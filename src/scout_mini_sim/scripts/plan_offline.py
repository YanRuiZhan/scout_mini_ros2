#!/usr/bin/env python3
"""离线视点规划：为每台设备生成并评分候选观测位姿，计算视点之间的路程矩阵，
写入 patrol_plan.yaml，供 ``poi_patrol.py --plan`` 使用。

为什么离线
----------
候选视点的评分（视野、入射角、遮挡、安全距离）只取决于地图和设备，不随每次巡检变化；
视点与视点之间的路程也一样。离线算一次存成文件，巡检启动时只需在线补算
"起点 -> 各视点"这一行，再求访问顺序。

整体流程
--------
1. 读入 POI（设备位置 target、检测面外法向 normal），转换到地图坐标系（--poi-transform）。
   设备检测面的中心高度 z 和宽高 w / h 按以下优先级取得：
   POI 文件里的 z / size  >  Gazebo world 的模型网格（--world）  >  按设备类型的缺省值。
2. 读入二维栅格地图（--map），算出每个格子到最近障碍物的距离（安全余量）；
   可选读入建图得到的 PCD（--pcd）做三维遮挡检查，没有 PCD 时用二维栅格保守检查。
3. 对每台设备，在检测面正前方按 (距离 r, 偏离法向的角度 phi) 采样候选位姿，车头朝向设备：
   距离上限默认 --r-max；装得高的设备整面入画需要退得更远时，自动放宽到 --r-limit。
   - 硬约束（不满足就丢弃，原因计入统计）：
       在地图内且是已知空闲格 / 离障碍物不小于 车身半径 + 余量 / 从起点可达 /
       设备整体落在相机视野内（竖直方向尤其关键）/ 可见比例不低于 --min-visibility
   - 软评分（0~1，按 WEIGHTS 加权求和）：
       size 成像大小接近 --target-fill；incidence 入射角越正越好；
       visibility 检测面可见比例；clearance 安全余量越大越好
4. 按得分从高到低挑 --keep 个视点，且彼此的偏角 phi 至少相差 --min-separation：
   最佳视点被临时遮挡时，备选来自不同方向，在线才有得换。
5. 在栅格上用代价感知的 8 邻域 A* 算所有入选视点两两之间的路程（tour_planner.GridMap）：
   代价模型与 Nav2 一致（膨胀层代价 + Smac 2D 的 cost_travel_multiplier），路径沿通道中线，
   比几何最短路更接近 Nav2 实际规划出的长度。
6. 写出 YAML，并记录 POI / 地图 / PCD 文件的 SHA-1；巡检时发现文件改过就会提示重新规划。

车身尺寸、膨胀层和 Smac 2D 的代价参数默认从 nav2_params.yaml 自动读取（--nav2-params），
与 Nav2 实际使用的保持一致；命令行显式给出时以命令行为准。

用法（在仓库根目录）
--------------------
    python3 src/scout_mini_sim/scripts/plan_offline.py \\
        --poi maps/warehouse_poi.yaml --map maps/warehouse.yaml --pcd maps/warehouse.pcd \\
        --world src/scout_mini_sim/worlds/warehouse.sdf --out maps/patrol_plan.yaml

只依赖 numpy 和 PyYAML，不需要启动 ROS。
"""
import argparse
import datetime
import hashlib
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import poi_patrol  # noqa: E402  复用 POI 读取、校验和坐标转换
from nav2_config import (  # noqa: E402  读取与 Nav2 一致的参数（巡检脚本做过期检查时也用它）
    NAV2_FALLBACK, default_nav2_params, footprint_radii, read_nav2_params)
import tour_planner  # noqa: E402  复用 PGM 读取和栅格 Dijkstra


# ============================================================================
# 可调常量（按现场情况修改）
# ============================================================================

# D435 彩色相机，与 URDF scout_mini_rdk.urdf.xacro 一致
CAMERA = {
    "hfov": 1.2113,         # 水平视场角 (rad)，约 69.4°
    "width": 1280,          # 图像宽 (px)
    "height": 720,          # 图像高 (px)
    "mount_height": 0.40,   # 光心离地高度 (m)，实车实测
    "forward": 0.222,       # 光心在 base_footprint 前方的距离 (m)
}

# 设备检测面的中心高度 z (m) 与宽 w、高 h (m)。
# 仅在 POI 文件没写 z / size、也没有 --world 可读时使用 —— 这是占位值，请按实物测量后修改。
TYPE_DEFAULTS = {
    "pressure_gauge": {"z": 1.2, "w": 0.3, "h": 0.3},
    "access_panel": {"z": 1.0, "w": 0.6, "h": 0.8},
    "level_tank": {"z": 1.0, "w": 0.8, "h": 1.2},
}
FALLBACK_GEOMETRY = {"z": 1.0, "w": 0.5, "h": 0.5}

# 软评分权重，和为 1。想更看重"拍得正"就加大 incidence，想更看重安全就加大 clearance。
WEIGHTS = {"size": 0.35, "incidence": 0.25, "visibility": 0.25, "clearance": 0.15}


def vertical_fov(camera=CAMERA):
    """由水平视场角和图像宽高比算竖直视场角 (rad)。D435 1280x720 约 42.6°。"""
    return 2 * math.atan(math.tan(camera["hfov"] / 2) * camera["height"] / camera["width"])


# ============================================================================
# 一、文件读取
# ============================================================================

def file_sha1(*paths):
    """若干文件内容合在一起的 SHA-1，用来判断离线结果是否过期。"""
    digest = hashlib.sha1()
    for path in paths:
        with open(path, "rb") as stream:
            for chunk in iter(lambda: stream.read(1 << 20), b""):
                digest.update(chunk)
    return digest.hexdigest()


def map_files(yaml_path):
    """地图 YAML 及其引用的图片，两者一起决定地图内容。"""
    meta = yaml.safe_load(Path(yaml_path).read_text())
    return [Path(yaml_path), Path(yaml_path).parent / meta["image"]]


def read_pcd_xyz(path):
    """读取未压缩的 ASCII / binary PCD，返回 N x 3 的 xyz 数组（丢弃非有限点）。"""
    header = {}
    with Path(path).open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"{path}: 缺少 PCD 的 DATA 行")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            key, *values = text.split()
            header[key] = values
            if key == "DATA":
                break
        fields, sizes, types = header["FIELDS"], list(map(int, header["SIZE"])), header["TYPE"]
        counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
        n = int(header["POINTS"][0])
        if header["DATA"][0] == "binary":
            dtype = np.dtype([(name, "<" + {"F": "f", "I": "i", "U": "u"}[kind] + str(size), (count,))
                              for name, size, kind, count in zip(fields, sizes, types, counts)])
            data = np.frombuffer(stream.read(), dtype=dtype, count=n)
            xyz = np.column_stack([data[a][:, 0] for a in ("x", "y", "z")]).astype(float)
        elif header["DATA"][0] == "ascii":
            data = np.loadtxt(stream, ndmin=2)
            offsets = np.cumsum([0] + counts[:-1])
            xyz = data[:, [offsets[fields.index(a)] for a in ("x", "y", "z")]]
        else:
            raise ValueError(f"{path}: 不支持压缩的 PCD（DATA binary_compressed）")
    return xyz[np.isfinite(xyz).all(axis=1)]


def geometry_from_world(world_sdf, ids):
    """从 Gazebo world 读设备检测面的中心高度和宽高。

    每台设备是 world 里的一个 <include>，名字为 poi_<id 小写，'-' 换成 '_'>，
    检测面网格在 <world 所在目录>/models/<名字>/meshes/face.obj：
    网格在模型自身坐标系下，检测面朝 -Y，宽度沿 X、高度沿 Z。
    找不到的设备不出现在返回值里。
    """
    root = ET.parse(world_sdf).getroot()
    models = Path(world_sdf).resolve().parent / "models"
    poses = {}
    for include in root.iter("include"):
        name, pose = include.findtext("name"), include.findtext("pose")
        if name and pose:
            poses[name] = [float(v) for v in pose.split()]
    found = {}
    for poi_id in ids:
        name = "poi_" + poi_id.lower().replace("-", "_")
        mesh = models / name / "meshes" / "face.obj"
        if name not in poses or not mesh.is_file():
            continue
        v = np.array([[float(t) for t in line.split()[1:4]]
                      for line in mesh.read_text().splitlines() if line.startswith("v ")])
        found[poi_id] = {"z": round(poses[name][2] + float(v[:, 2].mean()), 3),
                         "w": round(float(np.ptp(v[:, 0])), 3),
                         "h": round(float(np.ptp(v[:, 2])), 3)}
    return found


def device_geometry(poi, from_world):
    """返回 (几何, 来源)。优先级：POI 文件 > world 模型 > 类型缺省值。"""
    size = poi.get("size") or {}
    if "z" in poi and "w" in size and "h" in size:
        geo = {"z": float(poi["z"]), "w": float(size["w"]), "h": float(size["h"])}
        source = "poi"
    elif poi["id"] in from_world:
        geo, source = dict(from_world[poi["id"]]), "world"
    else:
        geo, source = dict(TYPE_DEFAULTS.get(poi["type"], FALLBACK_GEOMETRY)), "default"
    if not all(math.isfinite(v) and v > 0 for v in geo.values()):
        raise ValueError(f"{poi['id']}: 设备高度和尺寸必须为正数，得到 {geo}")
    return geo, source


# ============================================================================
# 二、与 Nav2 保持一致的参数：从 nav2_params.yaml 自动读取
# ============================================================================

def resolve_nav2_params(p, log=print):
    """补全 p 里与 Nav2 相关、命令行没有给出的参数。

    优先级：命令行显式给出 > nav2_params.yaml（--nav2-params，默认自动查找）> NAV2_FALLBACK。
    这样改了 nav2_params.yaml 后，离线规划自动跟着变，不用手动同步。
    """
    if p.nav2_params:
        path = Path(p.nav2_params)
        if not path.is_file():
            raise ValueError(f"--nav2-params 文件不存在: {path}")
    else:
        path = default_nav2_params()
    values = read_nav2_params(path) if path else dict(NAV2_FALLBACK)
    from_cli = [k for k in NAV2_FALLBACK if getattr(p, k) is not None]
    for key, value in values.items():
        if getattr(p, key) is None:
            setattr(p, key, value)
    p.nav2_params = str(path) if path else None
    log(f"Nav2 参数：{path or '未找到 nav2_params.yaml，用内置默认值'}\n"
        f"  车身内切 {p.inscribed_radius:.3f} m / 外接 {p.robot_radius:.3f} m，"
        f"膨胀半径 {p.inflation_radius:g} m，cost_scaling_factor {p.cost_scaling:g}，"
        f"cost_travel_multiplier {p.travel_multiplier:g}"
        + (f"（命令行覆盖：{', '.join(from_cli)}）" if from_cli else ""))
    return p


# ============================================================================
# 三、地图：二维栅格（必需）与三维体素（可选）
# ============================================================================

class Grid:
    """map_server 格式的二维栅格。数组下标为 [行, 列]，行号从下往上（第 0 行在 origin 处）。

    occ        占据格
    known_free 确定空闲的格子（机器人停车的位置必须是它）
    clearance  每格到最近占据格的近似距离 (m)，最大记到 max_clearance
    """

    def __init__(self, yaml_path, max_clearance=2.0):
        meta = yaml.safe_load(Path(yaml_path).read_text())
        width, height, pixels = tour_planner.read_pgm(Path(yaml_path).parent / meta["image"])
        image = np.array(pixels, dtype=float).reshape(height, width)[::-1]   # 翻成从下往上
        prob = image / 255.0 if int(meta.get("negate", 0)) else (255.0 - image) / 255.0
        self.occ = prob > float(meta.get("occupied_thresh", 0.65))
        self.known_free = prob < float(meta.get("free_thresh", 0.25))
        self.res = float(meta["resolution"])
        self.origin = (float(meta["origin"][0]), float(meta["origin"][1]))
        self.clearance = self._clearance(max_clearance)

    def _clearance(self, max_m):
        """从占据格出发逐圈膨胀：奇数圈只走上下左右，偶数圈加上对角，近似欧氏距离（误差 < 8%）。"""
        dist = np.full(self.occ.shape, max_m)
        reached = self.occ.copy()
        dist[reached] = 0.0
        for k in range(1, int(max_m / self.res) + 1):
            grown = reached.copy()
            grown[1:] |= reached[:-1]
            grown[:-1] |= reached[1:]
            grown[:, 1:] |= reached[:, :-1]
            grown[:, :-1] |= reached[:, 1:]
            if k % 2 == 0:
                grown[1:, 1:] |= reached[:-1, :-1]
                grown[1:, :-1] |= reached[:-1, 1:]
                grown[:-1, 1:] |= reached[1:, :-1]
                grown[:-1, :-1] |= reached[1:, 1:]
            dist[grown & ~reached] = k * self.res
            reached = grown
        return dist

    def cell(self, x, y):
        """世界坐标 -> (行, 列)；在地图外返回 None。"""
        row, col = int((y - self.origin[1]) // self.res), int((x - self.origin[0]) // self.res)
        if 0 <= row < self.occ.shape[0] and 0 <= col < self.occ.shape[1]:
            return row, col
        return None

    def segment_blocked(self, a, b, skip_end):
        """平面线段 a -> b（忽略最后 skip_end 米）是否穿过占据格。地图外视为不挡。"""
        total = math.dist(a, b)
        for t in np.arange(0.0, max(total - skip_end, 0.0), self.res / 2):
            s = t / total
            c = self.cell(a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]))
            if c is not None and self.occ[c]:
                return True
        return False


class Voxels:
    """PCD 体素化后的三维占据，用于"视线是否被挡住"。

    坐标为 (x, y, 离地高度)。离地低于 min_height 的点当作地面丢掉。
    ground_z 为 PCD 坐标系下地面的 z；None 时取所有点 z 的 1% 分位数（仓库里地面点最多）。
    """

    def __init__(self, xyz, res=0.1, ground_z=None, min_height=0.08):
        self.ground_z = float(np.percentile(xyz[:, 2], 1.0)) if ground_z is None else ground_z
        pts = xyz.copy()
        pts[:, 2] -= self.ground_z
        pts = pts[pts[:, 2] > min_height]
        self.res = res
        self.lo = pts.min(axis=0) if len(pts) else np.zeros(3)
        idx = np.floor((pts - self.lo) / res).astype(int)
        self.occ = np.zeros(idx.max(axis=0) + 1 if len(idx) else (1, 1, 1), dtype=bool)
        self.occ[tuple(idx.T)] = True

    def segment_blocked(self, a, b, skip_end):
        """三维线段 a -> b（忽略最后 skip_end 米）是否穿过占据体素。"""
        a, b = np.asarray(a, float), np.asarray(b, float)
        length = np.linalg.norm(b - a)
        steps = np.arange(0.0, max(length - skip_end, 0.0), self.res / 2)
        if not len(steps):
            return False
        pts = a + np.outer(steps / length, b - a)
        idx = np.floor((pts - self.lo) / self.res).astype(int)
        inside = np.all((idx >= 0) & (idx < self.occ.shape), axis=1)
        return bool(self.occ[tuple(idx[inside].T)].any())


# ============================================================================
# 四、候选视点：采样、约束检查、评分、挑选
# ============================================================================

def face_points(poi, geo, n=3):
    """检测面上 n x n 个采样点 (x, y, 离地高度)，覆盖面的中间 80%。"""
    tx, ty = poi["target"]["x"], poi["target"]["y"]
    nx, ny = unit(poi["normal"])
    ux, uy = -ny, nx                            # 检测面内的水平方向
    offsets = np.linspace(-0.4, 0.4, n)
    return [(tx + a * geo["w"] * ux, ty + a * geo["w"] * uy, geo["z"] + b * geo["h"])
            for a in offsets for b in offsets]


def unit(vec):
    norm = math.hypot(vec["x"], vec["y"])
    return vec["x"] / norm, vec["y"] / norm


def fit_distance(geo, p, camera=CAMERA):
    """正对检测面时，整面完整入画所需的最小距离 r（base_footprint 到检测面，m）。

    竖直方向：相机水平安装、离地 mount_height，检测面上沿高出相机 top、下沿低于相机 bottom，
    要求 top / r_cam ≤ tan(半竖直视场)、bottom / r_cam ≤ tan(半竖直视场)；
    水平方向：半宽 / r_cam ≤ tan(半水平视场)。三者取最大，再加上相机在车体前方的距离。
    例：面板上沿离地 1.9 m、相机 0.4 m，高出 1.5 m，半竖直视场留余量后约 18.4°，
        需要 r_cam ≥ 1.5 / tan(18.4°) ≈ 4.5 m —— 这就是"装得高的设备要退远"的来源。
    """
    tan_v = math.tan(vertical_fov(camera) / 2 - p.fov_margin)
    tan_h = math.tan(camera["hfov"] / 2 - p.fov_margin)
    top = geo["z"] + geo["h"] / 2 - camera["mount_height"]
    bottom = camera["mount_height"] - (geo["z"] - geo["h"] / 2)
    need = max(max(top, 0.0) / tan_v, max(bottom, 0.0) / tan_v, geo["w"] / 2 / tan_h)
    return need + camera["forward"]


def distance_range(geo, p):
    """这台设备的采样距离范围 [r_min, r_hi]。

    整面入画所需距离超过 --r-max 时，把上限放宽到"所需距离 + 1 m"，最多到 --r-limit；
    不超过时保持 --r-max，不额外增加计算量。
    """
    r_fit = fit_distance(geo, p)
    r_hi = p.r_max if r_fit <= p.r_max - 0.5 else min(p.r_limit, max(p.r_max, r_fit + 1.0))
    return p.r_min, r_hi


def sample_candidates(poi, p, r_hi=None):
    """在检测面正前方按 (r, phi) 网格采样候选位姿，车头朝向设备中心。"""
    tx, ty = poi["target"]["x"], poi["target"]["y"]
    nx, ny = unit(poi["normal"])
    r_hi = p.r_max if r_hi is None else r_hi
    for r in map(float, np.arange(p.r_min, r_hi + 1e-9, p.r_step)):
        for phi_deg in map(float, np.arange(-p.phi_max, p.phi_max + 1e-9, p.phi_step)):
            phi = math.radians(phi_deg)
            dx = nx * math.cos(phi) - ny * math.sin(phi)      # 法向旋转 phi
            dy = nx * math.sin(phi) + ny * math.cos(phi)
            yield {"x": tx + r * dx, "y": ty + r * dy, "yaw": math.atan2(-dy, -dx),
                   "r": r, "phi_deg": phi_deg}


def evaluate(c, poi, geo, grid, voxels, reach, p, camera=CAMERA):
    """检查一个候选位姿：不满足硬约束返回 (None, 原因)；否则返回 (各项得分, None)。"""
    # ---- 1. 停车位置：在地图内、已知空闲、离障碍物足够远、从起点可达 ----
    cell = grid.cell(c["x"], c["y"])
    if cell is None or not grid.known_free[cell]:
        return None, "not_free"
    need = p.robot_radius + p.clearance_margin
    if grid.clearance[cell] < need:
        return None, "too_close"
    if reach is not None and not reach(c["x"], c["y"]):
        return None, "unreachable"

    # ---- 2. 视野：设备上下沿、左右沿都要在画面内（留 fov_margin 余量）----
    r_cam = c["r"] - camera["forward"]          # 相机在车体前方，比 base_footprint 离设备更近
    if r_cam <= 0.3:
        return None, "fov"
    phi = math.radians(c["phi_deg"])
    half_v = vertical_fov(camera) / 2 - p.fov_margin
    half_h = camera["hfov"] / 2 - p.fov_margin
    top = geo["z"] + geo["h"] / 2 - camera["mount_height"]
    bottom = geo["z"] - geo["h"] / 2 - camera["mount_height"]
    side = geo["w"] * math.cos(phi) / 2         # 斜着看时检测面在水平方向上变窄
    if (math.atan2(top, r_cam) > half_v or math.atan2(bottom, r_cam) < -half_v
            or math.atan2(side, r_cam) > half_h):
        return None, "fov"

    # ---- 3. 遮挡：从相机光心到检测面各采样点的视线有多少没被挡住 ----
    cam = (c["x"] + camera["forward"] * math.cos(c["yaw"]),
           c["y"] + camera["forward"] * math.sin(c["yaw"]), camera["mount_height"])
    targets = face_points(poi, geo)
    if voxels is not None:
        blocked = [voxels.segment_blocked(cam, t, p.skip_end) for t in targets]
    else:   # 没有 PCD：二维栅格里任何占据格都当作"一直顶到天花板"，偏保守
        blocked = [grid.segment_blocked(cam[:2], t[:2], p.skip_end) for t in targets]
    visibility = 1.0 - sum(blocked) / len(blocked)
    if visibility < p.min_visibility:
        return None, "occluded"

    # ---- 4. 软评分（各项 0~1）----
    fill = max(geo["w"] * math.cos(phi) / (2 * r_cam * math.tan(camera["hfov"] / 2)),
               geo["h"] / (2 * r_cam * math.tan(vertical_fov(camera) / 2)))
    scores = {
        "size": math.exp(-((fill - p.target_fill) / 0.15) ** 2),   # 画面占比接近 target_fill 最好
        "incidence": math.cos(phi),                                  # 正对检测面最好
        "visibility": visibility,
        "clearance": min(1.0, (grid.clearance[cell] - need) / 0.5),  # 多出 0.5 m 余量就算满分
    }
    return scores, None


def select_diverse(scored, keep, min_separation):
    """按得分从高到低挑 keep 个，且偏角两两相差至少 min_separation 度。"""
    chosen = []
    for c in sorted(scored, key=lambda c: -c["score"]):
        if all(abs(c["phi_deg"] - o["phi_deg"]) >= min_separation - 1e-9 for o in chosen):
            chosen.append(c)
            if len(chosen) == keep:
                break
    return chosen


def explain_failure(r_fit, rejected, p):
    """一台设备没有可用视点时，给出最可能的原因和改法。"""
    if r_fit > p.r_limit:
        return (f"原因：整面入画要退到 {r_fit:.1f} m，超过 --r-limit {p.r_limit:g} m。"
                "可加大 --r-limit（前方要有这么大空地）、把相机调成上仰，"
                "或在 POI 里只写需要看清的区域的 z / size（如面板上的仪表区）")
    top = max(rejected, key=rejected.get) if rejected else ""
    hints = {
        "fov": f"原因：{p.r_min:g}～{p.r_limit:g} m 内能放下整面的位置都被其它约束排除了，"
               "检查设备前方是否有足够空地",
        "too_close": "原因：设备前方太窄，停车位置离障碍物不够远；可减小 --clearance-margin",
        "not_free": "原因：设备前方在地图里是障碍物或未知区域，检查 POI 坐标和法向",
        "occluded": "原因：视线被挡住；检查 PCD 里设备前方的障碍物，或减小 --min-visibility",
        "unreachable": "原因：从 --start 开不到设备前方",
    }
    return hints.get(top, "原因不明，请检查设备坐标、法向和地图")


def plan_device(poi, geo, grid, voxels, reach, p):
    """一台设备的候选视点：返回 (入选列表, 被丢弃原因的计数)。"""
    scored, rejected = [], {}
    for c in sample_candidates(poi, p, distance_range(geo, p)[1]):
        scores, reason = evaluate(c, poi, geo, grid, voxels, reach, p)
        if scores is None:
            rejected[reason] = rejected.get(reason, 0) + 1
            continue
        c.update(score=float(sum(WEIGHTS[k] * v for k, v in scores.items())),
                 visibility=float(scores["visibility"]),
                 clearance=float(grid.clearance[grid.cell(c["x"], c["y"])]))
        scored.append(c)
    return select_diverse(scored, p.keep, p.min_separation), rejected


# ============================================================================
# 五、主流程
# ============================================================================

def make_plan(p, log=print):
    """完整的离线规划，返回写入 YAML 的字典。p 为 argparse 结果。"""
    poi_args = argparse.Namespace(poi=p.poi, only=p.only, frame=p.frame,
                                  poi_transform=p.poi_transform, dry_run=False)
    pois, _ = poi_patrol.prepare_pois(poi_args)
    from_world = geometry_from_world(p.world, [x["id"] for x in pois]) if p.world else {}

    log(f"读取地图 {p.map} …")
    grid = Grid(p.map)
    voxels = None
    if p.pcd:
        log(f"读取 PCD {p.pcd} …")
        voxels = Voxels(read_pcd_xyz(p.pcd), res=p.voxel, ground_z=p.ground_z)
        log(f"  地面 z = {voxels.ground_z:.3f}，体素 {voxels.occ.shape}")

    # 路程计算用的栅格：按车身内切半径膨胀（与 Smac 2D 一致），并带 Nav2 膨胀层代价
    gm = tour_planner.GridMap(p.map, robot_radius=p.inscribed_radius, step=p.grid_step, costmap={
        "inflation_radius": p.inflation_radius, "cost_scaling_factor": p.cost_scaling,
        "travel_multiplier": p.travel_multiplier, "inscribed_radius": p.inscribed_radius})
    # 可达性：从起点做一次 Dijkstra，能到的格子都可达
    start_cell = gm.free_cell_near(*p.start)
    if start_cell is None:
        raise ValueError(f"起点 {p.start} 附近没有可通行的格子，请检查 --start")
    reached = gm._dijkstra(start_cell)

    def reach(x, y):
        return gm.cell(x, y) in reached

    devices, viewpoints = [], []
    for poi in pois:
        geo, source = device_geometry(poi, from_world)
        chosen, rejected = plan_device(poi, geo, grid, voxels, reach, p)
        r_fit = fit_distance(geo, p)
        summary = ", ".join(f"{k} {v}" for k, v in sorted(rejected.items())) or "无"
        log(f"{poi['id']:<8} 几何来源 {source:<7} 中心高 {geo['z']:.2f} m，宽×高 "
            f"{geo['w']:.2f}×{geo['h']:.2f} m，整面入画需 r ≥ {r_fit:.2f} m；"
            f"入选 {len(chosen)} 个；丢弃：{summary}")
        if not chosen:
            log("         " + explain_failure(r_fit, rejected, p))
        indices = []
        for rank, c in enumerate(chosen, 1):
            indices.append(len(viewpoints))
            viewpoints.append({
                "device": poi["id"], "rank": rank,
                "x": round(c["x"], 3), "y": round(c["y"], 3), "yaw": round(c["yaw"], 4),
                "r": round(c["r"], 3), "phi_deg": round(c["phi_deg"], 1),
                "score": round(c["score"], 4), "visibility": round(c["visibility"], 3),
                "clearance": round(c["clearance"], 3),
            })
        devices.append({
            "id": poi["id"], "type": poi["type"],
            "target": {k: round(v, 4) for k, v in poi["target"].items()},
            "normal": {k: round(v, 4) for k, v in poi["normal"].items()},
            "geometry": dict(geo, source=source, fit_distance=round(r_fit, 3)),
            "viewpoints": indices,
        })

    # 视点之间的路程矩阵；起点那一行单独存（起点固定时可直接用，否则巡检时在线重算）
    n = len(viewpoints)
    jobs = n * (n - 1) // 2 if p.search == "astar" else n
    log(f"计算 {n} 个视点两两之间的路程（代价感知 8 邻域 {p.search.upper()}，共 {jobs} 次搜索）…")
    started = datetime.datetime.now()

    def progress(k, total):
        if k == total or k % max(1, total // 10) == 0:
            log(f"  {k}/{total}，已用 {(datetime.datetime.now() - started).seconds} s")

    points = [(v["x"], v["y"]) for v in viewpoints]
    matrix = gm.distances(points, method=p.search, progress=progress) if points else []
    # 起点那一行：同一代价模型，从起点到每个视点
    cells = [gm.free_cell_near(v["x"], v["y"]) for v in viewpoints]
    if p.search == "astar":
        start_row = [gm.search(start_cell, c) * gm.res if c is not None else math.inf for c in cells]
    else:
        lengths = gm.search(start_cell)
        start_row = [lengths.get(c, math.inf) * gm.res if c is not None else math.inf for c in cells]

    sources = {"poi": {"path": str(p.poi), "sha1": file_sha1(p.poi)},
               # 记录 nav2_params.yaml 里读到的值；巡检时与当前文件比较，相关参数改过就提示重新规划
               "nav2_params": ({"path": p.nav2_params, "sha1": file_sha1(p.nav2_params),
                                "values": read_nav2_params(p.nav2_params)}
                               if p.nav2_params else None),
               "map": {"path": str(p.map), "sha1": file_sha1(*map_files(p.map))},
               "pcd": {"path": str(p.pcd), "sha1": file_sha1(p.pcd)} if p.pcd else None,
               "world": str(p.world) if p.world else None}
    params = {k: v for k, v in vars(p).items()
              if k not in ("poi", "map", "pcd", "world", "out", "only", "nav2_params")}
    params.update(camera=CAMERA, weights=WEIGHTS)
    return {
        "version": 1,
        "frame_id": p.frame,
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "sources": sources,
        "params": params,
        "distance": f"grid_{p.search}_cost_aware",
        "devices": devices,
        "viewpoints": viewpoints,
        "matrix": [[round(float(d), 3) for d in row] for row in matrix],
        "start": {"x": p.start[0], "y": p.start[1],
                  "row": [round(float(d), 3) for d in start_row]},
    }


def build_parser():
    ap = argparse.ArgumentParser(description="离线视点规划：候选视点 + 路程矩阵 -> patrol_plan.yaml")
    # --- 输入 ---
    ap.add_argument("--poi", required=True, help="POI 配置文件（与 poi_patrol.py 相同）")
    ap.add_argument("--map", required=True, help="二维栅格地图 YAML（与 Nav2 用的同一张）")
    ap.add_argument("--pcd", help="建图得到的 PCD，用于三维遮挡检查；不给则用二维栅格保守检查")
    ap.add_argument("--world", help="Gazebo world SDF，从模型网格读取设备高度和尺寸（仿真用）")
    ap.add_argument("--out", default="patrol_plan.yaml", help="输出文件")
    ap.add_argument("--only", nargs="+", help="只规划这些设备编号")
    # --- 坐标系 ---
    ap.add_argument("--frame", default="map", help="地图坐标系名，写入输出文件")
    ap.add_argument("--poi-transform", type=float, nargs=3, metavar=("TX", "TY", "YAW"),
                    help="POI 坐标转到地图坐标：p_map = R(YAW)*p_poi + [TX, TY]")
    ap.add_argument("--start", type=float, nargs=2, default=[0.0, 0.0], metavar=("X", "Y"),
                    help="出发位置（地图坐标），用于可达性检查；FAST-LIO 地图的原点就是建图起点")
    ap.add_argument("--ground-z", type=float, help="PCD 中地面的 z；默认自动估计")
    # --- 候选采样 ---
    ap.add_argument("--r-min", type=float, default=1.5, help="离检测面的最小距离 (m)")
    ap.add_argument("--r-max", type=float, default=4.0, help="离检测面的常规最大距离 (m)")
    ap.add_argument("--r-limit", type=float, default=7.0,
                    help="装得高或尺寸大的设备在 --r-max 内放不进画面时，最多退到这个距离 (m)")
    ap.add_argument("--r-step", type=float, default=0.25)
    ap.add_argument("--phi-max", type=float, default=60.0, help="偏离法向的最大角度 (°)")
    ap.add_argument("--phi-step", type=float, default=10.0)
    # --- 约束与评分 ---
    ap.add_argument("--robot-radius", type=float,
                    help="车身外接圆半径 (m)；默认由 nav2_params.yaml 的 footprint 算出（约 0.44）")
    ap.add_argument("--clearance-margin", type=float, default=0.15, help="在车身半径之外再留的余量 (m)")
    ap.add_argument("--fov-margin", type=float, default=0.05, help="视野边缘留出的角度 (rad)")
    ap.add_argument("--min-visibility", type=float, default=0.75, help="检测面最低可见比例")
    ap.add_argument("--target-fill", type=float, default=0.35, help="理想的画面占比（宽或高）")
    ap.add_argument("--skip-end", type=float, default=0.25,
                    help="视线检查忽略靠近设备的最后这段 (m)，设备本身和它所在的墙不算遮挡")
    ap.add_argument("--voxel", type=float, default=0.1, help="PCD 体素边长 (m)")
    # --- 输出 ---
    ap.add_argument("--keep", type=int, default=3, help="每台设备保留的视点数")
    ap.add_argument("--min-separation", type=float, default=30.0, help="入选视点偏角的最小差 (°)")
    # --- 路程矩阵（仿照 Nav2：Smac Planner 2D + 全局代价地图膨胀层）---
    ap.add_argument("--search", default="astar", choices=("astar", "dijkstra"),
                    help="astar：每对视点一次 8 邻域 A*；dijkstra：每个视点一次一对全部，点多时更快，结果相同")
    ap.add_argument("--grid-step", type=int, default=2, help="路程计算时栅格降采样倍数（越大越快越粗）")
    # 下面四项默认从 nav2_params.yaml 读取（见 resolve_nav2_params），命令行给出时以命令行为准
    ap.add_argument("--nav2-params",
                    help="Nav2 参数文件；默认自动找 scout_mini_navigation 的 config/nav2_params.yaml")
    ap.add_argument("--inscribed-radius", type=float,
                    help="车身内切半径 (m)，路径搜索按它膨胀障碍物；默认由 footprint 算出（约 0.31）")
    ap.add_argument("--inflation-radius", type=float, help="全局代价地图膨胀半径 (m)")
    ap.add_argument("--cost-scaling", type=float, help="膨胀层 cost_scaling_factor")
    ap.add_argument("--travel-multiplier", type=float, help="Smac 2D cost_travel_multiplier")
    return ap


def main(argv=None):
    p = build_parser().parse_args(argv)
    try:
        resolve_nav2_params(p)
        positive = [p.r_min, p.r_max, p.r_limit, p.r_step, p.phi_step, p.robot_radius, p.voxel,
                    p.target_fill, p.inscribed_radius, p.inflation_radius, p.cost_scaling]
        if any(not math.isfinite(v) or v <= 0 for v in positive) or not p.r_min <= p.r_max <= p.r_limit:
            raise ValueError("距离、步长、半径、体素、画面占比和代价参数必须为正数，"
                             "且 --r-min ≤ --r-max ≤ --r-limit")
        if not math.isfinite(p.travel_multiplier) or p.travel_multiplier < 0:
            raise ValueError("--travel-multiplier 必须为非负数")
        if p.keep < 1 or p.grid_step < 1 or not 0 <= p.min_visibility <= 1:
            raise ValueError("--keep、--grid-step 至少为 1，--min-visibility 在 0~1 之间")
        plan = make_plan(p)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        print(f"离线规划错误: {error}", file=sys.stderr)
        return 1
    Path(p.out).parent.mkdir(parents=True, exist_ok=True)
    with open(p.out, "w", encoding="utf-8") as stream:
        stream.write("# 由 plan_offline.py 生成，请勿手改；地图或 POI 改动后重新生成\n")
        yaml.safe_dump(plan, stream, allow_unicode=True, sort_keys=False, default_flow_style=None)
    missing = [d["id"] for d in plan["devices"] if not d["viewpoints"]]
    print(f"已写入 {p.out}：{len(plan['devices'])} 台设备，{len(plan['viewpoints'])} 个视点"
          + (f"；没有可用视点：{', '.join(missing)}" if missing else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
