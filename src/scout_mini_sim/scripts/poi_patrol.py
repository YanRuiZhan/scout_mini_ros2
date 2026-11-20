#!/usr/bin/env python3
"""Scout Mini POI 巡检：生成观测位姿、Nav2 导航、到点拍照并记录 CSV。

整体流程
--------
1. 读取 POI 配置（YAML），校验每个目标的 id / type / target / normal。
2. 若 POI 不在 Nav2 的 map 坐标系下，用 --poi-transform 转换坐标。
3. 对每个 POI 确定"观测位姿"（机器人面朝目标）：
   - 默认：沿目标表面外法向依次退后 --dist 列出的距离，取第一个 Nav2 能规划到的；
   - --plan：使用 plan_offline.py 离线评分好的候选视点，按评分依次尝试。
4. 求访问顺序（tour_planner.py，默认精确最优，--order 可选 greedy / two_opt）：
   - 默认：用 Nav2 规划器在线求起点与各观测点两两之间的路径长度；
   - --plan：视点之间用离线路程矩阵，只在线算"起点 -> 各视点"这一行。
5. 依次：导航 → 到点稳定 → 拍一张新图像 → 写一行 CSV；某个点失败时，
   以当前位置为起点对剩余的点重新排序。
   近点兜底：小车已到视点附近（--near-radius）却停着不动超过 --stuck-time，
   说明 Nav2 卡在到点判定的边界上；这时取消 Nav2 任务，用 Spin 原地对准朝向后照常拍照，
   结果记为 reached_near（原理见 NearGoalMonitor）。
6. 全部巡检完后导航回出发时的位姿（--no-return-home 可关闭），也记一行 CSV。
7. 结束或 Ctrl+C 时取消导航目标，并向 /cmd_vel 发零速度停车。

使用前提
--------
先启动 sim.launch.py（cameras:=true）和 navigation.launch.py，并在 RViz 完成定位；
或用 sim_patrol.launch.py（scout_mini_navigation 包）一键启动仿真、导航和本脚本。
无需 ROS 的检查：python3 poi_patrol.py --poi <配置.yaml> --dry-run
运行与坐标转换说明见 scout_mini_sim/README.md。

代码结构
--------
- 纯 Python 部分（不依赖 ROS，可单独测试 / dry-run）：
  load_pois、view_pose、angle_diff、NearGoalMonitor、order_pois、prepare_pois、load_plan、write_log
- 访问顺序：tour_planner.py（同目录的独立模块，也可单独运行做离线分析）
- ROS 部分（ROS 相关模块在函数内部才导入，dry-run 时不需要 ROS 环境）：
  wait_for_navigation、run
- 命令行入口：main
"""
import argparse
import csv
import math
import os
from pathlib import Path
import re
import sys
from tempfile import mkdtemp
import time

import yaml

# tour_planner.py 与本脚本在同一目录（源码目录或安装后的 lib/scout_mini_sim）
sys.path.insert(0, str(Path(__file__).resolve().parent))
import tour_planner  # noqa: E402
import nav2_config  # noqa: E402  读取 nav2_params.yaml 里与离线规划相关的参数


# CSV 日志的列，每个 POI 一行：
#   order        巡检顺序（从 1 开始）；最后一行 id=HOME 为返回起点
#   result       导航结果：reached / reached_near / no_plan / rejected / timeout / canceled /
#                failed / interrupted。reached_near = 近点卡住后由本脚本对准朝向（见 NearGoalMonitor）
#   view_dist    实际采用的观测距离 (m)，即视点离检测面的距离
#   view_rank    采用的是离线规划的第几候选（1 = 评分最高）；未用 --plan 时为空
#   plan_m       出发前规划的这一段路程 (m)，按 Nav2 路径长度（或 --distance 指定的方式）
#   goal_*       发给 Nav2 的目标位姿；robot_* 为到达后从 TF 读到的实际位置
#   pos_err_m    实际位置与目标位置的距离 (m)
#   yaw_err_deg  实际朝向减目标朝向 (°)，正数 = 车头偏左（逆时针）
#   time_s       导航耗时（墙钟时间，s）
#   image        保存的图片文件名；image_result 为 saved / no_image / error
LOG_FIELDS = [
    "order", "id", "type", "result", "view_dist", "view_rank", "plan_m", "goal_x", "goal_y", "goal_yaw",
    "robot_x", "robot_y", "pos_err_m", "yaw_err_deg", "time_s", "image", "image_result",
]

# 视为"到达"、会拍照的结果
REACHED = ("reached", "reached_near")

# lifecycle_msgs/State 中 PRIMARY_STATE_ACTIVE 的取值：节点已激活，可以接收任务。
LIFECYCLE_ACTIVE = 3


# ============================================================================
# 一、POI 数据处理（纯 Python，不依赖 ROS）
# ============================================================================

def load_pois(path):
    """读取并校验 POI 文件，返回 POI 列表。

    文件格式（见 config/poi_warehouse_inspection.yaml）::

        frame_id: world          # 可选，POI 坐标所在坐标系
        pois:
          - id: PG-W01           # 唯一编号
            type: pressure_gauge # 目标类型，仅用于记录
            target: {x: 7.65, y: -14.0}   # 目标表面中心
            normal: {x: 0.0, y: 1.0}      # 表面外法向（指向可观测的一侧）

    所有数据问题都在启动 ROS 之前以 ValueError 报出，避免车跑到一半才发现配置错误。
    target / normal 会被统一转换成 float。
    """
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get("pois"), list):
        raise ValueError("POI 文件必须包含 pois 列表")

    ids = set()
    for index, poi in enumerate(data["pois"], 1):
        # --- id 和 type：非空字符串，id 不能重复 ---
        if not isinstance(poi, dict):
            raise ValueError(f"第 {index} 个 POI 必须是映射")
        if not isinstance(poi.get("id"), str) or not poi["id"].strip():
            raise ValueError(f"第 {index} 个 POI 缺少非空字符串 id")
        if poi["id"] in ids:
            raise ValueError(f"POI id 重复: {poi['id']}")
        ids.add(poi["id"])
        if not isinstance(poi.get("type"), str) or not poi["type"].strip():
            raise ValueError(f"POI {poi['id']} 缺少非空字符串 type")

        # --- target 和 normal：必须有有限的数字 x / y ---
        for group in ("target", "normal"):
            try:
                xy = [float(poi[group][axis]) for axis in ("x", "y")]
            except (KeyError, TypeError, ValueError):
                raise ValueError(f"POI {poi['id']} 的 {group} 必须包含数字 x/y") from None
            if not all(math.isfinite(value) for value in xy):
                raise ValueError(f"POI {poi['id']} 的 {group}.x/y 必须是有限数")
            poi[group] = dict(zip(("x", "y"), xy))

        # --- 法向量不能是零向量，否则无法确定观测方向 ---
        norm = math.hypot(poi["normal"]["x"], poi["normal"]["y"])
        if not math.isfinite(norm) or norm == 0:
            raise ValueError(f"POI {poi['id']} 的 normal 不能为零或溢出向量")
    return data["pois"]


def view_pose(poi, dist):
    """计算观测位姿 (x, y, yaw)：从目标沿外法向退后 dist 米，车头 (+X) 朝向目标。

    设单位法向 n = (nx, ny)：
        观测点   p = target + dist * n
        朝向   yaw = atan2(-ny, -nx)      # 即 -n 的方向，从观测点指回目标
    """
    target, normal = poi["target"], poi["normal"]
    norm = math.hypot(normal["x"], normal["y"])
    nx, ny = normal["x"] / norm, normal["y"] / norm  # 归一化，配置里的法向不必是单位向量
    return target["x"] + dist * nx, target["y"] + dist * ny, math.atan2(-ny, -nx)


def yaw_from_quaternion(q):
    """四元数 -> 绕 Z 轴的航向角 (rad)。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    """角度差 a - b，归一化到 [-pi, pi)。例如 angle_diff(170°, -170°) = -20°，而不是 340°。"""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class NearGoalMonitor:
    """判断机器人是否"已经到了视点附近，却停在那里不动"（近点卡住）。

    为什么需要它
    ------------
    Humble 的 RPP 控制器在离"路径终点"不到 xy_goal_tolerance 时就停止前进、原地调朝向；
    而到点判定要求离"真实目标"不到同一个 xy_goal_tolerance。路径终点落在栅格中心上，
    和真实目标差几厘米，于是可能出现：RPP 认为到了，到点判定认为没到，小车原地不动，
    10 多秒后 Nav2 报 "Failed to make progress"，行为树开始原地转圈恢复。
    这个类发现这种情况后，由 navigate() 取消 Nav2 任务、自己把车头转正再拍照。

    判定规则
    --------
    - 离目标超过 radius：不算近点，计时清零；
    - 在 radius 以内：以当前位置为锚点开始计时；只要离锚点移动超过 move_tol 就重新计时
      （说明 Nav2 还在正常往里开）；
    - 在锚点附近连续停留 stuck_time 秒：判定为近点卡住，update() 返回 True。
    radius <= 0 表示关闭此功能，update() 永远返回 False。
    """

    def __init__(self, goal_xy, radius, stuck_time, move_tol=0.1):
        self.goal = tuple(goal_xy)
        self.radius = radius
        self.stuck_time = stuck_time
        self.move_tol = move_tol
        self.anchor = None      # 进入近点范围后的参考位置
        self.since = None       # 在参考位置附近开始停留的时刻

    def update(self, xy, now):
        """输入当前位置 (x, y) 和当前时间 now (s)，返回是否已判定为近点卡住。"""
        if self.radius <= 0 or math.dist(xy, self.goal) > self.radius:
            self.anchor = None
            return False
        if self.anchor is None or math.dist(xy, self.anchor) > self.move_tol:
            self.anchor, self.since = tuple(xy), now
            return False
        return now - self.since >= self.stuck_time


def order_pois(pois, start_xy, dist, method="auto", return_home=True):
    """按直线距离排列 POI（dry-run 和测试用；正式巡检用 Nav2 路径长度，见 run）。"""
    views = [view_pose(poi, dist)[:2] for poi in pois]
    plan = tour_planner.plan_tour(start_xy, views, method=method, return_home=return_home)
    return [pois[k] for k in plan.order], plan


def order_nearest(pois, start_xy, dist):
    """贪心最近邻（直线距离、不回起点），保留给旧的调用方。不修改输入列表。"""
    return order_pois(pois, start_xy, dist, method="greedy", return_home=False)[0]


def path_length(path):
    """nav_msgs/Path 的长度 (m)；没有路径时为 inf。"""
    if path is None or not path.poses:
        return math.inf
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def prepare_pois(args):
    """加载 POI、按 --only 过滤、做坐标转换，返回 (pois, 坐标系名)。

    坐标系规则：
    - 文件里的 frame_id 表示 POI 坐标所在坐标系；没写时按 --frame（默认 map）处理。
    - 给了 --poi-transform TX TY YAW 时，把 POI 从源坐标系转到 --frame：
          p_goal = R(YAW) * p_poi + [TX, TY]
      其中 target 是"点"，要旋转再平移；normal 是"方向"，只旋转不平移。
    - 正式导航时，源坐标系必须与 --frame 一致，否则拒绝运行（dry-run 例外）。
      例如 warehouse 配置是 Gazebo world 坐标，不能直接当作 map 坐标使用。
    """
    pois = load_pois(args.poi)
    with open(args.poi, encoding="utf-8") as stream:
        source_frame = yaml.safe_load(stream).get("frame_id", args.frame)
    if not isinstance(source_frame, str) or not source_frame.strip():
        raise ValueError("frame_id 必须是非空字符串")

    # 只保留 --only 指定的编号；拼错的编号直接报错，而不是悄悄跳过。
    if args.only is not None:
        missing = set(args.only) - {poi["id"] for poi in pois}
        if missing:
            raise ValueError("POI 文件中不存在这些 id: " + ", ".join(sorted(missing)))
        pois = [poi for poi in pois if poi["id"] in args.only]
    if not pois:
        raise ValueError("没有可巡检的 POI")

    # 二维刚体变换：先旋转 YAW，再平移 (TX, TY)。
    if args.poi_transform is not None:
        tx, ty, yaw = args.poi_transform
        c, s = math.cos(yaw), math.sin(yaw)
        for poi in pois:
            for group in ("target", "normal"):
                x, y = poi[group]["x"], poi[group]["y"]
                poi[group] = {"x": c * x - s * y, "y": s * x + c * y}
            poi["target"]["x"] += tx  # 只有点需要平移，法向量不平移
            poi["target"]["y"] += ty
        source_frame = args.frame

    if source_frame != args.frame and not args.dry_run:
        raise ValueError(
            f"POI 属于 {source_frame}，目标坐标系为 {args.frame}；请提供 "
            "--poi-transform TX TY YAW（目标系中的平移和弧度旋转），或使用已转换的 POI 文件"
        )
    return pois, source_frame


def file_sha1(*paths):
    """若干文件内容合在一起的 SHA-1（与 plan_offline.py 的算法一致），用于判断离线规划是否过期。"""
    import hashlib
    digest = hashlib.sha1()
    for path in paths:
        with open(path, "rb") as stream:
            for chunk in iter(lambda: stream.read(1 << 20), b""):
                digest.update(chunk)
    return digest.hexdigest()


def load_plan(args):
    """读取 plan_offline.py 生成的离线规划，返回 (pois, 规划)。

    规划是一个字典：
      candidates  {设备 id: [候选视点, ...]}，按离线评分从高到低；
                  每个候选含 x / y / yaw / r / rank，index 为它在离线路程矩阵里的下标
      matrix      视点两两之间的离线路程 (m)
    过期检查（任何一项不通过都拒绝运行，提示重新运行 plan_offline.py）：
      - 给了 --map / --poi：比较文件 SHA-1 与生成规划时记录的是否一致，不一致说明地图或设备改过；
      - Nav2 参数：比较 nav2_params.yaml 里车身尺寸、膨胀层、cost_travel_multiplier 这几个值
        与生成规划时是否一致（--nav2-params 指定文件，默认自动查找；找不到时只警告、跳过）。
        只比这几个值，调控制器速度等无关参数不会让离线规划失效。
    """
    with open(args.plan, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get("version") != 1:
        raise ValueError(f"{args.plan} 不是 plan_offline.py 生成的离线规划")
    if data.get("frame_id") != args.frame:
        raise ValueError(f"离线规划属于坐标系 {data.get('frame_id')}，与 --frame {args.frame} 不一致")

    # ---- 过期检查 ----
    sources = data.get("sources") or {}
    checks = []
    if args.map:
        meta = yaml.safe_load(Path(args.map).read_text())
        checks.append(("地图", file_sha1(args.map, Path(args.map).parent / meta["image"]),
                       (sources.get("map") or {}).get("sha1")))
    if args.poi:
        checks.append(("POI 文件", file_sha1(args.poi), (sources.get("poi") or {}).get("sha1")))
    for name, now, then in checks:
        if now != then:
            raise ValueError(f"{name}在生成离线规划之后改动过，请重新运行 plan_offline.py")
    check_nav2_params(sources.get("nav2_params"), args)

    # ---- 设备与候选视点 ----
    viewpoints = data["viewpoints"]
    pois, candidates = [], {}
    for device in data["devices"]:
        pois.append({key: device[key] for key in ("id", "type", "target", "normal")})
        candidates[device["id"]] = [dict(viewpoints[i], index=i) for i in device["viewpoints"]]
    if args.only is not None:
        missing = set(args.only) - set(candidates)
        if missing:
            raise ValueError("离线规划中不存在这些 id: " + ", ".join(sorted(missing)))
        pois = [poi for poi in pois if poi["id"] in args.only]
    if not pois:
        raise ValueError("没有可巡检的 POI")
    return pois, {"candidates": candidates, "matrix": data["matrix"]}


def check_nav2_params(recorded, args):
    """离线规划用到的 Nav2 参数有没有改过；改过抛 ValueError。

    recorded 为规划文件里的 sources.nav2_params（含生成时读到的 values）；旧版规划文件没有它，跳过。
    """
    planned = (recorded or {}).get("values")
    if not planned:
        return
    if args.nav2_params:
        path = Path(args.nav2_params)
        if not path.is_file():
            raise ValueError(f"--nav2-params 文件不存在: {path}")
    else:
        path = nav2_config.default_nav2_params()
        if path is None:
            print("警告：找不到 nav2_params.yaml，跳过 Nav2 参数的过期检查", file=sys.stderr)
            return
    changed = nav2_config.compare_nav2_values(planned, nav2_config.read_nav2_params(path))
    if changed:
        detail = "；".join(f"{nav2_config.NAV2_LABELS.get(k, k)} {old:g} → {new}"
                          for k, old, new in changed)
        raise ValueError(f"Nav2 参数在生成离线规划之后改动过（{detail}），请重新运行 plan_offline.py")


def write_log(path, rows):
    """整体重写 CSV。每完成一个 POI 调用一次，中途崩溃也能保留已完成的记录。"""
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=LOG_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


# ============================================================================
# 二、ROS 部分：等待 Nav2 就绪、执行巡检
# ============================================================================

def wait_for_navigation(nav, buffer, args):
    """等待 Nav2 各节点激活、定位 TF 可用，返回当前 map -> base_footprint 变换。

    没有使用 BasicNavigator.waitUntilNav2Active()：它会自动发布 initialpose，
    可能覆盖用户在 RViz 里设置的 2D Pose Estimate。这里只"查询"不"设置"。
    所有等待共享一个截止时间 (--startup-timeout)，超时抛 RuntimeError。
    """
    import rclpy
    from lifecycle_msgs.srv import GetState
    from rclpy.time import Time
    from tf2_ros import TransformException

    deadline = time.monotonic() + args.startup_timeout

    # ---- 第 1 步：逐个查询生命周期节点，直到状态为 active ----
    nodes = ["bt_navigator", "planner_server", "controller_server"]
    if args.localizer not in ("none", "auto"):
        nodes.insert(0, args.localizer)  # 指定了定位节点名时，它也必须已激活
    # auto：不关心是哪个定位节点（AMCL 或三维 NDT），下面等到定位 TF 可用即可
    for name in nodes:
        client = nav.create_client(GetState, name.rstrip("/") + "/get_state")
        try:
            while rclpy.ok():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"等待 {name} active 超时，请先启动仿真和 Nav2")
                # 服务还没出现：处理一下回调再重试
                if not client.wait_for_service(timeout_sec=0.2):
                    rclpy.spin_once(nav, timeout_sec=0.1)
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(nav, future, timeout_sec=0.5)
                if future.done() and future.result() is not None:
                    if future.result().current_state.id == LIFECYCLE_ACTIVE:
                        break
                else:
                    # 本次请求没有及时返回，丢弃它，避免请求越积越多
                    client.remove_pending_request(future)
                rclpy.spin_once(nav, timeout_sec=0.1)
        finally:
            nav.destroy_client(client)

    # ---- 第 2 步：等待 /clock 和定位 TF ----
    while rclpy.ok():
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"等待 /clock 或 {args.frame} -> {args.base_frame} TF 超时；"
                "AMCL 模式请先在 RViz 设置 2D Pose Estimate"
            )
        rclpy.spin_once(nav, timeout_sec=0.1)
        # 仿真时间下，收到第一条 /clock 之前时钟为 0，此时 TF 查询没有意义
        if args.use_sim_time and nav.get_clock().now().nanoseconds == 0:
            continue
        try:
            # Time() 即时间 0，表示"取最新可用的变换"
            return buffer.lookup_transform(args.frame, args.base_frame, Time())
        except TransformException:
            continue  # AMCL 尚未收到初始位姿时，map -> odom 还不存在
    raise RuntimeError("ROS 已关闭")


def run(args, pois, plan=None):
    """连接 ROS，按顺序巡检所有 POI；结果写入 <out>/<时间_随机后缀>/。

    plan 为 load_plan() 的结果时使用离线候选视点和离线路程矩阵，只在线补算起点那一行；
    为 None 时按 --dist 在线生成视点、在线求整个路程矩阵。
    """
    # ROS 相关模块在这里才导入，这样 dry-run 和单元测试不需要 ROS 环境。
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from sensor_msgs.msg import Image
    from tf2_ros import Buffer, TransformException, TransformListener

    # 关闭 rclpy 自带的 SIGINT 处理，保留 Python 的 KeyboardInterrupt，
    # 这样 Ctrl+C 后 finally 块还能在 ROS context 关闭前取消目标、发送停车指令。
    rclpy.init(args=["--ros-args", "-p", f"use_sim_time:={str(args.use_sim_time).lower()}"],
               signal_handler_options=SignalHandlerOptions.NO)

    nav = None
    rows = []               # 所有 POI 的记录（CSV 行）
    log_path = None
    motion_started = False  # 是否已经发过导航目标；发过才需要在退出时停车
    task_active = False     # 当前是否有未结束的导航任务；有则退出时需要取消
    try:
        # ------------------------------------------------------------------
        # 初始化：导航器、TF、相机订阅、停车用的速度发布者
        # ------------------------------------------------------------------
        nav = BasicNavigator(node_name="poi_patrol")
        buffer = Buffer(node=nav)
        listener = TransformListener(buffer, nav)  # noqa: F841  需保持引用，否则不再接收 TF
        bridge = CvBridge()

        # 图像缓存。after_ns 为 None 时不接收任何图像；
        # 拍照时设为"当前时间"，只接收时间戳晚于它的新帧，避免拿到导航途中的旧图。
        latest = {"img": None, "after_ns": None}

        def on_image(msg):
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if latest["after_ns"] is not None and stamp_ns > latest["after_ns"]:
                latest["img"] = msg

        nav.create_subscription(Image, args.image_topic, on_image, qos_profile_sensor_data)
        stop_pub = nav.create_publisher(Twist, args.cmd_vel_topic, 10)

        # ------------------------------------------------------------------
        # 等待 Nav2 就绪，并以当前位置为起点排序
        # ------------------------------------------------------------------
        nav.get_logger().info("等待 Nav2 和定位 TF；AMCL 模式请在 RViz 设置初始位姿")
        current = wait_for_navigation(nav, buffer, args)
        # 出发时的真实位姿（map 坐标），巡检结束后回到这里
        home = (current.transform.translation.x, current.transform.translation.y,
                yaw_from_quaternion(current.transform.rotation))
        if args.start_x is None:
            start_xy = (current.transform.translation.x, current.transform.translation.y)
        else:
            start_xy = (args.start_x, args.start_y)  # 仅影响排序，不会重置定位

        # 每次任务建一个独立目录（时间 + 随机后缀），避免重跑时覆盖照片和 CSV。
        os.makedirs(args.out, exist_ok=True)
        output = Path(mkdtemp(prefix=time.strftime("%Y%m%d_%H%M%S_"), dir=args.out))
        log_path = output / "patrol_log.csv"
        write_log(log_path, rows)

        # ------------------------------------------------------------------
        # 每个 POI 用到的小步骤
        # ------------------------------------------------------------------
        def make_pose(x, y, yaw):
            """平面位姿 -> PoseStamped。绕 Z 轴旋转 yaw 的四元数为 (0, 0, sin(yaw/2), cos(yaw/2))。"""
            pose = PoseStamped()
            pose.header.frame_id = args.frame
            pose.header.stamp = nav.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            return pose

        def candidates_for(poi):
            """一台设备的候选视点，按优先顺序排列。

            有离线规划：plan_offline.py 评分最高的几个（来自不同方向）；
            没有：按 --dist 列出的距离，在检测面正前方依次生成。
            """
            if plan is not None:
                return plan["candidates"].get(poi["id"], [])
            return [dict(zip(("x", "y", "yaw"), view_pose(poi, dist)), r=dist, rank="", index=None)
                    for dist in args.dist]

        def choose_goal(poi, row):
            """按优先顺序尝试候选视点，返回 (第一个能规划出路径的目标, 候选)；都不行返回 (None, None)。

            例如最佳视点此刻被临时堆放的货物挡住（Nav2 规划不过去），就换下一个候选。
            """
            for cand in candidates_for(poi):
                goal = make_pose(cand["x"], cand["y"], cand["yaw"])
                # use_start=False：从机器人当前位姿规划，只用来检查这个视点能否到达。
                path = nav.getPath(PoseStamped(), goal, use_start=False)
                if path is not None and path.poses:
                    row.update(view_dist=cand["r"], view_rank=cand["rank"],
                               goal_x=cand["x"], goal_y=cand["y"], goal_yaw=cand["yaw"])
                    return goal, cand
            return None, None

        def robot_pose():
            """当前机器人位姿 (x, y, yaw)，取自 TF（--frame -> --base-frame）；没有 TF 时返回 None。"""
            try:
                t = buffer.lookup_transform(args.frame, args.base_frame, Time())
            except TransformException:
                return None
            return (t.transform.translation.x, t.transform.translation.y,
                    yaw_from_quaternion(t.transform.rotation))

        def wait_task():
            """阻塞等待当前 Nav2 任务（导航或 Spin）结束，返回 TaskResult。

            isTaskComplete() 内部会 spin 节点，所以循环期间回调照常执行。
            """
            nonlocal task_active
            while not nav.isTaskComplete():
                pass
            task_active = False
            return nav.getResult()

        def align_heading(goal_yaw):
            """近点兜底的第二步：用 Nav2 behavior_server 的 Spin 行为原地转到目标朝向。

            朝向误差不超过 --align-tolerance 时不转。返回 True 表示朝向已对准。
            Spin 自带碰撞检查，转动中会避开局部代价地图里的障碍物。
            """
            nonlocal task_active
            pose = robot_pose()
            if pose is None:
                nav.get_logger().warning("近点兜底：没有定位 TF，无法对准朝向")
                return False
            error = angle_diff(goal_yaw, pose[2])        # 正数 = 需要逆时针转
            if abs(error) <= args.align_tolerance:
                return True
            nav.get_logger().info(f"近点兜底：原地转 {math.degrees(error):.1f}° 对准设备")
            nav.feedback = None
            task_active = True
            # time_allowance 为整数秒：按 0.4 rad/s 的低速估算，再留 5 s 余量
            if not nav.spin(spin_dist=error, time_allowance=int(math.ceil(abs(error) / 0.4)) + 5):
                task_active = False
                return False
            return wait_task() == TaskResult.SUCCEEDED

        def navigate(goal, row):
            """发送导航目标并阻塞等待结果，把结果和耗时写入 row。

            row["result"] 的取值：
              reached       Nav2 正常到达
              reached_near  近点卡住，已由本脚本取消 Nav2 并用 Spin 对准朝向（照常拍照）
              timeout       超过 --timeout，已取消
              rejected / canceled / failed / unknown  Nav2 返回的其它结果
            """
            nonlocal motion_started, task_active
            started = time.monotonic()
            motion_started = True
            goal_xy = (goal.pose.position.x, goal.pose.position.y)
            goal_yaw = yaw_from_quaternion(goal.pose.orientation)
            monitor = NearGoalMonitor(goal_xy, args.near_radius, args.stuck_time)
            # Commander 的 getPath 和 goToPose 共用 result_future / feedback，先清掉上一次的状态。
            nav.feedback = None
            task_active = True
            if not nav.goToPose(goal):  # 返回 False 表示目标被 bt_navigator 拒绝
                task_active = False
                row["result"] = "rejected"
                return
            # 等待导航结束；期间检查两件事，每件只触发一次：
            #   1) 墙钟超时 -> 取消任务，结果记为 timeout；
            #   2) 近点卡住（见 NearGoalMonitor）-> 取消任务，稍后自己对准朝向。
            timed_out = near_stuck = False
            while not nav.isTaskComplete():
                now = time.monotonic()
                if timed_out or near_stuck:
                    continue                      # 已发过取消，只等任务真正结束
                if now - started > args.timeout:
                    timed_out = True
                    nav.cancelTask()
                    continue
                pose = robot_pose()
                if pose is not None and monitor.update(pose[:2], now):
                    near_stuck = True
                    nav.get_logger().warning(
                        f"已在视点 {args.near_radius} m 内停留 {args.stuck_time:.0f} s，"
                        "取消 Nav2 任务，改为原地对准朝向")
                    nav.cancelTask()
            task_active = False
            result = nav.getResult()

            if near_stuck and result != TaskResult.SUCCEEDED:
                # 近点兜底：位置已经足够近，只差朝向；转正后按到达处理
                row["result"] = "reached_near" if align_heading(goal_yaw) else "failed"
            else:
                row["result"] = "timeout" if timed_out else {
                    TaskResult.SUCCEEDED: "reached", TaskResult.CANCELED: "canceled",
                    TaskResult.FAILED: "failed",
                }.get(result, "unknown")
            row["time_s"] = round(time.monotonic() - started, 2)

        def record_position(poi, row):
            """读取到达后的实际位姿，计算与目标的位置偏差 (m) 和朝向偏差 (°)。"""
            pose = robot_pose()
            if pose is None:
                nav.get_logger().warning(f"{poi['id']}: 无最终定位 TF，未记录位置误差")
                return
            rx, ry, ryaw = pose
            row.update(robot_x=rx, robot_y=ry,
                       pos_err_m=math.hypot(rx - row["goal_x"], ry - row["goal_y"]),
                       yaw_err_deg=round(math.degrees(angle_diff(ryaw, row["goal_yaw"])), 1))

        def capture_image(index, poi, row):
            """到点后等待车身稳定，再拍一张"新"图像保存为 PNG。"""
            # 1) 静置 settle_time 秒，让车停稳、画面不再晃动
            until = time.monotonic() + args.settle_time
            while time.monotonic() < until:
                rclpy.spin_once(nav, timeout_sec=0.05)
            # 2) 只接收此刻之后采集的图像，排除 DDS 队列里还没处理的旧帧
            latest.update(img=None, after_ns=nav.get_clock().now().nanoseconds)
            until = time.monotonic() + args.image_timeout
            while latest["img"] is None and time.monotonic() < until:
                rclpy.spin_once(nav, timeout_sec=0.1)
            # 3) 保存图像
            if latest["img"] is None:
                row["image_result"] = "no_image"
                nav.get_logger().warning(
                    f"{poi['id']}: 未收到新图像，请检查 {args.image_topic} 和 cameras:=true"
                )
            else:
                safe_id = re.sub(r"[^\w.-]", "_", poi["id"])  # 去掉不适合做文件名的字符
                filename = output / f"{index:02d}_{safe_id}.png"
                try:
                    image = bridge.imgmsg_to_cv2(latest["img"], desired_encoding="bgr8")
                    if not cv2.imwrite(str(filename), image):
                        raise OSError(f"图片写入失败: {filename}")
                    row.update(image=filename.name, image_result="saved")
                except Exception as error:
                    row["image_result"] = "error"
                    nav.get_logger().error(f"{poi['id']}: {error}")
            # 4) 停止接收图像，直到下一次拍照
            latest.update(img=None, after_ns=None)

        def planned_distance(a, b):
            """a -> b 的 Nav2 规划路径长度 (m)，a/b 为 (x, y, yaw)；规划失败为 inf。"""
            if args.distance == "euclidean":
                return math.dist(a[:2], b[:2])
            return path_length(nav.getPath(make_pose(*a), make_pose(*b), use_start=True))

        def quiet(enabled):
            """getPath 每次调用都会打印日志；求距离矩阵时暂时只保留警告。"""
            try:
                from rclpy.logging import LoggingSeverity
                nav.get_logger().set_level(LoggingSeverity.WARN if enabled else LoggingSeverity.INFO)
            except (ImportError, AttributeError):
                pass

        # ------------------------------------------------------------------
        # 出发前规划：选观测点 -> 求距离矩阵 -> tour_planner 排序
        # ------------------------------------------------------------------
        goals, views, plan_rows, offline_index = {}, [], [], []
        for poi in pois:
            row = dict.fromkeys(LOG_FIELDS, "")
            row.update(id=poi["id"], type=poi["type"], result="error")
            goal, cand = choose_goal(poi, row)
            if goal is None:
                row.update(order="-", result="no_plan")
                nav.get_logger().warning(f"{poi['id']}: 所有候选视点均无路径，跳过")
                rows.append(row)
                continue
            goals[poi["id"]] = goal
            views.append((row["goal_x"], row["goal_y"], row["goal_yaw"]))
            plan_rows.append((poi, row))
            offline_index.append(cand["index"])
        start = (start_xy[0], start_xy[1], home[2])
        quiet(True)
        try:
            if plan is None:
                # 在线：起点和所有视点两两求 Nav2 路径长度
                nav.get_logger().info(
                    f"计算 {len(views) + 1} 个位置两两之间的路径长度（{args.distance}）…")
                D = tour_planner.distance_matrix([start] + views, planned_distance,
                                                 symmetric=args.distance == "nav")
            else:
                # 离线矩阵 + 在线补算"起点 -> 各视点"这一行（起点每次可能不同）
                nav.get_logger().info(f"使用离线路程矩阵，在线计算起点到 {len(views)} 个视点的路径长度…")
                n = len(views)
                D = [[0.0] * (n + 1) for _ in range(n + 1)]
                for i in range(n):
                    D[0][i + 1] = D[i + 1][0] = planned_distance(start, views[i])
                    for j in range(n):
                        D[i + 1][j + 1] = float(plan["matrix"][offline_index[i]][offline_index[j]])
        finally:
            quiet(False)
        plan = tour_planner.plan_tour(start, views, method=args.order,
                                      return_home=args.return_home, matrix=[r[:] for r in D])
        for k in plan.unreachable:
            poi, row = plan_rows[k]
            row.update(order="-", result="no_plan")
            nav.get_logger().warning(f"{poi['id']}: 从起点规划不出路径，跳过")
            rows.append(row)
        queue = list(plan.order)          # 剩余待访问点（plan_rows / D 的下标为 k / k+1）
        leg_plan = dict(zip(queue, plan.legs))
        home_plan = plan.legs[-1] if plan.return_home and plan.legs else math.nan
        nav.get_logger().info(
            f"访问顺序（{plan.method}，预计 {plan.total:.1f} m"
            + ("，含返回起点" if plan.return_home else "") + "）："
            + " → ".join(plan_rows[k][0]["id"] for k in queue)
        )
        write_log(log_path, rows)

        def replan(from_pose):
            """失败后以当前位置为起点，对剩余的点重新排序；返回起点那一段仍以出发点为终点。"""
            nonlocal queue, leg_plan, home_plan
            if len(queue) < 2:
                return
            quiet(True)
            try:
                here = [planned_distance(from_pose, views[k]) for k in queue]
            finally:
                quiet(False)
            sub = [[0.0] + here]
            for i, ki in enumerate(queue):
                # 第 0 列是"回到起点"：用到出发点的距离，而不是到当前位置的距离
                sub.append([D[ki + 1][0]] + [D[ki + 1][kj + 1] for kj in queue])
            again = tour_planner.plan_tour(from_pose, [views[k] for k in queue], method=args.order,
                                           return_home=args.return_home, matrix=sub)
            queue = [queue[k] for k in again.order]
            leg_plan = dict(zip(queue, again.legs))
            if again.return_home:
                home_plan = again.legs[-1]
            nav.get_logger().info("重新排序剩余的点：" + " → ".join(plan_rows[k][0]["id"] for k in queue))

        # ------------------------------------------------------------------
        # 主循环：导航 -> 记录位置 -> 拍照 -> 写 CSV
        # ------------------------------------------------------------------
        index = 0
        total = len(queue)
        while queue:
            k = queue.pop(0)
            poi, row = plan_rows[k]
            index += 1
            row.update(order=index, plan_m=round(leg_plan.get(k, math.nan), 2))
            rows.append(row)
            try:
                nav.get_logger().info(f"[{index}/{total}] 前往 {poi['id']}")
                navigate(goals[poi["id"]], row)
                if row["result"] != "rejected":
                    record_position(poi, row)
                if row["result"] in REACHED:
                    capture_image(index, poi, row)
                elif queue:
                    # 没到达：以当前位置为起点，对剩余的点重新排序
                    pose = robot_pose()
                    if pose is not None:
                        replan(pose)
                    else:
                        nav.get_logger().warning("无定位 TF，按原顺序继续")
            except KeyboardInterrupt:
                row["result"] = "interrupted"
                raise
            finally:
                write_log(log_path, rows)  # 无论成功失败，每个点结束都落盘

        # ------------------------------------------------------------------
        # 返回起点：所有 POI 结束后（无论成败）回到出发时的位置和朝向
        # ------------------------------------------------------------------
        reached = sum(row["result"] in REACHED for row in rows)
        if args.return_home:
            row = dict.fromkeys(LOG_FIELDS, "")
            row.update(order=index + 1, id="HOME", type="return_home", result="error",
                       goal_x=home[0], goal_y=home[1], goal_yaw=home[2],
                       plan_m=round(home_plan, 2))
            rows.append(row)
            try:
                nav.get_logger().info(
                    f"巡检结束，返回起点 ({home[0]:.2f}, {home[1]:.2f}, yaw {home[2]:.2f})"
                )
                navigate(make_pose(*home), row)
                if row["result"] != "rejected":
                    record_position({"id": "HOME"}, row)
                if row["result"] not in REACHED:
                    nav.get_logger().warning(f"未能返回起点：{row['result']}")
            except KeyboardInterrupt:
                row["result"] = "interrupted"
                raise
            finally:
                write_log(log_path, rows)
        nav.get_logger().info(
            f"完成: 到达 {reached}/{len(pois)} 个 POI"
            + (f"，返回起点 {rows[-1]['result']}" if args.return_home else "")
            + f"，记录: {log_path.resolve()}"
        )
    finally:
        # ------------------------------------------------------------------
        # 清理：取消未完成的目标 -> 发零速度停车 -> 销毁节点 -> 关闭 rclpy
        # 嵌套 try/finally 保证前一步出错时后一步仍会执行。
        # ------------------------------------------------------------------
        if nav is not None:
            try:
                try:
                    if task_active and rclpy.ok():
                        nav.cancelTask()
                finally:
                    if motion_started and rclpy.ok():
                        # Gazebo 底盘没有命令超时；取消本脚本目标后显式停车。
                        for _ in range(3):
                            stop_pub.publish(Twist())
                            rclpy.spin_once(nav, timeout_sec=0.05)
            finally:
                nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ============================================================================
# 三、命令行入口
# ============================================================================

def dry_run_plan(pois, plan, args):
    """不连 ROS，按离线规划打印首选视点和访问顺序（起点那一行用直线距离近似）。"""
    start = (0.0, 0.0) if args.start_x is None else (args.start_x, args.start_y)
    usable = [(poi, plan["candidates"][poi["id"]][0]) for poi in pois if plan["candidates"][poi["id"]]]
    skipped = [poi["id"] for poi in pois if not plan["candidates"][poi["id"]]]
    n = len(usable)
    D = [[0.0] * (n + 1) for _ in range(n + 1)]
    for i, (_, a) in enumerate(usable):
        D[0][i + 1] = D[i + 1][0] = math.dist(start, (a["x"], a["y"]))
        for j, (_, b) in enumerate(usable):
            D[i + 1][j + 1] = float(plan["matrix"][a["index"]][b["index"]])
    order = tour_planner.plan_tour(start, [(c["x"], c["y"]) for _, c in usable], method=args.order,
                                   return_home=args.return_home, matrix=D)
    print(f"离线规划: {args.plan}；排序起点: {start}；方法: {order.method}（起点一行为直线距离）")
    print("order  id       rank   view_x   view_y   yaw(rad)   r(m)  phi(°)  score   leg_m")
    for index, (k, leg) in enumerate(zip(order.order, order.legs), 1):
        poi, c = usable[k]
        print(f"{index:5d}  {poi['id']:<8} {c['rank']:4d} {c['x']:8.2f} {c['y']:8.2f} {c['yaw']:8.3f}"
              f" {c['r']:6.2f} {c['phi_deg']:7.1f} {c['score']:6.3f} {leg:7.1f}")
    if args.return_home:
        print(f"{'':5}  {'HOME':<8} {'':4} {start[0]:8.2f} {start[1]:8.2f}"
              f" {'':8} {'':6} {'':7} {'':6} {order.legs[-1]:7.1f}")
    print(f"合计 {order.total:.1f} m" + (f"；没有可用视点：{', '.join(skipped)}" if skipped else ""))

def main(argv=None):
    parser = argparse.ArgumentParser(description="Scout Mini POI 巡检 (ROS 2 Humble / Nav2)")
    # --- 输入与观测 ---
    parser.add_argument("--poi", help="包含 pois 清单的 YAML 文件；与 --plan 同时给出时只用于过期检查")
    parser.add_argument("--plan", help="plan_offline.py 生成的离线规划（候选视点 + 路程矩阵）；"
                                       "给出后不再使用 --dist / --poi-transform")
    parser.add_argument("--map", help="与 --plan 一起用：Nav2 的地图 YAML，用于检查离线规划是否过期")
    parser.add_argument("--nav2-params", help="与 --plan 一起用：Nav2 参数文件，检查车身尺寸和代价参数是否改过；"
                                              "默认自动找 scout_mini_navigation 的 config/nav2_params.yaml")
    parser.add_argument("--only", nargs="+", help="只巡检这些编号，如 PG-W01 AP-W03")
    parser.add_argument("--dist", type=float, nargs="+", default=[2.5, 3.0, 2.0],
                        help="观测距离候选 (m)，按顺序尝试；默认兼顾低位水平相机视野")
    # --- 坐标系 ---
    parser.add_argument("--frame", default="map", help="Nav2 目标坐标系，须与全局代价地图一致")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--poi-transform", type=float, nargs=3, metavar=("TX", "TY", "YAW"),
                        help="POI 源坐标转入 --frame: p_goal=R(YAW)*p_poi+[TX,TY]，YAW 为弧度")
    parser.add_argument("--start-x", type=float, help="仅影响排序；运行时默认读取当前 TF")
    parser.add_argument("--start-y", type=float, help="与 --start-x 一同提供，属于 --frame")
    # --- ROS 话题与节点 ---
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--localizer", default="auto",
                        help="auto（默认，只等定位 TF）、lidar_localization、amcl、slam_toolbox 或 none")
    parser.add_argument("--use-sim-time", action=argparse.BooleanOptionalAction, default=True,
                        help="默认使用 /clock；实车传 --no-use-sim-time")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel", help="退出时停车话题")
    # --- 时间参数（均为墙钟时间，秒）---
    parser.add_argument("--timeout", type=float, default=300.0, help="单目标导航墙钟超时 (s)")
    parser.add_argument("--startup-timeout", type=float, default=60.0, help="启动准备墙钟超时 (s)")
    parser.add_argument("--image-timeout", type=float, default=3.0, help="到点后等待新图像的时间 (s)")
    parser.add_argument("--settle-time", type=float, default=0.5, help="到点后拍照前的静置时间 (s)")
    # --- 近点兜底（见 NearGoalMonitor）---
    parser.add_argument("--near-radius", type=float, default=0.4,
                        help="离视点多近时启用近点兜底 (m)；0 关闭")
    parser.add_argument("--stuck-time", type=float, default=8.0,
                        help="在视点附近基本不动多久判定为卡住 (s)；应小于 Nav2 进度检查的 15 s")
    parser.add_argument("--align-tolerance", type=float, default=0.1,
                        help="兜底时朝向误差小于它就不再转 (rad)，0.1 rad 约 5.7°")
    # --- 输出 ---
    parser.add_argument("--out", default="patrol_out", help="输出根目录，每次任务建立独立子目录")
    parser.add_argument("--order", default="auto", choices=tour_planner.METHODS,
                        help="访问顺序：auto（默认，点少时精确最优）、exact、two_opt、greedy")
    parser.add_argument("--distance", default="nav", choices=("nav", "euclidean"),
                        help="排序用的距离：nav（默认，Nav2 规划路径长度）或 euclidean（直线）")
    parser.add_argument("--return-home", action=argparse.BooleanOptionalAction, default=True,
                        help="巡检结束后回到出发时的位姿（默认开启；--no-return-home 关闭）")
    parser.add_argument("--dry-run", action="store_true", help="只打印观测位姿，不连接 ROS")
    args = parser.parse_args(argv)

    try:
        # ---- 参数检查：尽早报错，不要等到连上 ROS 之后 ----
        positive = args.dist + [args.timeout, args.startup_timeout, args.image_timeout,
                                args.stuck_time, args.align_tolerance]
        if any(not math.isfinite(value) or value <= 0 for value in positive):
            raise ValueError("距离、各项超时、--stuck-time 和 --align-tolerance 必须为有限正数")
        if any(not math.isfinite(value) or value < 0 for value in (args.settle_time, args.near_radius)):
            raise ValueError("--settle-time 和 --near-radius 必须为有限非负数")
        if (args.start_x is None) != (args.start_y is None):
            raise ValueError("--start-x 和 --start-y 必须一同提供")
        coordinates = ([] if args.start_x is None else [args.start_x, args.start_y])
        if not all(math.isfinite(value) for value in coordinates + (args.poi_transform or [])):
            raise ValueError("起点及坐标转换参数必须为有限数")

        if args.plan:
            pois, plan = load_plan(args)
            source_frame = args.frame
        elif args.poi:
            pois, source_frame = prepare_pois(args)
            plan = None
        else:
            raise ValueError("需要 --poi（在线生成视点）或 --plan（使用离线规划）")

        # ---- dry-run + 离线规划：打印每台设备的首选视点和按离线矩阵求出的顺序 ----
        if args.dry_run and plan is not None:
            dry_run_plan(pois, plan, args)
            return 0

        # ---- dry-run：只打印排序后的观测位姿，用来检查配置和坐标转换 ----
        if args.dry_run:
            start = (0.0, 0.0) if args.start_x is None else (args.start_x, args.start_y)
            ordered, plan = order_pois(pois, start, args.dist[0], args.order, args.return_home)
            print(f"坐标系: {source_frame}；观测距离: {args.dist[0]} m；排序起点: {start}；"
                  f"方法: {plan.method}（直线距离；离线按地图路径长度见 tour_planner.py --map）")
            print("order  id       type              view_x   view_y   yaw(rad)   leg_m")
            for index, (poi, leg) in enumerate(zip(ordered, plan.legs), 1):
                x, y, yaw = view_pose(poi, args.dist[0])
                print(f"{index:5d}  {poi['id']:<8} {poi['type']:<17} {x:7.2f} {y:7.2f} {yaw:8.3f} {leg:7.1f}")
            if args.return_home:
                print(f"{'':5}  {'HOME':<8} {'return_home':<17} {start[0]:7.2f} {start[1]:7.2f}"
                      f" {'':8} {plan.legs[-1]:7.1f}")
            print(f"合计 {plan.total:.1f} m")
            return 0

        run(args, pois, plan)
    except (OSError, ValueError, KeyError, RuntimeError, yaml.YAMLError) as error:
        print(f"巡检错误: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("巡检已中断，已保存当前记录并取消导航", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
