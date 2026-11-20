#!/usr/bin/env python3
"""巡检路径点规划器：决定按什么顺序访问一组路径点（观测点）。

与 ROS 无关的纯 Python 模块，poi_patrol.py 调用它；也可单独运行做离线分析。

核心接口
--------
    plan = plan_tour(start, waypoints, distance, method="auto", return_home=True)
    plan.order        访问顺序（waypoints 的下标）
    plan.legs         每段长度（m），含最后返回起点的一段
    plan.total        总长度（m）
    plan.unreachable  从起点无法到达的路径点下标（不参与排序）

``distance(a, b)`` 由调用方提供，返回 a 到 b 的路程（m），不可达返回 ``math.inf``：

- poi_patrol.py 正式巡检时：Nav2 规划器给出的实际路径长度；
- 离线分析：``GridMap(地图 yaml).distances(...)``，在二维栅格上求最短路径；
- 预览或测试：``euclidean``，直线距离。

求解方法
--------
- ``greedy``：贪心最近邻，每次去最近的下一个点；
- ``two_opt``：贪心结果再用 2-opt（翻转一段子路径）反复改进；
- ``exact``：Held-Karp 动态规划，精确最优，复杂度 O(n² · 2ⁿ)，适合约 13 个点以内；
- ``auto``（默认）：点数不超过 ``exact_limit`` 用 exact，否则用 two_opt。

回到起点的路线正反两个方向等长，exact / two_opt 统一取"先去离起点更近的一端"
（见 ``orient_tour``），保证同一组点每次得到同一个顺序。

命令行（离线分析，详见 --help）::

    python3 tour_planner.py --poi poi.yaml --map maps/warehouse.yaml --start 0 0 --method all
"""
import argparse
from dataclasses import dataclass, field
import heapq
import itertools
import math
from pathlib import Path
import sys

METHODS = ("auto", "exact", "two_opt", "greedy")
DEFAULT_EXACT_LIMIT = 13


@dataclass
class TourPlan:
    order: list                  # 访问顺序：waypoints 的下标
    legs: list                   # 每段长度 (m)：起点 → 第 1 个 … → 最后一个（→ 起点）
    total: float                 # 总长度 (m)
    method: str                  # 实际使用的求解方法
    return_home: bool
    unreachable: list = field(default_factory=list)


# ============================================================================
# 距离
# ============================================================================

def euclidean(a, b):
    """二维直线距离；a、b 为 (x, y) 或更长的序列，只取前两个分量。"""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def distance_matrix(points, distance, symmetric=False):
    """points[0] 为起点。返回 (n+1) x (n+1) 矩阵 D[i][j] = distance(points[i], points[j])。

    symmetric=True 时只算一半（适合规划器给出的路径长度，节省调用次数）。
    """
    n = len(points)
    D = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j or (symmetric and j < i):
                continue
            D[i][j] = float(distance(points[i], points[j]))
            if symmetric:
                D[j][i] = D[i][j]
    return D


# ============================================================================
# 求解
# ============================================================================

def tour_legs(order, D, return_home):
    """order 为矩阵下标（1..n），返回各段长度（含可选的返回段）。"""
    seq = [0] + list(order) + ([0] if return_home else [])
    return [D[a][b] for a, b in zip(seq, seq[1:])]


def tour_length(order, D, return_home):
    return sum(tour_legs(order, D, return_home))


def solve_greedy(D, nodes, return_home=True):
    """从起点 0 出发，每次走到最近的未访问节点。"""
    left, order, cur = list(nodes), [], 0
    while left:
        nxt = min(left, key=lambda k: D[cur][k])
        left.remove(nxt)
        order.append(nxt)
        cur = nxt
    return order


def solve_two_opt(D, nodes, return_home=True, start_order=None):
    """2-opt 局部改进：反复翻转一段子路径，直到总长度不再下降。距离可以不对称。"""
    order = list(start_order) if start_order is not None else solve_greedy(D, nodes, return_home)
    best = tour_length(order, D, return_home)
    improved = True
    while improved:
        improved = False
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                length = tour_length(cand, D, return_home)
                if length < best - 1e-9:
                    order, best, improved = cand, length, True
    return order


def solve_exact(D, nodes, return_home=True):
    """Held-Karp 动态规划：起点 0 出发、访问 nodes 各一次（return_home 时回到 0）的最短路线。"""
    nodes = list(nodes)
    m = len(nodes)
    if m == 0:
        return []
    bit = {k: 1 << i for i, k in enumerate(nodes)}
    # best[(mask, k)] = (到达 k 且已访问 mask 的最短长度, 上一个节点)
    best = {(bit[k], k): (D[0][k], 0) for k in nodes}
    for size in range(2, m + 1):
        for subset in itertools.combinations(nodes, size):
            mask = sum(bit[k] for k in subset)
            for k in subset:
                prev_mask = mask & ~bit[k]
                best[(mask, k)] = min(
                    ((best[(prev_mask, j)][0] + D[j][k], j) for j in subset if j != k),
                    key=lambda t: t[0],
                )
    full = (1 << m) - 1
    _, last = min(((best[(full, k)][0] + (D[k][0] if return_home else 0.0), k) for k in nodes),
                  key=lambda t: t[0])
    order, mask, k = [], full, last
    while k:
        order.append(k)
        prev = best[(mask, k)][1]
        mask &= ~bit[k]
        k = prev
    return order[::-1]


def orient_tour(order, D, return_home=True):
    """固定闭合路线的走向，使同一组点每次得到同一个顺序。

    回到起点的路线是一个环：正着走和反着走总长相同（距离对称时），求解器选哪个方向
    取决于浮点误差和点的输入顺序。这里统一规则：
      1. 反向明显更短（距离不对称时可能出现）就用反向；
      2. 两个方向等长时，先去离起点更近的一端；
      3. 仍然相同则先去下标较小的点。
    开放路线（不回起点）的方向由起点决定，原样返回。
    """
    if not return_home or len(order) < 2:
        return list(order)
    fwd, rev = list(order), list(order)[::-1]
    f_len, r_len = tour_length(fwd, D, True), tour_length(rev, D, True)
    tol = 1e-6 * max(1.0, abs(f_len))
    if r_len < f_len - tol:
        return rev
    if r_len > f_len + tol:
        return fwd
    f_first, r_first = D[0][fwd[0]], D[0][rev[0]]
    if abs(f_first - r_first) > tol:
        return fwd if f_first < r_first else rev
    return fwd if fwd[0] < rev[0] else rev


def plan_tour(start, waypoints, distance=euclidean, method="auto", return_home=True,
              exact_limit=DEFAULT_EXACT_LIMIT, symmetric=False, matrix=None):
    """规划访问顺序。

    start      起点 (x, y, ...)；waypoints 路径点列表；distance(a, b) -> 路程 (m)。
    matrix     可直接给出 (n+1) x (n+1) 距离矩阵（下标 0 为起点），此时不再调用 distance。
    """
    if method not in METHODS:
        raise ValueError(f"method 必须是 {', '.join(METHODS)} 之一，收到 {method!r}")
    D = matrix if matrix is not None else distance_matrix([start] + list(waypoints), distance, symmetric)
    nodes = [k for k in range(1, len(D)) if math.isfinite(D[0][k])]
    unreachable = [k - 1 for k in range(1, len(D)) if not math.isfinite(D[0][k])]
    # 两个可达点之间偶尔规划失败（例如规划超时）：按"经由起点"估计，避免整个求解失效
    for i in [0] + nodes:
        for j in nodes:
            if i != j and not math.isfinite(D[i][j]):
                D[i][j] = D[i][0] + D[0][j] if math.isfinite(D[i][0]) else 1e9
    used = method
    if method == "auto":
        used = "exact" if len(nodes) <= exact_limit else "two_opt"
    solver = {"exact": solve_exact, "two_opt": solve_two_opt, "greedy": solve_greedy}[used]
    order = solver(D, nodes, return_home)
    if used != "greedy":          # greedy 是"每步去最近点"的基线，保持其原意不翻转
        order = orient_tour(order, D, return_home)
    legs = tour_legs(order, D, return_home)
    return TourPlan([k - 1 for k in order], legs, sum(legs), used, return_home, unreachable)


# ============================================================================
# 离线距离：在 ROS 二维栅格地图上求最短路径
# ============================================================================

def read_pgm(path):
    """读取 P5（二进制）或 P2（文本）PGM，返回 (宽, 高, 按行展开的像素列表)。"""
    data = Path(path).read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b"P5":
        pixels = list(data[pos + 1:pos + 1 + width * height])
    elif magic == b"P2":
        pixels = [int(v) for v in data[pos:].split()][:width * height]
    else:
        raise ValueError(f"{path}: 只支持 P5/P2 PGM")
    if maxval != 255:
        pixels = [round(v * 255 / maxval) for v in pixels]
    return width, height, pixels


SQRT2 = math.sqrt(2.0)
# 8 邻域：(行偏移, 列偏移, 步长/格)
STEPS8 = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


class GridMap:
    """ROS map_server 格式的二维栅格（yaml + pgm），用于离线估计路径长度。

    占据格按 robot_radius 膨胀后视为不可通行；未知格可通行（与 Nav2 的 allow_unknown 一致）。
    step 为搜索时的降采样倍数（2 → 0.05 m 地图按 0.1 m 搜索），用于提速。

    costmap 为 None 时只求几何最短路（每步代价 = 步长）。
    给出 costmap（字典，键见 _build_cost）时仿照 Nav2：膨胀层按离障碍物的距离给每格一个代价，
    Smac Planner 2D 每一步的代价 = 步长 × (1 + cost_travel_multiplier × 代价/252)，
    于是路径会像 Nav2 一样沿通道中线走；返回的仍是这条路径的几何长度。
    """

    def __init__(self, yaml_path, robot_radius=0.45, step=2, costmap=None):
        import yaml
        meta = yaml.safe_load(Path(yaml_path).read_text())
        width, height, pixels = read_pgm(Path(yaml_path).parent / meta["image"])
        self.res = float(meta["resolution"]) * step
        self.origin = (float(meta["origin"][0]), float(meta["origin"][1]))
        occ_thresh = float(meta.get("occupied_thresh", 0.65))
        negate = int(meta.get("negate", 0))
        occupied = set()
        for idx, v in enumerate(pixels):
            p = v / 255.0 if negate else (255 - v) / 255.0
            if p > occ_thresh:
                row, col = divmod(idx, width)
                occupied.add(((height - 1 - row) // step, col // step))   # 行号从下往上
        self.w, self.h = (width + step - 1) // step, (height + step - 1) // step
        r = int(math.ceil(robot_radius / self.res))
        disk = [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1) if dy * dy + dx * dx <= r * r]
        self.blocked = set()
        for (cy, cx) in occupied:
            for dy, dx in disk:
                self.blocked.add((cy + dy, cx + dx))
        self.mult = {}          # 每格的代价倍率；不在字典里的格子为 1.0（离障碍物足够远）
        if costmap:
            self._build_cost(occupied, **costmap)

    def _build_cost(self, occupied, inflation_radius=1.0, cost_scaling_factor=2.0,
                    travel_multiplier=2.0, inscribed_radius=0.31):
        """按 Nav2 膨胀层公式给障碍物附近的格子算代价倍率。

        Nav2 膨胀层：离最近障碍物 d 米处的代价
            d <= inscribed_radius                      : 253（车身必然碰撞）
            inscribed_radius < d <= inflation_radius   : 252 × exp(-cost_scaling_factor × (d - inscribed_radius))
            d > inflation_radius                       : 0
        Smac Planner 2D（Node2D::getTraversalCost）：一步的代价 = 步长 × (1 + travel_multiplier × 代价/252)。
        默认值与 nav2_params.yaml 的全局代价地图和 Smac 2D 一致。
        """
        # 多源 Dijkstra：每格到最近占据格的距离（格），只算到 inflation_radius 以内
        limit = inflation_radius / self.res
        dist = {c: 0.0 for c in occupied}
        pq = [(0.0, c) for c in occupied]
        heapq.heapify(pq)
        while pq:
            d, (y, x) = heapq.heappop(pq)
            if d > dist[(y, x)]:
                continue
            for dy, dx, c in STEPS8:
                v = (y + dy, x + dx)
                nd = d + c
                if nd <= limit and nd < dist.get(v, math.inf):
                    dist[v] = nd
                    heapq.heappush(pq, (nd, v))
        for cell, d in dist.items():
            if cell in self.blocked:
                continue
            meters = d * self.res
            cost = 252.0 if meters <= inscribed_radius else \
                252.0 * math.exp(-cost_scaling_factor * (meters - inscribed_radius))
            self.mult[cell] = 1.0 + travel_multiplier * cost / 252.0

    def cell(self, x, y):
        return (int((y - self.origin[1]) / self.res), int((x - self.origin[0]) / self.res))

    def free_cell_near(self, x, y, search=1.0):
        cy, cx = self.cell(x, y)
        r = int(search / self.res)
        best = None
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                c = (cy + dy, cx + dx)
                if 0 <= c[0] < self.h and 0 <= c[1] < self.w and c not in self.blocked:
                    d = dy * dy + dx * dx
                    if best is None or d < best[0]:
                        best = (d, c)
        return None if best is None else best[1]

    def _dijkstra(self, src):
        """几何最短路（不计代价），一对全部；用于可达性判断。返回 {格子: 距离（格）}。"""
        dist = {src: 0.0}
        pq = [(0.0, src)]
        while pq:
            d, (y, x) = heapq.heappop(pq)
            if d > dist[(y, x)]:
                continue
            for dy, dx, c in STEPS8:
                v = (y + dy, x + dx)
                if 0 <= v[0] < self.h and 0 <= v[1] < self.w and v not in self.blocked:
                    nd = d + c
                    if nd < dist.get(v, math.inf):
                        dist[v] = nd
                        heapq.heappush(pq, (nd, v))
        return dist

    def search(self, src, goal=None):
        """代价感知的 8 邻域最短路搜索，最小化 Σ 步长 × 代价倍率，返回路径的几何长度（格）。

        goal 给出时为 A*：启发函数取八方向距离 (octile)，即无障碍、代价全为 1 时的最短步长和；
        代价倍率 ≥ 1，所以它不会高估，A* 求得的仍是最优路径。到不了返回 inf。
        goal 为 None 时为 Dijkstra（一次求出到所有格子的结果），返回 {格子: 几何长度}。
        两者在同一代价模型下给出相同的最优路径，A* 适合单对点，Dijkstra 适合一对全部。
        """
        def heuristic(c):
            if goal is None:
                return 0.0
            dy, dx = abs(c[0] - goal[0]), abs(c[1] - goal[1])
            return dx + dy + (SQRT2 - 2.0) * min(dx, dy)

        cost = {src: 0.0}       # 累计代价 g
        length = {src: 0.0}     # 沿当前最优路径的几何长度
        pq = [(heuristic(src), src)]
        while pq:
            f, u = heapq.heappop(pq)
            if u == goal:
                return length[u]
            gu = cost[u]
            if f - heuristic(u) > gu + 1e-9:
                continue            # 过期的队列项
            for dy, dx, step in STEPS8:
                v = (u[0] + dy, u[1] + dx)
                if 0 <= v[0] < self.h and 0 <= v[1] < self.w and v not in self.blocked:
                    ng = gu + step * self.mult.get(v, 1.0)
                    if ng < cost.get(v, math.inf):
                        cost[v], length[v] = ng, length[u] + step
                        heapq.heappush(pq, (ng + heuristic(v), v))
        return length if goal is None else math.inf

    def distances(self, points, method="dijkstra", progress=None):
        """points 两两之间的路程矩阵 (m)。

        method="astar"：每对点做一次 A*（距离对称，只算一半）；
        method="dijkstra"：每个点做一次一对全部的搜索，点多时更快，结果相同。
        progress(i, n) 用于打印进度，可省略。
        """
        cells = [self.free_cell_near(p[0], p[1]) for p in points]
        n = len(points)
        D = [[0.0] * n for _ in range(n)]
        if method == "astar":
            pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]
            for k, (i, j) in enumerate(pairs, 1):
                if cells[i] is None or cells[j] is None:
                    d = math.inf
                else:
                    d = self.search(cells[i], cells[j]) * self.res
                D[i][j] = D[j][i] = d
                if progress:
                    progress(k, len(pairs))
            return D
        for i, ci in enumerate(cells):
            lengths = self.search(ci) if ci is not None else {}
            for j, cj in enumerate(cells):
                if i != j:
                    D[i][j] = lengths.get(cj, math.inf) * self.res if cj is not None else math.inf
            if progress:
                progress(i + 1, n)
        return D


# ============================================================================
# 命令行：离线比较不同方法
# ============================================================================

def main(argv=None):
    parser = argparse.ArgumentParser(description="巡检路径点规划器（离线分析）")
    parser.add_argument("--poi", required=True, help="POI 配置文件（与 poi_patrol.py 相同）")
    parser.add_argument("--poi-transform", type=float, nargs=3, metavar=("TX", "TY", "YAW"))
    parser.add_argument("--frame", default="map")
    parser.add_argument("--start", type=float, nargs=2, default=[0.0, 0.0], metavar=("X", "Y"),
                        help="起点（与 POI 同一坐标系）")
    parser.add_argument("--dist", type=float, default=2.5, help="观测距离 (m)")
    parser.add_argument("--map", help="二维栅格地图 yaml；不给则用直线距离")
    parser.add_argument("--robot-radius", type=float, default=0.45)
    parser.add_argument("--method", default="all", choices=METHODS + ("all",))
    parser.add_argument("--no-return-home", dest="return_home", action="store_false")
    args = parser.parse_args(argv)

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import poi_patrol

    pois, frame = poi_patrol.prepare_pois(argparse.Namespace(
        poi=args.poi, only=None, frame=args.frame, dry_run=True, poi_transform=args.poi_transform))
    start = tuple(args.start)
    views = [poi_patrol.view_pose(p, args.dist)[:2] for p in pois]
    if args.map:
        print(f"在 {args.map} 上计算最短路径（机器人半径 {args.robot_radius} m）…", file=sys.stderr)
        D = GridMap(args.map, args.robot_radius).distances([start] + views)
        kind = "地图路径长度"
    else:
        D = distance_matrix([start] + views, euclidean)
        kind = "直线距离"
    print(f"坐标系 {frame}；起点 {start}；{len(pois)} 个点；距离 = {kind}；"
          f"{'回到起点' if args.return_home else '不回起点'}")
    for method in (METHODS[1:] if args.method == "all" else (args.method,)):
        plan = plan_tour(start, views, method=method, return_home=args.return_home,
                         matrix=[row[:] for row in D])
        names = " → ".join(pois[k]["id"] for k in plan.order) + (" → 起点" if plan.return_home else "")
        print(f"{plan.method:8} {plan.total:8.1f} m   {names}")
        if plan.unreachable:
            print(f"{'':8} 不可达：{', '.join(pois[k]['id'] for k in plan.unreachable)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
