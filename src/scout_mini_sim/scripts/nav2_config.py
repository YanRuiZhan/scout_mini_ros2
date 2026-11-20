#!/usr/bin/env python3
"""读取离线规划需要与 Nav2 保持一致的参数（只依赖 PyYAML）。

plan_offline.py 用它补全车身尺寸和代价参数；poi_patrol.py --plan 用它检查：
生成离线规划之后，nav2_params.yaml 里这些参数有没有改过（改过就要重新规划）。
只比较这几个值，不比较整个文件，所以调控制器速度、到点容差等无关参数不会让离线规划失效。
"""
import math
from pathlib import Path

import yaml

# 找不到 nav2_params.yaml 时才用这组内置值（与仓库当前配置相同）
NAV2_FALLBACK = {"robot_radius": 0.44, "inscribed_radius": 0.31, "inflation_radius": 1.0,
                 "cost_scaling": 2.0, "travel_multiplier": 2.0}


def default_nav2_params():
    """默认的 nav2_params.yaml：先找已安装的 scout_mini_navigation 包，再找源码目录；都没有返回 None。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("scout_mini_navigation")) / "config" / "nav2_params.yaml"
        if path.is_file():
            return path
    except Exception:       # 没有 source ROS 环境、或包未安装
        pass
    # 源码目录：本脚本在 src/scout_mini_sim/scripts/，配置在 src/scout_mini_navigation/config/
    path = Path(__file__).resolve().parents[2] / "scout_mini_navigation" / "config" / "nav2_params.yaml"
    return path if path.is_file() else None


def footprint_radii(points, padding=0.0):
    """车身多边形 -> (内切半径, 外接半径)，都加上 footprint_padding。

    内切半径 = 车体中心到各条边的最短距离（Nav2 以此判断"必然碰撞"）；
    外接半径 = 中心到最远顶点的距离（车身原地转一圈扫过的范围）。
    """
    def to_edge(a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, -(ax * dx + ay * dy) / (dx * dx + dy * dy)))
        return math.hypot(ax + t * dx, ay + t * dy)
    edges = zip(points, points[1:] + points[:1])
    inscribed = min(to_edge(a, b) for a, b in edges)
    circumscribed = max(math.hypot(x, y) for x, y in points)
    return inscribed + padding, circumscribed + padding


def read_nav2_params(path):
    """从 nav2_params.yaml 读出离线路程计算要与 Nav2 一致的参数。

    global_costmap   footprint + footprint_padding  -> 内切 / 外接半径
                     （没有 footprint 时用 robot_radius，内切 = 外接）
                     膨胀层（plugins 里类型为 InflationLayer 的那个）-> inflation_radius、cost_scaling_factor
    planner_server   第一个规划器插件的 cost_travel_multiplier（Smac 2D）
    缺的项用 NAV2_FALLBACK 补上。
    """
    data = yaml.safe_load(Path(path).read_text())
    gc = data["global_costmap"]["global_costmap"]["ros__parameters"]
    out = dict(NAV2_FALLBACK)
    footprint = gc.get("footprint")
    if footprint and footprint != "[]":
        points = yaml.safe_load(footprint) if isinstance(footprint, str) else footprint
        out["inscribed_radius"], out["robot_radius"] = footprint_radii(
            [tuple(map(float, xy)) for xy in points], float(gc.get("footprint_padding", 0.01)))
    elif "robot_radius" in gc:
        out["inscribed_radius"] = out["robot_radius"] = float(gc["robot_radius"])
    for name in gc.get("plugins", []):
        layer = gc.get(name)
        if isinstance(layer, dict) and str(layer.get("plugin", "")).endswith("InflationLayer"):
            out["inflation_radius"] = float(layer.get("inflation_radius", out["inflation_radius"]))
            out["cost_scaling"] = float(layer.get("cost_scaling_factor", out["cost_scaling"]))
    planner = data.get("planner_server", {}).get("ros__parameters", {})
    plugin = (planner.get("planner_plugins") or ["GridBased"])[0]
    out["travel_multiplier"] = float(planner.get(plugin, {}).get("cost_travel_multiplier",
                                                                  out["travel_multiplier"]))
    return out


def compare_nav2_values(planned, current, tol=1e-6):
    """比较生成规划时记录的值与当前文件里的值，返回有变化的项 [(名称, 旧值, 新值), ...]。"""
    return [(key, planned[key], current.get(key)) for key in sorted(planned)
            if current.get(key) is None or abs(float(planned[key]) - float(current[key])) > tol]


# 输出给人看的参数名
NAV2_LABELS = {
    "inscribed_radius": "车身内切半径", "robot_radius": "车身外接半径",
    "inflation_radius": "膨胀半径", "cost_scaling": "cost_scaling_factor",
    "travel_multiplier": "cost_travel_multiplier",
}
