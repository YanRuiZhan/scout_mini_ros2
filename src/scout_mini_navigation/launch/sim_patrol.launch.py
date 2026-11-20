"""一键启动：Gazebo 仿真 + Nav2（含三维定位）+ POI 巡检脚本。

在仓库根目录运行（相对路径都相对于当前目录）::

    # 在线生成视点（与单独运行 poi_patrol.py --poi 相同）
    ros2 launch scout_mini_navigation sim_patrol.launch.py \\
        map:=maps/warehouse.yaml poi:=maps/warehouse_poi.yaml

    # 使用 plan_offline.py 生成的离线规划
    ros2 launch scout_mini_navigation sim_patrol.launch.py \\
        map:=maps/warehouse.yaml plan:=maps/patrol_plan.yaml

    # 只起仿真和 Nav2，不跑巡检
    ros2 launch scout_mini_navigation sim_patrol.launch.py map:=maps/warehouse.yaml patrol:=false

启动顺序：仿真 → 等 nav_delay 秒 → Nav2 → 再等 patrol_delay 秒 → 巡检脚本。
巡检脚本自己还会等待 Nav2 各节点激活、定位 TF 就绪（最多 startup_timeout 秒），
所以两个延时只是错开启动时的 CPU 高峰，不需要很精确；电脑慢就调大。
巡检结束后仿真和 Nav2 默认继续运行，方便查看结果；shutdown_on_exit:=true 时随巡检一起退出。
需要独显渲染时整条命令前加 prime-run，子进程会继承。
"""

from pathlib import Path
import shlex
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
    Shutdown, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# ============================================================================
# 纯 Python 部分（test/test_sim_patrol_launch.py 有单元测试）
# ============================================================================

def existing_path(text, what):
    """把路径转成绝对路径；文件不存在时立刻报错，而不是等到巡检开始才发现。"""
    path = Path(text).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f"{what} 不存在: {path}（相对路径相对于运行 ros2 launch 时的当前目录）")
    return str(path)


def sim_arguments(gui, world, default_world):
    """传给 sim.launch.py 的参数。world 必须总是显式给出（绝对路径）。

    IncludeLaunchDescription 会把本文件的 launch 参数继承给被包含的文件：本文件的 world 默认为空，
    若不显式传，sim.launch.py 会继承到空字符串，Gazebo 以 `ign gazebo ""` 启动，
    停在"快速开始"界面——既没有仓库场景，也生成不了小车。
    """
    return {"gui": gui, "rviz": "false", "cameras": "true",
            "world": existing_path(world, "world") if world else str(default_world)}


def patrol_command(script, map_file, poi="", plan="", startup_timeout=180.0, extra=""):
    """组装巡检脚本的命令行。

    plan 非空：使用离线规划，并传 --map 让脚本检查规划是否过期（地图改过会拒绝运行）；
    否则：用 --poi 在线生成视点。extra 是额外参数原样附加，如 "--only PG-W01 --order greedy"。
    """
    if not plan and not poi:
        raise RuntimeError("需要 poi:=<POI 文件> 或 plan:=<离线规划文件>")
    cmd = [sys.executable, str(script)]
    if plan:
        cmd += ["--plan", plan, "--map", map_file]
    else:
        cmd += ["--poi", poi]
    cmd += ["--startup-timeout", f"{float(startup_timeout):g}"]
    return cmd + shlex.split(extra)


# ============================================================================
# 启动
# ============================================================================

def launch_all(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    sim_share = Path(get_package_share_directory("scout_mini_sim"))
    nav_share = Path(get_package_share_directory("scout_mini_navigation"))
    map_file = existing_path(arg("map"), "地图 map")

    # ---- 1. 仿真：Gazebo + 机器人 + 桥接 + EKF（RViz 用 Nav2 的那个）----
    sim_args = sim_arguments(arg("gui"), arg("world"), sim_share / "worlds" / "warehouse.sdf")
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / "launch" / "sim.launch.py")),
        launch_arguments=sim_args.items(),
    )]

    # ---- 2. Nav2 + 定位：等仿真先起来，避免 TF / 时钟还没就绪 ----
    nav_args = {"map": map_file, "localizer": arg("localizer"), "initial_pose": arg("initial_pose"),
                "rviz": arg("rviz")}
    if arg("pcd_map"):
        nav_args["pcd_map"] = existing_path(arg("pcd_map"), "PCD 地图 pcd_map")
    actions.append(TimerAction(period=float(arg("nav_delay")), actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav_share / "launch" / "navigation.launch.py")),
        launch_arguments=nav_args.items(),
    )]))

    # ---- 3. 巡检脚本 ----
    if arg("patrol").lower() == "true":
        plan = existing_path(arg("plan"), "离线规划 plan") if arg("plan") else ""
        poi = existing_path(arg("poi"), "POI 文件 poi") if not plan else ""
        script = Path(get_package_prefix("scout_mini_sim")) / "lib" / "scout_mini_sim" / "poi_patrol.py"
        cmd = patrol_command(script, map_file, poi=poi, plan=plan,
                             startup_timeout=float(arg("startup_timeout")), extra=arg("patrol_args"))
        on_exit = ([Shutdown(reason="巡检结束")]
                   if arg("shutdown_on_exit").lower() == "true" else None)
        actions.append(TimerAction(
            period=float(arg("nav_delay")) + float(arg("patrol_delay")),
            actions=[ExecuteProcess(cmd=cmd, name="poi_patrol", output="screen", on_exit=on_exit)],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        # --- 地图与任务 ---
        DeclareLaunchArgument("map", default_value="maps/warehouse.yaml",
                              description="二维栅格地图 YAML；同名 .pcd 用于三维定位"),
        DeclareLaunchArgument("poi", default_value="maps/warehouse_poi.yaml",
                              description="POI 文件（map 坐标系）；给了 plan 时不使用"),
        DeclareLaunchArgument("plan", default_value="",
                              description="plan_offline.py 生成的离线规划；为空则在线生成视点"),
        DeclareLaunchArgument("patrol_args", default_value="",
                              description="附加给 poi_patrol.py 的参数，如 '--only PG-W01 --order greedy'"),
        # --- 定位与界面 ---
        DeclareLaunchArgument("localizer", default_value="ndt", choices=["ndt", "amcl"]),
        DeclareLaunchArgument("pcd_map", default_value="", description="默认：map 同名 .pcd"),
        DeclareLaunchArgument("initial_pose", default_value="0 0 0",
                              description="初始位姿 'x y yaw'（map 坐标）；默认出生点即地图原点"),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"],
                              description="Gazebo 图形界面；false 更省资源"),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("world", default_value="", description="默认：scout_mini_sim 的 warehouse.sdf"),
        # --- 启动节奏 ---
        DeclareLaunchArgument("patrol", default_value="true", choices=["true", "false"],
                              description="false：只起仿真和 Nav2"),
        DeclareLaunchArgument("nav_delay", default_value="8.0", description="仿真启动后多久启动 Nav2 (s)"),
        DeclareLaunchArgument("patrol_delay", default_value="5.0", description="Nav2 启动后多久启动巡检 (s)"),
        DeclareLaunchArgument("startup_timeout", default_value="180",
                              description="巡检脚本等待 Nav2 和定位就绪的最长时间 (s)"),
        DeclareLaunchArgument("shutdown_on_exit", default_value="false", choices=["true", "false"],
                              description="巡检结束后关闭仿真和 Nav2"),
        OpaqueFunction(function=launch_all),
    ])
