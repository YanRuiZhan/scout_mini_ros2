"""Nav2 on top of an already running Scout Mini simulation.

initial_pose:="x y yaw" (default "0 0 0") makes AMCL start at that pose in the
map frame, so no 2D Pose Estimate click is needed. A FAST-LIO map built from the
default spawn has its origin at the spawn point, so "0 0 0" matches it.
Pass initial_pose:=none to set the pose by hand in RViz instead.
"""

import math
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def parse_initial_pose(text):
    """"x y yaw" -> (x, y, yaw); "none" -> None (set the pose in RViz)."""
    text = text.strip()
    if text.lower() in ("", "none", "false"):
        return None
    try:
        x, y, yaw = (float(v) for v in text.replace(",", " ").split())
    except ValueError:
        raise RuntimeError(
            f"initial_pose must be 'x y yaw' (metres, radians) or 'none', got {text!r}"
        ) from None
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise RuntimeError(f"initial_pose values must be finite, got {text!r}")
    return x, y, yaw


def with_initial_pose(params_file, pose):
    """Copy the Nav2 params with AMCL's initial pose filled in; return the new path."""
    with open(params_file, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    amcl = params.setdefault("amcl", {}).setdefault("ros__parameters", {})
    amcl["set_initial_pose"] = pose is not None
    if pose is not None:
        x, y, yaw = pose
        amcl["initial_pose"] = {"x": x, "y": y, "z": 0.0, "yaw": yaw}
    out = tempfile.NamedTemporaryFile(
        "w", prefix="scout_nav2_", suffix=".yaml", delete=False, encoding="utf-8"
    )
    with out:
        yaml.safe_dump(params, out, allow_unicode=True, sort_keys=False)
    return out.name


def launch_navigation(context):
    bringup = Path(get_package_share_directory("nav2_bringup"))
    slam = LaunchConfiguration("slam").perform(context).lower() == "true"
    map_file = LaunchConfiguration("map").perform(context)
    if map_file:
        map_file = str(Path(map_file).expanduser().resolve())
    if not slam and (not map_file or not Path(map_file).is_file()):
        raise RuntimeError(
            f"Map file not found: {map_file!r}. "
            "Localization requires map:=/absolute/path/map.yaml. "
            "Use slam:=true to create a new 2D map. PCD/PLY files are not occupancy maps."
        )
    pose = None if slam else parse_initial_pose(
        LaunchConfiguration("initial_pose").perform(context)
    )
    params_file = with_initial_pose(LaunchConfiguration("params_file").perform(context), pose)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch" / "bringup_launch.py")),
            launch_arguments={
                "slam": "True" if slam else "False",
                "map": map_file,
                "use_sim_time": "true",
                "params_file": params_file,
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(bringup / "rviz" / "nav2_default_view.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("scout_mini_navigation"))
    return LaunchDescription([
        DeclareLaunchArgument("slam", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("map", default_value="", description="Absolute path to a 2D map YAML"),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("params_file", default_value=str(share / "config" / "nav2_params.yaml")),
        DeclareLaunchArgument(
            "initial_pose", default_value="0 0 0",
            description="AMCL start pose 'x y yaw' in the map frame (m, rad); "
                        "'none' to set it with 2D Pose Estimate. Ignored with slam:=true",
        ),
        OpaqueFunction(function=launch_navigation),
    ])
