"""Nav2 on top of an already running Scout Mini simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch" / "bringup_launch.py")),
            launch_arguments={
                "slam": "True" if slam else "False",
                "map": map_file,
                "use_sim_time": "true",
                "params_file": LaunchConfiguration("params_file"),
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
        OpaqueFunction(function=launch_navigation),
    ])
