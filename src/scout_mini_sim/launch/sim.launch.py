"""Launch Scout Mini in Gazebo Fortress without the real CAN hardware."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_sim(context):
    share = Path(get_package_share_directory("scout_mini_sim"))
    cameras = LaunchConfiguration("cameras").perform(context).lower() == "true"
    robot_description = xacro.process_file(
        str(share / "urdf" / "scout_mini_rdk.urdf.xacro"),
        mappings={"enable_cameras": "true" if cameras else "false"},
    ).toxml()
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    world = LaunchConfiguration("world").perform(context)
    world_models = Path(world).resolve().parent / "models"
    # Fortress resolves package:// URIs relative to the parent of a package share.
    resource_env = {
        name: os.pathsep.join(filter(None, [
            str(world_models) if world_models.is_dir() else None,
            str(share.parent), os.environ.get(name),
        ]))
        for name in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH")
    }
    gazebo_args = ["ign", "gazebo", "-r", "-v", "3", world]
    if not gui:
        gazebo_args += ["-s", "--headless-rendering"]

    return [
        ExecuteProcess(
            cmd=gazebo_args,
            additional_env=resource_env,
            output="screen",
            on_exit=Shutdown(reason="Gazebo exited"),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description,
                          "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", "robot_description", "-name", "scout_mini_rdk",
                "-allow_renaming", "false",
                "-x", LaunchConfiguration("x"),
                "-y", LaunchConfiguration("y"),
                "-z", LaunchConfiguration("z"),
                "-Y", LaunchConfiguration("yaw"),
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="scout_mini_bridge",
            parameters=[{
                "config_file": str(share / "config" / "bridge.yaml"),
                "use_sim_time": True,
            }],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "config" / "scout_mini.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("scout_mini_sim"))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "cameras", default_value="true", choices=["true", "false"],
            description="Enable the D435 color/depth sensors; turn off to keep "
                        "lidar and IMU at full rate (e.g. when running FAST-LIO)",
        ),
        DeclareLaunchArgument(
            "world", default_value=str(
                share / "worlds" / "warehouse.sdf"
            ),
            description="World SDF with the physics, scene, sensor and IMU systems",
        ),
        # Warehouse's south doorway is centered at x=0 on the y=-25 wall.
        # Spawn inside the doorway and face +Y into the warehouse.
        DeclareLaunchArgument("x", default_value="0.0",
                              description="Spawn X; default is the warehouse doorway centreline"),
        DeclareLaunchArgument("y", default_value="-22.5",
                              description="Spawn Y; default is 2.5 m inside the south doorway"),
        DeclareLaunchArgument(
            "z", default_value="0.02",
            description="Initial base_footprint height above the ground (m)",
        ),
        DeclareLaunchArgument("yaw", default_value="1.5707963267948966",
                              description="Spawn heading in radians; +pi/2 faces into the warehouse"),
        OpaqueFunction(function=launch_sim),
    ])
