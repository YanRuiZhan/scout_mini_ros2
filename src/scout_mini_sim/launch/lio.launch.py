"""Run FAST-LIO2 against the Scout Mini Gazebo simulation.

Start ``sim.launch.py`` first. This launch adds the point-field adapter and
starts FAST-LIO with the simulation's instantaneous-frame timing mode.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = Path(get_package_share_directory("scout_mini_sim"))

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument(
            "connect_odom", default_value="true", choices=["true", "false"],
            description="Publish camera_init -> odom from time-aligned LIO/TF; "
                        "disable when AMCL or another localizer owns odom's parent",
        ),
        Node(
            package="scout_mini_sim",
            executable="lidar_time.py",
            name="lidar_time_pad",
            parameters=[{
                "input_topic": "/scout/lidar/points",
                "output_topic": "/scout/lidar/points_time",
            }],
            output="screen",
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            parameters=[
                str(sim_share / "config" / "fast_lio_scout.yaml"),
                {"use_sim_time": True},
            ],
            output="screen",
        ),
        Node(
            package="scout_mini_sim",
            executable="lio_odom_bridge.py",
            name="lio_odom_bridge",
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("connect_odom")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(sim_share / "config" / "fast_lio.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
