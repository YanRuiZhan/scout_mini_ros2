"""Run FAST-LIO2 against the Scout Mini Gazebo simulation.

Start ``sim.launch.py`` first and keep the robot still until FAST-LIO logs
``IMU Initial Done``. This launch adds the point-field adapter, starts FAST-LIO
with the simulation's instantaneous-frame timing mode, and feeds its pose into
the odometry EKF through ``lio_to_odom.py``.

- Mapping (default ``mapping:=true``): accumulate and publish the full map and
  save it as PCD on shutdown.
- Navigation (``mapping:=false``): odometry only. The accumulated map, the PCD
  buffer and the path grow with every scan, so they are turned off to keep the
  per-scan cost constant during long runs.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = Path(get_package_share_directory("scout_mini_sim"))
    mapping = ParameterValue(LaunchConfiguration("mapping"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument(
            "mapping", default_value="true", choices=["true", "false"],
            description="true: build and save the map; false: odometry only for navigation",
        ),
        DeclareLaunchArgument(
            "settle_time", default_value="1.0",
            description="Seconds of LiDAR timestamps to wait before IMU initialization; "
                        "keep the robot stationary during startup",
        ),
        Node(
            package="scout_mini_sim",
            executable="lidar_time.py",
            name="lidar_time_pad",
            parameters=[{
                "input_topic": "/scout/lidar/points",
                "output_topic": "/scout/lidar/points_time",
                "startup_delay_sec": ParameterValue(
                    LaunchConfiguration("settle_time"), value_type=float,
                ),
            }],
            output="screen",
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            parameters=[
                str(sim_share / "config" / "fast_lio_scout.yaml"),
                {
                    "use_sim_time": True,
                    "publish.map_en": mapping,
                    "publish.path_en": mapping,
                    "pcd_save.pcd_save_en": mapping,
                },
            ],
            output="screen",
        ),
        # FAST-LIO /Odometry -> /lio/odom (odom frame), fused by the EKF in sim.launch.py.
        Node(
            package="scout_mini_sim",
            executable="lio_to_odom.py",
            name="lio_to_odom",
            parameters=[{"use_sim_time": True}],
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
