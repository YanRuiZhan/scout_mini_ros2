"""Preview the simulation URDF in RViz without Gazebo or real hardware."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("scout_mini_sim"))
    gui = LaunchConfiguration("gui")
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            str(share / "urdf" / "scout_mini_rdk.urdf.xacro"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true", choices=["true", "false"],
            description="Show joint sliders; otherwise publish zero wheel positions",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", choices=["true", "false"],
            description="Open RViz to display the model",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"use_sim_time": False}],
            condition=IfCondition(gui),
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"use_sim_time": False}],
            condition=UnlessCondition(gui),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "config" / "display_sim_urdf.rviz")],
            parameters=[{"use_sim_time": False}],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
