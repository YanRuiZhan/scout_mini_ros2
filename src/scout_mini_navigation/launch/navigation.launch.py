"""Nav2 on top of an already running Scout Mini simulation.

Global localization (who publishes map -> odom), chosen with ``localizer``:

- ``ndt`` (default): lidar_localization_ros2 registers each 3D scan to the PCD
  map that the 2D grid was made from (``pcd_map``, default: the map YAML's
  path with a ``.pcd`` suffix). Uses all 16 LiDAR rings.
- ``amcl``: Nav2's 2D particle filter on the middle LiDAR ring.
- ``slam:=true``: SLAM Toolbox builds a new 2D map; no prior map, no localizer.

``initial_pose:="x y yaw"`` (default "0 0 0") starts the localizer at that pose
in the map frame, so no 2D Pose Estimate click is needed. A FAST-LIO map built
from the default spawn has its origin at the spawn point, so "0 0 0" matches.
Pass ``initial_pose:=none`` to set the pose by hand in RViz instead.
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


# ============================================================================
# Helpers (pure Python, unit tested in test/test_initial_pose.py)
# ============================================================================

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


def resolve_pcd_map(map_file, pcd_map):
    """The PCD for 3D localization: pcd_map if given, else <map yaml stem>.pcd."""
    path = Path(pcd_map).expanduser() if pcd_map else Path(map_file).with_suffix(".pcd")
    path = path.resolve()
    if not path.is_file():
        raise RuntimeError(
            f"PCD map not found: {str(path)!r}. localizer:=ndt needs the FAST-LIO PCD the "
            "2D map was made from; pass pcd_map:=/absolute/path/map.pcd, or use localizer:=amcl."
        )
    return str(path)


def ndt_overrides(pcd_path, pose):
    """lidar_localization parameters that the launch arguments decide."""
    overrides = {"map_path": pcd_path, "set_initial_pose": pose is not None}
    if pose is not None:
        x, y, yaw = pose
        overrides.update(
            initial_pose_x=x, initial_pose_y=y,
            initial_pose_qx=0.0, initial_pose_qy=0.0,
            initial_pose_qz=math.sin(yaw / 2.0), initial_pose_qw=math.cos(yaw / 2.0),
        )
    return overrides


# ============================================================================
# Launch
# ============================================================================

def ndt_localization(map_file, params_file, pcd_path, pose):
    """map_server for the 2D grid + lidar_localization publishing map -> odom."""
    share = Path(get_package_share_directory("scout_mini_navigation"))
    return [
        # The planner still plans on the 2D occupancy grid.
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[params_file, {"yaml_filename": map_file, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}],
            output="screen",
        ),
        Node(
            package="lidar_localization_ros2",
            executable="lidar_localization_node",
            name="lidar_localization",
            parameters=[
                str(share / "config" / "lidar_localization.yaml"),
                ndt_overrides(pcd_path, pose),
            ],
            # Raw Gazebo cloud; the node transforms it to base_footprint through TF.
            remappings=[("/cloud", "/scout/lidar/points")],
            output="screen",
        ),
        # Configure + activate the lifecycle node (as upstream's Nav2 launch does).
        Node(
            package="lidar_localization_ros2",
            executable="start_lifecycle_node.py",
            arguments=["lidar_localization"],
            output="screen",
        ),
    ]


def launch_navigation(context):
    bringup = Path(get_package_share_directory("nav2_bringup"))
    slam = LaunchConfiguration("slam").perform(context).lower() == "true"
    localizer = LaunchConfiguration("localizer").perform(context)
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

    actions = []
    if slam or localizer == "amcl":
        # nav2_bringup: SLAM Toolbox, or map_server + AMCL, plus the navigation servers.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch" / "bringup_launch.py")),
            launch_arguments={
                "slam": "True" if slam else "False",
                "map": map_file,
                "use_sim_time": "true",
                "params_file": params_file,
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ))
    else:
        pcd_path = resolve_pcd_map(map_file, LaunchConfiguration("pcd_map").perform(context))
        actions += ndt_localization(map_file, params_file, pcd_path, pose)
        # Navigation servers only (planner, controller, BT navigator, ...), no AMCL.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch" / "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": params_file,
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ))
    actions.append(Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(bringup / "rviz" / "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("scout_mini_navigation"))
    return LaunchDescription([
        DeclareLaunchArgument("slam", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("map", default_value="", description="Absolute path to a 2D map YAML"),
        DeclareLaunchArgument(
            "localizer", default_value="ndt", choices=["ndt", "amcl"],
            description="ndt: 3D NDT against the PCD map; amcl: 2D particle filter",
        ),
        DeclareLaunchArgument(
            "pcd_map", default_value="",
            description="PCD map for localizer:=ndt; default: the map YAML path with .pcd",
        ),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("params_file", default_value=str(share / "config" / "nav2_params.yaml")),
        DeclareLaunchArgument(
            "initial_pose", default_value="0 0 0",
            description="Start pose 'x y yaw' in the map frame (m, rad); "
                        "'none' to set it with 2D Pose Estimate. Ignored with slam:=true",
        ),
        OpaqueFunction(function=launch_navigation),
    ])
