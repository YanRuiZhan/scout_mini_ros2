"""navigation.launch.py helpers: AMCL / NDT start pose and the PCD map lookup."""

import importlib.util
import math
from pathlib import Path

import pytest
import yaml

pytest.importorskip("launch_ros")
ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("navigation_launch", ROOT / "launch/navigation.launch.py")
nav = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(nav)
PARAMS = ROOT / "config/nav2_params.yaml"


def amcl(path):
    return yaml.safe_load(Path(path).read_text())["amcl"]["ros__parameters"]


@pytest.mark.parametrize("text, expected", [
    ("0 0 0", (0.0, 0.0, 0.0)),
    ("1.5, -2 1.5708", (1.5, -2.0, 1.5708)),
    ("none", None),
    ("", None),
])
def test_parse_initial_pose(text, expected):
    assert nav.parse_initial_pose(text) == expected


@pytest.mark.parametrize("text", ["1 2", "a b c", "1 2 nan"])
def test_parse_initial_pose_rejects_bad_input(text):
    with pytest.raises(RuntimeError, match="initial_pose"):
        nav.parse_initial_pose(text)


def test_rewritten_params_set_the_pose_and_keep_everything_else():
    path = nav.with_initial_pose(PARAMS, (1.0, -2.0, 0.5))
    try:
        assert amcl(path)["set_initial_pose"] is True
        assert amcl(path)["initial_pose"] == {"x": 1.0, "y": -2.0, "z": 0.0, "yaw": 0.5}
        original, rewritten = yaml.safe_load(PARAMS.read_text()), yaml.safe_load(Path(path).read_text())
        for params in (original, rewritten):
            del params["amcl"]
        assert rewritten == original
    finally:
        Path(path).unlink()


def test_none_leaves_the_pose_to_rviz():
    path = nav.with_initial_pose(PARAMS, None)
    try:
        assert amcl(path)["set_initial_pose"] is False
    finally:
        Path(path).unlink()


def test_pcd_map_defaults_to_the_grid_yaml_stem(tmp_path):
    grid = tmp_path / "warehouse.yaml"
    grid.write_text("image: warehouse.pgm\n")
    with pytest.raises(RuntimeError, match="PCD map not found"):
        nav.resolve_pcd_map(str(grid), "")
    (tmp_path / "warehouse.pcd").write_text("")
    assert nav.resolve_pcd_map(str(grid), "") == str(tmp_path / "warehouse.pcd")
    other = tmp_path / "other.pcd"
    other.write_text("")
    assert nav.resolve_pcd_map(str(grid), str(other)) == str(other)


def test_ndt_overrides_turn_yaw_into_a_quaternion():
    overrides = nav.ndt_overrides("/maps/w.pcd", (1.0, -2.0, math.pi / 2))
    assert overrides["map_path"] == "/maps/w.pcd"
    assert overrides["set_initial_pose"] is True
    assert (overrides["initial_pose_x"], overrides["initial_pose_y"]) == (1.0, -2.0)
    assert overrides["initial_pose_qz"] == pytest.approx(math.sqrt(0.5))
    assert overrides["initial_pose_qw"] == pytest.approx(math.sqrt(0.5))
    # z comes from config/lidar_localization.yaml (base_footprint height in the map)
    assert "initial_pose_z" not in overrides
    assert nav.ndt_overrides("/maps/w.pcd", None) == {
        "map_path": "/maps/w.pcd", "set_initial_pose": False}


def test_ndt_config_matches_this_robot_and_the_ekf():
    config = yaml.safe_load((ROOT / "config/lidar_localization.yaml").read_text())
    params = config["lidar_localization"]["ros__parameters"]
    # Only map -> odom; odom -> base_footprint belongs to the EKF in scout_mini_sim.
    assert params["enable_map_odom_tf"] is True
    assert params["base_frame_id"] == "base_footprint"
    assert params["use_odom_tf_prediction"] is True
    # The IMU is already fused by the EKF; upstream defaults this to true.
    assert params["use_imu_preintegration"] is False
    # Gazebo's GPU lidar has no per-point time to deskew with.
    assert params["use_continuous_time_deskew"] is False
