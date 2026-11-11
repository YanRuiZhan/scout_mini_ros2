"""Keep the indoor LiDAR range compatible with FAST-LIO's local-map window."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_local_map_has_room_for_both_detection_margins():
    params = yaml.safe_load((ROOT / "config/fast_lio_scout.yaml").read_text())
    params = params["/**"]["ros__parameters"]
    # lasermap_fov_segment uses MOV_THRESHOLD=1.5 on each side of the cube.
    assert params["cube_side_length"] > 3.0 * params["mapping"]["det_range"]


def test_detection_margin_covers_the_configured_lidar_range():
    params = yaml.safe_load((ROOT / "config/fast_lio_scout.yaml").read_text())
    params = params["/**"]["ros__parameters"]
    model = ET.parse(ROOT / "urdf/scout_mini_rdk.urdf.xacro")
    maximum = float(model.find(".//sensor[@type='gpu_lidar']/lidar/range/max").text)
    assert params["mapping"]["det_range"] >= maximum > 0.0


def test_physics_step_divides_lidar_and_imu_periods():
    # Gazebo updates sensors only on physics steps. A period that is not a whole
    # number of steps makes sample intervals alternate (e.g. 9/12 ms), which
    # FAST-LIO integrates as uneven IMU dt and jittered LiDAR stamps.
    world = ET.parse(ROOT / "worlds/warehouse.sdf")
    step_ns = round(float(world.find(".//physics/max_step_size").text) * 1e9)
    model = ET.parse(ROOT / "urdf/scout_mini_rdk.urdf.xacro")
    for sensor_type in ("gpu_lidar", "imu"):
        rate = float(model.find(f".//sensor[@type='{sensor_type}']/update_rate").text)
        period_ns = round(1e9 / rate)
        assert period_ns % step_ns == 0, (sensor_type, rate, step_ns)


def test_degenerate_plane_guard_scales_with_the_map_resolution():
    # The guard rejects matches whose map neighbours are collinear. The most
    # compact real patch in a voxel map (3 + 2 points on adjacent rows) spreads
    # 0.45 voxels across, so a larger threshold would reject valid planes too.
    params = yaml.safe_load((ROOT / "config/fast_lio_scout.yaml").read_text())
    params = params["/**"]["ros__parameters"]
    spread = params["mapping"]["plane_min_spread"]
    assert 0.0 < spread < 0.4 * params["filter_size_map"]


def test_level_constraint_is_tight_enough_to_act():
    # Each scan constrains attitude to a few millidegrees; a looser
    # pseudo-measurement is outvoted and the map still tilts.
    params = yaml.safe_load((ROOT / "config/fast_lio_scout.yaml").read_text())
    mapping = params["/**"]["ros__parameters"]["mapping"]
    assert mapping["level_constraint_en"] is True
    assert 0.0 < mapping["level_constraint_std_deg"] <= 0.01


def test_imu_bridge_queue_survives_a_one_second_stall():
    bridge = yaml.safe_load((ROOT / "config/bridge.yaml").read_text())
    imu = next(entry for entry in bridge if entry["ros_topic_name"] == "/imu/data")
    model = ET.parse(ROOT / "urdf/scout_mini_rdk.urdf.xacro")
    rate = float(model.find(".//sensor[@type='imu']/update_rate").text)
    assert imu["publisher_queue"] >= rate
    assert imu["subscriber_queue"] >= rate
