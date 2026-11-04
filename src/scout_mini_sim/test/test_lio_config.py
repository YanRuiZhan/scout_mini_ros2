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
