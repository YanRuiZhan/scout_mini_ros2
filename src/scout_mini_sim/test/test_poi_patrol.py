"""POI data/coordinate regressions and patrol orchestration without robot motion."""
import csv
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("poi_patrol", ROOT / "scripts/poi_patrol.py")
patrol = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(patrol)
CONFIG = ROOT / "config/poi_warehouse_inspection.yaml"


def sample_poi():
    return {"id": "test", "type": "gauge", "target": {"x": 1.0, "y": 2.0},
            "normal": {"x": 0.0, "y": 2.0}}


def test_surface_normals_and_pose():
    poi = sample_poi()
    assert patrol.view_pose(poi, 1.5) == pytest.approx((1, 3.5, -math.pi / 2))
    ordered = patrol.order_nearest([poi, dict(poi, id="near", target={"x": 0, "y": -1})],
                                   (0, 0), 1)
    assert [p["id"] for p in ordered] == ["near", "test"]


def test_warehouse_pois_match_scene_faces():
    # Validate all ten points against independent SDF poses and mesh vertices,
    # including the rotated tank whose normal has two nonzero components.
    scene_path = ROOT / "worlds/warehouse.sdf"
    if not scene_path.exists():
        pytest.skip("local warehouse assets unavailable")
    scene = ET.parse(scene_path)
    models = {include.findtext("name"): include for include in scene.findall(".//include")}
    pois = patrol.load_pois(CONFIG)
    assert len(pois) == 10
    for poi in pois:
        model_name = "poi_" + poi["id"].lower().replace("-", "_")
        pose = list(map(float, models[model_name].findtext("pose").split()))
        mesh = ROOT / "worlds/models" / model_name / "meshes/face.obj"
        if not mesh.exists():
            pytest.skip("local face meshes unavailable")
        vertices = [list(map(float, line.split()[1:]))
                    for line in mesh.read_text().splitlines() if line.startswith("v ")]
        center = [sum(v[i] for v in vertices) / len(vertices) for i in range(3)]
        c, s = math.cos(pose[5]), math.sin(pose[5])
        expected = (pose[0] + c * center[0] - s * center[1],
                    pose[1] + s * center[0] + c * center[1])
        assert (poi["target"]["x"], poi["target"]["y"]) == pytest.approx(expected, abs=0.002)
        norm = math.hypot(poi["normal"]["x"], poi["normal"]["y"])
        assert (poi["normal"]["x"] / norm, poi["normal"]["y"] / norm) == pytest.approx(
            (s, -c), abs=0.002)


@pytest.mark.parametrize("change, message", [
    ({"normal": {"x": 0, "y": 0}}, "normal"),
    ({"target": {"x": float("nan"), "y": 0}}, "有限"),
    ({"target": {}}, "数字"),
    ({"id": ""}, "id"),
])
def test_invalid_input(tmp_path, change, message):
    path = tmp_path / "poi.yaml"
    path.write_text(yaml.safe_dump({"pois": [dict(sample_poi(), **change)]}))
    with pytest.raises(ValueError, match=message):
        patrol.load_pois(path)


def test_filter_dry_run_and_frame_guard(capsys):
    assert patrol.main(["--poi", str(CONFIG), "--only", "PG-W01", "--dry-run"]) == 0
    text = capsys.readouterr().out
    assert "world" in text and "PG-W01" in text and "AP-W01" not in text
    assert patrol.main(["--poi", str(CONFIG), "--only", "missing", "--dry-run"]) == 1
    assert "不存在" in capsys.readouterr().err
    assert patrol.main(["--poi", str(CONFIG)]) == 1
    assert "--poi-transform" in capsys.readouterr().err


def test_transform_rotates_targets_and_normals(tmp_path):
    path = tmp_path / "poi.yaml"
    path.write_text(yaml.safe_dump({"frame_id": "world", "pois": [sample_poi()]}))
    args = SimpleNamespace(poi=path, only=None, frame="map", dry_run=False,
                           poi_transform=[10, 20, math.pi / 2])
    pois, frame = patrol.prepare_pois(args)
    assert frame == "map"
    assert patrol.view_pose(pois[0], 1) == pytest.approx((7, 21, 0), abs=1e-10)


def test_empty_config_is_rejected_before_ros(tmp_path, capsys):
    path = tmp_path / "poi.yaml"
    path.write_text("pois: []\n")
    assert patrol.main(["--poi", str(path)]) == 1
    assert "没有可巡检" in capsys.readouterr().err


@pytest.mark.parametrize("mode", ["reached", "rejected", "timeout", "no_plan", "interrupted"])
def test_patrol_records_and_cancels_without_robot(tmp_path, monkeypatch, mode):
    # Real Humble message classes + CvBridge/OpenCV, fake only the navigation/TF
    # transport. No DDS node is created, and no velocity is sent to a robot.
    rclpy = pytest.importorskip("rclpy")
    cv2 = pytest.importorskip("cv2")
    commander = pytest.importorskip("nav2_simple_commander.robot_navigator")
    tf2 = pytest.importorskip("tf2_ros")
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import Image

    state = SimpleNamespace(cancel=0, spin=0, closed=False, stopped=0, ticks=0, callback=None)
    transform = TransformStamped()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 4.5

    class Navigator:
        def __init__(self, **kwargs):
            self.feedback = None

        def create_subscription(self, _, topic, callback, qos):
            assert topic == "/camera/color/image_raw"
            state.callback = callback

        def create_publisher(self, *args):
            def publish(msg):
                assert msg.linear.x == 0 and msg.angular.z == 0
                state.stopped += 1
            return SimpleNamespace(publish=publish)

        def get_clock(self):
            return SimpleNamespace(now=lambda: rclpy.time.Time(seconds=1))

        def get_logger(self):
            return SimpleNamespace(info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None)

        def getPath(self, start, goal, use_start):
            assert use_start is False
            return None if mode == "no_plan" else SimpleNamespace(poses=[goal])

        def goToPose(self, goal):
            return mode != "rejected"

        def isTaskComplete(self):
            if mode == "interrupted":
                raise KeyboardInterrupt
            state.ticks += 1
            return mode != "timeout" or state.cancel > 0

        def cancelTask(self):
            state.cancel += 1

        def getResult(self):
            return commander.TaskResult.CANCELED if mode == "timeout" else commander.TaskResult.SUCCEEDED

        def destroy_node(self):
            state.closed = True

    def spin_once(*args, **kwargs):
        state.spin += 1
        msg = Image()
        # The first frame predates settling and must be discarded.
        msg.header.stamp.sec = 0 if state.spin == 1 else 2
        msg.width, msg.height, msg.step, msg.encoding = 1, 1, 3, "rgb8"
        msg.data = bytes([255, 0, 0])
        state.callback(msg)

    monkeypatch.setattr(rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(rclpy, "ok", lambda: True)
    monkeypatch.setattr(rclpy, "spin_once", spin_once)
    monkeypatch.setattr(commander, "BasicNavigator", Navigator)
    monkeypatch.setattr(tf2, "Buffer", lambda **kw: SimpleNamespace(lookup_transform=lambda *a: transform))
    monkeypatch.setattr(tf2, "TransformListener", lambda *a: None)
    monkeypatch.setattr(patrol, "wait_for_navigation", lambda *a: transform)
    # Advance wall time per check so timeout behavior never needs a real sleep.
    clock = iter(range(10000))
    monkeypatch.setattr(patrol.time, "monotonic", lambda: next(clock) * 0.01)
    args = SimpleNamespace(use_sim_time=True, frame="map", base_frame="base_footprint",
                           start_x=None, start_y=None, out=str(tmp_path), dist=[2.5],
                           cmd_vel_topic="/cmd_vel", image_topic="/camera/color/image_raw",
                           timeout=0.005, settle_time=0, image_timeout=3)
    if mode == "interrupted":
        with pytest.raises(KeyboardInterrupt):
            patrol.run(args, [sample_poi()])
    else:
        patrol.run(args, [sample_poi()])
    with next(tmp_path.glob("*/patrol_log.csv")).open() as stream:
        row = list(csv.DictReader(stream))[0]
    assert row["result"] == mode
    assert state.closed
    assert state.cancel == (1 if mode in ("timeout", "interrupted") else 0)
    if mode == "reached":
        assert row["image_result"] == "saved" and state.spin >= 2
        image = cv2.imread(str(next(tmp_path.glob("*/*.png"))))
        assert image[0, 0].tolist() == [0, 0, 255]
        assert float(row["pos_err_m"]) == 0
