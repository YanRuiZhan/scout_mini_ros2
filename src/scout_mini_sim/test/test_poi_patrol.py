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


def test_angle_diff_wraps_to_shortest_turn():
    assert patrol.angle_diff(math.radians(170), math.radians(-170)) == pytest.approx(math.radians(-20))
    assert patrol.angle_diff(0.0, -math.pi / 2) == pytest.approx(math.pi / 2)
    assert patrol.angle_diff(1.0, 1.0) == 0


def test_near_goal_monitor_needs_staying_close_and_still():
    m = patrol.NearGoalMonitor((0, 0), radius=0.4, stuck_time=8.0)
    assert not m.update((1.0, 0), 0)          # far away: never stuck
    assert not m.update((0.3, 0), 1)          # entered the radius, timer starts
    assert not m.update((0.32, 0), 8.9)       # still close, 7.9 s
    assert m.update((0.33, 0), 9.0)           # 8 s without moving more than 0.1 m
    # moving inward resets the timer: Nav2 is still making progress
    m = patrol.NearGoalMonitor((0, 0), radius=0.4, stuck_time=8.0)
    for t, x in enumerate([0.39, 0.33, 0.27, 0.21, 0.15]):
        assert not m.update((x, 0), t * 3)
    # leaving the radius clears it; radius 0 disables the check
    assert not m.update((2.0, 0), 100)
    off = patrol.NearGoalMonitor((0, 0), radius=0, stuck_time=1)
    assert not any(off.update((0, 0), t) for t in range(5))


@pytest.mark.parametrize("mode, return_home, count", [
    ("reached", False, 1), ("rejected", False, 1), ("timeout", False, 1), ("no_plan", False, 1),
    ("interrupted", False, 1), ("reached", True, 1), ("no_plan", True, 1), ("interrupted", True, 1),
    ("timeout", False, 3), ("reached", True, 3), ("near", False, 1), ("near", True, 1),
    ("plan", True, 3),
])
def test_patrol_records_and_cancels_without_robot(tmp_path, monkeypatch, mode, return_home, count):
    # Real Humble message classes + CvBridge/OpenCV, fake only the navigation/TF
    # transport. No DDS node is created, and no velocity is sent to a robot.
    rclpy = pytest.importorskip("rclpy")
    cv2 = pytest.importorskip("cv2")
    commander = pytest.importorskip("nav2_simple_commander.robot_navigator")
    tf2 = pytest.importorskip("tf2_ros")
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import Image

    # task: which Nav2 task is running ("nav" = NavigateToPose, "spin" = Spin behavior)
    state = SimpleNamespace(cancel=0, spin=0, closed=False, stopped=0, ticks=0, callback=None,
                            paths=[], spins=[], task=None)
    goals = []
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
            # use_start=False: reachability from the robot; True: pairwise lengths for ordering
            state.paths.append(use_start)
            if mode == "no_plan":
                return None
            if not use_start and goal.pose.position.x == BLOCKED_X:
                return None     # this viewpoint is unreachable right now
            return SimpleNamespace(poses=[start, goal] if use_start else [goal])

        def goToPose(self, goal):
            goals.append(goal)
            state.task, state.cancel_at_start = "nav", state.cancel
            return mode != "rejected"

        def spin(self, spin_dist, time_allowance):
            state.spins.append(spin_dist)
            state.task = "spin"
            return True

        def isTaskComplete(self):
            if mode == "interrupted":
                raise KeyboardInterrupt
            state.ticks += 1
            if state.task == "spin":
                return True
            # "timeout" and "near": Nav2 never finishes on its own, only after a cancel
            return mode not in ("timeout", "near") or state.cancel > state.cancel_at_start

        def cancelTask(self):
            state.cancel += 1

        def getResult(self):
            if state.task == "nav" and mode in ("timeout", "near"):
                return commander.TaskResult.CANCELED
            return commander.TaskResult.SUCCEEDED

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
                           timeout=0.005 if mode == "timeout" else 300, settle_time=0,
                           image_timeout=3, return_home=return_home, order="auto", distance="nav",
                           near_radius=0.4, stuck_time=0.05, align_tolerance=0.1)
    pois = [dict(sample_poi(), id=f"P{i}", target={"x": 1.0 + 4 * i, "y": 2.0}) for i in range(count)]
    plan = offline_plan(pois) if mode == "plan" else None
    if mode == "interrupted":
        with pytest.raises(KeyboardInterrupt):
            patrol.run(args, pois)
    else:
        patrol.run(args, pois, plan)
    with next(tmp_path.glob("*/patrol_log.csv")).open() as stream:
        all_rows = list(csv.DictReader(stream))
    row = all_rows[0]
    if return_home and mode != "interrupted":
        # Even after a failed POI the robot heads back to where it started.
        home = all_rows[-1]
        assert len(all_rows) == count + 1 and home["id"] == "HOME"
        assert home["result"] in patrol.REACHED
        assert (float(home["goal_x"]), float(home["goal_y"])) == (1.0, 4.5)
        last = goals[-1].pose
        assert (last.position.x, last.position.y) == (1.0, 4.5)
    else:
        assert len(all_rows) == (1 if mode == "interrupted" else count)
    if count == 3:
        visited = [r["id"] for r in all_rows[:3]]
        if return_home:
            # a closed tour has the same length either way round
            assert visited in (["P0", "P1", "P2"], ["P2", "P1", "P0"])
        else:
            # robot starts next to P0, so the open tour sweeps P0 -> P1 -> P2
            assert visited == ["P0", "P1", "P2"]
    if mode == "plan":
        # offline matrix: only the start -> viewpoint row is asked from Nav2 (3 queries);
        # P1's first-choice viewpoint is unreachable, so its second candidate is used
        assert state.paths.count(True) == count
        ranks = {r["id"]: r["view_rank"] for r in all_rows}
        assert ranks == {"P0": "1", "P1": "2", "P2": "1", "HOME": ""}
    if mode == "timeout" and count == 3:
        # 6 pairwise lengths up front; after the first failure the two remaining points are
        # re-ordered from the robot's pose (2 queries); a single remaining point needs none
        assert state.paths.count(True) == 3 * 4 // 2 + 2
    if mode != "no_plan":
        # the viewpoint is checked from the robot, then the ordering asks Nav2 for path lengths
        assert state.paths[0] is False and True in state.paths
        assert row["plan_m"] != ""
    assert row["result"] == {"near": "reached_near", "plan": "reached"}.get(mode, mode)
    assert state.closed
    # the fake navigator finishes a task only after it is cancelled: one cancel per stuck task
    expected_cancel = {"timeout": count, "interrupted": 1, "near": 2 if return_home else 1}
    assert state.cancel == expected_cancel.get(mode, 0)
    if mode == "near":
        # the robot (yaw 0) sits on P0's viewpoint, which faces -y: the fallback cancels Nav2,
        # spins -90 deg itself and still takes the photo; at home it is already aligned
        assert state.spins == [pytest.approx(-math.pi / 2)]
        assert row["image_result"] == "saved"
        assert float(row["yaw_err_deg"]) == pytest.approx(90.0)
    if mode == "reached":
        assert row["image_result"] == "saved" and state.spin >= 2
        image = cv2.imread(str(next(tmp_path.glob("*/*.png"))))
        assert image[0, 0].tolist() == [0, 0, 255]
        if count == 1:      # the fake robot never leaves its start pose
            assert float(row["pos_err_m"]) == 0


BLOCKED_X = 99.0     # x of a viewpoint that the fake planner cannot reach


def offline_plan(pois):
    """What load_plan() returns: two candidates per POI, the first of P1 unreachable."""
    candidates, points = {}, []
    for poi in pois:
        x, y, yaw = patrol.view_pose(poi, 2.5)
        first = BLOCKED_X if poi["id"] == "P1" else x
        candidates[poi["id"]] = []
        for rank, cx in enumerate((first, x), 1):
            candidates[poi["id"]].append({"x": cx, "y": y, "yaw": yaw, "r": 2.5, "rank": rank,
                                          "phi_deg": 0.0, "score": 1.0, "index": len(points)})
            points.append((cx, y))
    matrix = [[math.dist(a, b) for b in points] for a in points]
    return {"candidates": candidates, "matrix": matrix}


def write_plan_files(tmp_path):
    """A tiny map + POI file and a plan that records their SHA-1 like plan_offline.py does."""
    (tmp_path / "m.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([254] * 4))
    (tmp_path / "m.yaml").write_text("image: m.pgm\nresolution: 1.0\norigin: [0, 0, 0]\n")
    poi_file = tmp_path / "poi.yaml"
    poi_file.write_text(yaml.safe_dump({"frame_id": "map", "pois": [sample_poi()]}))
    pois = [dict(sample_poi(), id=f"P{i}", target={"x": 1.0 + 4 * i, "y": 2.0}) for i in range(3)]
    plan = offline_plan(pois)
    viewpoints = [dict(c, device=pid) for pid, cs in plan["candidates"].items() for c in cs]
    data = {
        "version": 1, "frame_id": "map",
        "sources": {"poi": {"sha1": patrol.file_sha1(poi_file)},
                    "map": {"sha1": patrol.file_sha1(tmp_path / "m.yaml", tmp_path / "m.pgm")}},
        "devices": [{"id": poi["id"], "type": poi["type"], "target": poi["target"],
                     "normal": poi["normal"],
                     "viewpoints": [c["index"] for c in plan["candidates"][poi["id"]]]}
                    for poi in pois],
        "viewpoints": [{k: v for k, v in vp.items() if k != "index"} for vp in viewpoints],
        "matrix": plan["matrix"],
    }
    (tmp_path / "plan.yaml").write_text(yaml.safe_dump(data))
    return tmp_path / "plan.yaml", tmp_path / "m.yaml", poi_file


def test_plan_dry_run_staleness_and_filter(tmp_path, capsys):
    plan, map_yaml, poi_file = write_plan_files(tmp_path)
    base = ["--plan", str(plan), "--map", str(map_yaml), "--poi", str(poi_file), "--dry-run"]
    assert patrol.main(base + ["--start-x", "1", "--start-y", "4.5"]) == 0
    text = capsys.readouterr().out
    assert "离线规划" in text and "P0" in text and "HOME" in text and "合计" in text
    # P1's first candidate sits at x = 99 in the plan; dry-run shows the planner's first choice
    assert "99.00" in text
    assert patrol.main(base + ["--only", "P2"]) == 0
    assert "P0" not in capsys.readouterr().out
    assert patrol.main(base + ["--only", "nope"]) == 1
    assert "不存在" in capsys.readouterr().err
    # editing the map or the POI file after planning makes the plan stale
    map_yaml.write_text(map_yaml.read_text() + "# edited\n")
    assert patrol.main(base) == 1
    assert "地图" in capsys.readouterr().err
    poi_file.write_text(poi_file.read_text() + "# edited\n")
    assert patrol.main(["--plan", str(plan), "--poi", str(poi_file), "--dry-run"]) == 1
    assert "POI" in capsys.readouterr().err
    # neither --poi nor --plan
    assert patrol.main(["--dry-run"]) == 1


def test_dry_run_reports_order_and_length(capsys):
    args = ["--poi", str(CONFIG), "--start-x", "0", "--start-y", "-22.5", "--dry-run"]
    assert patrol.main(args) == 0
    exact = capsys.readouterr().out
    assert patrol.main(args + ["--order", "greedy"]) == 0
    greedy = capsys.readouterr().out
    total = [float(text.strip().splitlines()[-1].split()[1]) for text in (exact, greedy)]
    assert total[0] < total[1]            # exact never loses to greedy on the same distances
    assert "HOME" in exact and "exact" in exact
