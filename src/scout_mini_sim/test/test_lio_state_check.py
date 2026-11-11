"""Check the flat-floor acceptance report built from FAST-LIO's state log."""

import importlib.util
import math
from pathlib import Path

import numpy as np


SPEC = importlib.util.spec_from_file_location(
    "lio_state_check", Path(__file__).resolve().parents[1] / "scripts/lio_state_check.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_log(path, pitch_per_metre_deg=0.0, ba_x=0.0):
    """2 s stationary, 20 m straight at 0.5 m/s, a 90 degree turn, 7.5 m straight."""
    lines = []
    x = y = z = yaw = 0.0
    for i in range(0, 600):
        t = 0.3 + 0.1 * i
        if i < 20:
            v, w = 0.0, 0.0
        elif i < 420:
            v, w = 0.5, 0.0
        elif i < 450:
            v, w = 0.3, math.radians(30.0)
        else:
            v, w = 0.5, 0.0
        yaw += w * 0.1
        x += v * math.cos(yaw) * 0.1
        y += v * math.sin(yaw) * 0.1
        dist = math.hypot(x, y) if i < 420 else 0.0
        pitch = -pitch_per_metre_deg * dist if i < 420 else -pitch_per_metre_deg * 20.0
        z += v * 0.1 * math.tan(math.radians(-pitch))
        # mat_pre.txt: t, euler[deg], pos, ext_euler, ext_T, vel, bg, ba, grav
        row = [t, 0.0, pitch, math.degrees(yaw), x, y, z, 0, 0, 0, 0.05, 0, 0.3,
               v * math.cos(yaw), v * math.sin(yaw), 0.0, 0, 0, 0, ba_x, 0, 0, 0, 0, -9.81]
        lines.append(" ".join(f"{value:.9g}" for value in row))
    path.write_text("IMU log header that is not a state line\n" + "\n".join(lines) + "\n")
    return path


def test_flat_run_passes_every_limit(tmp_path):
    state = MODULE.derive(MODULE.load(write_log(tmp_path / "mat_pre.txt")))
    lines, passed = MODULE.report(state, every=10.0)
    assert passed
    assert all(ok for *_, ok in MODULE.checks(state))
    assert abs(state["dist"][-1] - (20.0 + 0.9 + 7.5)) < 0.2
    assert any("stationary" in line for line in lines)


def test_tilt_that_builds_while_driving_is_attributed_to_straight_motion(tmp_path):
    log = write_log(tmp_path / "mat_pre.txt", pitch_per_metre_deg=0.1, ba_x=-0.15)
    state = MODULE.derive(MODULE.load(log))
    growth = MODULE.growth_by_motion(state)
    assert growth["straight"][2] > 1.5          # ~2 deg gained over the 20 m leg
    assert abs(growth["stationary"][2]) < 0.01
    assert abs(growth["turning"][2]) < 0.2
    assert state["slope_deg"] > 0.5             # nose-up tilt makes the path climb
    failed = {name for name, _, _, ok in MODULE.checks(state) if not ok}
    assert failed == {"max |roll|, |pitch| deg", "max |accel bias| m/s2", "|z slope| deg"}
    _, passed = MODULE.report(state, every=0.0)
    assert not passed


def test_motion_classes_follow_speed_and_yaw_rate(tmp_path):
    state = MODULE.derive(MODULE.load(write_log(tmp_path / "mat_pre.txt")))
    motion = state["motion"]
    assert motion[5] == "stationary"
    assert motion[200] == "straight"
    assert motion[435] == "turning"
    assert np.isclose(state["tilt"].max(), 0.0, atol=1e-6)
