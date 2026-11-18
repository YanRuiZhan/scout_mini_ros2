"""Pose math of lio_to_odom.py: aligning camera_init to odom and expressing the base pose."""

import importlib.util
import math
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
SPEC = importlib.util.spec_from_file_location(
    "lio_to_odom", Path(__file__).resolve().parents[1] / "scripts/lio_to_odom.py",
)
lio = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(lio)


def pose(x, y, z=0.0, yaw=0.0):
    return (x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def assert_pose(actual, expected):
    (p, q), (ep, eq) = actual, expected
    assert p == pytest.approx(ep, abs=1e-9)
    # q and -q are the same rotation
    assert q == pytest.approx(eq, abs=1e-9) or q == pytest.approx(tuple(-v for v in eq), abs=1e-9)


def test_inverse_and_compose_round_trip():
    a = pose(1.0, -2.0, 0.3, yaw=0.7)
    assert_pose(lio.compose(a, lio.inverse(a)), pose(0, 0))
    # rotate 0.7 rad, then move 0.5 m along the rotated x axis
    p, _ = lio.compose(a, pose(0.5, 0.0))
    assert p == pytest.approx((1 + 0.5 * math.cos(0.7), -2 + 0.5 * math.sin(0.7), 0.3))


def test_late_start_keeps_odom_continuous():
    # The EKF has already moved the robot to (3, 1) facing +y when FAST-LIO
    # starts; camera_init is wherever the IMU was at that moment.
    base_T_imu = pose(0.05, 0.0, 0.25)
    imu_T_base = lio.inverse(base_T_imu)
    odom_T_base_start = pose(3.0, 1.0, yaw=math.pi / 2)

    def lio_pose(start_T_base):
        """FAST-LIO output (camera_init -> imu) after the base moved by start_T_base."""
        return lio.compose(lio.compose(imu_T_base, start_T_base), base_T_imu)

    # Alignment exactly as LioToOdom.on_lio does it on the first frame.
    odom_T_imu = lio.compose(odom_T_base_start, base_T_imu)
    odom_T_ci = lio.compose(odom_T_imu, lio.inverse(lio_pose(pose(0, 0))))

    def to_odom(ci_T_imu):
        return lio.flatten(lio.compose(lio.compose(odom_T_ci, ci_T_imu), imu_T_base))

    # First LIO frame reproduces the EKF pose: no jump when fusion starts.
    assert_pose(to_odom(lio_pose(pose(0, 0))), odom_T_base_start)
    # 2 m forward along the start heading (odom +y), then a 90 deg left turn.
    assert_pose(to_odom(lio_pose(pose(2.0, 0.0, yaw=math.pi / 2))),
                pose(3.0, 3.0, yaw=math.pi))


def test_flatten_drops_height_roll_and_pitch():
    roll = 0.1
    tilted = ((1.0, 2.0, 0.4), lio.quat_mul(pose(0, 0, yaw=0.5)[1],
                                           (math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2))))
    assert_pose(lio.flatten(tilted), pose(1.0, 2.0, yaw=0.5))
