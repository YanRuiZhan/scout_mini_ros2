"""Exact-time TF composition contracts, isolated from the user's ROS graph."""

import importlib.util
import math
from pathlib import Path
import time
from unittest.mock import Mock

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("lio_bridge", ROOT / "scripts/lio_odom_bridge.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def transform(parent, child, sec=1, p=(0.0, 0.0, 0.0), yaw=0.0):
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.header.stamp = Time(seconds=sec).to_msg()
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = p
    msg.transform.rotation.z = math.sin(yaw / 2)
    msg.transform.rotation.w = math.cos(yaw / 2)
    return msg


def odometry(sec=2, p=(0.0, 0.0, 0.0), yaw=0.0):
    msg = Odometry()
    msg.header.frame_id = "camera_init"
    msg.child_frame_id = "body"
    msg.header.stamp = Time(seconds=sec).to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


@pytest.fixture
def node():
    rclpy.init()
    instance = None
    try:
        instance = MODULE.LioOdomBridge(parameter_overrides=[Parameter("use_sim_time", value=True)])
        instance.broadcaster = Mock()
        instance.buffer.set_transform_static(
            transform("base_footprint", "base_link", p=(0.0, 0.0, 0.180998)), "test",
        )
        instance.buffer.set_transform_static(transform("base_link", "imu_link"), "test")
        yield instance
    finally:
        if instance is not None:
            instance.destroy_node()
        rclpy.shutdown()


def add_odom(node, sec, p=(0.0, 0.0, 0.0), yaw=0.0):
    node.buffer.set_transform(transform("odom", "base_footprint", sec, p, yaw), "test")


def output(node):
    return node.broadcaster.sendTransform.call_args.args[0]


def test_interpolates_at_lio_time_not_latest_and_keeps_height(node):
    add_odom(node, 1, (0.0, 0.0, 0.0))
    add_odom(node, 3, (2.0, 0.0, 0.0))
    node.on_odometry(odometry(2, (11.0, 2.0, 0.0)))
    result = output(node)
    assert result.header.frame_id == "camera_init"
    assert result.child_frame_id == "odom"
    assert result.header.stamp.sec == 2
    assert result.transform.translation.x == pytest.approx(10.0)
    assert result.transform.translation.y == pytest.approx(2.0)
    assert result.transform.translation.z == pytest.approx(-0.180998)


def test_rotation_interpolation_and_translation_composition(node):
    add_odom(node, 1, (2.0, 0.0, 0.0), 0.0)
    add_odom(node, 3, (2.0, 0.0, 0.0), math.pi / 2)
    node.on_odometry(odometry(2, (10.0, 4.0, 0.0), 3 * math.pi / 4))
    result = output(node).transform
    assert result.translation.x == pytest.approx(10.0)
    assert result.translation.y == pytest.approx(2.0)
    assert result.rotation.z == pytest.approx(math.sqrt(0.5))
    assert result.rotation.w == pytest.approx(math.sqrt(0.5))


def test_full_3d_composition():
    # world_T_odom is a 90-degree roll, not just a planar yaw correction.
    q_wo = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    q_oi = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))
    p_oi = (1.0, 2.0, 3.0)
    q_wi = MODULE.quaternion_product(q_wo, q_oi)
    p, q = MODULE.world_to_odom(((5.0, 2.0, 8.0), q_wi), (p_oi, q_oi))
    assert p == pytest.approx((4.0, 5.0, 6.0))
    assert q == pytest.approx(q_wo)


def test_waits_for_delayed_tf_without_extrapolating(node):
    add_odom(node, 1)
    node.on_odometry(odometry(2))
    node.broadcaster.sendTransform.assert_not_called()
    assert len(node.pending) == 1
    add_odom(node, 3)
    node.flush()
    node.broadcaster.sendTransform.assert_called_once()
    assert not node.pending


def test_missing_tf_expires_without_publishing(node):
    node.on_odometry(odometry(2))
    stamp, pose, _ = node.pending[0]
    node.pending[0] = (stamp, pose, time.monotonic() - 1)
    node.flush()
    assert not node.pending
    node.broadcaster.sendTransform.assert_not_called()


@pytest.mark.parametrize("invalid", ["parent", "child", "zero_time", "negative_time", "quaternion", "nan"])
def test_rejects_invalid_input(node, invalid):
    msg = odometry(2)
    if invalid == "parent":
        msg.header.frame_id = "map"
    elif invalid == "child":
        msg.child_frame_id = "base_link"
    elif invalid == "zero_time":
        msg.header.stamp.sec = 0
    elif invalid == "negative_time":
        msg.header.stamp.sec = -1
    elif invalid == "quaternion":
        msg.pose.pose.orientation.w = 0.0
    else:
        msg.pose.pose.position.x = float("nan")
    node.on_odometry(msg)
    assert not node.pending
    node.broadcaster.sendTransform.assert_not_called()


def test_rejects_duplicate_and_out_of_order_input(node):
    add_odom(node, 1)
    add_odom(node, 3)
    node.on_odometry(odometry(2))
    node.on_odometry(odometry(2))
    node.on_odometry(odometry(1))
    node.broadcaster.sendTransform.assert_called_once()


def test_pending_queue_is_bounded(node):
    node.max_pending = 2
    for sec in (1, 2, 3):
        node.on_odometry(odometry(sec))
    assert len(node.pending) == 2
    assert node.pending[0][0].nanoseconds == 2_000_000_000


def test_clock_reset_clears_dynamic_history_and_accepts_new_epoch(node):
    node.get_clock().set_ros_time_override(Time(seconds=10))
    add_odom(node, 10)
    node.on_odometry(odometry(11))
    node.get_clock().set_ros_time_override(Time(seconds=1))
    assert not node.pending
    assert not node.buffer.can_transform("odom", "base_footprint", Time(seconds=10))
    add_odom(node, 1)
    add_odom(node, 3)
    node.on_odometry(odometry(2))
    node.broadcaster.sendTransform.assert_called_once()


def test_rviz_uses_shared_height_color_scale():
    config = yaml.safe_load((ROOT / "config/fast_lio.rviz").read_text())
    displays = {d["Name"]: d for d in config["Visualization Manager"]["Displays"]}
    for name in ("LaserMap", "Scan"):
        assert displays[name]["Color Transformer"] == "AxisColor"
        assert displays[name]["Axis"] == "Z"
        assert displays[name]["Use Fixed Frame"] is True
    bounds = displays["LaserMap"]["Autocompute Value Bounds"]
    assert bounds == displays["Scan"]["Autocompute Value Bounds"]
    assert bounds["Value"] is False
    assert bounds["Min Value"] < bounds["Max Value"]
