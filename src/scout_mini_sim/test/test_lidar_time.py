"""Check timestamp gating and point preservation before FAST-LIO initialization."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from sensor_msgs.msg import Imu, PointCloud2, PointField


SPEC = importlib.util.spec_from_file_location(
    "lidar_time", Path(__file__).resolve().parents[1] / "scripts/lidar_time.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def cloud(nanoseconds):
    msg = PointCloud2()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(nanoseconds, 1_000_000_000)
    msg.header.frame_id = "lidar_3d_link"
    msg.height, msg.width = 1, 1
    msg.point_step, msg.row_step = 32, 32
    msg.data = bytes(range(32))
    return msg


def adapter():
    return SimpleNamespace(
        publisher=Mock(), imu_publisher=Mock(), get_logger=Mock(), warned_layout=False,
        startup_delay_ns=1_000_000_000, first_stamp_ns=None,
        last_stamp_ns=None, startup_ready=False,
    )


def test_waits_for_sensor_time_and_preserves_measurements():
    node = adapter()
    for stamp in [100_000_000, 100_000_000, 1_099_999_999]:
        MODULE.LidarTimePad.callback(node, cloud(stamp))
    node.publisher.publish.assert_not_called()

    original = cloud(1_100_000_000)
    MODULE.LidarTimePad.callback(node, original)
    result = node.publisher.publish.call_args.args[0]
    assert result.header == original.header
    assert bytes(result.data[:20]) == bytes(original.data[:20])
    assert bytes(result.data[24:]) == bytes(original.data[24:])
    assert bytes(result.data[20:24]) == np.float32(0).tobytes()
    field = next(f for f in result.fields if f.name == "time")
    assert field.offset == 20 and field.datatype == PointField.FLOAT32


def test_clock_reset_restarts_wait():
    node = adapter()
    for stamp in [5_000_000_000, 6_000_000_000]:
        MODULE.LidarTimePad.callback(node, cloud(stamp))
    assert node.publisher.publish.call_count == 1
    for stamp in [0, 999_999_999]:
        MODULE.LidarTimePad.callback(node, cloud(stamp))
    assert node.publisher.publish.call_count == 1
    MODULE.LidarTimePad.callback(node, cloud(1_000_000_000))
    assert node.publisher.publish.call_count == 2


def test_falling_imu_is_not_buffered_for_initialization():
    node = adapter()
    msg = Imu()
    MODULE.LidarTimePad.imu_callback(node, msg)
    MODULE.LidarTimePad.callback(node, cloud(100_000_000))
    msg.header.stamp.nanosec = 500_000_000
    MODULE.LidarTimePad.imu_callback(node, msg)
    node.imu_publisher.publish.assert_not_called()
    msg.header.stamp.sec, msg.header.stamp.nanosec = 1, 100_000_000
    MODULE.LidarTimePad.imu_callback(node, msg)
    node.imu_publisher.publish.assert_called_once_with(msg)
