#!/usr/bin/env python3
"""Give Gazebo lidar clouds the ``time`` field FAST-LIO expects.

Gazebo's ``gpu_lidar`` publishes ``x, y, z, intensity, ring`` with a
32-byte point stride: the four bytes at offset 20..23 are padding. PCL
cannot convert such a cloud into FAST-LIO's velodyne point type because
that type also has a ``time`` member.

Gazebo's GPU LiDAR renders a frame at one simulation timestamp, so this
adapter zero-fills the time field over the existing padding. The matching
FAST-LIO simulation configuration enables ``preprocess.instantaneous_scan``
to prevent azimuth-based timing reconstruction. No point data is copied or
reordered beyond adding the field.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

# 2 s of 200 Hz IMU. FAST-LIO integrates every sample, so a short queue that
# overflows while a callback runs silently drops motion between LiDAR frames.
IMU_QUEUE_DEPTH = 400

# Layout produced by ign gazebo's PointCloudPacked for a gpu_lidar.
# 雷达点共32byte数据 20-23字节为空，在其中写入time
INPUT_STEP = 32
TIME_OFFSET = 20
TIME_BYTES = 4

# The simulator provides a single timestamp for the complete GPU LiDAR frame.
TIME_VALUE = 0.0


class LidarTimePad(Node):
    def __init__(self):
        super().__init__("lidar_time_pad")
        self.declare_parameter("input_topic", "/scout/lidar/points")
        self.declare_parameter("output_topic", "/scout/lidar/points_time")
        self.declare_parameter("startup_delay_sec", 1.0)
        self.declare_parameter("imu_input_topic", "/imu/data")
        self.declare_parameter("imu_output_topic", "/scout/imu/data_settled")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        delay = self.get_parameter("startup_delay_sec").value
        if not math.isfinite(delay) or delay < 0:
            raise ValueError("startup_delay_sec must be finite and nonnegative")
        self.startup_delay_ns = round(delay * 1_000_000_000)
        self.first_stamp_ns = None
        self.last_stamp_ns = None
        self.startup_ready = False

        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos_profile_sensor_data
        )
        self.imu_publisher = self.create_publisher(
            Imu, self.get_parameter("imu_output_topic").value,
            QoSProfile(depth=IMU_QUEUE_DEPTH, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.imu_subscription = self.create_subscription(
            Imu, self.get_parameter("imu_input_topic").value,
            self.imu_callback,
            QoSProfile(depth=IMU_QUEUE_DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.warned_layout = False
        self.get_logger().info(f"padding {input_topic} -> {output_topic}")
        if self.startup_delay_ns:
            self.get_logger().info(
                f"Waiting {delay:g} s of LiDAR timestamps for the robot to settle "
                "before FAST-LIO initialization."
            )

    def callback(self, msg: PointCloud2) -> None:
        if msg.point_step != INPUT_STEP or msg.width * msg.height == 0:
            if not self.warned_layout:
                self.get_logger().warning(
                    f"unexpected cloud layout: point_step={msg.point_step}, "
                    f"size={msg.width}x{msg.height}; expected point_step={INPUT_STEP}"
                )
                self.warned_layout = True
            return

        # FAST-LIO estimates gravity and an accelerometer scale from its first
        # IMU batch. A robot falling onto the floor can corrupt that estimate.
        # Gate both IMU and LiDAR: withholding only LiDAR would leave falling
        # IMU samples buffered inside FAST-LIO for its first initialization batch.
        # Use sensor timestamps so a slow/paused Gazebo cannot shorten the wait.
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.first_stamp_ns = None
            self.startup_ready = False
            self.get_logger().warning(
                "LiDAR clock moved backwards; restarting the settling delay. "
                "Restart FAST-LIO after resetting the world to clear its old map."
            )
        self.last_stamp_ns = stamp_ns
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp_ns
        if stamp_ns - self.first_stamp_ns < self.startup_delay_ns:
            return
        if not self.startup_ready:
            self.startup_ready = True
            self.get_logger().info("Settling delay complete; forwarding LiDAR clouds.")

        # The padding bytes are not guaranteed to be zero by the sender.
        # FAST-LIO's instantaneous_scan mode ignores this field's values.
        points = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, INPUT_STEP).copy()
        points[:, TIME_OFFSET:TIME_OFFSET + TIME_BYTES] = np.frombuffer(
            np.float32(TIME_VALUE).tobytes(), dtype=np.uint8
        )

        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.is_bigendian = False
        out.point_step = INPUT_STEP
        out.row_step = INPUT_STEP * msg.width
        out.is_dense = False
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=TIME_OFFSET, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=24, datatype=PointField.UINT16, count=1),
        ]
        out.data = points.tobytes()
        self.publisher.publish(out)

    def imu_callback(self, msg: Imu) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.first_stamp_ns is None:
            return
        if stamp_ns < self.first_stamp_ns:
            self.first_stamp_ns = None
            self.startup_ready = False
            return
        if stamp_ns - self.first_stamp_ns >= self.startup_delay_ns:
            self.imu_publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = LidarTimePad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
