#!/usr/bin/env python3
"""Connect FAST-LIO's world frame to the existing odometry TF tree.

FAST-LIO's `body` is the IMU frame, named `imu_link` in this robot's URDF.
At the LIO timestamp: world_T_odom = world_T_imu * inverse(odom_T_imu).
tf2 interpolates the existing odometry; no latest-time lookup or extrapolation
is used. This node must be disabled when another localizer owns odom's parent.
"""

from collections import deque
import math
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quaternion_product(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (
        w * X + x * W + y * Z - z * Y,
        w * Y - x * Z + y * W + z * X,
        w * Z + x * Y - y * X + z * W,
        w * W - x * X - y * Y - z * Z,
    )


def rotate(q, v):
    return quaternion_product(
        quaternion_product(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3])
    )[:3]


def rigid_pose(position, orientation):
    p = (position.x, position.y, position.z)
    q = (orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(v) for v in (*p, *q)):
        raise ValueError("non-finite pose")
    norm = math.hypot(*q)
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    return p, tuple(v / norm for v in q)


def world_to_odom(world_imu, odom_imu):
    p_wi, q_wi = world_imu
    p_oi, q_oi = odom_imu
    q_wo = quaternion_product(q_wi, (-q_oi[0], -q_oi[1], -q_oi[2], q_oi[3]))
    offset = rotate(q_wo, p_oi)
    return tuple(a - b for a, b in zip(p_wi, offset)), q_wo


class LioOdomBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("lio_odom_bridge", **kwargs)
        defaults = {
            "lio_topic": "/Odometry",
            "world_frame": "camera_init",
            "lio_body_frame": "body",
            "odom_frame": "odom",
            "imu_frame": "imu_link",
            "tf_wait_timeout": 0.5,
            "max_pending": 100,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.world_frame = self.get_parameter("world_frame").value
        self.body_frame = self.get_parameter("lio_body_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.wait_timeout = self.get_parameter("tf_wait_timeout").value
        self.max_pending = self.get_parameter("max_pending").value
        frames = (self.world_frame, self.body_frame, self.odom_frame, self.imu_frame)
        if any(not f or f.startswith("/") for f in frames) or len(set(frames)) != 4:
            raise ValueError("frame names must be nonempty, distinct, without leading '/'")
        if not math.isfinite(self.wait_timeout) or self.wait_timeout <= 0 or self.max_pending < 1:
            raise ValueError("tf_wait_timeout and max_pending must be positive")

        self.pending = deque()
        self.last_received_ns = -1
        self.warning_times = {}
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, self.get_parameter("lio_topic").value,
            self.on_odometry, qos_profile_sensor_data,
        )
        # Retry without blocking TF callbacks; wall time also bounds waits if /clock stops.
        self.retry_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.retry_timer = self.create_timer(0.01, self.flush, clock=self.retry_clock)
        self.jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True,
            ),
            post_callback=self.on_time_jump,
        )
        self.get_logger().info(
            f"Publishing {self.world_frame} -> {self.odom_frame}; "
            f"matching {self.odom_frame} -> {self.imu_frame} at each LIO timestamp. "
            "Disable this node before starting AMCL or another global localizer."
        )

    def warn(self, key, message):
        now = time.monotonic()
        if now - self.warning_times.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self.warning_times[key] = now

    def on_time_jump(self, _jump):
        self.pending.clear()
        self.last_received_ns = -1
        self.buffer.clear()
        self.get_logger().warning("Clock changed/reset; discarded pending poses and TF history.")

    def on_odometry(self, msg):
        if msg.header.frame_id != self.world_frame or msg.child_frame_id != self.body_frame:
            self.warn("frames", "Ignoring LIO odometry with unexpected parent/child frame IDs.")
            return
        try:
            stamp = Time.from_msg(msg.header.stamp)
        except ValueError:
            self.warn("stamp", "Ignoring invalid LIO timestamp.")
            return
        # tf2 interprets zero as 'latest', so zero cannot be an exact-time sample.
        if stamp.nanoseconds <= 0 or stamp.nanoseconds <= self.last_received_ns:
            self.warn("stamp", "Ignoring zero, duplicate or out-of-order LIO timestamp.")
            return
        try:
            pose = rigid_pose(msg.pose.pose.position, msg.pose.pose.orientation)
        except ValueError as error:
            self.warn("pose", f"Ignoring invalid LIO pose: {error}")
            return
        self.last_received_ns = stamp.nanoseconds
        if len(self.pending) >= self.max_pending:
            self.pending.popleft()
            self.warn("queue", "TF wait queue full; dropped oldest LIO pose.")
        self.pending.append((stamp, pose, time.monotonic() + self.wait_timeout))
        self.flush()

    def flush(self):
        while self.pending:
            stamp, pose, deadline = self.pending[0]
            if time.monotonic() >= deadline:
                self.pending.popleft()
                self.warn(
                    "timeout", f"Dropped LIO pose: no timely exact-stamp TF "
                    f"{self.odom_frame} -> {self.imu_frame}. "
                    "Check /clock, use_sim_time, base odometry and robot_state_publisher.",
                )
                continue
            try:
                odom_imu = self.buffer.lookup_transform(self.odom_frame, self.imu_frame, stamp)
            except TransformException:
                return
            self.pending.popleft()
            try:
                p, q = world_to_odom(pose, rigid_pose(
                    odom_imu.transform.translation, odom_imu.transform.rotation,
                ))
            except ValueError as error:
                self.warn("tf_pose", f"Ignoring invalid odometry transform: {error}")
                continue
            output = TransformStamped()
            output.header.stamp = stamp.to_msg()
            output.header.frame_id = self.world_frame
            output.child_frame_id = self.odom_frame
            translation, rotation = output.transform.translation, output.transform.rotation
            translation.x, translation.y, translation.z = p
            rotation.x, rotation.y, rotation.z, rotation.w = q
            self.broadcaster.sendTransform(output)


def main():
    rclpy.init()
    node = LioOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
