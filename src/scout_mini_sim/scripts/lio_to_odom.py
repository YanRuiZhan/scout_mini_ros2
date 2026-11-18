#!/usr/bin/env python3
"""把 FAST-LIO2 的位姿转换成 odom 坐标系下的底盘位姿，供 EKF 融合。

数据流
------
FAST-LIO2  /Odometry (camera_init -> body)     10 Hz，body 即 imu_link
    │
    ▼  本节点：换算到 odom 坐标系、换成 base_footprint、压平到二维
/lio/odom  (odom -> base_footprint)
    │
    ▼  robot_localization EKF（config/ekf.yaml）
/odom 和 TF odom -> base_footprint             50 Hz
    （位置和航向以 LIO 为准，两帧雷达之间用轮速 + IMU 陀螺仪补齐）

坐标系对齐
----------
camera_init 是 FAST-LIO 启动时 IMU 所在的位置，odom 是 EKF 启动时底盘所在的位置，
两者一般不重合（例如先开仿真开了一段，再启动 FAST-LIO）。
收到第一帧 LIO 位姿时，从 TF 读取 EKF 此刻给出的 odom -> imu_link，求出

    odom_T_ci = odom_T_imu(t0) * inverse(ci_T_imu(t0))

此后它固定不变，并作为静态 TF odom -> camera_init 发布，FAST-LIO 的点云
（/cloud_registered 等）因此也能在 map / odom 下显示。TF 树：

    map ── odom ─┬─ base_footprint ── base_link ── 传感器    （EKF）
      （AMCL）   └─ camera_init ── body                    （FAST-LIO）

重启 FAST-LIO（重启 lio.launch.py）时本节点也会重启并重新对齐，odom 保持连续。
"""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


# ============================================================================
# 位姿运算（纯函数）。位姿用 (p, q) 表示：p=(x, y, z)，四元数 q=(x, y, z, w)。
# a_T_b 表示"b 坐标系在 a 坐标系中的位姿"，compose(a_T_b, b_T_c) = a_T_c。
# ============================================================================

def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate(q, v):
    """用四元数 q 旋转向量 v：q * v * q^-1。"""
    conj = (-q[0], -q[1], -q[2], q[3])
    return quat_mul(quat_mul(q, (*v, 0.0)), conj)[:3]


def compose(a, b):
    """a_T_b ∘ b_T_c -> a_T_c。"""
    (pa, qa), (pb, qb) = a, b
    offset = rotate(qa, pb)
    return tuple(x + y for x, y in zip(pa, offset)), quat_mul(qa, qb)


def inverse(a):
    """a_T_b -> b_T_a。"""
    p, q = a
    conj = (-q[0], -q[1], -q[2], q[3])
    return tuple(-v for v in rotate(conj, p)), conj


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def flatten(pose):
    """压平到地面：z=0、去掉 roll/pitch，只保留 x、y、yaw（Nav2 和 AMCL 都是二维的）。"""
    (x, y, _), q = pose
    yaw = yaw_of(q)
    return (x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def to_pose(position, orientation):
    """ROS 消息 -> (p, q)，同时检查有限性并归一化四元数。"""
    p = (position.x, position.y, position.z)
    q = (orientation.x, orientation.y, orientation.z, orientation.w)
    norm = math.sqrt(sum(v * v for v in q))
    if not all(math.isfinite(v) for v in (*p, *q)) or norm < 1e-9:
        raise ValueError("非有限位姿或零四元数")
    return p, tuple(v / norm for v in q)


# ============================================================================
# ROS 节点
# ============================================================================

class LioToOdom(Node):
    def __init__(self):
        super().__init__("lio_to_odom")
        defaults = {
            "lio_topic": "/Odometry",
            "output_topic": "/lio/odom",
            "lio_frame": "camera_init",     # FAST-LIO 的世界坐标系
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "imu_frame": "imu_link",        # FAST-LIO 的 body 就是 IMU 坐标系
            # 交给 EKF 的测量标准差。越小 EKF 越信任 LIO；它决定了 LIO 与轮速/IMU 的权重。
            "position_std": 0.02,           # m
            "yaw_std_deg": 0.5,             # 度
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value  # noqa: E731
        self.lio_frame, self.odom_frame = param("lio_frame"), param("odom_frame")
        self.base_frame, self.imu_frame = param("base_frame"), param("imu_frame")
        position_var = param("position_std") ** 2
        yaw_var = math.radians(param("yaw_std_deg")) ** 2
        # 6x6 协方差按 [x, y, z, roll, pitch, yaw] 排列；EKF 只融合 x、y、yaw。
        self.covariance = [0.0] * 36
        for index, value in ((0, position_var), (7, position_var), (35, yaw_var),
                             (14, 1e-6), (21, 1e-6), (28, 1e-6)):
            self.covariance[index] = value

        self.base_T_imu = None   # 静态外参，来自 robot_state_publisher
        self.odom_T_ci = None    # 首帧对齐结果
        self.last_ns = -1

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publisher = self.create_publisher(Odometry, param("output_topic"), 10)
        self.create_subscription(Odometry, param("lio_topic"), self.on_lio, qos_profile_sensor_data)
        self.get_logger().info(
            f"{param('lio_topic')} -> {param('output_topic')}；等待首帧 LIO 位姿以对齐 odom"
        )

    def lookup(self, parent, child, stamp):
        t = self.buffer.lookup_transform(parent, child, stamp)
        return to_pose(t.transform.translation, t.transform.rotation)

    def on_lio(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        # 仿真时间倒退（Gazebo 重置）：之前的对齐失效，重新对齐。
        if stamp.nanoseconds < self.last_ns:
            self.get_logger().warning("时间倒退（仿真重置？），重新对齐 odom 与 camera_init")
            self.odom_T_ci = None
        self.last_ns = stamp.nanoseconds
        try:
            ci_T_imu = to_pose(msg.pose.pose.position, msg.pose.pose.orientation)
        except ValueError as error:
            self.get_logger().warning(f"忽略无效 LIO 位姿：{error}", throttle_duration_sec=5.0)
            return

        try:
            # 1) 外参 base_footprint -> imu_link，只需读一次
            if self.base_T_imu is None:
                self.base_T_imu = self.lookup(self.base_frame, self.imu_frame, Time())
            # 2) 首帧对齐：用 EKF 在同一时刻给出的 odom -> imu_link
            if self.odom_T_ci is None:
                odom_T_imu = self.lookup(self.odom_frame, self.imu_frame, stamp)
                self.odom_T_ci = compose(odom_T_imu, inverse(ci_T_imu))
                self.publish_static_tf()
                self.get_logger().info("已对齐 odom 与 camera_init，开始输出 LIO 里程计")
        except TransformException as error:
            # 刚启动时 EKF 或 robot_state_publisher 的 TF 可能还没到，下一帧再试。
            self.get_logger().warning(f"等待 TF：{error}", throttle_duration_sec=5.0)
            return

        # 3) odom_T_base = odom_T_ci * ci_T_imu * imu_T_base，再压平到二维
        odom_T_base = flatten(compose(compose(self.odom_T_ci, ci_T_imu), inverse(self.base_T_imu)))
        out = Odometry()
        out.header.stamp = msg.header.stamp      # 保留雷达时间戳，EKF 据此回溯融合
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        (p, q) = odom_T_base
        pos, ori = out.pose.pose.position, out.pose.pose.orientation
        pos.x, pos.y, pos.z = p
        ori.x, ori.y, ori.z, ori.w = q
        out.pose.covariance = self.covariance
        self.publisher.publish(out)

    def publish_static_tf(self):
        (p, q) = self.odom_T_ci
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.lio_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.static_tf.sendTransform(t)


def main():
    rclpy.init()
    node = LioToOdom()
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
