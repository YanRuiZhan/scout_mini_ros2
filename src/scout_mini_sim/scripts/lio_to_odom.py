#!/usr/bin/env python3
"""把 FAST-LIO2 的位姿转换成 odom 坐标系下的底盘位姿，供 EKF 融合。

数据流
------
FAST-LIO2  /Odometry (camera_init -> body)     10 Hz，body 即 imu_link
    │
    ▼  本节点：换算到 odom 坐标系、换成 base_footprint、压平到二维、补偿延迟
/lio/odom  (odom -> base_footprint，时间戳为当前)
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

延迟补偿
--------
FAST-LIO 处理一帧要几十毫秒，位姿到达时车已经往前走了一段。若把旧位置当成现在的
位置交给 EKF，车越快位姿越往回拉。这里用 EKF 自己在这段时间里的运动把它推到当前：

    odom_T_base(now) = lio(t) * inverse(ekf(t)) * ekf(now)

即"LIO 在 t 时刻对 EKF 的修正量"作用到 EKF 的最新位姿上，再以最新时间戳发布。

跳变保护
--------
EKF 很信任 LIO，LIO 一旦发散（例如 IMU 大量丢失），odom 会被带着跳走，车在地图上
跳到别处甚至地图外，规划随之失败。因此 LIO 与 EKF 在同一时刻相差超过
max_jump_m 或 max_jump_yaw_deg 时丢弃该帧；连续 realign_after 帧都超限，
说明 LIO 已发散，按 EKF 当前位姿重新对齐，并提示重启 lio.launch.py。
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


def propagate(lio_at_t, ekf_at_t, ekf_now):
    """把 t 时刻的 LIO 位姿推算到 EKF 最新时刻：lio(t) * inverse(ekf(t)) * ekf(now)。"""
    return flatten(compose(compose(lio_at_t, inverse(ekf_at_t)), ekf_now))


def is_jump(lio_at_t, ekf_at_t, max_m, max_yaw):
    """LIO 与 EKF 同一时刻的位姿相差是否超过阈值（米、弧度）。"""
    (lp, lq), (ep, eq) = lio_at_t, ekf_at_t
    dyaw = math.remainder(yaw_of(lq) - yaw_of(eq), math.tau)
    return math.hypot(lp[0] - ep[0], lp[1] - ep[1]) > max_m or abs(dyaw) > max_yaw


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
            # 跳变保护：10 Hz 下正常的 LIO 修正只有厘米级
            "max_jump_m": 0.5,
            "max_jump_yaw_deg": 15.0,
            "realign_after": 20,            # 连续超限帧数（约 2 s）后重新对齐
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value  # noqa: E731
        self.lio_frame, self.odom_frame = param("lio_frame"), param("odom_frame")
        self.base_frame, self.imu_frame = param("base_frame"), param("imu_frame")
        self.max_jump_m = param("max_jump_m")
        self.max_jump_yaw = math.radians(param("max_jump_yaw_deg"))
        self.realign_after = param("realign_after")
        self.rejected = 0
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
        lio_at_t = flatten(compose(compose(self.odom_T_ci, ci_T_imu), inverse(self.base_T_imu)))
        # 4) 补偿处理延迟：用 EKF 在 t 时刻和最新时刻的位姿，把 LIO 位姿推到当前
        try:
            ekf_at_t = self.lookup(self.odom_frame, self.base_frame, stamp)
            latest = self.buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except TransformException as error:
            self.get_logger().warning(f"等待 TF：{error}", throttle_duration_sec=5.0)
            return
        ekf_now = to_pose(latest.transform.translation, latest.transform.rotation)
        # 5) 跳变保护：与 EKF 差太多的帧不交给 EKF
        if is_jump(lio_at_t, flatten(ekf_at_t), self.max_jump_m, self.max_jump_yaw):
            self.rejected += 1
            self.get_logger().warning(
                "LIO 位姿与 EKF 相差过大，已丢弃（LIO 可能发散，检查 IMU gap 警告）",
                throttle_duration_sec=2.0,
            )
            if self.rejected >= self.realign_after:
                self.get_logger().error(
                    "LIO 连续发散，已按 EKF 当前位姿重新对齐；建议重启 lio.launch.py 重建 LIO 地图"
                )
                self.odom_T_ci = None
                self.rejected = 0
            return
        self.rejected = 0
        odom_T_base = propagate(lio_at_t, ekf_at_t, ekf_now)
        out = Odometry()
        out.header.stamp = latest.header.stamp   # 已推算到 EKF 最新时刻
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
