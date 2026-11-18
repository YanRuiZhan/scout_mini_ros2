#!/usr/bin/env python3
"""Scout Mini POI 巡检：生成观测位姿、Nav2 导航、到点拍照并记录 CSV。

整体流程
--------
1. 读取 POI 配置（YAML），校验每个目标的 id / type / target / normal。
2. 若 POI 不在 Nav2 的 map 坐标系下，用 --poi-transform 转换坐标。
3. 对每个 POI，沿目标表面外法向退后若干米，得到"观测位姿"（机器人面朝目标）。
4. 从机器人当前位置出发，按最近邻顺序排列 POI，减少来回跑。
5. 依次：Nav2 规划 → 导航 → 到点稳定 → 拍一张新图像 → 写一行 CSV。
6. 结束或 Ctrl+C 时取消导航目标，并向 /cmd_vel 发零速度停车。

使用前提
--------
先启动 sim.launch.py（cameras:=true）和 navigation.launch.py，并在 RViz 完成定位。
无需 ROS 的检查：python3 poi_patrol.py --poi <配置.yaml> --dry-run
运行与坐标转换说明见 scout_mini_sim/README.md。

代码结构
--------
- 纯 Python 部分（不依赖 ROS，可单独测试 / dry-run）：
  load_pois、view_pose、order_nearest、prepare_pois、write_log
- ROS 部分（ROS 相关模块在函数内部才导入，dry-run 时不需要 ROS 环境）：
  wait_for_navigation、run
- 命令行入口：main
"""
import argparse
import csv
import math
import os
from pathlib import Path
import re
import sys
from tempfile import mkdtemp
import time

import yaml


# CSV 日志的列，每个 POI 一行：
#   order        巡检顺序（从 1 开始）
#   result       导航结果：reached / no_plan / rejected / timeout / canceled / failed / interrupted
#   view_dist    实际采用的观测距离 (m)
#   goal_*       发给 Nav2 的目标位姿；robot_* 为到达后从 TF 读到的实际位置
#   pos_err_m    实际位置与目标位置的距离 (m)
#   time_s       导航耗时（墙钟时间，s）
#   image        保存的图片文件名；image_result 为 saved / no_image / error
LOG_FIELDS = [
    "order", "id", "type", "result", "view_dist", "goal_x", "goal_y", "goal_yaw",
    "robot_x", "robot_y", "pos_err_m", "time_s", "image", "image_result",
]

# lifecycle_msgs/State 中 PRIMARY_STATE_ACTIVE 的取值：节点已激活，可以接收任务。
LIFECYCLE_ACTIVE = 3


# ============================================================================
# 一、POI 数据处理（纯 Python，不依赖 ROS）
# ============================================================================

def load_pois(path):
    """读取并校验 POI 文件，返回 POI 列表。

    文件格式（见 config/poi_warehouse_inspection.yaml）::

        frame_id: world          # 可选，POI 坐标所在坐标系
        pois:
          - id: PG-W01           # 唯一编号
            type: pressure_gauge # 目标类型，仅用于记录
            target: {x: 7.65, y: -14.0}   # 目标表面中心
            normal: {x: 0.0, y: 1.0}      # 表面外法向（指向可观测的一侧）

    所有数据问题都在启动 ROS 之前以 ValueError 报出，避免车跑到一半才发现配置错误。
    target / normal 会被统一转换成 float。
    """
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get("pois"), list):
        raise ValueError("POI 文件必须包含 pois 列表")

    ids = set()
    for index, poi in enumerate(data["pois"], 1):
        # --- id 和 type：非空字符串，id 不能重复 ---
        if not isinstance(poi, dict):
            raise ValueError(f"第 {index} 个 POI 必须是映射")
        if not isinstance(poi.get("id"), str) or not poi["id"].strip():
            raise ValueError(f"第 {index} 个 POI 缺少非空字符串 id")
        if poi["id"] in ids:
            raise ValueError(f"POI id 重复: {poi['id']}")
        ids.add(poi["id"])
        if not isinstance(poi.get("type"), str) or not poi["type"].strip():
            raise ValueError(f"POI {poi['id']} 缺少非空字符串 type")

        # --- target 和 normal：必须有有限的数字 x / y ---
        for group in ("target", "normal"):
            try:
                xy = [float(poi[group][axis]) for axis in ("x", "y")]
            except (KeyError, TypeError, ValueError):
                raise ValueError(f"POI {poi['id']} 的 {group} 必须包含数字 x/y") from None
            if not all(math.isfinite(value) for value in xy):
                raise ValueError(f"POI {poi['id']} 的 {group}.x/y 必须是有限数")
            poi[group] = dict(zip(("x", "y"), xy))

        # --- 法向量不能是零向量，否则无法确定观测方向 ---
        norm = math.hypot(poi["normal"]["x"], poi["normal"]["y"])
        if not math.isfinite(norm) or norm == 0:
            raise ValueError(f"POI {poi['id']} 的 normal 不能为零或溢出向量")
    return data["pois"]


def view_pose(poi, dist):
    """计算观测位姿 (x, y, yaw)：从目标沿外法向退后 dist 米，车头 (+X) 朝向目标。

    设单位法向 n = (nx, ny)：
        观测点   p = target + dist * n
        朝向   yaw = atan2(-ny, -nx)      # 即 -n 的方向，从观测点指回目标
    """
    target, normal = poi["target"], poi["normal"]
    norm = math.hypot(normal["x"], normal["y"])
    nx, ny = normal["x"] / norm, normal["y"] / norm  # 归一化，配置里的法向不必是单位向量
    return target["x"] + dist * nx, target["y"] + dist * ny, math.atan2(-ny, -nx)


def order_nearest(pois, start_xy, dist):
    """贪心最近邻排序：每次去离当前位置最近的观测点。不修改输入列表。

    这不是最优路线（旅行商问题），且用的是直线距离而非实际路径长度，
    但对少量 POI 足够简单有效。
    """
    left, ordered, current = list(pois), [], start_xy
    while left:
        nxt = min(left, key=lambda poi: math.dist(view_pose(poi, dist)[:2], current))
        left.remove(nxt)
        ordered.append(nxt)
        current = view_pose(nxt, dist)[:2]
    return ordered


def prepare_pois(args):
    """加载 POI、按 --only 过滤、做坐标转换，返回 (pois, 坐标系名)。

    坐标系规则：
    - 文件里的 frame_id 表示 POI 坐标所在坐标系；没写时按 --frame（默认 map）处理。
    - 给了 --poi-transform TX TY YAW 时，把 POI 从源坐标系转到 --frame：
          p_goal = R(YAW) * p_poi + [TX, TY]
      其中 target 是"点"，要旋转再平移；normal 是"方向"，只旋转不平移。
    - 正式导航时，源坐标系必须与 --frame 一致，否则拒绝运行（dry-run 例外）。
      例如 warehouse 配置是 Gazebo world 坐标，不能直接当作 map 坐标使用。
    """
    pois = load_pois(args.poi)
    with open(args.poi, encoding="utf-8") as stream:
        source_frame = yaml.safe_load(stream).get("frame_id", args.frame)
    if not isinstance(source_frame, str) or not source_frame.strip():
        raise ValueError("frame_id 必须是非空字符串")

    # 只保留 --only 指定的编号；拼错的编号直接报错，而不是悄悄跳过。
    if args.only is not None:
        missing = set(args.only) - {poi["id"] for poi in pois}
        if missing:
            raise ValueError("POI 文件中不存在这些 id: " + ", ".join(sorted(missing)))
        pois = [poi for poi in pois if poi["id"] in args.only]
    if not pois:
        raise ValueError("没有可巡检的 POI")

    # 二维刚体变换：先旋转 YAW，再平移 (TX, TY)。
    if args.poi_transform is not None:
        tx, ty, yaw = args.poi_transform
        c, s = math.cos(yaw), math.sin(yaw)
        for poi in pois:
            for group in ("target", "normal"):
                x, y = poi[group]["x"], poi[group]["y"]
                poi[group] = {"x": c * x - s * y, "y": s * x + c * y}
            poi["target"]["x"] += tx  # 只有点需要平移，法向量不平移
            poi["target"]["y"] += ty
        source_frame = args.frame

    if source_frame != args.frame and not args.dry_run:
        raise ValueError(
            f"POI 属于 {source_frame}，目标坐标系为 {args.frame}；请提供 "
            "--poi-transform TX TY YAW（目标系中的平移和弧度旋转），或使用已转换的 POI 文件"
        )
    return pois, source_frame


def write_log(path, rows):
    """整体重写 CSV。每完成一个 POI 调用一次，中途崩溃也能保留已完成的记录。"""
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=LOG_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


# ============================================================================
# 二、ROS 部分：等待 Nav2 就绪、执行巡检
# ============================================================================

def wait_for_navigation(nav, buffer, args):
    """等待 Nav2 各节点激活、定位 TF 可用，返回当前 map -> base_footprint 变换。

    没有使用 BasicNavigator.waitUntilNav2Active()：它会自动发布 initialpose，
    可能覆盖用户在 RViz 里设置的 2D Pose Estimate。这里只"查询"不"设置"。
    所有等待共享一个截止时间 (--startup-timeout)，超时抛 RuntimeError。
    """
    import rclpy
    from lifecycle_msgs.srv import GetState
    from rclpy.time import Time
    from tf2_ros import TransformException

    deadline = time.monotonic() + args.startup_timeout

    # ---- 第 1 步：逐个查询生命周期节点，直到状态为 active ----
    nodes = ["bt_navigator", "planner_server", "controller_server"]
    if args.localizer != "none":
        nodes.insert(0, args.localizer)  # amcl 或 slam_toolbox 也必须已激活
    for name in nodes:
        client = nav.create_client(GetState, name.rstrip("/") + "/get_state")
        try:
            while rclpy.ok():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"等待 {name} active 超时，请先启动仿真和 Nav2")
                # 服务还没出现：处理一下回调再重试
                if not client.wait_for_service(timeout_sec=0.2):
                    rclpy.spin_once(nav, timeout_sec=0.1)
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(nav, future, timeout_sec=0.5)
                if future.done() and future.result() is not None:
                    if future.result().current_state.id == LIFECYCLE_ACTIVE:
                        break
                else:
                    # 本次请求没有及时返回，丢弃它，避免请求越积越多
                    client.remove_pending_request(future)
                rclpy.spin_once(nav, timeout_sec=0.1)
        finally:
            nav.destroy_client(client)

    # ---- 第 2 步：等待 /clock 和定位 TF ----
    while rclpy.ok():
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"等待 /clock 或 {args.frame} -> {args.base_frame} TF 超时；"
                "AMCL 模式请先在 RViz 设置 2D Pose Estimate"
            )
        rclpy.spin_once(nav, timeout_sec=0.1)
        # 仿真时间下，收到第一条 /clock 之前时钟为 0，此时 TF 查询没有意义
        if args.use_sim_time and nav.get_clock().now().nanoseconds == 0:
            continue
        try:
            # Time() 即时间 0，表示"取最新可用的变换"
            return buffer.lookup_transform(args.frame, args.base_frame, Time())
        except TransformException:
            continue  # AMCL 尚未收到初始位姿时，map -> odom 还不存在
    raise RuntimeError("ROS 已关闭")


def run(args, pois):
    """连接 ROS，按顺序巡检所有 POI；结果写入 <out>/<时间_随机后缀>/。"""
    # ROS 相关模块在这里才导入，这样 dry-run 和单元测试不需要 ROS 环境。
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from sensor_msgs.msg import Image
    from tf2_ros import Buffer, TransformException, TransformListener

    # 关闭 rclpy 自带的 SIGINT 处理，保留 Python 的 KeyboardInterrupt，
    # 这样 Ctrl+C 后 finally 块还能在 ROS context 关闭前取消目标、发送停车指令。
    rclpy.init(args=["--ros-args", "-p", f"use_sim_time:={str(args.use_sim_time).lower()}"],
               signal_handler_options=SignalHandlerOptions.NO)

    nav = None
    rows = []               # 所有 POI 的记录（CSV 行）
    log_path = None
    motion_started = False  # 是否已经发过导航目标；发过才需要在退出时停车
    task_active = False     # 当前是否有未结束的导航任务；有则退出时需要取消
    try:
        # ------------------------------------------------------------------
        # 初始化：导航器、TF、相机订阅、停车用的速度发布者
        # ------------------------------------------------------------------
        nav = BasicNavigator(node_name="poi_patrol")
        buffer = Buffer(node=nav)
        listener = TransformListener(buffer, nav)  # noqa: F841  需保持引用，否则不再接收 TF
        bridge = CvBridge()

        # 图像缓存。after_ns 为 None 时不接收任何图像；
        # 拍照时设为"当前时间"，只接收时间戳晚于它的新帧，避免拿到导航途中的旧图。
        latest = {"img": None, "after_ns": None}

        def on_image(msg):
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if latest["after_ns"] is not None and stamp_ns > latest["after_ns"]:
                latest["img"] = msg

        nav.create_subscription(Image, args.image_topic, on_image, qos_profile_sensor_data)
        stop_pub = nav.create_publisher(Twist, args.cmd_vel_topic, 10)

        # ------------------------------------------------------------------
        # 等待 Nav2 就绪，并以当前位置为起点排序
        # ------------------------------------------------------------------
        nav.get_logger().info("等待 Nav2 和定位 TF；AMCL 模式请在 RViz 设置初始位姿")
        current = wait_for_navigation(nav, buffer, args)
        if args.start_x is None:
            start_xy = (current.transform.translation.x, current.transform.translation.y)
        else:
            start_xy = (args.start_x, args.start_y)  # 仅影响排序，不会重置定位
        pois = order_nearest(pois, start_xy, args.dist[0])

        # 每次任务建一个独立目录（时间 + 随机后缀），避免重跑时覆盖照片和 CSV。
        os.makedirs(args.out, exist_ok=True)
        output = Path(mkdtemp(prefix=time.strftime("%Y%m%d_%H%M%S_"), dir=args.out))
        log_path = output / "patrol_log.csv"
        write_log(log_path, rows)

        # ------------------------------------------------------------------
        # 每个 POI 用到的小步骤
        # ------------------------------------------------------------------
        def make_pose(x, y, yaw):
            """平面位姿 -> PoseStamped。绕 Z 轴旋转 yaw 的四元数为 (0, 0, sin(yaw/2), cos(yaw/2))。"""
            pose = PoseStamped()
            pose.header.frame_id = args.frame
            pose.header.stamp = nav.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            return pose

        def choose_goal(poi, row):
            """按 --dist 顺序尝试观测距离，返回第一个能规划出路径的目标；都不行返回 None。

            例如目标前 2.5 m 处被货架挡住，就退到 3.0 m、2.0 m 再试。
            """
            for dist in args.dist:
                x, y, yaw = view_pose(poi, dist)
                candidate = make_pose(x, y, yaw)
                # use_start=False：Humble 会忽略 start，从机器人在全局代价地图中的当前位姿规划。
                path = nav.getPath(PoseStamped(), candidate, use_start=False)
                if path is not None and path.poses:
                    row.update(view_dist=dist, goal_x=x, goal_y=y, goal_yaw=yaw)
                    return candidate
            return None

        def navigate(goal, row):
            """发送导航目标并阻塞等待结果，把结果和耗时写入 row。"""
            nonlocal motion_started, task_active
            started = time.monotonic()
            motion_started = True
            # Commander 的 getPath 和 goToPose 共用 result_future / feedback，先清掉上一次的状态。
            nav.feedback = None
            task_active = True
            if not nav.goToPose(goal):  # 返回 False 表示目标被 bt_navigator 拒绝
                task_active = False
                row["result"] = "rejected"
                return
            # isTaskComplete() 内部会 spin，所以这个循环不会卡死回调。
            timed_out = False
            while not nav.isTaskComplete():
                if not timed_out and time.monotonic() - started > args.timeout:
                    timed_out = True
                    nav.cancelTask()  # 只取消一次，然后继续等它真正结束
            task_active = False
            result = nav.getResult()
            row["time_s"] = round(time.monotonic() - started, 2)
            row["result"] = "timeout" if timed_out else {
                TaskResult.SUCCEEDED: "reached", TaskResult.CANCELED: "canceled",
                TaskResult.FAILED: "failed",
            }.get(result, "unknown")

        def record_position(poi, row):
            """读取到达后的实际位置，计算与目标点的偏差。"""
            try:
                actual = buffer.lookup_transform(args.frame, args.base_frame, Time())
                rx, ry = actual.transform.translation.x, actual.transform.translation.y
                row.update(robot_x=rx, robot_y=ry,
                           pos_err_m=math.hypot(rx - row["goal_x"], ry - row["goal_y"]))
            except TransformException:
                nav.get_logger().warning(f"{poi['id']}: 无最终定位 TF，未记录位置误差")

        def capture_image(index, poi, row):
            """到点后等待车身稳定，再拍一张"新"图像保存为 PNG。"""
            # 1) 静置 settle_time 秒，让车停稳、画面不再晃动
            until = time.monotonic() + args.settle_time
            while time.monotonic() < until:
                rclpy.spin_once(nav, timeout_sec=0.05)
            # 2) 只接收此刻之后采集的图像，排除 DDS 队列里还没处理的旧帧
            latest.update(img=None, after_ns=nav.get_clock().now().nanoseconds)
            until = time.monotonic() + args.image_timeout
            while latest["img"] is None and time.monotonic() < until:
                rclpy.spin_once(nav, timeout_sec=0.1)
            # 3) 保存图像
            if latest["img"] is None:
                row["image_result"] = "no_image"
                nav.get_logger().warning(
                    f"{poi['id']}: 未收到新图像，请检查 {args.image_topic} 和 cameras:=true"
                )
            else:
                safe_id = re.sub(r"[^\w.-]", "_", poi["id"])  # 去掉不适合做文件名的字符
                filename = output / f"{index:02d}_{safe_id}.png"
                try:
                    image = bridge.imgmsg_to_cv2(latest["img"], desired_encoding="bgr8")
                    if not cv2.imwrite(str(filename), image):
                        raise OSError(f"图片写入失败: {filename}")
                    row.update(image=filename.name, image_result="saved")
                except Exception as error:
                    row["image_result"] = "error"
                    nav.get_logger().error(f"{poi['id']}: {error}")
            # 4) 停止接收图像，直到下一次拍照
            latest.update(img=None, after_ns=None)

        # ------------------------------------------------------------------
        # 主循环：规划 -> 导航 -> 记录位置 -> 拍照 -> 写 CSV
        # ------------------------------------------------------------------
        for index, poi in enumerate(pois, 1):
            row = dict.fromkeys(LOG_FIELDS, "")
            # 默认记为 error；正常流程会覆盖它，异常时就保留 error
            row.update(order=index, id=poi["id"], type=poi["type"], result="error")
            rows.append(row)
            try:
                goal = choose_goal(poi, row)
                if goal is None:
                    row["result"] = "no_plan"
                    nav.get_logger().warning(f"{poi['id']}: 所有候选距离均无路径，跳过")
                    continue
                nav.get_logger().info(f"[{index}/{len(pois)}] 前往 {poi['id']}")
                navigate(goal, row)
                if row["result"] == "rejected":
                    continue
                record_position(poi, row)
                if row["result"] != "reached":
                    continue  # 没到点就不拍照
                capture_image(index, poi, row)
            except KeyboardInterrupt:
                row["result"] = "interrupted"
                raise
            finally:
                write_log(log_path, rows)  # 无论成功失败，每个点结束都落盘

        reached = sum(row["result"] == "reached" for row in rows)
        nav.get_logger().info(f"完成: 到达 {reached}/{len(rows)}，记录: {log_path.resolve()}")
    finally:
        # ------------------------------------------------------------------
        # 清理：取消未完成的目标 -> 发零速度停车 -> 销毁节点 -> 关闭 rclpy
        # 嵌套 try/finally 保证前一步出错时后一步仍会执行。
        # ------------------------------------------------------------------
        if nav is not None:
            try:
                try:
                    if task_active and rclpy.ok():
                        nav.cancelTask()
                finally:
                    if motion_started and rclpy.ok():
                        # Gazebo 底盘没有命令超时；取消本脚本目标后显式停车。
                        for _ in range(3):
                            stop_pub.publish(Twist())
                            rclpy.spin_once(nav, timeout_sec=0.05)
            finally:
                nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ============================================================================
# 三、命令行入口
# ============================================================================

def main(argv=None):
    parser = argparse.ArgumentParser(description="Scout Mini POI 巡检 (ROS 2 Humble / Nav2)")
    # --- 输入与观测 ---
    parser.add_argument("--poi", required=True, help="包含 pois 清单的 YAML 文件")
    parser.add_argument("--only", nargs="+", help="只巡检这些编号，如 PG-W01 AP-W03")
    parser.add_argument("--dist", type=float, nargs="+", default=[2.5, 3.0, 2.0],
                        help="观测距离候选 (m)，按顺序尝试；默认兼顾低位水平相机视野")
    # --- 坐标系 ---
    parser.add_argument("--frame", default="map", help="Nav2 目标坐标系，须与全局代价地图一致")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--poi-transform", type=float, nargs=3, metavar=("TX", "TY", "YAW"),
                        help="POI 源坐标转入 --frame: p_goal=R(YAW)*p_poi+[TX,TY]，YAW 为弧度")
    parser.add_argument("--start-x", type=float, help="仅影响排序；运行时默认读取当前 TF")
    parser.add_argument("--start-y", type=float, help="与 --start-x 一同提供，属于 --frame")
    # --- ROS 话题与节点 ---
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--localizer", default="amcl", help="amcl、slam_toolbox 或 none")
    parser.add_argument("--use-sim-time", action=argparse.BooleanOptionalAction, default=True,
                        help="默认使用 /clock；实车传 --no-use-sim-time")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel", help="退出时停车话题")
    # --- 时间参数（均为墙钟时间，秒）---
    parser.add_argument("--timeout", type=float, default=300.0, help="单目标导航墙钟超时 (s)")
    parser.add_argument("--startup-timeout", type=float, default=60.0, help="启动准备墙钟超时 (s)")
    parser.add_argument("--image-timeout", type=float, default=3.0, help="到点后等待新图像的时间 (s)")
    parser.add_argument("--settle-time", type=float, default=0.5, help="到点后拍照前的静置时间 (s)")
    # --- 输出 ---
    parser.add_argument("--out", default="patrol_out", help="输出根目录，每次任务建立独立子目录")
    parser.add_argument("--dry-run", action="store_true", help="只打印观测位姿，不连接 ROS")
    args = parser.parse_args(argv)

    try:
        # ---- 参数检查：尽早报错，不要等到连上 ROS 之后 ----
        positive = args.dist + [args.timeout, args.startup_timeout, args.image_timeout]
        if any(not math.isfinite(value) or value <= 0 for value in positive):
            raise ValueError("距离和各项超时必须为有限正数")
        if not math.isfinite(args.settle_time) or args.settle_time < 0:
            raise ValueError("--settle-time 必须为有限非负数")
        if (args.start_x is None) != (args.start_y is None):
            raise ValueError("--start-x 和 --start-y 必须一同提供")
        coordinates = ([] if args.start_x is None else [args.start_x, args.start_y])
        if not all(math.isfinite(value) for value in coordinates + (args.poi_transform or [])):
            raise ValueError("起点及坐标转换参数必须为有限数")

        pois, source_frame = prepare_pois(args)

        # ---- dry-run：只打印排序后的观测位姿，用来检查配置和坐标转换 ----
        if args.dry_run:
            start = (0.0, 0.0) if args.start_x is None else (args.start_x, args.start_y)
            print(f"坐标系: {source_frame}；观测距离: {args.dist[0]} m；排序起点: {start}")
            print("order  id       type              view_x   view_y   yaw(rad)")
            for index, poi in enumerate(order_nearest(pois, start, args.dist[0]), 1):
                x, y, yaw = view_pose(poi, args.dist[0])
                print(f"{index:5d}  {poi['id']:<8} {poi['type']:<17} {x:7.2f} {y:7.2f} {yaw:8.3f}")
            return 0

        run(args, pois)
    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as error:
        print(f"巡检错误: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("巡检已中断，已保存当前记录并取消导航", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
