#!/usr/bin/env python3
"""Scout Mini POI 巡检：生成观测位姿、Nav2 导航、到点拍照并记录 CSV。

先启动 sim.launch.py（cameras:=true）和 navigation.launch.py，并在 RViz 完成定位。
无需 ROS 的检查：python3 poi_patrol.py --poi <配置.yaml> --dry-run
运行与坐标转换说明见 scout_mini_sim/README.md。
"""
import argparse
import csv
import math
import os
from pathlib import Path
import re
import sys
import time

import yaml


LOG_FIELDS = [
    "order", "id", "type", "result", "view_dist", "goal_x", "goal_y", "goal_yaw",
    "robot_x", "robot_y", "pos_err_m", "time_s", "image", "image_result",
]


def load_pois(path):
    """兼容原来的 pois 列表格式，并在启动 ROS 前检查数据。"""
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get("pois"), list):
        raise ValueError("POI 文件必须包含 pois 列表")
    ids = set()
    for index, poi in enumerate(data["pois"], 1):
        if not isinstance(poi, dict):
            raise ValueError(f"第 {index} 个 POI 必须是映射")
        if not isinstance(poi.get("id"), str) or not poi["id"].strip():
            raise ValueError(f"第 {index} 个 POI 缺少非空字符串 id")
        if poi["id"] in ids:
            raise ValueError(f"POI id 重复: {poi['id']}")
        ids.add(poi["id"])
        if not isinstance(poi.get("type"), str) or not poi["type"].strip():
            raise ValueError(f"POI {poi['id']} 缺少非空字符串 type")
        for group in ("target", "normal"):
            try:
                xy = [float(poi[group][axis]) for axis in ("x", "y")]
            except (KeyError, TypeError, ValueError):
                raise ValueError(f"POI {poi['id']} 的 {group} 必须包含数字 x/y") from None
            if not all(math.isfinite(value) for value in xy):
                raise ValueError(f"POI {poi['id']} 的 {group}.x/y 必须是有限数")
            poi[group] = dict(zip(("x", "y"), xy))
        norm = math.hypot(poi["normal"]["x"], poi["normal"]["y"])
        if not math.isfinite(norm) or norm == 0:
            raise ValueError(f"POI {poi['id']} 的 normal 不能为零或溢出向量")
    return data["pois"]


def view_pose(poi, dist):
    """沿目标表面的外法向退后 dist 米，底盘 +X 朝向目标。"""
    target, normal = poi["target"], poi["normal"]
    norm = math.hypot(normal["x"], normal["y"])
    nx, ny = normal["x"] / norm, normal["y"] / norm
    return target["x"] + dist * nx, target["y"] + dist * ny, math.atan2(-ny, -nx)


def order_nearest(pois, start_xy, dist):
    """按观测位姿做最近邻排序；不修改输入清单。"""
    left, ordered, current = list(pois), [], start_xy
    while left:
        nxt = min(left, key=lambda poi: math.dist(view_pose(poi, dist)[:2], current))
        left.remove(nxt)
        ordered.append(nxt)
        current = view_pose(nxt, dist)[:2]
    return ordered


def prepare_pois(args):
    pois = load_pois(args.poi)
    with open(args.poi, encoding="utf-8") as stream:
        source_frame = yaml.safe_load(stream).get("frame_id", args.frame)
    if not isinstance(source_frame, str) or not source_frame.strip():
        raise ValueError("frame_id 必须是非空字符串")
    if args.only is not None:
        missing = set(args.only) - {poi["id"] for poi in pois}
        if missing:
            raise ValueError("POI 文件中不存在这些 id: " + ", ".join(sorted(missing)))
        pois = [poi for poi in pois if poi["id"] in args.only]
    if not pois:
        raise ValueError("没有可巡检的 POI")
    if args.poi_transform is not None:
        tx, ty, yaw = args.poi_transform
        c, s = math.cos(yaw), math.sin(yaw)
        for poi in pois:
            for group in ("target", "normal"):
                x, y = poi[group]["x"], poi[group]["y"]
                poi[group] = {"x": c * x - s * y, "y": s * x + c * y}
            poi["target"]["x"] += tx
            poi["target"]["y"] += ty
        source_frame = args.frame
    if source_frame != args.frame and not args.dry_run:
        raise ValueError(
            f"POI 属于 {source_frame}，目标坐标系为 {args.frame}；请提供 "
            "--poi-transform TX TY YAW（目标系中的平移和弧度旋转），或使用已转换的 POI 文件"
        )
    return pois, source_frame


def write_log(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=LOG_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def wait_for_navigation(nav, buffer, args):
    """有界等待生命周期与 TF；不触发 Commander 的默认 initialpose 发布。"""
    import rclpy
    from lifecycle_msgs.srv import GetState
    from rclpy.time import Time
    from tf2_ros import TransformException

    deadline = time.monotonic() + args.startup_timeout
    nodes = ["bt_navigator", "planner_server", "controller_server"]
    if args.localizer != "none":
        nodes.insert(0, args.localizer)
    for name in nodes:
        client = nav.create_client(GetState, name.rstrip("/") + "/get_state")
        try:
            while rclpy.ok():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"等待 {name} active 超时，请先启动仿真和 Nav2")
                if not client.wait_for_service(timeout_sec=0.2):
                    rclpy.spin_once(nav, timeout_sec=0.1)
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(nav, future, timeout_sec=0.5)
                if future.done() and future.result() is not None:
                    if future.result().current_state.id == 3:
                        break
                else:
                    client.remove_pending_request(future)
                rclpy.spin_once(nav, timeout_sec=0.1)
        finally:
            nav.destroy_client(client)
    while rclpy.ok():
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"等待 /clock 或 {args.frame} -> {args.base_frame} TF 超时；"
                "AMCL 模式请先在 RViz 设置 2D Pose Estimate"
            )
        rclpy.spin_once(nav, timeout_sec=0.1)
        if args.use_sim_time and nav.get_clock().now().nanoseconds == 0:
            continue
        try:
            return buffer.lookup_transform(args.frame, args.base_frame, Time())
        except TransformException:
            continue
    raise RuntimeError("ROS 已关闭")


def run(args, pois):
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import Image
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init(args=["--ros-args", "-p", f"use_sim_time:={str(args.use_sim_time).lower()}"])
    nav = None
    rows = []
    log_path = None
    motion_started = False
    task_active = False
    try:
        nav = BasicNavigator(node_name="poi_patrol")
        buffer = Buffer(node=nav)
        listener = TransformListener(buffer, nav)
        bridge = CvBridge()
        latest = {"img": None, "after_ns": None}

        def on_image(msg):
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if latest["after_ns"] is not None and stamp_ns > latest["after_ns"]:
                latest["img"] = msg

        nav.create_subscription(Image, args.image_topic, on_image, qos_profile_sensor_data)
        stop_pub = nav.create_publisher(Twist, args.cmd_vel_topic, 10)
        nav.get_logger().info("等待 Nav2 和定位 TF；AMCL 模式请在 RViz 设置初始位姿")
        current = wait_for_navigation(nav, buffer, args)
        if args.start_x is None:
            start_xy = (current.transform.translation.x, current.transform.translation.y)
        else:
            start_xy = (args.start_x, args.start_y)
        pois = order_nearest(pois, start_xy, args.dist[0])
        # 每次任务一个目录，避免重跑覆盖照片和 CSV。
        os.makedirs(args.out, exist_ok=True)
        from tempfile import mkdtemp
        output = Path(mkdtemp(prefix=time.strftime("%Y%m%d_%H%M%S_"), dir=args.out))
        log_path = output / "patrol_log.csv"
        write_log(log_path, rows)

        def make_pose(x, y, yaw):
            pose = PoseStamped()
            pose.header.frame_id = args.frame
            pose.header.stamp = nav.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            return pose

        for index, poi in enumerate(pois, 1):
            row = dict.fromkeys(LOG_FIELDS, "")
            row.update(order=index, id=poi["id"], type=poi["type"], result="error")
            rows.append(row)
            try:
                goal = None
                for dist in args.dist:
                    x, y, yaw = view_pose(poi, dist)
                    candidate = make_pose(x, y, yaw)
                    # Humble 会忽略 start，使用机器人在全局代价地图中的当前位姿。
                    path = nav.getPath(PoseStamped(), candidate, use_start=False)
                    if path is not None and path.poses:
                        goal = candidate
                        row.update(view_dist=dist, goal_x=x, goal_y=y, goal_yaw=yaw)
                        break
                if goal is None:
                    row["result"] = "no_plan"
                    nav.get_logger().warning(f"{poi['id']}: 所有候选距离均无路径，跳过")
                    continue
                nav.get_logger().info(f"[{index}/{len(pois)}] 前往 {poi['id']}")
                started = time.monotonic()
                motion_started = True
                # Commander 的规划和导航共用 result_future，必须先检查目标是否被接受。
                nav.feedback = None
                task_active = True
                if not nav.goToPose(goal):
                    task_active = False
                    row["result"] = "rejected"
                    continue
                timed_out = False
                while not nav.isTaskComplete():
                    if not timed_out and time.monotonic() - started > args.timeout:
                        timed_out = True
                        nav.cancelTask()
                task_active = False
                result = nav.getResult()
                row["time_s"] = round(time.monotonic() - started, 2)
                row["result"] = "timeout" if timed_out else {
                    TaskResult.SUCCEEDED: "reached", TaskResult.CANCELED: "canceled",
                    TaskResult.FAILED: "failed",
                }.get(result, "unknown")
                try:
                    actual = buffer.lookup_transform(args.frame, args.base_frame, Time())
                    rx, ry = actual.transform.translation.x, actual.transform.translation.y
                    row.update(robot_x=rx, robot_y=ry, pos_err_m=math.hypot(rx - x, ry - y))
                except TransformException:
                    nav.get_logger().warning(f"{poi['id']}: 无最终定位 TF，未记录位置误差")
                if row["result"] != "reached":
                    continue
                # 等待稳定后，仅接受在该时刻之后采集的图像，排除 DDS 队列中的旧帧。
                until = time.monotonic() + args.settle_time
                while time.monotonic() < until:
                    rclpy.spin_once(nav, timeout_sec=0.05)
                latest.update(img=None, after_ns=nav.get_clock().now().nanoseconds)
                until = time.monotonic() + args.image_timeout
                while latest["img"] is None and time.monotonic() < until:
                    rclpy.spin_once(nav, timeout_sec=0.1)
                if latest["img"] is None:
                    row["image_result"] = "no_image"
                    nav.get_logger().warning(
                        f"{poi['id']}: 未收到新图像，请检查 {args.image_topic} 和 cameras:=true"
                    )
                else:
                    safe_id = re.sub(r"[^\w.-]", "_", poi["id"])
                    filename = output / f"{index:02d}_{safe_id}.png"
                    try:
                        image = bridge.imgmsg_to_cv2(latest["img"], desired_encoding="bgr8")
                        if not cv2.imwrite(str(filename), image):
                            raise OSError(f"图片写入失败: {filename}")
                        row.update(image=filename.name, image_result="saved")
                    except Exception as error:
                        row["image_result"] = "error"
                        nav.get_logger().error(f"{poi['id']}: {error}")
                latest.update(img=None, after_ns=None)
            except KeyboardInterrupt:
                row["result"] = "interrupted"
                raise
            finally:
                write_log(log_path, rows)
        reached = sum(row["result"] == "reached" for row in rows)
        nav.get_logger().info(f"完成: 到达 {reached}/{len(rows)}，记录: {log_path.resolve()}")
    finally:
        if nav is not None:
            try:
                if task_active and rclpy.ok():
                    nav.cancelTask()
                if motion_started and rclpy.ok():
                    # Gazebo 底盘没有命令超时；停止前需先取消本脚本的导航目标。
                    for _ in range(3):
                        stop_pub.publish(Twist())
                        rclpy.spin_once(nav, timeout_sec=0.05)
            finally:
                nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Scout Mini POI 巡检 (ROS 2 Humble / Nav2)")
    parser.add_argument("--poi", required=True, help="包含 pois 清单的 YAML 文件")
    parser.add_argument("--dist", type=float, nargs="+", default=[2.5, 3.0, 2.0],
                        help="观测距离候选 (m)，按顺序尝试；默认兼顾低位水平相机视野")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--frame", default="map", help="Nav2 目标坐标系，须与全局代价地图一致")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--poi-transform", type=float, nargs=3, metavar=("TX", "TY", "YAW"),
                        help="POI 源坐标转入 --frame: p_goal=R(YAW)*p_poi+[TX,TY]，YAW 为弧度")
    parser.add_argument("--localizer", default="amcl", help="amcl、slam_toolbox 或 none")
    parser.add_argument("--use-sim-time", action=argparse.BooleanOptionalAction, default=True,
                        help="默认使用 /clock；实车传 --no-use-sim-time")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel", help="退出时停车话题")
    parser.add_argument("--start-x", type=float, help="仅影响排序；运行时默认读取当前 TF")
    parser.add_argument("--start-y", type=float, help="与 --start-x 一同提供，属于 --frame")
    parser.add_argument("--timeout", type=float, default=300.0, help="单目标导航墙钟超时 (s)")
    parser.add_argument("--startup-timeout", type=float, default=60.0, help="启动准备墙钟超时 (s)")
    parser.add_argument("--image-timeout", type=float, default=3.0)
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--only", nargs="+", help="只巡检这些编号，如 PG-W01 AP-W03")
    parser.add_argument("--out", default="patrol_out", help="输出根目录，每次任务建立独立子目录")
    parser.add_argument("--dry-run", action="store_true", help="只打印观测位姿，不连接 ROS")
    args = parser.parse_args(argv)
    try:
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
