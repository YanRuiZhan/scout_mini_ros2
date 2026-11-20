# Scout Mini 仿真

本包使用 ROS 2 Humble 和 Gazebo Fortress 运行 Scout Mini 仿真，不启动实车 CAN 硬件。

## 构建

在工作空间根目录执行：

```zsh
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src/scout_mini_sim src/third_party/fast_lio_ros2 --ignore-src -r -y
colcon build --symlink-install --packages-select scout_mini_sim fast_lio \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.zsh
```

## 启动仿真

```zsh
ros2 launch scout_mini_sim sim.launch.py
```

默认启动带巡检目标的仓库场景，出生点位于南侧入口内约 2.5 m 处 `(0, -22.5)`，沿入口中线朝向仓库深处（`yaw=π/2`，世界坐标 `+Y`），相机开启，不打开 RViz。入口位于南墙 `y=-25`，中心为 `x=0`。出生位姿可通过 `x:=`、`y:=`、`z:=`、`yaw:=` 覆盖。常用选项：

```zsh
ros2 launch scout_mini_sim sim.launch.py rviz:=true
ros2 launch scout_mini_sim sim.launch.py cameras:=false
ros2 launch scout_mini_sim sim.launch.py depth_camera:=true   # 额外开启深度相机
ros2 launch scout_mini_sim sim.launch.py gui:=false
```


### 性能

彩色相机默认 1280×720、5 Hz，深度相机默认关闭，以减小渲染负担。若日志出现
`libEGL warning: egl: failed to create dri2 screen`，说明 Gazebo 没有用上显卡，
雷达和相机都在 CPU 上软件渲染（可用 `glxinfo -B` 查看，`llvmpipe` 即软件渲染）。
此时 CPU 很容易被占满，表现为 FAST-LIO 报 `IMU gap`、EKF 报 `Failed to meet update rate`、
代价地图丢弃激光帧，定位和导航随之变差。巡检时建议加 `gui:=false` 关闭 Gazebo 窗口，
导航 RViz 也会占用不少 CPU。Gazebo 窗口右下角的 RTF 低于 0.8 时，应先降低负载。

Intel + NVIDIA 双显卡笔记本默认用核显。用 `prime-run` 包住 launch 命令即可让 Gazebo
（及其启动的所有子进程）使用独显，`nvidia-smi` 中会出现 `ruby` 进程：

```zsh
prime-run ros2 launch scout_mini_sim sim.launch.py
```

没有 `prime-run` 命令时，在同一终端设置等效的环境变量后再启动：

```zsh
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
```

日志出现 `[RTPS_TRANSPORT_SHM Error] Failed init_port` 时，进程间共享内存传输初始化失败、
退回 UDP，点云等大消息会挤占接收缓冲区，导致 IMU 等小消息一起丢失。通常是进程崩溃后
`/dev/shm` 残留锁文件，关闭所有 ROS 进程后清理：

```zsh
ros2 daemon stop
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

## 启动 FAST-LIO2建图

排查仓库入口的竖直点云薄片时，比较 RViz 中三路数据：`/scout/lidar/points` 是原始雷达点云，`/cloud_registered` 是FAST-LIO当前帧，`/Laser_map` 是累积地图。薄片若只出现在建图输出而未出现在原始点云，检查FAST-LIO启动时的IMU初始化和位姿漂移；该仓库门洞在南墙 `y=-25`、`x≈0` 处。Gazebo初始放置的底盘会下落并产生非静止IMU样本，不能让FAST-LIO把它们用作重力初始化数据。

需要先启动仿真：

```zsh
ros2 launch scout_mini_sim sim.launch.py
```

等小车落地后，在另一个终端启动 FAST-LIO2：

```zsh
ros2 launch scout_mini_sim lio.launch.py
```

建图启动后保持小车静止，等日志出现 `IMU Initial Done` 再行驶。导航时只需要 FAST-LIO 的里程计，用 `mapping:=false` 启动：它关闭累积地图、轨迹和 PCD 保存，这三项会随每帧增长，长时间运行会越来越慢。`settle_time` 控制转发点云前的等待时长，按传感器时间计算；低帧率或暂停仿真不会提前结束等待。需要更长落地时间时可使用 `ros2 launch scout_mini_sim lio.launch.py settle_time:=2.0`。重启或重置 Gazebo 后，也应重启 FAST-LIO，清除上一轮地图。


### 地图倾斜 / Z 漂移

16 线雷达在十米以外的墙面上只留下稀疏的水平线。FAST-LIO 用 5 个最近的地图点拟合平面，这些点几乎共线时，平面方向由测距噪声决定，结果近似水平面而不是墙面。行驶中这些错误的约束把 roll/pitch 持续拉偏（沿行驶方向“抬头”），地面被建成斜坡，零偏和重力估计随之被带偏。`config/fast_lio_scout.yaml` 用两个参数处理：

| 参数 | 作用 |
|---|---|
| `mapping.plane_min_spread` | 近邻点在第二主轴方向的展开小于该值（米）时丢弃此匹配，去掉共线近邻。需小于 `0.4 * filter_size_map`；`0.0` 关闭 |
| `mapping.level_constraint_en`、`mapping.level_constraint_std_deg` | 伪观测，使 roll/pitch 保持启动时的值。只适用于单一平整地面，有坡道或用于实车时关闭 |

日志若出现 `IMU gap of ...` 警告，说明 IMU 样本在到达 FAST-LIO 之前已丢失；同时出现的 `No point, skip this scan!` 表示某帧雷达时间段内没有 IMU 数据，整帧被跳过。先按上文"性能"确认 RTF 与共享内存传输正常，再分段测频率定位丢失位置：

```zsh
ros2 topic hz /imu/data                # bridge 输出，应约 200 Hz
ros2 topic hz /scout/imu/data_settled  # lidar_time.py 转发后，应与上一行一致
```


### 里程计和 TF

里程计由 `robot_localization` 的 EKF（`config/ekf.yaml`，随 `sim.launch.py` 启动）融合三路数据，发布 `/odom` 和 TF `odom → base_footprint`（50 Hz）：

| 来源 | 融合的量 | 作用 |
|---|---|---|
| FAST-LIO2 `/lio/odom`（10 Hz） | x、y、yaw | 位置和航向的主要来源，不受车轮打滑影响 |
| 轮式里程计 `/wheel/odom` | vx（及差速约束 vy=0） | 补齐两帧雷达之间的平移 |
| IMU `/imu/data` | 航向角速度 | 补齐两帧雷达之间的转动 |

`lio_to_odom.py`（随 `lio.launch.py` 启动）把 FAST-LIO 的 `/Odometry`（`camera_init → body`）换算成 odom 下的底盘位姿 `/lio/odom`。收到第一帧时，它用 EKF 当时的位姿对齐两个坐标系，并发布静态 TF `odom → camera_init`，所以先开仿真、晚些再启动 FAST-LIO 也不会让 odom 跳变。FAST-LIO 每帧有几十毫秒处理延迟，`lio_to_odom` 发布前用 EKF 在这段时间内的运动把位姿推算到当前时刻。EKF 不开启 `smooth_lagged_data`：它会因 IMU 与轮速交错到达而频繁回退重放，单轮耗时可达 0.1 s 以上，日志出现 `Failed to meet update rate`。

若 LIO 位姿与 EKF 同一时刻相差超过 0.5 m 或 15°，`lio_to_odom` 丢弃该帧；连续约 2 s 超限时判定 LIO 发散，重新对齐并报错，此时应检查 `IMU gap` 警告并重启 `lio.launch.py`。

没有启动 FAST-LIO 时，EKF 只用轮速 + IMU：航向仍不受打滑影响，位置会慢慢漂移，由 AMCL 修正。FAST-LIO 在仿真中仍有 `IMU gap` 问题，默认导航流程不启动它。

```text
map                          # AMCL（导航时）
└── odom
    ├── base_footprint       # EKF（LIO + 轮速 + IMU）
    │   └── base_link        # 机器人模型和传感器
    └── camera_init          # lio_to_odom（静态，首帧对齐）
        └── body             # FAST-LIO2
```

查看话题和 TF：

```zsh
ros2 topic hz /scout/lidar/points
ros2 topic hz /lio/odom
ros2 run tf2_ros tf2_echo odom base_footprint --ros-args -p use_sim_time:=true
```

重启 FAST-LIO 时整个 `lio.launch.py` 一起重启，`lio_to_odom` 会重新对齐，odom 保持连续。

## POI 自动巡检

`poi_patrol.py` 已安装为本包的可执行脚本。它在现有 Nav2 上依次规划、导航、拍照，
不启动 Gazebo 或导航节点。先重新构建并加载环境：

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select scout_mini_sim \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.zsh
```

运行依赖已写入 `package.xml`：`nav2_simple_commander`、`cv_bridge`、
`python3-yaml` 和 `python3-opencv`，缺少依赖时按本文构建部分的 `rosdep` 命令安装。
默认使用 `/clock`、`/camera/color/image_raw`、`map` 和 `base_footprint`，
与本仓库的 bridge 和 Nav2 配置一致。

配置 [config/poi_warehouse_inspection.yaml](config/poi_warehouse_inspection.yaml)
包含当前 `warehouse.sdf` 的 10 个目标。`target` 是模型贴图表面的 XY 中心，
`normal` 是朝向可观测侧的外法向；脚本会将法向归一化。
观测位置是 `target + dist * normal`，机器人朝向目标。
默认距离按 `2.5、3.0、2.0 m` 尝试，兼顾本车水平安装且离地约 `0.40 m`（与实车一致）的相机；
导航成功与图像保存分别记录，不代表已识别缺陷或已完整拍到目标。

先无需 ROS 检查生成的观测位姿，也可以只看部分编号：

```zsh
/usr/bin/python3 src/scout_mini_sim/scripts/poi_patrol.py \
  --poi src/scout_mini_sim/config/poi_warehouse_inspection.yaml \
  --only PG-W01 AP-W03 --start-x 0 --start-y -22.5 --dry-run
```

**坐标必须对齐**：这份配置标明 `frame_id: world`，属于 Gazebo 世界坐标；
FAST-LIO 导出的地图属于建图坐标，栅格转换也会保留它，不能直接认为它等于 world。
使用该配置导航时必须传 `--poi-transform TX TY YAW`，含义为：

```text
x_map = cos(YAW) * x_world - sin(YAW) * y_world + TX
y_map = sin(YAW) * x_world + cos(YAW) * y_world + TY
```

`YAW` 单位为弧度，`TX/TY` 是世界原点在 map 中的位置。应通过地图中的已知地标
对应关系确定转换，再用相同参数 dry-run 检查；地图 YAML 的 `origin` 只是栅格左下角，
不能直接作为这个转换。

**常用情况：地图由 FAST-LIO 从默认出生点建图**（`sim.launch.py` 默认 `x=0 y=-22.5 yaw=π/2`）。
FAST-LIO 地图原点就是建图起点，x 轴是起点处的车头方向（世界 +Y），因此：

```text
--poi-transform 22.5 0 -1.5708
```

即先绕原点旋转 −90°，再平移 `(22.5, 0)`。出生点或朝向不同时需重新计算。
**不要传 `--poi-transform 0 0 0`**：此时约一半目标会落到仓库外，规划器允许穿过
未知区域，小车会直接开出南侧大门。只有确认 map 与 world 完全重合时才可以这样传。也可以复制并转换这份配置，将坐标改成 map 下实测值并标明
`frame_id: map`，此时无需传转换参数。兼容旧格式：未写 `frame_id` 的 POI 文件
按 `--frame`（默认 map）解释。

正式巡检时先启动对应场景（相机默认开启）和已保存地图的 Nav2（每条 launch 使用独立终端，
单显卡电脑可省略 `prime-run`）：

```zsh
prime-run ros2 launch scout_mini_sim sim.launch.py
prime-run ros2 launch scout_mini_navigation navigation.launch.py map:=/绝对路径/对应地图.yaml
```

可选的 FAST-LIO 里程计在两者之间启动，并保持小车静止直到出现 `IMU Initial Done`。
AMCL 默认以出生点为初始位姿（`navigation.launch.py` 的 `initial_pose`，见导航说明），
在 RViz 确认激光与地图对齐后再开始巡检。脚本只等待定位，不自动重置初始位姿。
导航地图建立流程与 TF 约束见 [导航说明](../scout_mini_navigation/README.md)。
对已经转换到 map 的 POI 文件，可在工作空间根目录运行：

```zsh
ros2 run scout_mini_sim poi_patrol.py --poi /绝对路径/poi_in_map.yaml \
  --only PG-W01 --out patrol_out
```

`--localizer` 默认 `auto`：不检查具体的定位节点（三维 NDT 或 AMCL 均可），等到目标坐标系
到 `base_footprint` 的 TF 可用即开始。也可指定节点名（如 `lidar_localization`、`amcl`、
`slam_toolbox`），脚本会额外等待它进入 active。
运行时默认从当前 TF 决定排序起点；`--start-x/--start-y` 只影响排序，不会初始化定位。
启动准备默认等待 60 秒，可通过 `--startup-timeout` 调整。
`--timeout` 为单目标导航墙钟超时；`--image-timeout` 控制新帧等待时间。
相机若以 `cameras:=false` 关闭，会记录 `image_result=no_image`。

所有点巡检完后（无论各点成败），默认导航回出发时的位置和朝向，CSV 最后一行
`id=HOME` 记录返回结果；`--no-return-home` 可关闭。

**访问顺序**由独立模块 [`scripts/tour_planner.py`](scripts/tour_planner.py) 决定。出发前，脚本先为每个
设备选定观测点，再用 Nav2 规划器（`use_start=True`）求起点与各观测点两两之间的实际路径长度，
交给规划器求顺序；CSV 的 `plan_m` 列记录每段的规划路程。某个点失败时，以当前位置为起点
对剩余的点重新排序，终点仍为出发点。

| 参数 | 说明 |
|---|---|
| `--order auto`（默认） | 点数 ≤ 13 时用 `exact`，否则用 `two_opt` |
| `--order exact` | Held-Karp 动态规划，精确最优（含返回起点） |
| `--order two_opt` | 贪心结果再做 2-opt 改进 |
| `--order greedy` | 贪心最近邻（旧版行为，用于对比） |
| `--distance nav`（默认） / `euclidean` | 排序用 Nav2 路径长度 / 直线距离 |

回到起点的路线正着走、反着走一样长，`exact` / `two_opt` 固定**先去离起点更近的一端**，
同一组点每次得到同一个顺序（`greedy` 保持"每步去最近点"的原意，不做调整）。

**近点兜底**：Humble 的 RPP 离路径终点不到 0.25 m 就停下只调朝向，而到点判定要求离真实目标
不到 0.25 m；两者差几厘米时小车会停在视点旁不动，Nav2 随后报 `Failed to make progress` 并原地
转圈恢复。脚本发现小车在视点 `--near-radius`（默认 0.4 m）内停留超过 `--stuck-time`（默认 8 s）
时，取消 Nav2 任务，用 Nav2 的 Spin 行为把车头转到目标朝向（误差在 `--align-tolerance` 内就不转），
然后照常拍照，结果记为 `reached_near`。`--near-radius 0` 关闭。

`--dry-run` 不连接 ROS，按直线距离给出顺序和各段长度。按地图上的实际路径长度离线比较各方法：

```zsh
/usr/bin/python3 src/scout_mini_sim/scripts/tour_planner.py \
  --poi maps/warehouse_poi.yaml --map maps/warehouse.yaml --start 0 0 --method all
```

每次巡检在 `patrol_out/日期_时间_随机后缀/` 下保存 PNG 和 `patrol_log.csv`，
每完成一个点就写入 CSV。`result` 区分 `reached`、`reached_near`、`no_plan`、`rejected`、
`timeout`、`canceled`、`failed` 等状态；`pos_err_m` / `yaw_err_deg` 是到点后的位置和朝向误差；
`image_result` 单独记录 `saved`、
`no_image` 或 `error`。到点后默认等待 0.5 秒，随后只接收采集时间更新的图像。
Ctrl+C 时保留当前记录、请求取消导航并向 `/cmd_vel` 发布零速度。
本轮适配的自动测试没有验证 Gazebo 中整条路线的可达性，首次请用 `--only` 单点验证。

