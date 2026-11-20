# 常用参数速查

本文汇总调试时最常改动的参数：当前值、作用、调大或调小的效果。
表中的值与仓库当前配置一致；修改配置后请同步更新本文。

路径简称：

| 简称 | 文件 |
|---|---|
| `nav2` | `src/scout_mini_navigation/config/nav2_params.yaml` |
| `ndt` | `src/scout_mini_navigation/config/lidar_localization.yaml` |
| `ekf` | `src/scout_mini_sim/config/ekf.yaml` |
| `urdf` | `src/scout_mini_sim/urdf/scout_mini_rdk.urdf.xacro` |

改动 `nav2` / `ndt` 后重启导航即可；改动 `urdf` 后需要重新构建 `scout_mini_sim` 并**重启仿真**。

---

## 按需求查找

| 我想…… | 去改 |
|---|---|
| 跑得更快或更慢 | [1. 速度上限](#1-速度上限必须多处同步)（多处必须一起改） |
| 弯道更稳、不撞货架 | [2. 局部控制器 RPP](#2-局部控制器-rpp) 的弯道减速，[4. 代价地图](#4-代价地图) 的全局膨胀 |
| 路径离货架远一点 | [4. 代价地图](#4-代价地图) 的全局膨胀，[3. 全局规划](#3-全局规划-smac-planner-2da) 的 `cost_travel_multiplier` |
| 到点停得更准 | [2. 局部控制器 RPP](#2-局部控制器-rpp) 的到点判定 |
| 定位不稳、配准经常被拒绝 | [5. 三维定位 NDT](#5-三维定位-ndt) |
| 不用每次点 2D Pose Estimate | [6. 启动参数](#6-启动参数) 的 `initial_pose` |
| 仿真卡、RTF 低 | [9. 仿真](#9-仿真) |
| 拍照距离、超时 | [10. 巡检脚本](#10-巡检脚本-poi_patrolpy) |

---

## 1. 速度上限（必须多处同步）

| 参数 | 文件 | 当前值 | 说明 |
|---|---|---|---|
| `FollowPath.desired_linear_vel` | `nav2` | `1.0` m/s | RPP 直道速度 |
| `FollowPath.rotate_to_heading_angular_vel` | `nav2` | `0.8` rad/s | 原地转向角速度 |
| `FollowPath.regulated_linear_scaling_min_radius` | `nav2` | `1.25` m | 应等于 线速度 ÷ 角速度上限，弯道减速才能保证角速度不超限 |
| `velocity_smoother.max_velocity` | `nav2` | `[1.0, 0.0, 0.8]` | 最终发往 `/cmd_vel` 的上限 `[vx, vy, wz]` |
| `velocity_smoother.min_velocity` | `nav2` | `[-0.15, 0.0, -0.8]` | 倒车与反向转动上限 |
| `velocity_smoother.max_accel` / `max_decel` | `nav2` | `[1.0, 0.0, 1.5]` / `[-1.0, 0.0, -1.5]` | 加减速；1.0 m/s² 时从 1 m/s 刹停约 0.5 m |
| `behavior_server.max_rotational_vel` | `nav2` | `0.8` rad/s | 恢复行为（原地旋转）速度 |
| `max_linear_velocity` | `urdf` | `2.78` m/s | Gazebo 底盘上限，等于实车最高速度 10 km/h |
| `max_angular_velocity` | `urdf` | `0.8` rad/s | Gazebo 底盘上限，Nav2 的角速度不能超过它 |

**同步规则**（`src/scout_mini_sim/test/test_odometry_config.py` 会自动检查）：

- `desired_linear_vel` = `max_velocity[0]` ≤ Gazebo `max_linear_velocity`
- `rotate_to_heading_angular_vel` = `max_velocity[2]` = `max_rotational_vel` ≤ Gazebo `max_angular_velocity`
- `desired_linear_vel` ÷ `regulated_linear_scaling_min_radius` ≤ 角速度上限

经验：全速时的最小转弯半径 = 线速度 ÷ 角速度。货架通道约 2～3 m 宽，半径应明显小于通道宽度。

---

## 2. 局部控制器 RPP

`nav2` → `controller_server.FollowPath`（Regulated Pure Pursuit）

| 参数 | 当前值 | 作用 | 调整 |
|---|---|---|---|
| `min_lookahead_dist` / `max_lookahead_dist` | `0.4` / `1.2` m | 追踪点距离范围 | 大：更平滑但切弯更多；小：更贴路径但可能抖 |
| `lookahead_time` | `1.2` s | 追踪点距离 = 速度 × 该值 | 同上 |
| `regulated_linear_scaling_min_radius` | `1.25` m | 弯道半径小于它时按比例减速 | 见第 1 节同步规则 |
| `regulated_linear_scaling_min_speed` | `0.25` m/s | 弯道减速的下限 | 太小会在急弯里很慢 |
| `use_cost_regulated_linear_velocity_scaling` | `true` | 靠近障碍物时减速 | — |
| `cost_scaling_dist` / `cost_scaling_gain` | `0.6` m / `1.0` | 离障碍物多近开始减速、减多少 | 窄通道太慢时调小 |
| `inflation_cost_scaling_factor` | `3.0` | **必须等于局部代价地图膨胀层的 `cost_scaling_factor`** | 改一处要改另一处 |
| `rotate_to_heading_min_angle` | `0.785` rad（45°） | 车头偏离路径超过此角先原地转向 | — |
| `approach_velocity_scaling_dist` | `1.0` m | 离目标多远开始减速 | — |
| `min_approach_linear_velocity` | `0.05` m/s | 接近目标时的最低速度 | — |
| `max_allowed_time_to_collision_up_to_carrot` | `1.0` s | 预计多久内会撞上就停车 | 大：更保守 |

到点判定：`controller_server.general_goal_checker`

| 参数 | 当前值 | 说明 |
|---|---|---|
| `xy_goal_tolerance` | `0.25` m | 位置误差小于它算到达 |
| `yaw_goal_tolerance` | `0.25` rad（约 14°） | 朝向误差小于它算到达；巡检拍照对朝向敏感时可调小 |

卡住判定：`controller_server.progress_checker`：`movement_time_allowance: 15.0` s 内平移不足
`required_movement_radius: 0.2` m 即判定卡住，交给行为树恢复（原地转向不算移动）。

控制频率：`controller_server.controller_frequency: 10.0` Hz。仿真负载高时 20 Hz 实际只能跑到
5～10 Hz（日志 `Control loop missed its desired rate`），按实际能达到的频率设置。

注意：Humble 的 RPP 直接用 `xy_goal_tolerance` 作为"开始原地调朝向"的距离，两者无法分开设置。
路径终点与真实目标差几厘米时，小车可能停在视点旁不动；`poi_patrol.py` 的近点兜底处理这种情况
（见第 10 节 `--near-radius`）。

---

## 3. 全局规划 Smac Planner 2D（A*）

`nav2` → `planner_server.GridBased`

| 参数 | 当前值 | 作用 |
|---|---|---|
| `cost_travel_multiplier` | `2.0` | 越大越倾向走低代价格子，即离障碍物远的通道中间 |
| `tolerance` | `0.5` m | 目标不可达时，允许在其周围多远内找替代终点 |
| `allow_unknown` | `true` | 允许穿过未知区域；PCD 转出的栅格内部有未知孔洞，需要开启 |
| `max_planning_time` | `2.0` s | 单次规划最长时间 |
| `smoother.w_smooth` / `w_data` | `0.3` / `0.2` | 路径平滑程度 / 贴近原路径的程度 |

---

## 4. 代价地图

| 参数 | 全局 | 局部 | 作用 |
|---|---|---|---|
| `inflation_layer.inflation_radius` | `1.0` m | `0.65` m | 膨胀半径；全局较大，让路径沿通道中间走 |
| `inflation_layer.cost_scaling_factor` | `2.0` | `3.0` | 代价随距离衰减的速度；越小，离障碍物较远处的代价越高 |
| `width` / `height` | 整张地图 | `8 × 8` m | 局部地图需覆盖 RPP 的追踪点和碰撞检查范围 |
| `footprint` | `0.612 × 0.580` m | 同左 | 车身轮廓，按实车外形 |
| `footprint_padding` | `0.02` m | 同左 | 车身外扩 |
| `obstacle_max_range` / `raytrace_max_range` | `6.0` / `8.0` m | 同左 | 激光标记障碍 / 清除障碍的距离 |

车身外接圆半径约 0.44 m（含 padding）。膨胀半径只比它大一点时，路径会紧贴障碍物。

---

## 5. 三维定位 NDT

`ndt`（`lidar_localization` 节点，`localizer:=ndt` 时使用）

| 参数 | 当前值 | 作用 | 调整 |
|---|---|---|---|
| `score_threshold` | `6.0` | 配准得分高于它视为失败，沿用上一次的 `map → odom` | 经常被拒绝时调大；接受了明显错误的配准时调小 |
| `ndt_resolution` | `1.0` m | NDT 体素边长 | 0.5：更精细但对初值更敏感；2.0：更稳但精度下降 |
| `ndt_max_iterations` | `35` | 单帧最大迭代次数 | — |
| `ndt_num_threads` | `4` | 配准线程数 | CPU 紧张时调小 |
| `voxel_leaf_size` | `0.2` m | 当前帧降采样 | 大：更快；小：更准 |
| `scan_min_range` / `scan_max_range` | `0.5` / `40.0` m | 参与配准的点的距离范围 | — |
| `initial_pose_z` | `-0.18` m | 地面上的 `base_footprint` 在 FAST-LIO 地图中的高度 | 地图换成不同安装高度建的图时要改 |
| `level_map_to_odom` | `true` | `map → odom` 只保留 x、y、yaw | 二维导航保持开启 |
| `use_odom_tf_prediction` | `true` | 配准初值取自 EKF | 保持开启 |
| `use_imu_preintegration` | `false` | 上游默认是 `true`；IMU 已在 EKF 中使用，这里不再重复 | 保持关闭 |

配准初值、地图路径和初始位姿由启动参数设置，见第 6 节。

---

## 6. 启动参数

`ros2 launch scout_mini_navigation navigation.launch.py ...`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `map` | （必填） | 二维栅格地图 YAML 的绝对路径 |
| `localizer` | `ndt` | `ndt`：三维 NDT；`amcl`：二维 AMCL |
| `pcd_map` | 与 `map` 同名的 `.pcd` | 三维定位用的 PCD；必须和二维地图来自同一次建图 |
| `initial_pose` | `"0 0 0"` | 初始位姿 `"x y yaw"`（map 坐标，米、弧度）；`none` 表示在 RViz 手动设置 |
| `slam` | `false` | `true`：用 SLAM Toolbox 新建二维地图 |
| `rviz` | `true` | 是否打开导航 RViz |

`ros2 launch scout_mini_sim sim.launch.py ...`：见第 9 节。
`ros2 launch scout_mini_sim lio.launch.py ...`：见第 8 节。

---

## 7. AMCL（`localizer:=amcl` 时）

`nav2` → `amcl`

| 参数 | 当前值 | 作用 |
|---|---|---|
| `alpha1`～`alpha4` | `0.1` | 里程计运动噪声；里程计越不准越要调大，否则粒子跟不上 |
| `update_min_d` / `update_min_a` | `0.1` m / `0.1` rad | 移动多少更新一次；越小修正越频繁，单次跳变越小 |
| `max_beams` | `120` | 每帧参与计算的激光束数 |
| `min_particles` / `max_particles` | `500` / `2000` | 粒子数范围 |

---

## 8. 局部里程计 EKF 与可选 FAST-LIO

`ekf`（`robot_localization`，随 `sim.launch.py` 启动）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `frequency` | `50.0` Hz | `/odom` 和 `odom → base_footprint` 的发布频率 |
| `odom0`（`/wheel/odom`） | vx、vy | 轮速提供前进速度，vy=0 作为差速约束 |
| `imu0`（`/imu/data`） | vyaw | 陀螺仪提供航向角速度，不受打滑影响 |
| `odom1`（`/lio/odom`） | x、y、yaw | 仅在启动 `lio.launch.py` 时存在 |
| `smooth_lagged_data` | `false` | 开启会因回退重放导致 EKF 跟不上频率，保持关闭 |

`lio_to_odom.py` 节点参数（`lio.launch.py` 启动）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `position_std` / `yaw_std_deg` | `0.02` m / `0.5`° | 交给 EKF 的 LIO 测量标准差；越小 EKF 越信任 LIO |
| `max_jump_m` / `max_jump_yaw_deg` | `0.5` m / `15`° | LIO 与 EKF 相差超过它时丢弃该帧 |
| `realign_after` | `20` 帧（约 2 s） | 连续超限多少帧后判定 LIO 发散并重新对齐 |

`lio.launch.py` 启动参数：`mapping`（默认 `true` 建图；导航时用 `false`，关闭随时间增长的
累积地图、轨迹和 PCD 缓存），`settle_time`（默认 `1.0` s，启动前等待小车落地）。

---

## 9. 仿真

`sim.launch.py` 启动参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `cameras` | `true` | 彩色相机；巡检拍照需要 |
| `depth_camera` | `false` | 深度相机；渲染开销大，按需开启 |
| `gui` | `true` | Gazebo 窗口；`false` 可节省渲染 |
| `x` / `y` / `yaw` | `0` / `-22.5` / `π/2` | 出生位姿（世界坐标） |

`urdf` 中的传感器

| 参数 | 当前值 | 说明 |
|---|---|---|
| 雷达 `lidar_h_samples` | `900`（0.4°） | 水平采样数；`450` 更省算力 |
| 雷达 `update_rate` | `10` Hz | — |
| IMU `update_rate` | `200` Hz | 必须能整除物理步长（`warehouse.sdf` 中为 2.5 ms） |
| 彩色相机 | `1280×720`、`5` Hz | 巡检拍照够用；分辨率或帧率越高越占 CPU |
| 相机高度 `camera_joint` 的 z | `0.219` m（离地约 `0.40` m） | 按实车实测；离地高度 = 0.181（`base_height`）+ z |
| 雷达高度 `lidar_3d_joint` 的 z | `0.30` m（离地约 `0.48` m） | 改动后需同步 FAST-LIO 外参 `extrinsic_T` 和 `pcd_to_grid.py --sensor-height` |
| 深度相机 | `1280×720`、`30` Hz | 仅 `depth_camera:=true` 时存在 |
| `wheel_separation` | `0.90` m | 滑移转向的等效轮距（几何轮距 0.49 m），影响轮式里程计的航向 |

双显卡笔记本用 `prime-run ros2 launch ...` 让 Gazebo 和 RViz 使用独显。

---

## 10. 巡检脚本 `poi_patrol.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--poi` | — | POI 配置文件（在线生成视点）；与 `--plan` 同时给出时只用于过期检查 |
| `--plan` | — | `plan_offline.py` 的离线规划；给出后不再使用 `--dist`、`--poi-transform` |
| `--map` | — | 与 `--plan` 一起用：地图 YAML，检查离线规划是否过期 |
| `--poi-transform TX TY YAW` | 无 | POI 坐标转到地图坐标；默认出生点建图时为 `22.5 0 -1.5708` |
| `--dist` | `2.5 3.0 2.0` m | 观测距离候选，依次尝试直到规划成功 |
| `--only` | 全部 | 只巡检指定编号 |
| `--timeout` | `300` s | 单个目标的导航超时 |
| `--settle-time` | `0.5` s | 到点后拍照前的静置时间 |
| `--image-timeout` | `3.0` s | 等待新图像的时间 |
| `--near-radius` | `0.4` m | 近点兜底：离视点多近时启用；`0` 关闭 |
| `--stuck-time` | `8.0` s | 近点兜底：在视点附近基本不动多久判定为卡住（应小于进度检查的 15 s） |
| `--align-tolerance` | `0.1` rad | 近点兜底：朝向误差小于它就不再用 Spin 转 |
| `--order` | `auto` | 访问顺序：`auto`（≤ 13 点用 `exact`，否则 `two_opt`）、`exact`（精确最优）、`two_opt`、`greedy` |
| `--distance` | `nav` | 排序用的距离：`nav`（Nav2 规划路径长度）或 `euclidean`（直线） |
| `--return-home` | 开启 | 巡检结束后回到出发时的位姿；`--no-return-home` 关闭 |
| `--localizer` | `auto` | `auto` 只等定位 TF；也可指定节点名，如 `lidar_localization`、`amcl` |
| `--out` | `patrol_out` | 输出目录，每次巡检新建子目录 |
| `--dry-run` | — | 只打印观测位姿，不连接 ROS |

---

## 11. 离线视点规划 `plan_offline.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--poi` / `--map` | （必填） | POI 文件和二维栅格地图 |
| `--pcd` | 无 | 建图 PCD，三维遮挡检查；不给则用二维栅格保守检查 |
| `--world` | 无 | 仿真 world，从模型网格读设备高度和尺寸 |
| `--start X Y` | `0 0` | 出发位置，用于可达性检查 |
| `--r-min` / `--r-max` / `--r-step` | `1.5` / `4.0` / `0.25` m | 离检测面的距离范围和步长 |
| `--phi-max` / `--phi-step` | `60` / `10` ° | 偏离法向的角度范围和步长 |
| `--robot-radius` / `--clearance-margin` | `0.45` / `0.15` m | 停车位置离障碍物至少 半径 + 余量 |
| `--min-visibility` | `0.75` | 检测面最低可见比例 |
| `--target-fill` | `0.35` | 理想的画面占比，成像大小评分以它为中心 |
| `--keep` / `--min-separation` | `3` / `30` ° | 每台设备保留几个视点、偏角至少相差多少 |
| `--grid-step` | `2` | 路程计算的栅格降采样倍数，越大越快越粗 |

评分权重 `WEIGHTS`、相机参数 `CAMERA`、按类型的设备尺寸缺省值 `TYPE_DEFAULTS` 在脚本开头，直接修改。

## 12. 一键启动 `sim_patrol.launch.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `map` / `poi` / `plan` | `maps/warehouse.yaml` / `maps/warehouse_poi.yaml` / 空 | 给了 `plan` 时用离线规划 |
| `patrol` | `true` | `false` 只起仿真和 Nav2 |
| `patrol_args` | 空 | 附加给 `poi_patrol.py` 的参数 |
| `nav_delay` / `patrol_delay` | `8` / `5` s | 启动间隔，电脑慢就调大 |
| `startup_timeout` | `180` s | 巡检脚本等待 Nav2 和定位就绪的最长时间 |
| `gui` / `rviz` | `true` / `true` | Gazebo 界面、RViz |
| `localizer` / `initial_pose` / `pcd_map` | `ndt` / `"0 0 0"` / 空 | 同 `navigation.launch.py` |
| `shutdown_on_exit` | `false` | 巡检结束后一并退出 |
