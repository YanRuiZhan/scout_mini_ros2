# Scout Mini 导航仿真

ROS 2 Humble / Gazebo Fortress。复用 `scout_mini_sim` 的 `/scan`、`/odom`、
`/cmd_vel` 和 `odom → base_footprint`，所有导航节点使用仿真时间。
不需要将 Nav2 源码加入 `third_party`。

## 安装与构建

退出 conda 后，在工作空间根目录执行：

```zsh
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src/scout_mini_sim src/scout_mini_navigation \
  src/third_party/lidar_localization_ros2 src/third_party/ndt_omp_ros2 --ignore-src -r -y
# 限制并行数，避免三维定位器编译时内存耗尽卡死，详见根目录 README
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1 \
  --packages-up-to scout_mini_navigation --cmake-args -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.zsh
```

## FAST-LIO2 PCD 转二维地图（主流程）

先用 FAST-LIO2 离线建图并保存 PCD。以下命令在工作空间根目录执行，
无需运行 ROS 节点，也不需要安装 pip 包：

```zsh
/usr/bin/python3 src/scout_mini_navigation/scripts/pcd_to_grid.py \
  maps/room_03.pcd maps/room_03_grid --ground-z=-0.12 --sensor-height=0.5
```

`room_03.pcd` 的低处点云初步拟合地面约为 `z=-0.12 m`，这是 PCD 坐标系下的高度，
不是 Gazebo 地面高度；此参数仍需结合点云检查。默认分辨率 `0.05 m`，
地面容差 `±0.08 m`，雷达高度默认为距地面 `0.5 m`，障碍高度范围为
距地 `0.15～0.58 m`（含雷达上方 `0.08 m` 余量），每格至少 2 点。
若雷达安装高度不同，请显式设置 `--sensor-height`。
工具支持未压缩 binary / ASCII PCD，不支持 binary_compressed。
构建后也可通过 `ros2 run scout_mini_navigation pcd_to_grid.py` 调用。

输出 `maps/room_03_grid.pgm` 和 `maps/room_03_grid.yaml`，不覆盖已有输出。
保留 PCD 的 XY，不要求与 odom 原点重合；YAML origin 是栅格左下角。
黑色为障碍，白色为地面观测支持的候选空闲，灰色为未知，障碍优先。
地面证据只是平坦室内场景的近似，不是基于传感器轨迹的射线清除，
不能据此保证安全，也不能把所有无点区域填成空闲。

此高度范围仅依据雷达安装高度，若整车高于雷达，可能漏掉上方障碍；低于 0.15 m 的
障碍也可能被忽略，膨胀层无法补回未检测的障碍。该工具使用固定地面高度，未估计
局部地形。地面稀疏造成内部未知孔洞，当前是检查用初稿，未验证导航可用性；
障碍物上表面/内部可能仍为未知。不要直接全局填洞或打开未知区通行来掩盖问题。
后续可结合建图轨迹/原始扫描做射线重建，或在核实场景后采用受限的地面插补。

必须启动与 PCD 对应的 Gazebo 场景。三维定位直接使用这份 PCD：二维栅格与 PCD 同名
（`room_03_grid.yaml` 对应 `room_03_grid.pcd`）时自动找到，否则用 `pcd_map:=` 指定：

```zsh
prime-run ros2 launch scout_mini_navigation navigation.launch.py \
  map:=/绝对路径/maps/room_03_grid.yaml pcd_map:=/绝对路径/maps/room_03.pcd
```

PCD 转出的地图原点是建图起点；小车不在该点时用 `initial_pose:="x y yaw"` 指定，
或 `initial_pose:=none` 后在 RViz 设置 **2D Pose Estimate**。检查激光与地图对齐后再进行小范围导航测试。

## 可选：SLAM Toolbox 重新建立二维地图

终端 1 启动仿真（关闭仿真自带 RViz，避免重复窗口）：

```zsh
ros2 launch scout_mini_sim sim.launch.py rviz:=false cameras:=false
```

终端 2 加载工作空间环境后启动 SLAM Toolbox + Nav2 + 导航 RViz：

```zsh
ros2 launch scout_mini_navigation navigation.launch.py slam:=true
```

等待 `/map` 与 TF 可用后，在 RViz 中用 **Nav2 Goal** 向已观测到的空闲区域发送目标，
逐步扩展地图。也可手动遥控建图，但遥控时不要同时发送导航目标。
建图模式不需要用 **2D Pose Estimate** 初始化 AMCL。

终端 3 在工作空间根目录保存地图（可重复使用其他文件名前缀）：

```zsh
ros2 run nav2_map_server map_saver_cli -f maps/scout_mini --ros-args -p use_sim_time:=true
```

生成 `maps/scout_mini.yaml` 和图像文件。这是独立的二维建图备选方案，
使用 FAST-LIO2 PCD 转图主流程时不需要运行它。

## 使用已有地图导航

保留仿真，停止上面的建图导航 launch，再启动导航（`prime-run` 让 RViz 使用独显，
单显卡电脑可省略）：

```zsh
prime-run ros2 launch scout_mini_navigation navigation.launch.py map:=/绝对路径/maps/scout_mini.yaml
```

可选：在启动导航前运行 FAST-LIO 里程计，启动后保持小车静止直到出现 `IMU Initial Done`：

```zsh
ros2 launch scout_mini_sim lio.launch.py mapping:=false rviz:=false
```

三维定位需要 `<地图名>.pcd`（与二维地图 YAML 同名，或用 `pcd_map:=` 指定）；
SLAM Toolbox 建的二维地图没有 PCD，需加 `localizer:=amcl`。
定位器默认以 `initial_pose:="0 0 0"`（map 坐标，米、弧度）作为初始位姿，适用于从默认
出生点用 FAST-LIO 建的地图。其他情况传实际位姿，例如 `initial_pose:="1.0 -2.0 1.57"`；
传 `initial_pose:=none` 则在 RViz 中用 **2D Pose Estimate** 手动指定。
确认激光与地图对齐后，再用 **Nav2 Goal** 发送目标。`map` 必须与当前 Gazebo 场景一致。
默认 `slam:=false`；未指定有效地图时会明确报错，而不是加载无关的示例地图。
可用 `rviz:=false` 或 `params_file:=/绝对路径/参数.yaml`。

## 配置与限制

- `config/nav2_params.yaml` 基于本机 Humble 的 Nav2 / SLAM Toolbox 默认配置，
  保留 Humble 的插件名称和行为树插件列表。
- 全局坐标系 `map`，局部坐标系 `odom`，底盘参考系 `base_footprint`。
- 全局定位（发布 `map → odom`），用 `localizer:=` 选择：
  - `ndt`（默认）：[lidar_localization_ros2](../third_party/lidar_localization_ros2)
    把每帧 16 线点云用 NDT 配准到 FAST-LIO 建的 PCD 地图，配准初值取自 EKF 的
    `odom → base_footprint`。配置见 `config/lidar_localization.yaml`；
    配准得分高于 `score_threshold` 的帧被拒绝，保留上一次的 `map → odom`。
  - `amcl`：二维粒子滤波，只用雷达中间一线匹配二维栅格，每移动 `0.1 m` 或转动 `0.1 rad` 更新。
- 局部里程计 `odom → base_footprint` 来自 `scout_mini_sim` 的 EKF（轮速 + IMU 陀螺仪，
  可选融合 FAST-LIO2 位姿），航向由陀螺仪测量，转弯打滑时仍准确。
- 矩形 footprint 为实车外形 `0.612 × 0.580 m`，另加 `0.02 m` padding；
  线速度上限 `1.0 m/s`（加减速 `1.0 m/s²`），角速度上限 `0.8 rad/s`（全速时最小转弯半径约 1.25 m）。这些不是实车标定参数。
  角速度上限与 Gazebo DiffDrive 插件的 `max_angular_velocity` 相同；调整速度时，
  RPP、`velocity_smoother`、`behavior_server` 和 Gazebo 插件的上限需一起修改，
  详见 `nav2_params.yaml` 开头的注释。
- 局部控制器为 Regulated Pure Pursuit（RPP）：沿全局路径追踪前方一点，
  弯道半径小于 `1.25 m` 或靠近障碍物时自动减速，车头偏离路径超过 45° 时先原地转向。
  全局规划为 Smac Planner 2D（A*，代价参与路径代价，自带平滑），二维激光障碍层与膨胀层；
  不包含三维避障。
  全局代价地图膨胀半径 `1.0 m`（局部 `0.65 m`），让全局路径尽量沿通道中间走。
  雷达平面之外的低矮/悬空障碍不保证可检测。
- 不要启动实车 `scout_mini_base`，不要让遥控与 Nav2 同时发布速度。
- 仿真底盘没有命令超时停车。结束导航前应取消目标并显式停车；
  不能把关闭 launch 当作可靠的急停：

```zsh
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

以上配置已在 Gazebo 仓库场景中验证 POI 巡检可用；参数未在实车上标定。
