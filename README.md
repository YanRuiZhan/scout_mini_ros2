# Scout Mini ROS 2 工作空间

ROS 2 Humble / Ubuntu 22.04，功能包在 `src/`，在本目录执行构建。
仿真模型和使用说明见 [src/scout_mini_sim/README.md](src/scout_mini_sim/README.md)。

## 当前阶段：仅仿真

```zsh
# 如已激活 conda，先 conda deactivate，直至退出 conda 环境。
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select scout_mini_sim
source install/setup.zsh
ros2 launch scout_mini_sim display_sim_urdf.launch.py
# Gazebo Fortress 仿真：
# ros2 launch scout_mini_sim sim.launch.py
```

`colcon_defaults.yaml` 为本工作空间启用软链接安装，并将 CMake 的
`Python3_EXECUTABLE` / `PYTHON_EXECUTABLE` 固定为 `/usr/bin/python3`。
这避免 CMake 从 PATH 选中 conda Python 而找不到 `ament_package` 或 numpy；
不修改全局 conda / shell 设置，也不要往 conda 或系统 Python 用 pip 安装 ROS 依赖。
命令行显式传入 `--cmake-args` 时，请同时保留上述 Python 参数。

测试：

```zsh
colcon test --packages-select scout_mini_sim scout_mini_navigation --event-handlers console_direct+
colcon test-result --test-result-base build --verbose
```

## 快速开始：仓库巡检

以下流程已在 Gazebo 仓库场景中验证可用。每条命令使用独立终端，先 `source install/setup.zsh`。

```zsh
# 1. 构建（首次或拉取更新后）
rosdep install --from-paths src/scout_mini_sim src/scout_mini_navigation \
  src/third_party/fast_lio_ros2 --ignore-src -r -y
colcon build --symlink-install --packages-select scout_mini_sim scout_mini_navigation fast_lio

# 2. 仿真（双显卡笔记本用 prime-run 走独显，见下方"常见问题"）
prime-run ros2 launch scout_mini_sim sim.launch.py

# 3. 导航，加载已保存的二维地图
prime-run ros2 launch scout_mini_navigation navigation.launch.py map:=maps/warehouse.yaml

# 4. 在 RViz 用 2D Pose Estimate 设置初始位姿，确认激光与地图重合，再开始巡检
ros2 run scout_mini_sim poi_patrol.py --poi maps/warehouse_poi.yaml
```

`maps/` 不纳入版本库。POI 坐标与地图坐标系的对应关系见
[仿真说明 · POI 自动巡检](src/scout_mini_sim/README.md#poi-自动巡检)。
FAST-LIO 里程计为可选项（见下文"定位方案"），需要时在第 2、3 步之间启动。

## 导航仿真

导航配置位于 [`src/scout_mini_navigation`](src/scout_mini_navigation/README.md)，
支持 SLAM Toolbox 建图导航，以及二维地图 + AMCL 定位导航。
复用现有 Gazebo 仿真，Nav2 使用系统安装包，无需加入 `third_party`。

### 定位方案

分两层（ROS REP-105）：局部里程计连续平滑，全局定位消除长期漂移。

```text
map ──► odom ──► base_footprint ──► base_link ──► 雷达 / IMU / 相机
   AMCL      EKF               URDF（固定）    URDF（固定）
```

| 变换 | 是否变化 | 发布者 | 含义 |
|---|---|---|---|
| `map → odom` | 缓慢变化，修正时会跳 | AMCL | 里程计累积的漂移量 |
| `odom → base_footprint` | 持续变化（50 Hz） | EKF（`robot_localization`） | 车从起点走了多远、转了多少 |
| `base_footprint → base_link → 传感器` | 固定 | `robot_state_publisher`（URDF） | 传感器在车上的安装位置 |

| 层 | 方法 | 输入 |
|---|---|---|
| 局部里程计 | EKF 融合 | 轮式里程计前进速度 + IMU 陀螺仪角速度（转弯打滑时航向仍准确）；可选 FAST-LIO2 位姿 x/y/yaw |
| 全局定位 | AMCL 粒子滤波 | 2D 激光（16 线雷达中间一线）匹配先验二维栅格地图 |

为什么有了全局定位还需要局部里程计：

- **频率与延迟**：控制器 20 Hz 运行，需要"此刻"的位姿和速度；AMCL 依赖 10 Hz 雷达，
  且每移动 `0.1 m` 或转动 `0.1 rad` 才更新一次，算出时车已前进一段。
- **连续性**：AMCL 每次修正都会让位姿突变；控制器和局部代价地图建立在 `odom` 下，
  需要不跳变的位姿，否则会猛打方向、障碍物错位。
- **AMCL 依赖里程计**：粒子滤波先用里程计预测运动，再用激光校正；里程计不准，
  全局定位也会出错。
- **速度**：控制器需要当前速度（`/odom` 的 twist），AMCL 只输出位置。

FAST-LIO2 内部是三维（6 自由度）的，但只作为里程计使用，不知道先验地图；
其位姿由 `lio_to_odom.py` 压平为二维后交给 EKF。最终定位与 Nav2 规划都是二维的。
当前 FAST-LIO 在仿真中仍会出现 `IMU gap`（IMU 样本丢失），默认导航流程不启动它；
`lio_to_odom.py` 会丢弃与 EKF 相差超过 `0.5 m / 15°` 的位姿，LIO 发散时不会带偏定位。
若货架通道等二维特征重复的区域定位不稳，可考虑三维点云匹配 PCD 地图的全局定位。

配置：`src/scout_mini_sim/config/ekf.yaml`（EKF）、
`src/scout_mini_navigation/config/nav2_params.yaml`（AMCL、规划、控制、速度上限）。
细节见 [仿真说明 · 里程计和 TF](src/scout_mini_sim/README.md#里程计和-tf)。

### 规划与控制

| 模块 | 方案 | 要点 |
|---|---|---|
| 全局规划 | NavFn | 全局代价地图膨胀半径 `1.0 m`，路径尽量沿通道中间 |
| 局部控制 | Regulated Pure Pursuit（RPP） | 追踪路径前方一点；弯道半径小于 `1.25 m` 或靠近障碍物时自动减速；车头偏离路径超过 45° 先原地转向 |
| 速度上限 | `1.0 m/s`、`0.8 rad/s` | 与 Gazebo 底盘上限一致；改动时需同步多处，见 `nav2_params.yaml` 开头注释，`test_odometry_config.py` 会检查 |

### 常见问题

| 现象 | 原因与处理 |
|---|---|
| 日志 `libEGL warning: egl: failed to create dri2 screen`，Gazebo 右下角 RTF 明显低于 100% | Gazebo 未用上显卡，在 CPU 上软件渲染。双显卡笔记本用 `prime-run ros2 launch ...` 启动仿真和导航（`nvidia-smi` 中应出现 `ruby` 进程）；若在 conda 环境中，先 `conda deactivate` |
| EKF 报 `Failed to meet update rate`、FAST-LIO 报 `IMU gap`、代价地图丢弃激光帧 | CPU 被占满。确认 RTF 接近 100%；可加 `gui:=false` 关闭 Gazebo 窗口；深度相机默认已关闭 |
| 日志 `[RTPS_TRANSPORT_SHM Error] Failed init_port` | 进程崩溃后 `/dev/shm` 残留锁文件，进程间通信退回 UDP，大消息易丢包。关闭所有 ROS 进程后执行 `ros2 daemon stop` 与 `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` |
| 巡检所有目标 `Getting path failed with status code: 6` | 规划器中止：车的位姿不在地图内或未设置初始位姿。在 RViz 勾选 RobotModel 检查车的位置，重新设置 2D Pose Estimate |
| 巡检目标落在仓库外，小车开出大门 | POI 坐标系与地图不一致，见仿真说明中的 `--poi-transform` |

## 全量构建（含实车包）

裸 `colcon build` 会同时构建实车包，需要额外的 ros2_control、ros2_controllers、
twist_mux、can_msgs、ros2_socketcan 和 ros2_socketcan_msgs 等依赖。
当前仅使用仿真，不必为了构建实车包安装这些组件。

需要实车时先解析并安装依赖，再构建：

```zsh
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

如 rosdep 无法解析 SocketCAN 依赖，需要补充对应的 Humble 兼容源码包或系统包，
而不是删除 `find_package` 来绕过依赖检查。

目录移动后如出现 CMake 源路径不一致，可加 `--cmake-clean-cache` 重新配置；
若仍引用旧安装路径，用未加载旧工作空间的新终端，清理生成目录后重建。
