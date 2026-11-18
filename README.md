# Scout Mini ROS 2 工作空间

ROS 2 Humble / Ubuntu 22.04，功能包在 `src/`，在本目录执行构建。
仿真模型和使用说明见 [src/scout_mini_sim/README.md](src/scout_mini_sim/README.md)。

## 当前阶段：仅构建仿真

```zsh
# 如已激活 conda，先 conda deactivate，直至退出 conda 环境。
source /opt/ros/humble/setup.zsh
colcon build --packages-select scout_mini_sim
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
colcon test --packages-select scout_mini_sim --event-handlers console_direct+
colcon test-result --test-result-base build/scout_mini_sim --verbose
```

## 导航仿真

导航配置位于 [`src/scout_mini_navigation`](src/scout_mini_navigation/README.md)，
支持 SLAM Toolbox 建图导航，以及二维地图 + AMCL 定位导航。
复用现有 Gazebo 仿真，Nav2 使用系统安装包，无需加入 `third_party`。

### 定位方案

分两层（ROS REP-105）：局部里程计保证连续平滑，全局定位消除长期漂移。

```text
map ──── odom ──── base_footprint
    AMCL      EKF（robot_localization）
```

| 层 | 发布 | 方法 | 输入 |
|---|---|---|---|
| 局部里程计 | `odom → base_footprint`、`/odom`（50 Hz） | `robot_localization` EKF 融合 | FAST-LIO2 位姿 x/y/yaw（10 Hz，主力，不受车轮打滑影响）<br>轮式里程计前进速度 + IMU 陀螺仪角速度（补齐两帧雷达之间） |
| 全局定位 | `map → odom` | AMCL 粒子滤波 | 2D 激光（16 线雷达中间一线）匹配先验二维栅格地图 |

- FAST-LIO2 内部是三维（6 自由度）的，但只作为里程计使用，并不知道先验地图；
  其位姿由 `lio_to_odom.py` 压平为二维后交给 EKF。最终定位与 Nav2 规划都是二维的。
- 未启动 FAST-LIO 时 EKF 退化为轮速 + IMU，航向仍准确，但位置会逐渐漂移。
- 导航启动顺序：`sim.launch.py` → `lio.launch.py mapping:=false`（静止至 `IMU Initial Done`）
  → `navigation.launch.py map:=...` → RViz 中设置 2D Pose Estimate。
- 配置：`src/scout_mini_sim/config/ekf.yaml`（EKF）、
  `src/scout_mini_navigation/config/nav2_params.yaml`（AMCL、速度上限）。
  细节见 [仿真说明 · 里程计和 TF](src/scout_mini_sim/README.md)。
- 若货架通道等二维特征重复的区域定位不稳，可考虑升级为三维点云匹配 PCD 地图的全局定位。

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
