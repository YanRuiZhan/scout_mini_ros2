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
