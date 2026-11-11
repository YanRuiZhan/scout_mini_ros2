# Scout Mini 导航仿真（初始配置）

ROS 2 Humble / Gazebo Fortress。复用 `scout_mini_sim` 的 `/scan`、`/odom`、
`/cmd_vel` 和 `odom → base_footprint`，所有导航节点使用仿真时间。
不需要将 Nav2 源码加入 `third_party`。

## 安装与构建

退出 conda 后，在工作空间根目录执行：

```zsh
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src/scout_mini_sim src/scout_mini_navigation --ignore-src -r -y
colcon build --packages-select scout_mini_sim scout_mini_navigation
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

必须启动与 PCD 对应的 Gazebo 场景；默认 `scout_mini.sdf` 不一定是 room_03 的建图场景。
导航时关闭 LIO 和 SLAM Toolbox，以 AMCL 管理全局定位：

```zsh
ros2 launch scout_mini_navigation navigation.launch.py map:=/绝对路径/maps/room_03_grid.yaml
```

先设置 **2D Pose Estimate** 并检查激光与地图是否对齐，再进行小范围导航测试。

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

保留仿真，停止上面的建图导航 launch，再执行：

```zsh
ros2 launch scout_mini_navigation navigation.launch.py map:=/绝对路径/maps/scout_mini.yaml
```

在 RViz 中先用 **2D Pose Estimate** 指定小车当前位姿，确认激光与地图对齐，
再用 **Nav2 Goal** 发送目标。`map` 必须与当前 Gazebo 场景一致。
默认 `slam:=false`；未指定有效地图时会明确报错，而不是加载无关的示例地图。
可用 `rviz:=false` 或 `params_file:=/绝对路径/参数.yaml`。

## 配置与限制

- `config/nav2_params.yaml` 基于本机 Humble 的 Nav2 / SLAM Toolbox 默认配置，
  保留 Humble 的插件名称和行为树插件列表。
- 全局坐标系 `map`，局部坐标系 `odom`，底盘参考系 `base_footprint`。
- 初始保守矩形 footprint 为 `0.66 × 0.60 m`（覆盖底盘与车轮），另加 `0.02 m` padding；
  线速度上限 `0.30 m/s`，角速度上限 `0.80 rad/s`。这些不是实车标定参数。
- DWB 局部控制、NavFn 全局规划，二维激光障碍层与膨胀层；不包含三维避障。
  雷达平面之外的低矮/悬空障碍不保证可检测。
- 不要同时启动 `lio.launch.py` 的默认 TF 连接节点：SLAM / AMCL 独占 `map → odom`，
  LIO 连接节点也会为 `odom` 指定父坐标系。初次验证建议完全不启动 LIO。
- 不要启动实车 `scout_mini_base`，不要让遥控与 Nav2 同时发布速度。
- 仿真底盘没有命令超时停车。结束导航前应取消目标并显式停车；
  不能把关闭 launch 当作可靠的急停：

```zsh
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

本包是初始接线与参数配置，仍需在 Gazebo 中验证定位、目标到达、避障与停车行为。
