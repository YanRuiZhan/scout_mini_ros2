# Scout Mini 仿真

本包使用 ROS 2 Humble 和 Gazebo Fortress 运行 Scout Mini 仿真，不启动实车 CAN 硬件。

## 构建

在工作空间根目录执行：

```zsh
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src/scout_mini_sim src/third_party/fast_lio_ros2 --ignore-src -r -y
colcon build --symlink-install --packages-select scout_mini_sim fast_lio \ --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
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
ros2 launch scout_mini_sim sim.launch.py gui:=false
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

建图启动后保持小车静止，等日志出现 `IMU Initial Done` 再行驶。`settle_time` 控制转发点云前的等待时长，按传感器时间计算；低帧率或暂停仿真不会提前结束等待。需要更长落地时间时可使用 `ros2 launch scout_mini_sim lio.launch.py settle_time:=2.0`。重启或重置 Gazebo 后，也应重启 FAST-LIO，清除上一轮地图。


### 里程计和 TF

仿真会发布轮式里程计 `/odom` 和 TF `odom → base_footprint`。FAST-LIO2 会发布 `/Odometry` 和 TF `camera_init → body`。`lio_odom_bridge` 再发布 `camera_init → odom`，把两棵 TF 树连接起来；它不会融合两路里程计，也不提供已保存地图的重定位。

```text
camera_init
├── body                 # FAST-LIO2
└── odom                 # lio_odom_bridge
    └── base_footprint   # 仿真底盘
        └── base_link    # 机器人模型和传感器
```

查看话题和 TF：

```zsh
ros2 topic hz /scout/lidar/points
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo camera_init base_link --ros-args -p use_sim_time:=true
```

`lio_odom_bridge` 只应运行一份。若 AMCL 或其他全局定位节点负责 `odom` 的父坐标系，启动 LIO 时加 `connect_odom:=false`，避免 TF 冲突。

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
默认距离按 `2.5、3.0、2.0 m` 尝试，兼顾本车水平安装且离地约 `0.27 m` 的相机；
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
不能直接作为这个转换。如果已确认当前 map 与 world 完全重合，可显式传
`--poi-transform 0 0 0`。也可以复制并转换这份配置，将坐标改成 map 下实测值并标明
`frame_id: map`，此时无需传转换参数。兼容旧格式：未写 `frame_id` 的 POI 文件
按 `--frame`（默认 map）解释。

正式巡检时先启动相机、对应场景和已保存地图的 Nav2（每条 launch 使用独立终端）：

```zsh
ros2 launch scout_mini_sim sim.launch.py cameras:=true
ros2 launch scout_mini_navigation navigation.launch.py map:=/绝对路径/对应地图.yaml
```

AMCL 模式先在 RViz 设置 **2D Pose Estimate**，确认激光与地图对齐。
脚本只等待定位，不自动重置初始位姿。不要运行默认的 `lio.launch.py` TF 连接节点；
导航地图建立流程与 TF 约束见 [导航说明](../scout_mini_navigation/README.md)。
对已经转换到 map 的 POI 文件，可在工作空间根目录运行：

```zsh
ros2 run scout_mini_sim poi_patrol.py --poi /绝对路径/poi_in_map.yaml \
  --only PG-W01 --out patrol_out
```

SLAM Toolbox 导航模式使用 `--localizer slam_toolbox`；外部定位且没有生命周期节点时
使用 `--localizer none`，仍要求目标坐标系到 `base_footprint` 的 TF 可用。
运行时默认从当前 TF 决定排序起点；`--start-x/--start-y` 只影响排序，不会初始化定位。
启动准备默认等待 60 秒，可通过 `--startup-timeout` 调整。
`--timeout` 为单目标导航墙钟超时；`--image-timeout` 控制新帧等待时间。
相机若以 `cameras:=false` 关闭，会记录 `image_result=no_image`。

每次巡检在 `patrol_out/日期_时间_随机后缀/` 下保存 PNG 和 `patrol_log.csv`，
每完成一个点就写入 CSV。`result` 区分 `reached`、`no_plan`、`rejected`、
`timeout`、`canceled`、`failed` 等状态；`image_result` 单独记录 `saved`、
`no_image` 或 `error`。到点后默认等待 0.5 秒，随后只接收采集时间更新的图像。
Ctrl+C 时保留当前记录、请求取消导航并向 `/cmd_vel` 发布零速度。
本轮适配的自动测试没有验证 Gazebo 中整条路线的可达性，首次请用 `--only` 单点验证。

