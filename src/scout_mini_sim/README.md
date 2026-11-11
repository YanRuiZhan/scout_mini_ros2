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

