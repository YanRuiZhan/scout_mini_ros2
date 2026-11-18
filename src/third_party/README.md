# 第三方包

| 包 | 上游 | 版本 | 本仓库的改动 |
|---|---|---|---|
| `fast_lio_ros2` | FAST-LIO2 的 ROS 2 移植 | — | 有本地修改：共线平面匹配剔除、水平约束、IMU 断档警告、地图累积等（见提交历史） |
| `livox_ros_driver2` | Livox ROS 2 驱动 | — | 只保留 FAST-LIO 编译需要的消息定义 |
| `lidar_localization_ros2` | https://github.com/rsasaki0109/lidar_localization_ros2 | `main` @ `6303352`（2026-11-18） | 未修改源码；未纳入 `images/`、`docker/`、`.github/` |
| `ndt_omp_ros2` | https://github.com/rsasaki0109/ndt_omp_ros2 | `master` @ `17ce1e1`（与 `humble` 分支相同） | 未修改；未纳入 `.github/`、`.vscode/` |

`lidar_localization_ros2` 由 `scout_mini_navigation` 的 `navigation.launch.py`（`localizer:=ndt`，默认）
启动，配置在 `src/scout_mini_navigation/config/lidar_localization.yaml`。更新上游版本时，
用同样的方式替换整个目录，并重新核对该配置中的参数名（`test_initial_pose.py` 会检查关键项）。
