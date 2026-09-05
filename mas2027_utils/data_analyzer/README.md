# data_analyzer

从上游 `navi_minco_bit/src/utils/data_analyzer` 搬来的实时 MINCO 跟踪对照。默认话题已经改成本仓库链路，不进主 launch。

对比 `/opt_path`（规划速度/加速度）和 IMU 积分得到的“实测”速度，并记 CSV。关窗口后出一张轨迹/速度/加速度报告图。

## 话题对照

| 用途 | 上游 | 本仓库默认 |
|---|---|---|
| 规划轨迹 | `/opt_path`（`ros_interfaces/MpcPositionCommand`） | `/opt_path`（`interfaces/MpcPositionCommand`） |
| 里程计 | `/aft_mapped_to_init` | `/Odometry`（best_effort，必须用 SensorDataQoS） |
| IMU | `/livox/imu` | `/mid360_driver/imu` |
| 控制频率 | `/cmd_vel` | `/cmd_vel`（smoother → fake_vel 之后，约 20 Hz） |
| 杆臂 / roll | `y=-0.20`，roll 20° | `0` / `0`（与 `nav2_params.yaml` 的 `lidar_offset_*` 一致） |

## 运行

容器里、已经 `source /home/ros2_ws/install/setup.bash`，导航栈在跑：

```bash
python3 /home/ros2_ws/src/mas2027_utils/data_analyzer/scripts/nav2_performance_analyzer.py
```

需要 `python3-matplotlib` 和 `python3-tk`（Dockerfile 已加）。X11 要通，和 RViz 一样 `xhost +si:localuser:root`。

覆盖话题或安装偏置：

```bash
python3 /home/ros2_ws/src/mas2027_utils/data_analyzer/scripts/nav2_performance_analyzer.py --ros-args \
  -p odom_topic:=/Odometry \
  -p imu_topic:=/mid360_driver/imu \
  -p cmd_vel_topic:=/cmd_vel \
  -p sensor_offset_y:=0.0 \
  -p sensor_roll_deg:=0.0
```

CSV 和关窗后的 PNG 写到 `/tmp/mas_nav_data_analyzer/`（可用环境变量 `MAS_NAV_ANALYZER_DIR` 改）。
