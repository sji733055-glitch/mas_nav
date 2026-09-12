# MAS 2027 原生导航

本仓库使用 ROS 2 Jazzy，运行链路已经统一为独立 `nav_executor`，不再启动或依赖
Nav2 server、Behavior Tree、Costmap 插件、`minco_planner` 插件包或
`minco_controller` 插件包。

## 在线链路

```text
MID360 → small_point_lio → /Odometry + /cloud_registered
                              │              │
                              │              └→ ROGMap → A* / MINCO
                              └────────────────────────→ MPC → /cmd_vel

lab3_terrain.msgpack → terrain_map_server → /cost_map + /direction_map（RViz）
lab3.pcd             → odom_localizer      → map → odom
tf_maintainer                                → odom → base_link
```

`terrain_map_server` 当前只提供 HW 地形代价图和方向图可视化；规划避障使用在线
ROGMap。规划模式固定为 `EXPLORATION`，规划、轨迹优化和跟踪全部由
`mas2027_nav_executor` 完成。

## 环境与编译

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd /home/mas/mas_nav_2027_native
rosdep install -r --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 4
source install/setup.bash
```

额外系统库：

```bash
sudo apt-get install -y --no-install-recommends \
  ros-jazzy-rmw-cyclonedds-cpp libdw-dev libomp-dev python3-pip
```

## 启动

```bash
source /opt/ros/jazzy/setup.bash
source /home/mas/mas_nav_2027_native/install/setup.bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py
```

常用参数：

```bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py \
  use_rviz:=True \
  use_odom_localizer:=True \
  use_ros2_comm:=False \
  output_topic:=/cmd_vel
```

首次上车建议保持 `use_ros2_comm:=False`。RViz 的 2D Goal Pose 发布到
`/goal_pose`；轨迹、ROGMap、`HW Cost Map` 和 `HW Direction Map` 已在
`nav_executor_view.rviz` 中配置。

## 核心配置

- `mas2027_nav_executor/config/node_params.yaml`：控制频率、话题和坐标系。
- `mas2027_nav_executor/config/planner_params.yaml`：ROGMap、A*、MINCO 与恢复参数。
- `mas2027_nav_executor/config/mpc_params.yaml`：MPC 约束和权重。
- `mas2027_nav_bringup/config/small_point_lio_params.yaml`：雷达与 LIO 参数。
- `mas2027_perception/Localization/odom_localizer/config/params.yaml`：先验 PCD 定位。
- `mas2027_nav_bringup/map/lab3_terrain.msgpack`：HW 地形可视化地图。

MINCO、MPC 和 qpOASES 的运行时源码位于 `mas2027_nav_executor/vendor/`，属于执行器
内部实现，不再由其他 ROS 包跨目录提供。

## 地图更新

运行时使用两份同坐标系的先验：

- `mas2027_nav_bringup/pcd/lab3.pcd`：供 `odom_localizer` 定位；
- `mas2027_nav_bringup/map/lab3_terrain.msgpack`：供地形图 RViz 显示。

从已有 PGM/YAML 重新生成地形图：

```bash
python3 mas2027_perception/map_server/scripts/pgm_to_terrain_msgpack.py \
  mas2027_nav_bringup/map/lab3.yaml \
  mas2027_nav_bringup/map/lab3_terrain.msgpack
```

保存 LIO 点云并生成 PGM/YAML 的离线脚本仍可使用：

```bash
bash mas2027_nav_bringup/scripts/save_pcd_and_make_map.sh lab3
```

离线地图工具不会进入在线导航链路。更详细的数据流、TF 和排障说明见
[`docs/README.md`](docs/README.md)，每次修改记录见
[`docs/change-history.md`](docs/change-history.md)。
