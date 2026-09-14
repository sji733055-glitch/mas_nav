# MAS 2027 原生导航

本仓库使用 ROS 2 Jazzy，运行链路已经统一为独立 `nav_executor`，不再启动或依赖
Nav2 server、Behavior Tree、Costmap 插件、`minco_planner` 插件包或
`minco_controller` 插件包。

## 在线链路

```text
MID360 → small_point_lio → /Odometry + /cloud_registered → map_server（当前动态检测）
                                  └────────────────────────────→ ROGMap（在线局部占据/距离）
                                                                ↓
lab3_terrain.msgpack → terrain_map_server → 静态地形约束 → TaskManager → 全向 Kino A* → MINCO → MPC → /cmd_vel

terrain_map_server → /cost_map + /direction_map（静态地形层）
lab3.pcd             → odom_localizer      → map → odom
tf_maintainer                                → odom → base_link
```

`terrain_map_server` 的代价图与方向图现已进入规划链路：全向 Kino A* 在静态地形图上
联合搜索位置、速度方向与速度档，按底盘加速度约束扩展，并叠加 `/dynamic_cost_map`
当前帧；弧长速度剖面用速度平方的前后向可达性传播及分段加速/巡航/制动时间
为 MINCO 提供时间种子。最终发布的时间参数化仍由 MINCO 优化并验收。
MINCO 的走廊、优化器和轨迹碰撞检查使用 ROGMap 的在线二维占据与距离场；
静态地形方向约束和 `map_server` 的当前动态代价图仍用于全局搜索、轨迹验收与
MPC 制动。ROGMap 也检查全局搜索落在其局部滑窗内的部分以及 MPC 下一段指令；
滑窗外仍由地形图和当前动态代价图负责。地图、ROGMap 距离场或 `map→odom`
TF 缺失时不接受目标或不输出运动命令。

目标接纳、重规划及轨迹执行许可现在由 `task_manager/TaskManager` 管理，
`MincoPlanner` 内置的 `MincoFsm` 已移除。新目标抢占时，旧轨迹即使晚到也不能
重新获得运动许可；旧的限时推离恢复由 `TaskManager` 接管，且仍受地图安全检查；
里程计超过 `node.odom_timeout_s` 时规划与执行都会停止。
执行器源码按职责放在 `path_planner/search`、`path_planner/trajectory`、
`path_executor/mpc`、`path_executor/monitoring`、`path_executor/state` 和
`common/environment`；第三方数值算法及 qpOASES 仍在 `vendor/`。

ROGMap 由 `nav_executor_planner` 进程内持有，订阅 `/cloud_registered` 和
`/Odometry`，不是另起一个 `rog_map_node`；RViz 的 `ROGMAP` 分组默认显示紫色
`Occupied` 立方体在线占据（边长 0.05 m）与淡色 `2D Distance Field`。动态/静态/合成投影栅格、类型图
及高度分析可在分组内按需勾选；`Map Bound` 默认显示局部地图、可视和更新范围。
ROGMap 的先验图融合当前关闭，所以 `Static Layer Value`
默认关闭且不能代表 `map_server` 的静态地形图。其投影高度、地面/墙体阈值和
`node.rog_map_clearance`（默认 0.30 m）
需在静止实车上标定；若双雷达外参或时间不同步，点云重影也会进入 ROGMap。
为避免车体振动导致静态目标被误判为运动，已移除目标跟踪与未来障碍预测；
这两段是针对全向底盘的适配，不是原样复制 HW 的前向曲率运动原语、指导走廊、
台阶速度窗或后置 MINCO 的完整速度优化器；因此仍不是 HW 规划/执行算法的完整等价实现。
`lab3_terrain.msgpack`
由 PGM 生成，只有平地/障碍，**没有**坡道、台阶及方向数据；需要标注真实语义地图
才能验证方向通行效果。保留原有 `/cmd_vel` 底盘协议，不移植 HW 腿部模式或其
LPV/FDDP 底盘控制模型。规划模式仍为 `EXPLORATION`。
当前 MPC 使用全向运动模型，输出车体系 `/cmd_vel.linear.x`、`linear.y`、
`angular.z`；默认允许转向（角速度 ±2 rad/s、角加速度 ±4 rad/s²），
这些限值需按实际底盘标定。它不是 HW 腿式底盘的 LPV/FDDP 控制器。

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
`/goal_pose`；轨迹、`HW Cost Map` 和 `HW Direction Map` 已在
`nav_executor_view.rviz` 中配置。默认的 `Planning Constraints` 是稀疏高对比度点层，
不会遮挡全局路径或 MINCO 轨迹：红色为当前不能通过，黄色为只允许沿语义方向通过的
地形；预测障碍不在 RViz 中显示。底层 `HW Cost Map` 半透明显示，便于对照原始代价
来源。`Dynamic Obstacles (cyan, current)` 单独以青色点显示当前动态障碍；需要查看
完整栅格时可手动启用 `Planning Constraints (raster diagnostic)`。
若点击目标后没有路径，先看终端是否出现 `Received goal on /goal_pose`；
`Queued goal` 仅代表目标入队，不代表轨迹已经生成。后续若出现
`MINCO trajectory not published`，按其失败原因检查地图/TF/安全距离；
传感器地图尚未就绪时需要重新点击目标。
若出现 `Trajectory clearance ... below required ...`，应先核实车体与静态地图的
配准及实际障碍距离，不要直接减小 `collision_dist` 来绕过安全检查。

## 核心配置

- `mas2027_nav_executor/config/node_params.yaml`：控制频率、话题和坐标系。
- `mas2027_nav_executor/config/planner_params.yaml`：A*、MINCO 与恢复参数。
- `mas2027_nav_executor/config/mpc_params.yaml`：MPC 约束和权重。
- `mas2027_nav_bringup/config/small_point_lio_params.yaml`：雷达与 LIO 参数。
- `mas2027_perception/Localization/odom_localizer/config/params.yaml`：先验 PCD 定位。
- `mas2027_nav_bringup/map/lab3_terrain.msgpack`：静态地形规划地图。

MINCO 搜索与轨迹逻辑位于 `mas2027_nav_executor/src/path_planner/`，MPC 求解与安全监测
位于 `mas2027_nav_executor/src/path_executor/`；第三方数值后端与 qpOASES 位于 `vendor/`，不由其他 ROS 包
跨目录提供。

## 地图更新

运行时使用两份同坐标系的先验：

- `mas2027_nav_bringup/pcd/lab3.pcd`：供 `odom_localizer` 定位，也供 `map_server` 静态点云差异检测；
- `mas2027_nav_bringup/map/lab3_terrain.msgpack`：供静态地形规划及 RViz 显示。

定位器使用 GICP 的完整六自由度 `map→odom` 结果，不再固定 Z；先验 PCD 必须与
当前雷达安装、地面高度和天花板高度一致。修改先验点云或雷达外参后，应先在 RViz
确认地板和天花板重合，再启用动态点云差分。

不依赖真机的动态代价图烟测：

```bash
ROS_DOMAIN_ID=231 python3 mas2027_perception/map_server/test/smoke_dynamic_cost_map.py \
  mas2027_nav_bringup/map/lab3_terrain.msgpack \
  mas2027_perception/map_server/test/fixture_static_floor.pcd
ROS_DOMAIN_ID=232 python3 mas2027_nav_executor/test/smoke_goal.py \
  mas2027_nav_bringup/map/lab3_terrain.msgpack \
  mas2027_nav_executor/config \
  mas2027_perception/map_server/test/fixture_static_floor.pcd
```

贴地障碍的差分以 `map_server` 参数 `local_map.low_obstacle_min_height`
（默认 0.06 m）及 `local_map.low_obstacle_match_distance`（默认 0.07 m）控制；
小于地面分割噪声的凸起无法可靠区分，需实车标定，不能直接降低到零。
动态栅格在短时漏检后由 `local_map.dropout_hold_seconds`（默认 0.3 s）继续保留，
避免原地闪烁；新障碍立即生效。这里的“动态”是相对于先验 PCD 的差异，
并非运动速度判断；持续不匹配的静止障碍仍须参与避障，应校准先验点云与实时点云配准。

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
