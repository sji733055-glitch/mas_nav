# MAS 2027 原生导航

本仓库使用 ROS 2 Jazzy，运行链路已经统一为独立 `nav_executor`，不再启动或依赖
Nav2 server、Behavior Tree、Costmap 插件、`minco_planner` 插件包或
`minco_controller` 插件包。

## 在线链路

```text
MID360 → small_point_lio → /Odometry + /cloud_registered → map_server（当前动态检测）
                                  └────────────────────────────→ ROGMap（在线局部占据/距离）
                                                                ↓
lab3_terrain.msgpack → terrain_map_server → 静态地形约束 → TaskManager → SMAC 2D → MINCO → MPC → /cmd_vel

terrain_map_server → /cost_map + /direction_map（静态地形层）
lab3.pcd             → odom_localizer      → map → odom
tf_maintainer                                → odom → base_link
```

`terrain_map_server` 的代价图与方向图现已进入规划链路：**全局主搜索是从
`mas_nav_2027` 移植过来的 SMAC 2D**（`path_planner/search/smac/`），在
静态地形图与 `/dynamic_cost_map` 当前帧合并出的二维栅格上做 8 邻域 A\*，
并用同一张图烘焙的距离场做 ESDF 势场软代价（`smac_2d.*`），让折线远离墙面、
贴近走廊中线。搜索输出与旧工程一致是纯几何折线，不含速度/加速度可行性约束；
运动学可行性由下游的弧长速度剖面与 MINCO 保证 —— 弧长速度剖面用速度平方的
前后向可达性传播及分段加速/巡航/制动时间为 MINCO 提供时间种子，
最终发布的时间参数化仍由 MINCO 优化并验收。
`planner.use_smac: false` 可退回原来的 Astar（NavFn 波前）分支；SMAC 失败时
**不会自动回退**，直接失败并打端点诊断，与旧工程一致。
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

### 净空判据（发布前校验 / 运行时监视 / MPC 指令检查）

三处安全检查共用 `common/environment/clearance_gate.hpp` 的一条判据，要求：

```
required(v) = collision_dist + max(v * replan_react_time, monitor_margin)
```

- 发布前校验（`validateTrajectory`）与运行时监视（`checkCollision()`）必须是同一个公式。
  两者曾经分别用 `collision_dist`(0.30) 和 `collision_dist + monitor_margin`(0.50)，
  结果是轨迹刚发布就被监视器判不安全 → 急停 + 重规划，`/cmd_vel` 只在发布瞬间有值、
  车走不起来，RViz 里规划轨迹与急停轨迹来回闪。`monitor_margin` 默认 `0.0`。
- 运行时监视额外减去 `kMonitorClearanceTolerance`（0.05 m，一个 ROGMap 体素）作为抖动余量，
  方向是让监视器略宽于发布门，避免 ESDF 逐帧更新把刚通过校验的轨迹立刻否决。
- **近场放宽**：机器人当前所在位置附近（沿轨迹弧长 `collision_dist` 以内）是它已经占住的
  地方，那里不可能满足完整净空。该段只要求「不比当前实测净空更差」（留 0.02 m 抖动余量），
  离开近场后必须满足 `required(v)`。没有这条规则时，只要车停在离墙比 `collision_dist`
  更近的地方，任何轨迹都会在起点被拒，表现为「发目标点后车不动」。
  日志出现 `Near-field exemption: start clearance ...` 说明正在使用该规则。
- MPC 下一段指令检查（`node.rog_map_clearance`）用同一判据，起点同样按近场处理。

排查「车不动」的顺序：先看 `/cmd_vel` 是否持续非零，再看终端里
`MINCO trajectory not published: <原因>` 与 `Committed path became unsafe` 是否在刷。
反复急停说明发布门与监视门又不一致；只有 `not published` 说明近场外的
`required(v)` 满足不了，此时应核实车体与地图配准、ROGMap 在线占据是否把车体自身
（雷达盲区、云台/枪管）算成了障碍，而不是直接减小 `collision_dist`。

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
早先自研的「全向 Kino A\*」（`path_planner/search/omni_kino_astar.*`，状态含位置、
运动方向与速度档）已**移出关键路径**：现场实测它在真实 lab3 图上 3 m 起大量方向无解、
5 m 后几乎全灭，每次失败烧光 50 000 次扩展预算、耗时 0.76~1.03 s，瓶颈是状态空间爆炸
而非无解。模块与 `test_omni_kino_astar` 保留备用，现已无生产调用点。
`lab3_terrain.msgpack`
由 PGM 生成，只有平地/障碍，**没有**坡道、台阶及方向数据；需要标注真实语义地图
才能验证方向通行效果。方向层当前实测全 0，因此 `TerrainMapQuery` 交给全局搜索的
是二值栅格、不含方向；逐边方向约束只保留在 `validateTrajectory` 的
`terrain->transition()` 里（失败记 `TERRAIN_COLLISION_OR_DIRECTION`）。
保留原有 `/cmd_vel` 底盘协议，不移植 HW 腿部模式或其
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

**全局折线与轨迹是两条不同的线，RViz 里分开显示**（这是排障时最快能看出问题的地方）：

| RViz 显示 | 话题 | 含义 |
|---|---|---|
| `Global Plan (SMAC search, thick)` | `/nav_executor/debug/global_plan`（Marker） | 全局搜索（SMAC 2D / Astar）输出的折线，默认 **0.15 m 青色**粗线 + 终点球 |
| `Global Plan (Path)` | `/nav_executor/global_plan`（Path） | 同一条折线的 `nav_msgs/Path`，给需要读坐标/点数的工具 |
| `MINCO Trajectory` | `/nav_executor/debug/minco_trajectory`（Marker） | MINCO 轨迹，0.07 m、按速度染色 |
| `MINCO Trajectory (Path)` | `/nav_executor/global_path`（Path） | 同一条轨迹的 `nav_msgs/Path`（**这个名字是历史遗留，发的不是全局折线**，`smoke_goal.py` 靠它数轨迹点数） |

所以「搜索想让它怎么绕」看粗青线，「车准备怎么走」看细彩线；两条明显不重合时，
问题在 MINCO 的走廊/净空，而不是全局搜索。注意粗青线的最后一段是从搜索的到点容差
（`planner.tolerance`，默认 0.30 m）跳到精确目标点的，可能有一小截不在格点上。
样式由 `planner` 之外的 `node.visualization.global_plan_*` 控制（发布频率、线宽、RGB）。

若点击目标后没有路径，先看终端是否出现 `Received goal on /goal_pose`；
`Queued goal` 仅代表目标入队，不代表轨迹已经生成。后续若出现
`MINCO trajectory not published`，按其失败原因检查地图/TF/安全距离；
传感器地图尚未就绪时需要重新点击目标。
若出现 `Trajectory clearance ... below required ...`，应先核实车体与静态地图的
配准及实际障碍距离，不要直接减小 `collision_dist` 来绕过安全检查。

## 核心配置

- `mas2027_nav_executor/config/node_params.yaml`：控制频率、话题和坐标系。
- `mas2027_nav_executor/config/planner_params.yaml`：全局搜索（`use_smac`、
  `smac_2d.*`、`allow_unknown`、`tolerance`）、MINCO 与恢复参数。
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
