# 学习顺序与各文件作用

按数据流动读，比按目录字母序读快。下面每一层读完，应该能回答括号里的问题再往下走。

## 建议顺序

### 1. 启动把什么拉起来了？

先读两个 launch，对照 `ros2 node list`：

| 文件 | 作用 |
|---|---|
| `mas2027_nav_bringup/launch/rm_navigation_small_point_lio_launch.py` | 总入口：雷达、LIO、`map→odom`、桥接节点、再 include Nav2 |
| `mas2027_nav_bringup/launch/nav2_launch.py` | Nav2 生命周期节点 + `fake_vel_transform` + `ros2_comm` |
| `mas2027_nav_bringup/launch/robot_state_publisher_launch.py` | 从 URDF 发静态 TF（`base_link → lidar_link` 等） |
| `mas2027_nav_bringup/config/nav2_params.yaml` | Nav2 / MINCO / ROG-Map 参数，文件头注释就是当前链路摘要 |
| `mas2027_nav_bringup/config/small_point_lio_params.yaml` | 驱动、LIO、`fake_vel_transform` 参数 |
| `mas2027_nav_bringup/behavior_trees/*.xml` | `planner_id` 必须是 `MincoPlanner`，`controller_id` 是 `FollowPath`。浏览器编辑：`mas2027_utils/bt_editor/bt_editor.html` |

启动开关（都在总 launch 里）：

| 参数 | 默认 | 实际控制什么 |
|---|---|---|
| `use_nav2` | True | 是否起 Nav2 + fake_vel + ros2_comm |
| `use_rog_map` | True | 是否起 `layer_value_to_cloud`（不是 `rog_map_node`） |
| `use_odom_localizer` | True | 动态 `map→odom`；False 则发单位静态 TF |
| `use_fake_vel_transform` | True | `base_link_fake` 与速度旋转 |
| `use_ros2_comm` | True | `/cmd_vel` UDP 下发 |

读完应能回答：默认启动里有没有 `rog_map_node`？（没有。）

### 2. 雷达和里程计

| 路径 | 作用 |
|---|---|
| `mas2027_perception/mid360_driver/src/mid360_driver.cpp` | UDP 收 MID360，发 `/mid360_driver/lidar`、`/mid360_driver/imu` |
| `mas2027_perception/Odometry/small_point_lio/src/small_point_lio_node.cpp` | 节点壳：订阅雷达/IMU，发布 `/Odometry`、`/cloud_registered*`、TF `odom→base_link` |
| `.../small_point_lio/src/small_point_lio/` | 紧耦合 LIO 核心 |
| `mas2027_robot_description/urdf/mas2027_sentry.urdf` | 车体与雷达安装外参（`lidar_joint`） |

读完应能回答：`/cloud_registered` 在哪个坐标系？（`odom`。）`/Odometry` 的 twist 在哪个轴系？（`base_link`。）

### 3. 定位与先验图

| 路径 | 作用 |
|---|---|
| `mas2027_perception/Localization/odom_localizer/` | 在线点云对 `mas2027_nav_bringup/pcd/` 先验 PCD 做 GICP，发 `map→odom` |
| `mas2027_perception/Localization/odom_localizer/config/params.yaml` | 先验 PCD 路径、GICP 门限 |
| `mas2027_nav_bringup/map/` | 二维 PGM/YAML（给 map_server / ROG-Map `prior_map`） |
| `mas2027_nav_bringup/pcd/` | 三维先验（给 odom_localizer；当前 `lab3.pcd`） |
| `mas2027_utils/pcd2pgm/`、`map_edit/`、`pcd_trans/`、`pcd2ele/`、`pcd2esdf/` | 点云切片、修图、变换、高程、离线 ESDF |

Nav2 的 `global_frame` 仍是 `odom`。`map` 帧主要给 ROG-Map 的二维先验投影用。

### 4. 局部地图（ROG-Map）

不要先翻整个 `src/rog_map/`。按接口读：

| 路径 | 作用 |
|---|---|
| `mas2027_planner/minco_planner/src/minco_core/minco_planner.cpp` 的 `configureRogMap()` | 唯一创建 `ROGMapROS` 的地方 |
| `mas2027_perception/rog_map/include/rog_map_ros/rog_map_ros2.hpp` | ROS 适配：订阅 odom/点云、定时更新、可视化发布 |
| `mas2027_perception/rog_map/src/rog_map/rog_map.cpp` | 三维概率栅格入口 |
| `.../prob_map.cpp`、`sliding_map.cpp` | 射线更新、滑动窗 |
| `.../projection_layer.cpp` | 三维柱 → 二维 FREE/PASSABLE/OCCUPIED/UNKNOWN |
| `.../field_layer.cpp`、`esdf_map.cpp` | 有符号距离场，MINCO 查的就是这个 |
| `.../prior_map.cpp` | 把 PGM 先验投到当前滑动网格 |
| `.../map_query_interface.hpp`、`query_adapter.cpp` | 规划器进程内查询，不走话题 |
| `.../map_registry.cpp` | 进程内单例；不能跨到 `controller_server` |
| `.../src/rog_map_bridge/layer_value_to_cloud.cpp` | `/rog_map/layer_value` → `/rog_map/terrain_map`，喂两个 costmap |
| `.../src/rog_map_node/rog_map_node.cpp` | 独立宿主，导航时不要启动 |

读完应能回答：costmap 看到的障碍从哪来？（viz timer 发 `layer_value`，桥接成点云。）MINCO 避障查的是哪一层？（进程内 ESDF，不是 costmap。）

### 5. 规划

| 路径 | 作用 |
|---|---|
| `minco_planner/src/minco_core/minco_planner.cpp` | Nav2 插件生命周期、读参、建图、`createPlan()` |
| `.../minco_fsm.cpp` | INIT / WAIT_GOAL / GENERATE_TRAJ / FOLLOW_TRAJ / RECOVERING |
| `.../components/global_path_searcher.cpp` | PRIORMAP（costmap/SMAC）或 EXPLORATION（ROG 边界） |
| `.../smac_search/` | 2D 搜索初值，可叠 ESDF 代价 |
| `.../components/local_path_processor.cpp` | 沿全局路径截 lookahead |
| `.../corridor_generator.cpp` | 安全走廊 |
| `.../traj_opt/minco_optimizer.cpp` | MINCO 多项式优化 |
| `.../components/trajectory_safety_checker.cpp` | 硬碰撞检查（`collision_dist`） |
| `.../components/recovery_behaivor.cpp` | 规划器内部脱困，和 Nav2 `behavior_server` 不是同一套 |
| `third_party/interfaces/` | `/opt_path` 的消息定义 |

`minco_planner/README.md` 仍留着上游话题名（`/aft_mapped_to_init`）。本仓库以 `nav2_params.yaml` 为准：`/Odometry`、`odom`。

### 6. 控制与底盘

| 路径 | 作用 |
|---|---|
| `minco_controller/src/minco_mpc_controller.cpp` | Nav2 Controller 插件：跟踪 `/opt_path`，解 QP |
| `minco_controller/src/mpc_solver.cpp` | qpOASES |
| `mas2027_utils/fake_vel_transform/src/fake_vel_transform.cpp` | `base_link_fake` TF、twist 旋转、叠加 `/cmd_spin`、转发 `/Odometry_world_fixed` |
| `mas2027_utils/ros2_comm/src/ros2_comm.cpp` | `/cmd_vel` → UDP（127.0.0.1:8889），上限约 50 Hz |
| `mas2027_utils/pb_nav2_plugins/src/layers/` | `IntensityVoxelLayer`，costmap 读点云强度 |

### 7. 可以后读的

| 路径 | 何时需要 |
|---|---|
| `mas2027_nav_bringup/scripts/waypoint_navigator.py` | 多点巡航 |
| `mas2027_nav_bringup/scripts/measure_lidar_mount.py` | 重标雷达安装时 |
| `mas2027_utils/bt_editor/bt_editor.html` | 改行为树 XML |
| `mas2027_utils/data_analyzer/scripts/nav2_performance_analyzer.py` | 看规划 vs 实测速度 |

## 包级目录

```text
mas_nav_2027/
├── mas2027_nav_bringup/      启动、参数、行为树、RViz、二维地图
├── mas2027_perception/       雷达驱动、LIO、ROG-Map、odom_localizer
├── mas2027_planner/          MincoPlanner + MincoMpcController
├── mas2027_robot_description/  URDF / mesh
├── mas2027_utils/            fake_vel、ros2_comm、costmap 插件、离线建图、bt_editor、data_analyzer
└── third_party/interfaces/   MINCO /opt_path 消息
```
