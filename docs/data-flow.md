# 数据链路

在线导航默认 `use_nav2:=True`、`use_rog_map:=True`、`use_odom_localizer:=True`。下面按传感器 → 地图 → 规划 → 控制 → 底盘写。

## 总图

```text
MID360
  |  /mid360_driver/lidar    PointCloud2
  |  /mid360_driver/imu      Imu
  v
small_point_lio
  |  TF  odom → base_link
  |  /Odometry               pose 在 odom，twist 在 base_link，best_effort
  |  /cloud_registered       odom 系去畸变点云，SensorDataQoS keep_last(1)
  |  /cloud_registered_full  当前与上一话题内容相同
  |
  +--------------------------+
  v                          v
ROGMapROS（在 planner_server 里）    odom_localizer
  | 进程内 MapQueryInterface          | GICP vs 先验 PCD
  | /rog_map/layer_value (10 Hz)      | TF map → odom
  v
layer_value_to_cloud
  |  /rog_map/terrain_map    PointXYZI
  v
local_costmap / global_costmap   IntensityVoxelLayer
  |
  v
bt_navigator  --ComputePathToPose(MincoPlanner)-->  nav_msgs/Path（只给 BT / RViz）
                  |
                  |  MincoPlanner 内部 FSM ~20 Hz
                  v
                /opt_path     interfaces/MpcPositionCommand
                  v
controller_server / FollowPath = MincoMpcController
  |  返回 TwistStamped（轴系 = odom = base_link_fake）
  |  同时发 /local_plan
  v
/cmd_vel_nav  -->  velocity_smoother  -->  /cmd_vel_nav_smoothed
                  v
            fake_vel_transform
              |  Rz(-yaw_chassis) + /cmd_spin
              |  /Odometry_world_fixed（twist 旋到 base_link_fake）
              v
            /cmd_vel
              v
            ros2_comm  UDP 127.0.0.1:8889  -->  下位机
```

## 1. 感知输入

| 话题 | 类型 | 发布 | 订阅 |
|---|---|---|---|
| `/mid360_driver/lidar` | PointCloud2 | `mid360_driver` | `small_point_lio` |
| `/mid360_driver/imu` | Imu | `mid360_driver` | `small_point_lio` |

驱动参数在 `small_point_lio_params.yaml` 的 `mid360_driver` 段。没有 `192.168.1.x` 网卡时驱动绑不上，整条链从这里断。

双雷达目标态仍是这两条话题：两台 MID360 在 `mid360_driver` 里按源 IP 配对、后雷达变到 `lidar_link` 后发出。硬件、PTP、外参与拟改参数见 [dual-lidar.md](dual-lidar.md)。当前代码还没融合；`is_topic_name_with_lidar_ip: false` 时插第二台会把两路点无外参拼进同一发布器。

## 2. LIO 输出

| 话题 | QoS | 用途 |
|---|---|---|
| `/Odometry` | best_effort, depth 10 | LIO 位姿。`MincoPlanner`、ROG-Map、`fake_vel_transform` 都订这个 |
| `/cloud_registered` | SensorData, keep_last(1) | ROG-Map 建图、odom_localizer GICP、离线 bag |
| `/cloud_registered_full` | 同上 | 注释里的「稠密主输出」；当前实现与 `/cloud_registered` 同内容 |

点云必须 `keep_last(1)`：深度太大时下游吃到的是排队的旧帧，ESDF 会滞后。录 bag 时 `/cloud_registered` 是 best_effort，必须用 `mas2027_nav_bringup/config/qos_mapping.yaml` 覆盖，否则 bag 里点云是 0 条。

`/Odometry` 与 `fake_vel_transform` 的订阅也必须 best_effort。节点里已经写了：默认 reliable 会一条都收不到，`current_robot_base_angle_` 永远停在 0。

## 3. ROG-Map 与 costmap

ROG-Map **不是** launch 出来的节点。`planner_server` configure 时 `MincoPlanner::configureRogMap()` 构造 `ROGMapROS`，绑在 `planner_server` 这个 LifecycleNode 上。

参数：`nav2_params.yaml` → `planner_server.MincoPlanner.rog_map`。

当前订阅：

- `ros_callback.odom_topic: /Odometry`
- `ros_callback.cloud_topic: /cloud_registered`
- `update_period_ms: 20`

两条下游：

| 下游 | 通道 | 用途 |
|---|---|---|
| MINCO / SMAC / 碰撞检查 | `MapQueryInterface` 指针 | 查占据和 ESDF 距离/梯度 |
| 两个 Nav2 costmap | 话题 | 恢复行为、碰撞检查、RViz；PRIORMAP 模式下 MINCO 全局搜索也会用 global_costmap |

话题桥：

```text
visualization.timer (必须 visualization.enable: true)
  --> /rog_map/layer_value     OccupancyGrid，100=OCCUPIED，其它为 0
  --> layer_value_to_cloud
  --> /rog_map/terrain_map     PointXYZI，z=0.2，intensity=1.0
  --> IntensityVoxelLayer（local + global）
```

`visualization.rate: 10.0` 就是 costmap 实际拿到障碍的频率。关掉 `use_rog_map` 或 `visualization.enable`，两个 costmap 都没有障碍观测。

其它 `/rog_map/*`（occupied、esdf、layer_type）是可视化，规划不依赖它们。

## 4. 规划与控制之间

| 话题 | 类型 | 说明 |
|---|---|---|
| Nav2 `ComputePathToPose` | action | BT 以 3 Hz 调 `MincoPlanner::createPlan()` |
| 返回的 `nav_msgs/Path` | Path | 只驱动 BT 的 `FollowPath` 和 RViz。被挡目标时可能仍返回 2 个点的「成功」Path |
| `/opt_path` | `MpcPositionCommand` | MINCO FSM 周期发布，MPC 真正跟踪的参考 |
| `/opt_path_vis`、`/astar_path_vis` | Path / Marker | 诊断：有折线无 `/opt_path` = 优化被拒 |
| `/local_plan` | Path | 本仓库给 MPC 补的；`fake_vel_transform` 用来判断控制器激活，并和 `/Odometry` 做 ApproximateTime 同步 |

里程计各用各的，不要混：

| 消费者 | 话题 | 原因 |
|---|---|---|
| `MincoPlanner` | `/Odometry` | 原始 LIO，twist 在 `base_link` |
| `MincoMpcController` 插件内 `odom_topic` | `/Odometry_world_fixed` | 内部 `compensateLeverArm()` 会按 yaw 旋 body 速度；不要喂双重旋转的数据 |
| `controller_server` Nav2 速度反馈 | `/Odometry_world_fixed` | Nav2 不变换 twist，当作 `robot_base_frame` 速度 |
| `bt_navigator` | `/Odometry_world_fixed` | 与 controller_server 同源；twist 在 `base_link_fake`。`/Odometry_fake` 这个名字已不发 |

## 5. 速度下发

```text
controller_server  remap cmd_vel --> /cmd_vel_nav
        |
velocity_smoother  /cmd_vel_nav --> /cmd_vel_nav_smoothed   （限加速度，超时停）
        |
fake_vel_transform
        输入：/cmd_vel_nav_smoothed（odom / base_link_fake 轴，yaw=0 所以两者重合）
        旋转：Rz(-yaw_chassis) 到 base_link
        叠加：/cmd_spin --> angular.z
        输出：/cmd_vel
        |
ros2_comm
        订阅 /cmd_vel（depth 1，只要最新）
        UDP 发 vx, vy, nav_state；硬上限 50 Hz，保活 20 Hz
        目标 127.0.0.1:8889（compose 用 host 网络）
```

MPC 的 `omega` 锁在 `[0, 0]`。自旋只走 `/cmd_spin`。`/cmd_vel_mpc` 是控制器调试回显，下位机不看它。

## 6. 旧 PolarBear 链路（已删）

`terrain_analysis` / `terrain_analysis_ext` / `loam_interface` / `sensor_scan_generation` 以及 `pointcloud_to_laserscan` 已从仓库删掉。costmap 只订 `/rog_map/terrain_map`，没有回退开关。
