# 问题排查与定位

先分层：传感器 → TF → 地图 → 规划 → 控制 → 底盘。每一层用「有没有数据」和「数据在不在对的坐标系」两个问题卡住，再往代码里钻。

杀后台节点用记下的 PID，或 `pkill -x` 精确命令名，不要 `pkill -f`。

## 0. 启动后 30 秒体检

```bash
ros2 node list
ros2 topic hz /mid360_driver/lidar /Odometry /cloud_registered
ros2 topic hz /rog_map/layer_value /rog_map/terrain_map /opt_path /cmd_vel
ros2 run tf2_ros tf2_echo odom base_link
ros2 lifecycle get /planner_server /controller_server
```

lifecycle 卡在 configuring、到不了 active：通常是 costmap 等 `odom → base_link` 等不到。只起 `nav2_launch.py`、没有 `small_point_lio` 时就会这样，这是预期，不是插件没加载。

## 1. 没点云 / 没里程计

| 现象 | 优先查 | 常见原因 |
|---|---|---|
| `/mid360_driver/lidar` 无数据 | 网卡、`host_ip`、雷达 IP | 本机没有 `192.168.1.x` 时驱动绑不上 |
| 有雷达无 `/Odometry` | LIO 日志、URDF TF | `base_link → lidar_link` 还没 lookup 到；IMU 话题名不对 |
| `/cloud_registered` hz=0 但 LIO 在跑 | QoS | 订阅端用了 reliable；或 keep_last 太大看起来像「有数据」其实全是旧帧 |
| bag 里点云 0 条 | 录包 QoS | 必须用 `mas2027_nav_bringup/config/qos_mapping.yaml` 覆盖 best_effort |
| 插了第二台雷达后 LIO 漂 / 整帧被丢 | `is_topic_name_with_lidar_ip`、时间跨度 | 默认 `false` 会把两路点**无外参**拼进同一话题。先 `true` 分话题确认两路 UDP，再按 [dual-lidar.md](dual-lidar.md) 做融合；不要直接喂现有驱动 |

ROG-Map 打 `No odom received, skip cloud callback`：先有 `/Odometry` 再有点云。`Odom timeout`：odom 和点云时间差超过 `odom_timeout`（0.08 s）。

## 2. TF 断、帧名错

| 现象 | 优先查 |
|---|---|
| costmap / planner 一直 inactive | `tf2_echo odom base_link`；LIO 是否在发 |
| RViz 车模在原点打转、地图乱跳 | 是否两个节点同时发 `map→odom` |
| 先验墙贴在开机点周围 | `prior_map.frame_id` 被改成 `odom`，或 localizer 没锁上 |
| 障碍距离过滤以场地原点为中心 | costmap `sensor_frame` 没填 `lidar_link` |
| Nav2 报 `base_link_fake` 不存在 | `use_fake_vel_transform:=False`，或 fake_vel 没收到 `/Odometry`（QoS） |

改 URDF 雷达安装后，LIO 盲区球心、`lidar_offset_*`、ROG `center_offset` 是同一组标定量，要一起改。一键重测：`bash mas2027_nav_bringup/scripts/measure_lidar_mount.sh`（须先停导航）。

## 3. 地图不对：空的、全是墙、闪烁、范围太小

先分清你看的是哪一层：

- RViz `/rog_map/layer_type`：ProjectionLayer 四分类
- `/rog_map/terrain_map`：桥接后的障碍点，只有 OCCUPIED
- local/global costmap：再经过体素和膨胀
- MINCO 实际避障：进程内 ESDF，RViz `/rog_map/field` 才接近它

| 现象 | 可能 |
|---|---|
| 两个 costmap 全空 | `use_rog_map:=False`；`visualization.enable: false`（layer_value 在 viz timer 里发）；桥接节点没起；QoS |
| costmap 有障碍但车照撞 | MINCO 不读 costmap 做优化；查 ESDF / `safe_dist` / 点云是否滞后 |
| 远处没障碍，开近了才出现 | ROG `map_size` 只有 ±5 m，global_costmap 30 m 外圈无观测 |
| 地面变成墙 | 雷达倾角、`scan_z_min_abs`、ProjectionLayer 阈值；远处地面被当障碍就是换 ProjectionLayer 的原因 |
| 障碍格子远多于真实墙 | `classifyCell` 末尾兜底是 OCCUPIED。`H≥0.45` 且 occupancy ratio < 0.80 会把「地面 + 一个飞点」标成墙。打开 `MincoPlanner.rog_map.performance.print_enable`，看 1 Hz 的 `classification: thin=/wall=/tunnel=/ambiguous=`，ambiguous 占优就是这个 |
| `/rog_map/layer_value` 在两套栅格间跳 | 同时起了 `rog_map_node` 和 MincoPlanner 内的那份 |
| `[ROG WARN] Unfinished frame cnt > 1` | 多半是 OpenMP 线程太多导致调度延迟，不是算法本身。`parallel_raycast_enable` 保持 false；小规模 ESDF 不要开宽 team。CSV 在容器 `/tmp/rog_map_perf_*.csv` |

## 4. 规划：有目标但不动、穿障、贴障

BT `planner_id` 必须是 `MincoPlanner`。配成 `GridBased` 时启动正常，发目标才失败。

| 现象 | 怎么区分 |
|---|---|
| 一直 INIT | 没有 `/Odometry`，或 `configureRogMap` 失败（看 planner_server 日志） |
| `ComputePathToPose` 成功，车原地、`/cmd_vel` 为 0 | 看有没有 `/opt_path`。没有 = MINCO 优化后被 `validateTrajectory` 拒绝。被墙挡住的目标会 20 Hz 刷 `collision detected` / `Rejecting`，BT 仍认为规划成功，**不会进恢复** |
| 有 `/astar_path_vis` 无 `/opt_path` | 搜索通了，优化或安全检查没过：`safe_dist`/`collision_dist`、速度加速度上限、ESDF 是否有效 |
| 轨迹贴障/穿障 | 点云时延、投影高度、`field.inflation_radius`（必须 0；非零会把净空抬到车钻不过去，L-BFGS 三次迭代就退出） |
| 会撞倒障碍、动态障碍来不及让 | MINCO 在 `FOLLOW_TRAJ` 里以前 1 Hz 才重规划（YAML 写了 5 Hz 但 C++ 没读），安全检查用贴车的 `collision_dist: 0.30`，失败还继续跟旧 `/opt_path`，MPC 不看 ESDF。现已接到：监视净空 `collision_dist + max(v·0.35, 0.20)`、强制重规划 5 Hz、不安全 10 Hz 封顶、`ReplanLocal` 失败则急停 `/opt_path`（不改 `last_traj_` / 冷热启动）。速度三处 3.0。需重建 `minco_planner`。查 `[MincoPlanner] Trajectory collision detected` 和有没有新的 `/opt_path` |
| 0.95 m 目标开成爬行 | `penalty_weight_time` 过小（代码默认 0.01，本仓库已改成 100） |
| 优化日志 cost 地板 1e4 且 iter 很少 | Pos 惩罚被抬成常数，检查 ESDF inflation 和 `safe_dist` |

性能：`/tmp/minco_perf_detailed.csv`，`MincoPlanner.performance.*`。比赛前关掉详细 CSV 和刷屏。

## 5. 控制：方向反、横移、振荡、延迟

| 现象 | 优先查 |
|---|---|
| `/cmd_vel_nav` 无输出 | lifecycle 是否 active；有没有 `/opt_path`；MPC odom 是否在到 |
| 速度越高横漂越大 | Nav2 速度反馈用了未旋转的 `/Odometry`（twist 在 base_link），应该用 `/Odometry_world_fixed` |
| 原地转伴随平移 | `lidar_offset_*` 符号；不要给 MPC 喂已旋转的 fake odom |
| 自旋时速度方向乱跳 | `/local_plan` 没了，fake_vel 走 CONTROLLER_TIMEOUT，yaw 没和 cmd 对齐 |
| 速度方向和直觉相反 | 输出是全局/fake 轴，到 `base_link` 才是车体系；下位机约定是否也是车体系 |
| 指令延迟一整秒 | `ros2_comm` 发太快，UDP 队列里全是旧包；保持 ≤50 Hz |
| QP 不可行 | 参考突变、加速度边界比 smoother 更松、初始误差过大 |
| 小陀螺 | 本车 `use_small_gyro_mode: false`，自旋只走 `/cmd_spin`。打开会和 fake_vel 的 spin 叠两份 |

`bt_navigator` 和 `controller_server` 的 `odom_topic` 都是 `/Odometry_world_fixed`（`fake_vel_transform` 的 `output_odom_topic`）。`/Odometry_fake` 是旧名，库里没有节点再发这个话题。

## 6. 按日志关键字跳文件

| 日志 / 话题 | 文件 |
|---|---|
| `Failed to lookup transform from base_link` | `small_point_lio_node.cpp`、URDF |
| `No odom received` / `Odom timeout` / `Unfinished frame cnt` | `rog_map_ros2.hpp` 的 cloud/update 回调 |
| `ROGMap is created inside MincoPlanner` | `minco_planner.cpp` `configureRogMap` |
| `Trajectory validation failed` / `Rejecting` | `trajectory_safety_checker.cpp`、`minco_optimizer.cpp` |
| `planner_id` / ComputePathToPose 失败 | `behavior_trees/*.xml` |
| `CONTROLLER_TIMEOUT` 行为（无 local_plan） | `fake_vel_transform.cpp` |
| UDP / `MIN_SEND_INTERVAL` | `ros2_comm.cpp` |

## 7. 已知未修（读代码时不要当成自己的回归）

1. **被挡目标静默停住。** `createPlan` 在 MINCO 全拒时仍返回成功 Path，BT 不触发 recovery。修法在规划失败语义，不是调参能好的。
2. **costmap `inflation_radius: 0.20` < 内切半径 0.30。** InflationLayer 会出一圈 lethal、几乎没有 1–98 的代价梯度。MINCO 不靠这个梯度，但恢复/其它插件会。
3. **ROG 窗口 ±5 m vs global_costmap 30 m。** 远处当自由。
4. **包内 README 话题名过时。** `minco_planner/README.md`、`minco_controller/README.md`、部分 `rog_map/README.md` 还写 `/aft_mapped_to_init`、`/cloud_registered_full`。以 `nav2_params.yaml` 为准。
