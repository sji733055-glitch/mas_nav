# 感知、定位、规划、控制：组件原理

按问题拆，而不是按 ROS 包名拆。每个组件只回答一件事。

## 感知：点从哪来

### mid360_driver

Livox MID360 的轻量 UDP 驱动。把设备点云和 IMU 变成 ROS 话题 `/mid360_driver/lidar`、`/mid360_driver/imu`。它不做去畸变、不发 TF。雷达 IP / 本机 `host_ip` 配错时，后面全部静音。双雷达时融合也放这里（按源 IP 配对 + 后→前外参），不换官方驱动、不搬 HERO 的 merge 节点；详见 [dual-lidar.md](dual-lidar.md)。

### Small Point-LIO

紧耦合激光惯性里程计。IMU 预测、点到平面/点到地图约束，输出：

- 位姿：`/Odometry`（header `odom`，child `base_link`）
- 去畸变点云：`/cloud_registered`（已经变到 odom）
- TF：`odom → base_link`（需要先从 URDF lookup 到 `base_link → lidar_link`）

`align_odom_with_gravity: true` 时，会把 odom 的 z 对齐重力，避免车体倾斜把地面扫成一堵墙。盲区球心（`blind` 相关）必须是 **base_link 原点在 lidar_link 中的坐标**；改 URDF `lidar_joint` 后要跑 `scripts/measure_lidar_mount.sh` 重算，否则车身点会被当成障碍。

LIO 是相对定位：开机点就是 odom 原点。开得越久漂得越多，这是后面 odom_localizer 存在的原因。

## 定位：odom 对到 map

### 两层定位

| 层 | 节点 | 输出 | 作用 |
|---|---|---|---|
| 相对 | `small_point_lio` | `odom → base_link` | 高频、平滑，短时可用 |
| 绝对 | `odom_localizer` | `map → odom` | 把当前 odom 钉到建图时的场地上 |

`odom_localizer` 用 small_gicp 把最近几帧 `/cloud_registered` 对到 `mas2027_nav_bringup/pcd/` 里的先验 PCD（当前默认 `lab3.pcd`）。先验云的坐标系就是建图会话的 odom，本仓库把它当作 `map`。不要对这张 PCD 再乘 `T_base_lidar`。

点云是 SensorDataQoS，localizer 必须用同样的 QoS，否则配准源为空。

Nav2 仍然在 **odom** 里跑。`map` 帧的消费者主要是 ROG-Map 的 PGM 先验投影。关掉 localizer（`use_odom_localizer:=False`）等于承认「我不要全局场」，适合无图探索或录建图 bag。

### 离线出先验（不是在线节点）

```text
LIO 绕场 + /map_save（或拷一份已有 PCD）
  --> 放到 mas2027_nav_bringup/pcd/<name>.pcd
  --> 可选 pcd_trans 平移/旋转
  --> pcd2pgm + map_edit    高度切片、修墙，得到 pgm/yaml
```

细节和命令见根 README **离线静态地图**。`pcd2ele` 出高程图，`pcd2esdf` 从 PGM 烘焙 2D 距离场，运行时 MINCO 不吃这两份，ESDF 仍由 ROG 在线算。

## 建图：ROG-Map 在规划器进程里

上游 ROG-Map 是库，不是节点。本仓库的用法：

1. `MincoPlanner::configure()` 里 `new ROGMapROS`，绑到 `planner_server`
2. 它自己订 `/Odometry` + `/cloud_registered`，20 ms 更新一次滑动三维栅格
3. 射线 hit/miss 更新占据概率，可选时间衰减清掉动态障碍
4. ProjectionLayer 把每个 XY 柱分类成 UNKNOWN / FREE / PASSABLE / OCCUPIED
5. FieldLayer 在二维障碍上做有符号 ESDF（`field.enable: true`，`inflation_radius: 0.0`）
6. 可选把 PGM 先验并进投影（只加硬障碍，不清除在线障碍）

MINCO 通过 `MapQueryInterface` 直接查内存，不序列化大地图。

costmap 看不懂 OccupancyGrid 的滑动 origin，也和 best_effort QoS 对不上 StaticLayer，所以用 `layer_value_to_cloud` 把 OCCUPIED 格子变成假点云，喂给原来的 `IntensityVoxelLayer`。z 和 intensity 是合成值，分类已经在 ProjectionLayer 做完了。

滑动窗 `map_size: [10, 10, 1.5]` 只有车体 ±5 m。global_costmap 是 30×30 m，外圈没有观测；`track_unknown_space: false` 时未知当自由，远处障碍要开近了才出现。这是设计限制，不是没接到点云。

`rog_map_node` 做的是同一份 `ROGMapROS`，只是自己 spin。和 `planner_server` 同时跑会有两套滑动原点抢 `/rog_map/layer_value`。

## 规划：搜索给拓扑，MINCO 给可执行轨迹

`MincoPlanner` 实现 `nav2_core::GlobalPlanner`。`createPlan(start, goal)` 主要是把目标塞进内部状态；真正的搜索、优化、重规划在 FSM 定时器里跑（约 20 Hz，与 `minco_optimizer.opt_freq` 对齐）。

### 两种模式

| 模式 | 全局引导 | 适用 |
|---|---|---|
| `PRIORMAP` | Nav2 global_costmap + SMAC 2D | 有先验/有较大 costmap 时。当前 `nav2_params.yaml` 默认 |
| `EXPLORATION` | 在 ROG-Map 窗口内搜，目标出窗则裁到边界 | 无图、只靠在线滑动地图 |

行为树 XML 的 `planner_id` 必须等于插件名 `MincoPlanner`。写成 `GridBased` 时 configure 不会报错，要到发目标才失败。

### 算法步骤

1. 坐标系归一化到 `odom`
2. 离散搜索一条折线（SMAC，可加 ESDF 代价让初值离障）
3. 按 `lookahead_dist`（6 m）截局部段，稀疏化，分配时间
4. MINCO：分段多项式，L-BFGS 优化形状与时间。惩罚项包括净空（ESDF）、速度/加速度上限、总时长
5. `validateTrajectory`：硬阈值 `collision_dist`（0.30 m）必须小于优化软目标 `safe_dist`（0.40 m），否则会出现「惩罚为 0 但校验判碰撞」
6. 通过则发 `/opt_path`。跟随中 20 Hz 监视剩余轨迹（净空 `collision_dist + max(v·replan_react_time, monitor_margin)`），默认 5 Hz 强制 `ReplanLocal`；轨迹不安全且重规划失败则发急停，不再跟已经撞上的旧轨迹

`nav_msgs/Path` 仍然返回给 BT，所以「规划成功但车不动」很常见：Path 有两个点，`/opt_path` 没有。被挡在未知区后面的目标尤其如此（`unknown_as_occupied: true`），BT 不会进恢复。

## 控制：MPC 跟踪轨迹，自旋另走

`MincoMpcController` 实现 `nav2_core::Controller`。它几乎不看 BT 传来的折线，而是订 `/opt_path`。

- 状态 `[px, py, yaw]`，控制 `[vx, vy, wz]`
- 预测时域 `lookahead_time / dt = 0.5 / 0.05 = 10` 步
- qpOASES 解有约束 QP；本车 `vx/vy` ±3.0 m/s（动态障碍先从 4.0 降下来），`omega` 锁死 0
- 输出 twist 在 odom 轴（= base_link_fake 轴）
- 控制频率 `controller_frequency: 20` Hz（不是上游的 100 Hz），和 smoother、UDP 发送能力匹配

全向底盘 + 自旋扫描不能让 Nav2 把「当前车头」当路径切向，所以：

1. 规划/控制在 `base_link_fake`（yaw 固定）
2. `fake_vel_transform` 把速度旋回 `base_link`，并叠 `/cmd_spin`
3. Humble 的 cmd_vel 没有时间戳，用 `/local_plan` 和 `/Odometry` 做 ApproximateTime，才能取到匹配时刻的 yaw；没有 `local_plan` 会掉进 0.5 s 超时分支，用最新 odom yaw，自旋时会转错

`ros2_comm` 把 `/cmd_vel` 打成 UDP。发送硬上限 50 Hz：宿主机按「每轮一个数据报」消费，发太快内核队列里全是旧指令，表现为秒级恒定延迟。depth 1 保证积压时丢旧留新。

## Nav2 在这条链里还干什么

MINCO 不读 local_costmap 做轨迹优化。Nav2 仍负责：

- 生命周期（configure → activate）
- 行为树调度（3 Hz 重规划 + FollowPath）
- 两个 costmap：恢复、碰撞、RViz，PRIORMAP 的 SMAC 初值
- `velocity_smoother` 加速度限制和超时停车
- `behavior_server` 的 BackUp / Wait（和规划器内部 recovery 是两套）

没有 `map_server` / AMCL。costmap `rolling_window: true`，`global_frame: odom`。
