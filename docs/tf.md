# TF 链路

导航坐标系按 REP-105 的精神拆成 `map / odom / base_link`，再多一个本车特有的 `base_link_fake`。

## 树

```text
map
 └── odom                          动态：odom_localizer（GICP）
      └── base_link                动态：small_point_lio
           ├── base_link_fake      动态 50 Hz：fake_vel_transform
           ├── lidar_link          静态：robot_state_publisher ← URDF lidar_joint（前雷达）
           ├── lidar_back_link    静态：URDF lidar_back_joint，与右侧手性对称（Y 取反、roll 取反）；LIO 不 lookup
           ├── lidar_imu           LIO 根据 IMU 外参推（lookup 失败时发不出 odom TF）
           ├── chassis_link        静态 URDF（与 base_link 重合）
           └── base_footprint      静态 URDF，z = -0.1
```

`use_odom_localizer:=False` 时，`map→odom` 改成 `static_transform_publisher` 的单位变换（建图或纯里程计导航）。

不要同时开 odom_localizer 和其它 `map→odom` 发布器（例如 SLAM Toolbox）。

## 谁发布

| 变换 | 发布者 | 频率 / 性质 | 含义 |
|---|---|---|---|
| `map → odom` | `odom_localizer` 或 static | 默认 20 Hz | 把本次开机的 odom 原点对到建图时的 map |
| `odom → base_link` | `small_point_lio` | 随 LIO | 雷达/IMU 紧耦合里程计，表达在车体 |
| `base_link → base_link_fake` | `fake_vel_transform` | 50 Hz | `q.setRPY(0, 0, -yaw_chassis)`：fake 的 yaw 相对 odom 恒为 0 |
| `base_link → lidar_link` | `robot_state_publisher` | 静态 | 安装外参，当前 URDF `xyz="0.00 -0.15 0.11"` `rpy="0.8389 0 0"` |

Nav2 / costmap / BT 的 `global_frame` 是 **odom**，`robot_base_frame` 是 **base_link_fake**。它们不在 `map` 里规划。`map` 主要给 ROG-Map `prior_map.frame_id: map` 做二维先验投影。

## 各帧干什么

### map

离线建图会话的世界系。`mas2027_nav_bringup/pcd/lab3.pcd` 和 `lab3.pgm` 都在这套坐标里（建图时 map == 那次的 odom）。在线 GICP 拿 live `/cloud_registered`（当前 odom）去对这张 PCD，得到 `map→odom`。

### odom

LIO 开机清零的局部世界。点云 `/cloud_registered`、ROG-Map `frame_id`、两个 costmap、MINCO `frames.map_frame` / `rog_frame` 目前都填 `odom`。

开机后 LIO 会漂。有先验时靠 `map→odom` 把先验墙投对；没有先验时（`use_odom_localizer:=False`）整条导航都在漂移的 odom 里，短距离可以，回环对不齐。

### base_link

车体。`/Odometry.twist` 按 REP-105 在这个轴系。下位机收到的 `/cmd_vel` 也必须在这个轴系，所以 `fake_vel_transform` 要把 Nav2 的速度旋回来。

### base_link_fake

和 `base_link` 同原点、同滚转俯仰，yaw 钉在启动朝向。云台/底盘自旋时，Nav2 看到的「车头」不变，全向平移的 x/y 不会被自旋带跑。

因为 `base_link → base_link_fake` 是 `Rz(-yaw)`，fake 相对 odom 的 yaw 恒为 0，**odom 轴系与 `base_link_fake` 轴系重合**。所以 MPC 输出写在 `odom` 轴上，也满足 Nav2「twist 在 robot_base_frame」的约定。不要在控制器里再乘一次 `Rz(-yaw)`，会双重旋转。

### lidar_link

雷达光学中心。costmap 观测源的 `sensor_frame: lidar_link` 必须填：点云 header 是 `odom`，不填的话距离过滤会以 odom 原点（开机位置）为中心，而不是雷达。

双雷达时 `lidar_link` 仍是右侧原雷达；左侧点在驱动里变过来，LIO 不 lookup `lidar_back_link`。左侧与右侧手性对称：`xyz="0.00 0.15 0.11"` `rpy="-0.8389 0 0"`。详见 [dual-lidar.md](dual-lidar.md)。

## 速度轴系（和 TF 配套）

| 量 | 轴系 |
|---|---|
| `/Odometry.pose` | odom |
| `/Odometry.twist` | base_link |
| `/Odometry_world_fixed.pose` | 与 `/Odometry` 相同 |
| `/Odometry_world_fixed.twist` | base_link_fake（= odom 轴，yaw=0） |
| MPC 返回的 twist / `/cmd_vel_nav*` | odom = base_link_fake |
| `/cmd_vel`（给底盘） | base_link，angular.z 来自 `/cmd_spin` |

`MincoPlanner` 订 `/Odometry`。`bt_navigator`、`controller_server` 和 MPC 插件订 `/Odometry_world_fixed`。Nav2 不变换 odom 的 twist，直接当作 `robot_base_frame` 速度，所以必须把 twist 先旋到 fake 轴。

若把已经旋过的 twist 再喂给 `compensateLeverArm()`（它内部还会按 yaw 旋一次），会双重旋转，表现为横移。现场以插件 configure 日志里打出的 frame 名为准。

## 先验 TF

ROG-Map `prior_map.enable: true` 且 `frame_id: map` 时，`ROGMapROS` 用共享 TF buffer 查 `map ← odom`。查不到就把先验墙投错地方：`lab3.pgm` 的 origin 大约 `(-7.76, -5.97)`，写成 odom 等于把整张场贴到开机原点上。

真车要用先验：先有 `map→odom`（odom_localizer），`frame_id` 保持 `map`。

## 检查命令

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link base_link_fake
ros2 run tf2_ros tf2_echo base_link lidar_link
```

`odom→base_link` 一直超时：LIO 没起来，或 `lidar_link` 静态 TF 还没被 LIO lookup 到。
`map→odom` 一直单位阵且 odom_localizer 在跑：GICP 还没锁（先验 PCD 路径、停车位、点云 QoS）。
`base_link_fake` 没有：`use_fake_vel_transform` 为 False，Nav2 的 `robot_base_frame` 会对不上。
