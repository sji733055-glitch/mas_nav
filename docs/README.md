# 独立导航说明

当前唯一入口是：

```bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py
```

## 组件职责

| 组件 | 输入 | 输出 | 职责 |
|---|---|---|---|
| `small_point_lio` | MID360 点云与 IMU | `/Odometry`、`/cloud_registered` | 实时里程计与配准点云 |
| `odom_localizer` | `lab3.pcd`、实时点云 | 定位结果 | 先验点云配准 |
| `tf_maintainer` | LIO 与定位结果 | `map→odom→base_link` | 唯一 TF 发布者 |
| `ROGMap` | `/Odometry`、`/cloud_registered` | 进程内查询接口 | 在线占据、距离和梯度查询 |
| `PathPlanner` | `/goal_pose`、ROGMap | `/opt_path` | A* 搜索、MINCO 优化与安全重规划 |
| `PathExecutor` | `/opt_path`、`/Odometry` | `/cmd_vel` | MPC 轨迹跟踪和失败制动 |
| `terrain_map_server` | `lab3_terrain.msgpack` | `/cost_map`、`/direction_map` | HW 地形图可视化 |

规划器固定使用 ROGMap 的 `EXPLORATION` 模式。HW 地形图目前不参与轨迹优化，
因此 RViz 里的代价图和方向图是观察通道，不是规划输入。

## 关键话题

- 目标：`/goal_pose`
- 在线点云：`/cloud_registered`
- 里程计：`/Odometry`
- 优化轨迹：`/opt_path`
- 底盘速度：`/cmd_vel`
- 自旋叠加：`/cmd_spin`
- 地形代价：`/cost_map`
- 地形方向：`/direction_map`
- RViz 路径：`/nav_executor/global_path`
- RViz 彩色轨迹：`/nav_executor/minco_trajectory`

## TF 约束

```text
map → odom → base_link → lidar_link
```

`small_point_lio.publish_odom_tf=false`，`odom_localizer.tf.publish_direct=false`；两者只
提供数据，`tf_maintainer` 统一发布动态 TF。控制器以 `odom` 为全局控制坐标系，输出
可通过 `node.output_in_body_frame` 切换到车体坐标系。

## RViz

默认视图为 `mas2027_nav_bringup/rviz/nav_executor_view.rviz`。其中：

- `HW Cost Map` 显示 `/cost_map`；
- `HW Direction Map` 显示 `/direction_map`；
- `Global Path` 显示 `/nav_executor/global_path`；
- `MINCO Trajectory` 显示 `/nav_executor/minco_trajectory`。

独立执行器没有 `Global Costmap` / `Local Costmap`，这是预期行为。

## 快速检查

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic echo /cost_map --once
ros2 topic echo /direction_map --once
ros2 topic echo /opt_path --once
ros2 topic echo /cmd_vel --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

若点击目标后没有轨迹，依次确认 ROGMap 已收到点云和里程计、目标位于当前滑窗可达
范围、`/opt_path` 有发布。若 MPC 输入过期、TF 转换失败或求解失败，执行器会发布零速。
