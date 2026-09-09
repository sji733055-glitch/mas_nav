# mas_nav_2027 学习文档

这是 RoboMaster 2027 哨兵导航栈的阅读入口。根目录 `README.md` 讲环境、建图与离线地图流程；这里讲**在线导航时系统怎么转起来**。

当前主启动：

```bash
ros2 launch mas2027_nav_bringup rm_navigation_small_point_lio_launch.py
```

一句话：MID360 点云进 Small Point-LIO，得到 `odom → base_link` 和 `/cloud_registered`；`MincoPlanner` 在 `planner_server` 进程里建一份 ROG-Map（ESDF + 二维投影），优化轨迹发到 `/opt_path`；`MincoMpcController` 跟踪它，速度经 `fake_vel_transform` 旋到车体系，再由 `ros2_comm` UDP 下发到底盘。

## 文档地图

| 文档 | 内容 |
|---|---|
| [学习顺序](learning-order.md) | 建议读代码的顺序，以及每个包/关键文件干什么 |
| [数据链路](data-flow.md) | 话题从雷达到轮子怎么走，谁订阅谁 |
| [TF 链路](tf.md) | 坐标系树、谁发布、和速度轴系的关系 |
| [组件原理](components.md) | 感知、定位、建图、规划、控制各自解决什么问题 |
| [排查定位](troubleshooting.md) | 常见症状 → 该看哪一段、哪种误判 |
| [双 MID360 融合](dual-lidar.md) | 硬件接线、PTP、驱动内融合、外参与落地顺序；当前仓库仍是单雷达，文中「拟改」尚未合入 |
| [双 MID360 融合](dual-lidar.md) | 硬件接线、PTP、驱动内融合、外参与落地顺序；当前仓库仍是单雷达，文中「拟改」尚未合入 |

## 读代码前先记住的三件事

1. **导航时没有独立的 `rog_map_node`。** ROG-Map 是 `MincoPlanner` 在 `planner_server` 里 new 出来的库对象。`rog_map_node` 只用于不跑 Nav2 时单独调建图，不能和导航同时开。
2. **MINCO 不靠 `nav_msgs/Path` 开车。** `createPlan()` 返回的 Path 只喂行为树和 RViz；真正给控制器的是 `/opt_path`（`interfaces/msg/MpcPositionCommand`）。
3. **Nav2 跑在 `base_link_fake` 上。** 这个坐标系和 `base_link` 同原点，但 yaw 钉死在启动方向，用来把全向底盘的平移和车体自旋解耦。车体自旋走 `/cmd_spin`，不走 MPC 的 `omega`。

改参数几乎总是改这两份 YAML，不要在包内另起一份：

- `mas2027_nav_bringup/config/nav2_params.yaml` — Nav2、MINCO、ROG-Map（`MincoPlanner.rog_map` 是全仓库唯一一份 ROG-Map 参数）
- `mas2027_nav_bringup/config/small_point_lio_params.yaml` — 雷达驱动、LIO、`fake_vel_transform`
