# 独立导航执行器迁移记录（2026-09-12）

## 目标

从 Nav2 server 编排迁出一条独立的 ROGMap、MINCO、MPC 导航链路，并参考
`HWSentryNav26/nav_executor` 的职责划分：规划、控制和可视化均由独立执行器承担。

目标运行链路为：

```text
/cloud_registered + /Odometry
  -> ROGMapROS / MapQueryInterface
  -> PathPlanner / MINCO
  -> /opt_path
  -> PathExecutor / MPC
  -> /cmd_vel
```

## 已完成

### 独立启动与 TF

- 新增 `mas2027_nav_bringup/launch/nav_executor_launch.py`。
- 新增 `tf_maintainer`，在独立启动模式下统一发布 `odom -> base_link` 与
  `map -> odom`，避免 LIO 与定位器重复发布同一 TF 边。
- `small_point_lio` 增加 `publish_odom_tf` 参数；`odom_localizer` 增加
  `tf.publish_direct` 与 `tf.transform_topic` 参数。

### ROGMap 与规划

- executor 中的 `PathPlanner` 已拥有规划生命周期节点、`ROGMapROS`、
  `MapQueryInterface` 与 MINCO 规划调用。
- 新增 `RogMapQueryAdapter`，封装二维可通行性以及 ESDF 距离/梯度查询。
- 目标点会先经 ROGMap 可通行性检查；不可通行、未知或地图外目标不会进入规划。
- `MincoPlanner` 在进程中已有 `MapQueryInterface` 时复用它，不再新建第二份
  ROGMap。

### 控制与可视化

- `PathExecutor` 已接管 MPC solver 的所有权与 `solve()` 调用；求解算法仍为
  原 qpOASES MPC。
- 接收到 `/opt_path` 时，executor 发布：
  - `/nav_executor/global_path` (`nav_msgs/Path`)
  - `/nav_executor/debug/minco_trajectory` (`visualization_msgs/Marker`，速度蓝到红)
- 两个可视化发布者采用 `transient_local` QoS。
- 新增 `mas2027_nav_bringup/rviz/nav_executor_view.rviz`，显示注册点云、TF、里程计、
  全局路径和 MINCO Marker；独立启动默认加载该视图。

### 包级依赖迁移

- `mas2027_nav_executor` 不再声明对 `minco_controller`、`minco_planner` ROS 包的
  构建/运行依赖。
- MPC solver、qpOASES 与 MINCO 核心当前作为 executor 的私有静态目标编译。
- 为保持迁移过程可构建，私有目标暂时仍引用
  `mas2027_planner/minco_controller` 和 `mas2027_planner/minco_planner` 下的源码路径；
  尚未物理移动源码或删除旧包。

## 验证

以下构建已成功完成：

```bash
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  --parallel-workers 4 --packages-select mas2027_nav_executor

colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  --parallel-workers 4 --packages-select mas2027_nav_executor mas2027_nav_bringup
```

并已通过：

```bash
ros2 launch mas2027_nav_bringup nav_executor_launch.py --show-args
```

未启动真实硬件，也未向底盘发送测试速度指令。

## 当前未完成项

`PathExecutor` 目前只拥有 MPC solver。以下逻辑仍在
`mas2027_nav_executor/src/nav_executor_node.cpp`：

- `/opt_path` 到 `odom` 的 TF 转换；
- 最近点搜索、时间插值与 MPC 参考轨迹构造；
- 轨迹超时后的零速度保护；
- map/odom 系控制量到车体系速度的转换；
- `/cmd_vel` 发布。

因此主节点尚未只剩事件调度。下一次重构应将上述逻辑作为一个完整、可构建的
`PathExecutor::compute()` 单元迁移，避免保留会无条件输出零速度的半成品接口。

## 删除策略

用户已授权最终废弃 Nav2 与旧 `mas2027_planner` / `minco_controller` 包。但在
MINCO/MPC 源码已物理迁入 executor、完整控制循环迁移并经构建及上车验证前，不能
删除这些目录，否则独立 executor 会立即失去源码输入。删除时还需同步移除旧 Nav2
启动入口与 package 依赖。
