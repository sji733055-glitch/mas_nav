# 隧道、坡道、起伏路的区域控制

本文为早期实现记录；当前标签、mode 和协议以
[`mas2027_nav_executor/docs/region_control.md`](../mas2027_nav_executor/docs/region_control.md)
为准。以下历史编号和方向图描述不适用于当前运行配置。

导航侧实现日期：2026-09-24。

地图标签：`SLOPE=2`、`TUNNEL=7`、`UNDULATING=8`。`map_server` 发布原始
`/terrain_label_map` (`mono8`)；坡道方向仍取 `/direction_map`。当前仓库中的
terrain msgpack 未标出这三种区域，地图制作端需要写入对应原始标签。

`/opt_path` 到达导航节点后，`annotateRegions()` 沿最终轨迹以半个地图栅格的间距采样，
将连续同类标签转换为 `RegionSegment`。每段记录 `prepare_s`、`active_s`、
`commit_s`、`enter_s`、`exit_s`、`release_s`、mode 与速度参数，距离均为轨迹累计弧长。
同 mode 的重叠控制窗会合并；不同 mode 的控制窗重叠时拒绝该轨迹，避免在两种模式间
无确定规则地抢占。区域段与轨迹 ID 在节点内成对保存。

`RegionController` 以机器人当前位置投影到轨迹并保持进度单调。`PREPARING` 先应用
该区域速度/加速度上限并逐周期过渡，`ARMED` 开始下发 mode，`COMMITTED`、`INSIDE` 和
`RELEASING` 继续保持，超过 `release_s` 才恢复普通 mode。阶段和当前 label、
mode、进度发布到 `/nav_executor/region_status`。新轨迹重新建立区域段；没有有效
标注时停止下发非零速度，并保留上次特殊 mode。规划端的 MINCO 分段速度上限也
覆盖 `prepare_distance` 到 `release_distance` 的区域影响窗；执行端再限制实际
平移速度和加速度。

参数在 `mas2027_nav_executor/config/region_control.yaml`：每类区域单独配置
`mode`、`max_speed`、`max_acceleration`、`prepare_distance`、
`activation_distance`、`commit_distance` 和 `release_distance`。mode 默认 `-1`，
表示尚未与下位机确定编号；轨迹经过未配置区域会被拒绝。距离满足
`prepare_distance >= activation_distance >= commit_distance >= 0`。
`speed_blend_rate` 和 `acceleration_blend_rate` 控制 MPC 约束的逐周期过渡；
到达 `commit_s` 时仍未收敛的上限会钳到区域目标值。

`/nav_executor/chassis_cmd` 把 `vx`、`vy`、`mode` 放在同一 ROS 消息中；
`ros2_comm` 消费该消息而不再消费 `/cmd_vel`。其 `mode_transport` 有三种值：

| 取值 | UDP 下行内容 | 适用条件 |
|---|---|---|
| `legacy`（默认） | 原 9 字节 `vx/vy/nav_state`，mode 不出桥 | 当前 mas_vision 可接收，保留原速度链路 |
| `nav_state` | 原 9 字节，`nav_state` 承载 mode | 下位机明确把该字节当 mode，且编码已约定 |
| `extended` | 10 字节 `vx/vy/nav_state/mode` | mas_vision 同步扩展接收结构、长度校验和串口转发之后 |

目前核对到的 mas_vision `ROS2_RECV_PACKET` 只有 `vx`、`vy`、`nav_state`，
其接收代码只接受 9 字节 payload，串口层仅透传 `nav_state`。所以在确认硬件契约前
保持 `legacy` 默认值。要让区域 mode 实际到达下位机，需要确认普通/隧道/坡道/
起伏路四种 mode 编码，以及 mode 是否直接占用现有 `nav_state`；若不是，须在
mas_vision 和下位机侧配套扩展协议。本轮不改 mas_vision。

现阶段没有下位机 mode 生效回执。`COMMITTED` 是位置阶段，不能证明硬件已切换。
有回执后可在到达 `commit_s` 前增加确认门；未确认时停车，不进入区域。
