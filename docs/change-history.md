# 修改历史

本文件记录由开发任务产生的代码、配置、脚本、资源和文档变更。新记录追加在最上方，不改写旧记录。

## 2026-09-12 — RViz 增加 HW 地图显示项

- Nav2 与独立 nav_executor 两份 RViz 视图均新增默认启用的 `HW Cost Map`，直接显示 `/cost_map`。
- 两份视图均新增默认关闭的 `HW Direction Map`，需要时可在 Displays 面板勾选并显示 `/direction_map`。
- `nav_executor_launch.py` 现在同步启动 `terrain_map_server`；该模式不启动 Nav2 Costmap server，因此没有 Global/Local Costmap 话题。
- 验证：RViz 配置 YAML 可解析并随 `mas2027_nav_bringup` 安装；未启动图形界面检查渲染效果。

## 2026-09-12 — 固化修改记录规则

- 新增仓库级 `AGENTS.md`：每个产生文件改动的任务在交付前必须更新本文件。
- 将本文件加入文档索引。
- 验证：检查规则文件与文档链接；未涉及运行时行为。

## 2026-09-12 — HW 地形代价接入规划与控制

- `terrain_map_server` 发布的 `/cost_map` 接入 Nav2 全局和局部 `StaticLayer`；关闭三值化以保留连续膨胀代价，供 SMAC 全局搜索使用。
- `MincoMpcController` 增加前视代价采样：普通代价区渐进降速，致命代价区停车；参数位于 `FollowPath.terrain_cost_control`。
- 主导航启动文件自动启动 `terrain_map_server`，默认加载 `lab3_terrain.msgpack`。
- 验证：`map_server`、`minco_controller`、`mas2027_nav_bringup` 编译通过；launch 可解析；`/cost_map` 实测为 reliable + transient-local。
- 未验证：未启动完整真车导航，未向底盘发送速度。

## 2026-09-12 — HW map_server 首版移植

- 新增 `interfaces/msg/navigation/CostMaps.msg` 与 `map_server` ROS 2 包。
- 新增 PGM/YAML 到 HW msgpack 地形格式的转换脚本，并从 `lab3.pgm` 生成 `lab3_terrain.msgpack`。
- 当前转换仅包含 `FLAT` 和 `OBSTACLE`；PGM 无法推导坡道、台阶及方向语义。
- 验证：消息包与 map_server 编译通过；节点成功加载 770×347、0.05 m/px 的 lab3 地图并发布地图话题。
