# 修改历史

本文件记录由开发任务产生的代码、配置、脚本、资源和文档变更。新记录追加在最上方，不改写旧记录。

## 2026-09-15 — 新增导航调参与修复全过程文档

- 新增 `docs/nav_tuning_2026-09-15.md`：把当日从「车不动 / 一卡一卡」到「行驶流畅」、以及从「只能近处规划」到「可规划远处目标」的全部因果链与改动按**因果**重组（本文件按时间顺序、逐条含验证边界，两者互补）。
- 文档结构：① 卡顿/不动（近距盲区幻影障碍、速度相关净空的极限环、线速度死区、未观测语义、投影窗口与虚拟地面矛盾、动态障碍层、两次失败尝试、仍未处理的一处阈值不一致）；② 远处规划（`isFree` 把 255 当不可通行、Kino 无解降级、`allow_unknown` 不同源、降级改用地形图查询、先验图融合启用后关闭）；③ 与上游 `navi_minco_bit` / 旧工程 `mas_nav_2027` 的对齐结果（含**唯一剩下的实质差异：方向约束层**，以及两处有意保留的本工程取值）；④ 仍未解决/未验证清单；⑤ 按文件的改动清单。
- 仅新增文档，未改动任何代码、配置或参数。
- 验证：文档中每条结论的出处（现场日志文件名与关键行、源码文件与函数、实测数值）均在正文标注，数值取自当日实车日志与配置核对。**未验证**：文中"与上游对齐"的判断基于配置与源码的静态比对，未在 docker 内实跑对照；文档描述的是截至 2026-09-15 的状态，不随后续改动自动同步。

## 2026-09-15 — RViz 增加真正的全局折线显示（原来的 "Global Path" 发的其实是 MINCO 轨迹）

- 问题：RViz 里那个叫 `Global Path` 的显示挂在 `/nav_executor/global_path` 上，但该话题由
  `publish_trajectory_visualization()`（`nav_executor_node.cpp`）用 `trajectory.cmds` 填出来，
  发的是 **MINCO 轨迹**，不是全局搜索结果。真正的 SMAC 折线（`MincoPlanner::latest_global_path_`）
  从来没发布过 —— 也就是说「搜索给出的拓扑引导」在现场一直看不见，这是排障时最需要的一条信息。
- 修改（6 个文件）：
  1. `minco_planner.{hpp,cpp}`：新增 `copyLatestGlobalPath(std::vector<PoseStamped> &) const`，
     在 `path_mutex_` 下一次锁里连数据一起取出（避免调用方分两次加锁看到不同快照）。
  2. `path_planner.{hpp,cpp}`：`PathPlanner::copyLatestGlobalPath()` 透传到 `MincoPlanner`。
  3. `nav_executor_node.cpp`：新增 `/nav_executor/global_plan`（`nav_msgs/Path`）与
     `/nav_executor/debug/global_plan`（`visualization_msgs/Marker`，LINE_STRIP + 终点球）
     两个发布器，以及一个默认 5 Hz 的 `global_plan_timer_`。折线画在 z=0.03 抬高一点，
     避免与 z=0 的代价图/规划约束栅格闪面；线宽/颜色/频率来自 `node.visualization.global_plan_*`。
     没有可用折线时发一条空 Path 并 `DELETE` 两个 Marker，避免 RViz 上留着一条已经失效的旧线。
  4. `config/node_params.yaml`：新增两个话题名与四个样式参数（默认线宽 0.15 m、青色、5 Hz）。
  5. `nav_executor_view.rviz`：新增 `Global Plan (SMAC search, thick)` 与 `Global Plan (Path)`
     两项（放在显示列表最后 = 最后渲染 = 画在最上层）；把原来误名的 `Global Path` 改名成
     `MINCO Trajectory (Path)` 并调暗调细（0.6 alpha / 0.04 m），避免和真正的全局折线抢视线。
  6. `test/smoke_goal.py`：新增全局折线的端到端断言。
- 顺带修掉一个真实缺陷：Marker 发布器原本用 `rclcpp::QoS(1).transient_local()`。
  `transient_local` 的 durability 缓存只保留最后 depth 条，而折线与终点球是**两条独立消息**，
  depth=1 时**后打开 RViz 只能拿到终点球、看不到折线**。已改为 `QoS(10)`，并且 `publish_global_plan()`
  改为每个定时器周期都重发（而不是只在内容变化时发），这样 RViz 在本节点之后启动也能拿到完整折线。
- 验证：
  - `mas2027_nav_executor` Release 构建通过；执行器 8 项 CTest 全部通过。
  - `test/smoke_goal.py` 普通与 `--preempt` 两个变体均通过，新增断言覆盖：话题确实有消息、
    frame 为 `odom`、坐标全部有限、末点与精确目标重合（1e-6）、Marker 与 Path 点数一致、
    线宽 ≥ 0.10 m 且不透明、起点贴着机器人。
  - **判别性断言**：折线相邻点间距必须落在格点尺度（SMAC 直接在地图 0.05 m 格上扩展，
    故为 0.05 或 0.0707 m）。这条专门防「以后有人把这条线又接回 MINCO 轨迹」——
    轨迹按 dt 采样，间距小一个量级，一接错就立刻失败。实测普通版 52 轨迹点 / 48 折线点，
    preempt 版 54 / 56，两者点数不同也印证是两份数据。
  - 本次实测输出：`goal smoke passed: 52 trajectory poses, 48 global plan poses, marker width 0.150 m`。
  - RViz 配置用 `yaml.safe_load` 解析通过，四个相关显示项与 `Fixed Frame: map` 均正确。
- 未验证：
  - **未在真实 RViz 图形界面里看过**（沙箱内无法起 GUI）——线宽 0.15 m / 青色是否够醒目、
    0.03 m 抬高是否完全消除闪面，需要上车目视确认。
  - 未跑实车；`node.visualization.global_plan_publish_hz` 等参数只做了默认值路径的验证，
    没有做运行时 `ros2 param set` 实测。
  - 换目标瞬间会先发一条空 Path 去清 RViz，实车点击密集时是否会出现肉眼可见的一闪未评估。

## 2026-09-15 — 全局主搜索换成从 mas_nav_2027 移植的 SMAC 2D（含 ESDF 势场软代价）

- 决策：用户选定「只换搜索算法」——把 `mas_nav_2027` 的 `smac_search/` 移植进来替换现有 NavFn 式 `Astar` 主搜索，地图仍是本工程的地形 msgpack，其余链路不动；并选定「SMAC 失败直接失败并打诊断日志，不回退 A*」。
- 背景（上一条已详述）：上一条把主搜索从 Kino 换成了纯栅格 A\*，但那是 `Astar`（NavFn 波前），只是「与 SMAC2D 同口径」，并非同一个算法；旧工程实际跑的是自写简化版 SMAC 2D，且带 `smac_2d.use_esdf_cost` 的距离场软代价，本工程一直没有。
- 移植范围（**不是整包搬运**）：旧工程 `src/smac_search/` 共 1627 行，其中 `node_2d.cpp`（151 行）与 `collision_checker.cpp`（167 行）全仓库只有自己 include 自己，是死代码，未移植；实际搬入 `constants.hpp` / `types.hpp` / `smac_planner_2d_simple.{hpp,cpp}`，算法本体逐行保留（8 邻域扩展、octile 启发式、`tolerance` 到点判定、膨胀代价因子、ESDF 势场软代价、SoA 搜索缓冲）。
- 修改（7 个文件，新增 4 个）：
  1. 新增 `include/mas2027_nav_executor/path_planner/search/smac/{constants.hpp,types.hpp,smac_planner_2d_simple.hpp}` 与 `src/path_planner/search/smac/smac_planner_2d_simple.cpp`。相对旧工程的三处必要改动：去掉 `nav2_costmap_2d::Costmap2DROS` 成员与两个 `configure` 重载，改为 `configure(rclcpp::Logger)`（地图几何全部取自 `MapQueryInterface`，`createPath()` 本来每次都会刷新 origin/resolution/size）；代价常量改从本目录 `constants.hpp` 取（取值与 Nav2 相同 255/254/253/252）；`types.hpp` 删掉只服务 Nav2 平滑器的 `SmootherParams`（它带来 `rclcpp_lifecycle` 依赖）。命名空间为 `mas2027_nav_executor::smac`。
  2. `global_path_searcher.{hpp,cpp}`：`configure()` 增加 `smac`/`use_smac` 形参；`makePlanOnQuery()` 改为旧工程同构的 `if (use_smac_ && smac_) {...} else { ...Astar... }`，SMAC 分支失败即 `return false` 并打 `SMAC 2D failed to find path` + 端点诊断。`planner=%s` 的日志字段由硬编码 `"Astar"` 改为 `(use_smac_ && smac_) ? "SMAC2D" : "Astar"`（旧工程就是这么写的；不改会让现场排查误判实际用的是哪条搜索）。
  3. `minco_planner.{hpp,cpp}`：新增 `smac_planner_` 成员与 `use_smac_` / `smac_use_esdf_cost_` / `smac_esdf_weight_` / `smac_esdf_decay_` / `smac_esdf_max_cost_` 参数；构造点与 `setParameters(allow_unknown_, 1000000, tolerance_)` 取值对齐旧工程 `minco_planner.cpp:543-548`；`rebuildModeDependentQueries()` 与 `cleanup()` 同步。`onSetParameters`：`use_smac` 归入 configure-time（结构开关），`smac_2d.*` 四项支持在线调并即时下发到 `smac_planner_`。
  4. `config/planner_params.yaml`：新增 `use_smac: true` 与 `smac_2d.{use_esdf_cost,esdf_weight,esdf_decay,esdf_max_cost}`，取值与旧工程 `nav2_params.yaml` 的 `smac_2d` 段一致（true / 1.0 / 0.8 / 0.5）。
  5. `CMakeLists.txt`：planner 库加入新源文件；新增测试目标 `test_smac_planner_2d_simple`。
  6. 新增 `test/test_smac_planner_2d_simple.cpp`（4 项断言组）。
  7. `README.md`：在线链路与配置说明由「全向 Kino A\*」改为「SMAC 2D」，并补上 Kino 已移出关键路径、方向层当前全 0 的说明（上一条改了代码但没同步 README）。
- **顺带修掉一个移植过来的真实缺陷**：旧工程 `setMap()` 里有一句 `planning_id_ = 0u;`。旧工程只在 configure 时调一次 `setMap`，所以从未暴露；本工程沿用「每次搜索前把地图交给 SMAC」的接线方式，每次搜索都会调，于是归零后 `createPath()` 的 `++planning_id_` 又重新得到 1，与上一次搜索留在 `visited_/closed_/parent_` 里的 1 撞号。后果不是「搜索变慢」而是**静默返回陈旧路径**：`createPath` 末尾的失败判定是 `if (!goal_reached && closed_[goal_index] != planning_id_)`，goal 格带着上一轮的 id 使判定被跳过，于是直接顺着上一轮留下的 `parent_` 回退出上一条路径（实测 `iterations=1`、路径与上一轮逐点相同）。已改为 `planning_id_` 单调递增、`setESDFQuery` 变更时只失效 `esdf_cost_cache_id_`，并在代码注释里记下成因。
- 验证：
  - **回归测试确实是有效的（先证实再修）**：把 `planning_id_ = 0u;` 临时加回去重建，`test_smac_planner_2d_simple` 在第 224 行断言失败（`assert(!search(...))`），即「缺口封死后仍返回上一轮那条穿墙的陈旧路径」；恢复修复后通过。最初写的「两次搜索路径应逐点相同」版本抓不到该缺陷（陈旧路径恰好等于上一轮路径，比对通过），已改成「地图变化后必须基于新地图搜索或失败」。
  - `mas2027_nav_executor` Release 构建通过。
  - 执行器 8 项 CTest 全部通过（原 7 项 + 新增 `smac_planner_2d_simple`），`ROS_LOG_DIR` 指向工作区内。
  - **端到端离线冒烟**（`test/smoke_goal.py`，真实 `lab3_terrain.msgpack` + 真实 config，`ROS_DOMAIN_ID=232/230/229`）：日志出现 `SMAC 2D global search enabled: use_esdf_cost=true weight=1.000 decay=0.800 max_cost=0.500 tolerance=0.300` 与 `Terrain global search input: planner=SMAC2D`，`goal smoke passed: 52 trajectory poses`；`--preempt` 变体（连续两次全局搜索、复用同一个 SMAC 实例）`passed: 54 trajectory poses`；`use_smac: false` 的临时参数副本确认回退分支可用（日志 `SMAC 2D disabled (use_smac=false)`、`planner=Astar`、`passed: 52`）。
- 未验证：
  - **未跑实车**——SMAC+ESDF 偏置对实际路径形态、通行时间、窄道通过率的影响未实测；出厂值 1.0/0.8/0.5 是软偏置，单测里为了可观测用的是放大权重（100/0.8/无上限），两者不等价。
  - 旧工程的膨胀梯度（Nav2 `InflationLayer` 0.55/cost_scaling 2.5）**未移植**：本工程地形图经 `TerrainMapQuery` 折成二值 0/254，`evaluateInflationCost()` 对自由格恒返回 1.0，因此 `cost_penalty` 目前不起作用，真正改变路径的只有 ESDF 项。路径贴墙程度是否可接受需现场看。
  - 本工程每次全局搜索都会 `setMap()`（旧工程只在 configure 时调一次），虽然已按幂等实现，但高频调用下的开销未测。
  - `smac_2d.*` 的在线改参路径（`onSetParameters`）只做了代码与日志检查，未做 ros2 param set 实测。

## 2026-09-15 — 全局搜索改为「地形图纯栅格 A* 主搜索」（对齐 mas_nav_2027 的 SMAC2D），Kino 移出关键路径

- 决策：用户选定"把地形图纯栅格 A\* 提为主搜索，对齐 mas_nav_2027 的 SMAC"（上一条记录末尾留的"待用户决定"项即此项，现予落地；上一条的帧换算修复保留不动）。
- 背景（上一条已详述）：本工程 `planner_mode_context.cpp` 删掉 `PRIORMAP` 分支后强制 `EXPLORATION`，全局搜索换成速度格点 Kino（`searchOmniKinoPath`），纯栅格 A\* 只剩降级这一条路；实测 Kino 在真实 lab3 图上 3 m 起大量方向无解、5 m 后几乎全灭，每次失败烧光 50 000 次扩展预算、耗时 0.76~1.03 s。
- 修改（3 个文件）：
  1. `global_path_searcher.cpp`：`planExploration()` 的地形分支里，**主搜索**改为 `TerrainMapQuery`（= `terrain_->planningConstraints()`：静态地形 + 当前动态层）上的 `makePlanOnQuery()`，即纯栅格 A\*，与旧工程 SMAC2D 同口径；删掉 Kino 调用与其降级分支（降级逻辑已无意义——A\* 比 Kino 更宽松且完整），并删掉只服务于 Kino 的 `dynamic_free` 包装（含 ROGMap 滑窗判据；旧工程在 PRIORMAP 模式下同样不把 ROGMap 用于全局搜索）。`cancel_checker` 上提到函数开头，地形主搜索与无地形回退分支共用（此前降级调用传的是空 checker）。动态层就绪检查保留（`TerrainMapQuery` 只在 `dynamic_snapshot_` 存在时才并入动态障碍，故仍 fail closed）。**路径输出的 map→odom 换算保留**（上一条修的帧错配）。
  2. `global_path_searcher.hpp` / `minco_planner.cpp`：`plan()` 与 `planExploration()` 去掉已无消费者的 `start_velocity` / `max_speed` / `max_acceleration` 形参，调用点同步（`getCurrentSpeed()`、`minco_config.max_vel/max_acc` 不再传）；注释说明运动学输入已不需要。
- 未改动：`Astar`、`TerrainMapQuery`、`makePlanOnQuery`、Kino 模块本体与 `test_omni_kino_astar`（保留备用，现已无生产调用点）、ROGMap 与全部 `planner_params.yaml` 参数、无地形时的 ROGMap 回退分支。
- 验证：
  - **探针实测（`.scratch/kino_probe/probe_astar.cpp`，复用 `makePlanOnQuery` 的同一套 A\* 循环与端点投影，lab3.pgm + direction 全 0）**：对 Kino 失败的那批远处目标，A\* 全部给出路径且耗时 **1.0~13 ms**——d=5.0/45° → 120 点 4.1 ms、d=6.0/0° → 151 点 4.1 ms、d=7.0/0° → 151 点 4.2 ms、d=8.0/0° → 161 点 2.4 ms、d=9.0/0° → 181 点 1.9 ms（同批目标 Kino 分别耗时 0.76~1.03 s 后失败）。仍为 0 ms 即失败的样本是目标落在墙里或地图外（如 180° 方向超过 4 m 即出界），与搜索能力无关。
  - `mas2027_nav_executor` Release 构建通过（12.5 s）；执行器 7 项 CTest 全部通过（`ROS_LOG_DIR` 指向工作区内）。
- 未验证：**未跑实车**——远处目标能否稳定规划并真的开出去未实测；`Terrain` 主搜索在终端的新签名是 `[MincoPlanner] Terrain global search input: ...`（成功）与 `Terrain Astar failed to find path` + 端点诊断（失败），下次现场以它为准计数；A\* 路径不含速度/加速度可行性，是否会让 MINCO 的收敛率变化未评估；掉头/窄缝等 Kino 原本能给出运动学种子的场景是否退化未知（若退化，需把 Kino 作为"近处可选精修"重新接入）。

## 2026-09-15 — 降级路径补做 map→odom 换算（修「降级成功但车不动」）+ Kino 远处目标能力实测

- 现象（现场 20:50 运行 `mas2027_nav_executor_node_139128_1789476613275.log`，408 行）：点 4 个远处目标（odom 系 5.77/4.57/4.6x m）后，**每个目标都在约 1.1 s 后出现一条** `No acceleration-feasible route satisfies terrain and current dynamic obstacles; falling back to terrain-map A* direct plan`；降级 A\* 本身**成功**（全程没有 `Astar failed`、没有端点诊断），但紧接着是 **238 次 `MINCO path generation failed; retrying`**、车一动不动，`Braking: ...` 反复出现。
- 根因一（本次修复）：`makePlanOnQuery()` 的 `output_frame` 形参**只用于写 `header.frame_id`，不做任何坐标变换**：返回点位来自 `query->mapToWorld()`，即**地形图查询坐标系（map）**。上一条改动把降级查询从 ROGMap 换成 `TerrainMapQuery`、起终点改用 `start_map/goal_map` 时，仍把 `mode_context.outputFrame()`（= odom）当输出帧传了进去，于是种子路径**整体被贴上 odom 标签却是 map 坐标**。现场两者并不重合：`odom_localizer` 日志 `map->odom T=(0.316, 0.403) Q≈yaw 0.10 rad`，车停在 odom 原点时 map 系位置是 `(0.309, 0.391)`（即上面日志里 `start_world` 那一对）——种子路径起点离车 **0.5 m**、终点离用户点的目标 **0.5 m**，还多转 5.8°。MINCO 只能沿这条错位参考线优化，起点附近净空 0.24~0.29 m 反复低于 0.30 m 被判 `COLLISION`。
- 修改（1 个文件，`global_path_searcher.cpp`）：降级 `makePlanOnQuery` 成功后，逐点做 `tf2::doTransform(in, out, map_to_rog)` 把路径由地形图系换算到 `outputFrame()`——用的是**同一个 `map_to_rog`**、与紧随其后的 Kino 成功分支完全一致；注释记录成因与现场数字。未改 `makePlanOnQuery` 本体、未改任何参数。
- 根因二（本次只实测、未改代码）：**`searchOmniKinoPath` 在真实 lab3 图上够不到远处目标**。新写的独立探针（`.scratch/kino_probe/`，直接编译 `terrain_grid.cpp`+`omni_kino_astar.cpp`，用 lab3.pgm 按 map_server 同款翻转/阈值建图，direction 全 0 与 `pgm_to_terrain_msgpack.py` 一致）实测：**空图**上 1~8 m 全部有解、最慢 85 ms；**真实 lab3 图**上以车当前位置为起点，3 m 起大量方向无解、5 m 后几乎全灭，且每次失败都烧掉整个 50 000 次扩展预算、耗时 **0.76~1.03 s**（与现场日志里"目标→WARN 相隔 1.08 s"吻合）。即瓶颈是**带速度维的状态格点在有障碍空间里爆炸**（`speed_bins = max_speed/0.1 = 30`、8 个朝向 → 单节点最多 248 个后继），不是解不存在。
- 对照：旧工程 `mas_nav_2027` 的全局搜索是 **SMAC 2D 纯栅格**（`PlannerMode::PRIORMAP`，跑在整张先验图的 Nav2 costmap 上，`smac_planner_2d_simple`），不含速度/加速度可行性约束、也没有扩展预算，故"远处目标"从不会因此失败；本工程 `planner_mode_context.cpp` 已把 `PRIORMAP` 分支删掉、强制 `EXPLORATION`，全局搜索换成了速度格点 Kino，降级 A\* 反而成了唯一能覆盖远处的通路。**是否把纯栅格 A\* 提为主搜索（对齐旧工程）待用户决定，本次未动。**
- 验证：`mas2027_nav_executor` Release 构建通过（26.9 s）；执行器 7 项 CTest 全部通过（`ROS_LOG_DIR` 指向工作区内，因沙箱下 `~/.ros/log` 只读）；探针实测数据如上。
- 未验证：**未跑实车**——帧换算是否消除"降级成功但车不动"未实测；Kino 远处无解因此是否仍要每次白烧 1 s 未处理；日志里 `Near-field exemption: start clearance 0.253 m below required 0.300 m` 说明车当时本就停在离障碍物约 0.25 m 处，该因素是独立叠加项，未评估。

## 2026-09-15 — 降级全局搜索改用地形图查询（方案 A，消除占据类 TERRAIN 违规）

- 背景：Kino 无解时的降级 A\* 原先在 **ROGMAP** 查询上做纯栅格搜索，不看 HW 地形/方向层，于是 MINCO 沿该种子路径优化出的轨迹被 `validateTrajectory` 判 `TERRAIN_COLLISION_OR_DIRECTION`（现场实测一次运行 10 次），轨迹发不出去。
- 修改（2 个文件）：
  1. `global_path_searcher.cpp`：降级分支改为构造 `mas2027_nav_executor::TerrainMapQuery(terrain_)` 作为查询传给 `makePlanOnQuery`，**起终点改用已算好的 map 系 `start_map.pose` / `goal_map.pose`**（地形查询的坐标系是 map，不能再用 ROGMap 系的 `start_rog`/`goal_rog`）；日志前缀由 `ROGMap(fallback)` 改为 `Terrain(fallback)`。同时新增 `#include "mas2027_nav_executor/common/environment/terrain_map_query.hpp"`。
  2. `CMakeLists.txt`：把 `src/common/environment/terrain_map_query.cpp` 加入 `${PROJECT_NAME}_node` 目标——该实现此前只出现在 `test_terrain_map_query` 中，不加会报 `undefined reference to TerrainMapQuery::TerrainMapQuery / vtable`（首次构建即踩到）。
- 已知限制（写在代码注释里）：`TerrainMapQuery` 的取值是**二值**的（`kLethalCost` 254 / `kFreeCost` 0），**不含方向信息**；而方向约束是逐边判定的（Kino 用 `transition(from,to)`，朴素 A\* 没有该概念）。因此本次只预期消除"穿过不可通行格"一类违规，**"逆向穿过方向受限格"仍可能被否**。
- 未改动：Kino 搜索、`makePlanOnQuery` 本体、`Astar`、ROGMap 与 planner 的全部参数。
- 验证：`mas2027_nav_executor` Release 构建通过（7.66 s）；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车——`TERRAIN_COLLISION_OR_DIRECTION` 下降多少未实测；**`TerrainMapQuery::refresh()` 为私有、本次未调用**，该查询是否在降级时刻已由构造/惰性路径填好快照未确认（若为空则 A\* 失败、降级返回 false，行为退回本次改动前）；方向类违规的残留量未知（需现场计数后决定走"边感知方向"还是放开方向约束）。

## 2026-09-15 — 回退 safe_dist 至 0.40：0.50 使 MINCO 不收敛

- 现象（现场运行时日志 `mas2027_nav_executor_node_122922_*.log`，141 行）：单击一个目标点后，全局路径**已正常生成**（仅 1 次降级调用，日志含 `allow_unknown=true` 且无 `Astar failed`，说明此前的 `isFree`/`allow_unknown` 修复生效），但随后出现 **93 次 `MINCO path generation failed; retrying`**、15 次 `MINCO trajectory not published: COLLISION`、17 次净空不足（如 `0.282 / 0.255 m below required 0.300 m at (1.17,-0.07) / (1.32,-0.06)`）、8 次 `TERRAIN_COLLISION_OR_DIRECTION`。即瓶颈从"全局搜索"转移到了 **MINCO 优化本身不收敛**。
- 判断：降级 A\* 给出的种子路径要穿过未观测区（该次日志 `goal_cost=255(unknown)`），此类区域几何通常很紧；而此前为把"软目标 vs 硬判据"交叉点推到 1.33 m/s 将 `safe_dist` 由 0.40 抬到 **0.50**，在窄处/未知区成为**够不到的软目标**，位置罚项恒不满足 → L-BFGS 不收敛 → `path generation failed`。硬判据只需 `collision_dist`(0.30)，故回退。
- 修改：`planner_params.yaml` 中 `minco_optimizer.safe_dist` 由 0.50 改回 **0.40**，注释记录抬高的动机、实测代价与回退理由。配置改动，无需重新编译。
- 未改动：`collision_dist`(0.30)、`replan_react_time`(0.0)、`node.rog_map_clearance`、全部 rog_map 参数、发布前校验（仍为已回退的 `requiredClearance(v)`）。
- 验证：`yaml.safe_load` 解析通过并打印确认（`safe_dist = 0.4`、`collision_dist = 0.3`、`replan_react_time = 0.0`）。**未跑实车**：93 次优化失败是否由此消除未实测。
- 未验证/遗留：8 次 `TERRAIN_COLLISION_OR_DIRECTION` 与 15~17 次净空不足仍存在——前者是 HW 地形/方向图否决降级 A\* 给出的纯栅格路径（该路线已知不保证方向可行性），后者是"监视器减容差、发布前校验不减"的不一致（该不一致仍成立但已回退，需在干净现场数据下再评估）。`safe_dist` 是否应介于 0.40~0.45 之间未调参。

## 2026-09-15 — 回退「发布前校验补容差」（按用户要求）

- 回退内容：`minco_planner.cpp` 的 `checkCollision(const traj_opt::Trajectory &)` 中 `options.check_dist` 恢复为 `requiredClearance(v)`（即上一条记录改回原样），并在注释中保留该次尝试的结论与"监视器与本处阈值不一致"这一仍成立的事实。
- 回退理由：改动后 `validateTrajectory: collision detected` 确实不再出现（改动生效），但随后现场报告"完全无法规划"。核对日志（`mas2027_nav_executor_node_114441_*.log`，161 行）显示瓶颈在更上游：**31 次 `Global path search failed` + 33 次 Kino 降级尝试 → 无全局路径 → MINCO 无种子 → 46 次 `MINCO path generation failed`**，与本处校验阈值无因果关系。同时发现 7 次目标点中有 **3 次被 `Ignoring goal: odometry is stale` 直接拒收**（`/Odometry` 过期，属独立问题）。
- 未改动：`kMonitorClearanceTolerance` 常量、`collision_dist`、`replan_react_time`（0.0）、`node.rog_map_clearance`、全部 rog_map 参数。
- 验证：`mas2027_nav_executor` Release 构建通过（26.9 s）；执行器 7 项 CTest 全部通过；回退后用 grep 确认第 1683 行已恢复为 `options.check_dist = requiredClearance(v);`（第 1652 行的监视器分支仍为 `monitor_dist`，即减容差的版本，未受影响）。
- 未验证：未跑实车确认"无法规划"是否随之消失——按日志判断该现象由全局搜索失败与 odom 卡顿造成，**回退本身预期不会解决它**；`Global path search failed` 与 `odometry is stale` 两条线都尚未定位。

## 2026-09-15 — 发布前校验补上净空容差，修复"过某个特定点必卡"

- 现象：同一条路径经过某个特定点必然卡住。日志（`mas2027_nav_executor_node_97120_*.log`，315 行）中失败样本沿一条直线排开，净空为 **0.300 / 0.299 / 0.298 / 0.297 / 0.296 m，而要求恰为 0.300 m** —— 即差 **0~4 毫米**；终端同时出现 `validateTrajectory: collision detected.` + `Trajectory validation failed! Rejecting`。
- 根因：同一份代码里发布前校验与运行时监视的阈值**不一致**。运行时监视（`minco_planner.cpp:1643`）用 `requiredClearance(v) - kMonitorClearanceTolerance`；发布前校验（`checkCollision(const Trajectory&)`，原第 1676 行）用**不减容差**的严格 `requiredClearance(v)`。而该函数正上方的注释写的是"发布前校验：阈值必须与运行时监视一致"——**代码违背了自己的注释**。于是净空停在 0.296~0.300 的轨迹：监视器放行（阈值 0.25），发布前校验毫米级否决，新轨迹发不出去；旧轨迹走完，车就停在原地。
- 修改：`checkCollision(const traj_opt::Trajectory & traj)` 中 `options.check_dist` 改为 `std::max(0.0, requiredClearance(v) - kMonitorClearanceTolerance)`，与监视器对齐；注释补充该不一致的成因与现场证据。
- 未改动：`kMonitorClearanceTolerance` 取值、`collision_dist`、`replan_react_time`（仍为 0.0）、`node.rog_map_clearance`、`safe_dist`、全部 rog_map 参数。
- 验证：`mas2027_nav_executor` Release 构建通过（30.1 s）；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车确认该点不再卡；**该点净空本就只有约 0.296~0.300 m**（释放容差后为 0.25 阈值，理论可通过），但该处是否仍有真实窄点导致速度受限未评估；容差 0.05 是否过大（相当于把有效净空降到 0.25，与本车内切半径 0.30 的关系）未复核。

## 2026-09-15 — 关闭先验图融合：实测未对齐，会在返程制造幻影墙

- 现象：开启先验图融合（lab3）后，返程（第二个目标）出现一次明显卡顿。时序证据（`mas2027_nav_executor_node_92610_*.log`，49 行）：去程 +0→+19 s 到达、无任何告警；返程 +24 s 出发后，在 **(3.16~3.31, −0.02~−0.04)** 一带出现 4 次 `Trajectory clearance 0.272~0.284 m below required 0.275~0.300 m` 与 **10 次** `Braking: current dynamic obstacle intersects the MPC reference horizon`，返程 26 秒中有约 20 秒耗在该点。
- 根因：`fusePriorMapProjection()` 把先验图的占据格当**硬障碍**叠加进二维投影/mask。RViz 确认该处 `/rog_map/layer_value_static` 显示有墙、而 `/rog_map/layer_value_dynamic`（实测）为空——即**先验图与现场未对齐**，净空由约 0.4 m 被压到 0.272~0.285 m，低于 `collision_dist` 0.30，于是规划器否决、执行器急停。
- 另注（排查误导点）：`command_safety.cpp:56-64` 中「动态图不自由」与「ROGMAP 净空不足」**共用同一个 `DYNAMIC_BLOCKED` 状态**，调用方统一打印 `Braking: current dynamic obstacle ...`。本次靠坐标与净空告警交叉比对才区分开——该层已由 `bypass_dynamic_obstacle: True` 关闭，并非复活。
- 修改：`planner_params.yaml` 的 `rog_map.projection.prior_map.enable` 由 `true` 改回 **`false`**，并按第 1 步"只验对齐、对不上不继续"的约定暂停后续放大窗口与切换全局搜索。`yaml_path`/`pgm_path`/`frame_id` 原样保留，便于对齐修好后一键恢复。配置改动，无需重新编译。
- 未改动：`map_size`、全部 `minco_optimizer` 参数、`node.rog_map_clearance`、`collision_dist` 均未动（避免又是一次口径打架）。
- 验证：`yaml.safe_load` 解析通过并打印确认 `prior_map.enable = False`。
- 未验证：未跑实车确认返程卡顿消失；**先验图错位的成因未查明**（可能方向：`prior_map.cpp` 的 `loadPriorMap` 对 PGM 的 y 翻转/origin 处理，或 `map`↔`odom` 标定，或 lab3.pgm 与 lab3.pcd 本身不同源）；对齐修好前，先验图融合与"全局搜索改用 A*"的路线保持暂停。

## 2026-09-15 — 启用 ROGMap 先验图融合（lab3），为全局搜索改用 A* 做准备

- 背景与目标：远处目标规划一直受"全局搜索跑在 HW 地形图上、且受 ROGMap 10×10 m 滑窗牵制"的限制；docker 内的上游（`/home/mas/nav_opensource/navi_minco_bit`）是把整张先验图交给 Nav2 StaticLayer、由 SMAC 在**整张图**上规划，`rog_map.projection.prior_map` 只在滑窗内给 MINCO 当 ESDF。本工程已移除 Nav2，故按路线 A 对齐：**先验图融合进 ROGMap，再由 A\* 规划**。本次只做第 1 步。
- 上游配置对照（`src/navigation/navi2_bringup/params/sentry1.yaml:393-398`）：`prior_map: {enable, yaml_path, pgm_path, frame_id}` 四个键与 `rog_map_core/config.hpp:287-290` 读取的 `projection.prior_map.*` **完全一致**。
- 改动：`planner_params.yaml` 的 `planner.rog_map.projection` 下新增 `prior_map` 段——`enable: true`、`yaml_path` 指向 `mas2027_nav_bringup/map/lab3.yaml`（origin 为 `[-4.6, -7.94, 0]`，resolution 0.05）、`pgm_path: ""`（留空即用 YAML 的 `image` 字段并相对 YAML 目录解析）、`frame_id: map`。
- **本步刻意不动 `map_size`**：保持 10×10 m 窗口，先验证对齐。理由：先验图与现场若对不上，后面放大窗口与切换搜索都是白做；`map`↔`odom` 由 `odom_localizer` + `tf_maintainer` 提供。
- 未改动：`map_size`、`ray_range`、两个 `unknown_as_occupied`、`scan_z_min_abs`、`virtual_ground_height`、`minco_optimizer` 全部参数；无代码改动。
- 验证：`yaml.safe_load` 解析通过并打印确认该段取值；隔离 `ROS_DOMAIN_ID=230` 直启 `mas2027_nav_executor_node`（补 `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu`）9 秒，日志出现 **`[ROGMap] loaded prior map 770x347 at 0.050 m/cell in frame 'map'`**（770×347 即 lab3.pgm 的实际尺寸），`[MincoPlanner]` 正常初始化、无异常退出。
- 未验证：**未验证先验图与现场是否对齐**（需真机 RViz 看 `/rog_map/layer_value_static` 与实测墙体是否吻合）——这是本步的核心判据，也是后续放大窗口与切换全局搜索的前提；未跑实车。

## 2026-09-15 — allow_unknown 与 exploration.unknown_as_occupied 同源，修复降级 A* 仍失败

- 现象（终端/日志 `mas2027_nav_executor_node_76587_*.log`）：Kino 无解后降级已被触发（出现 `falling back to ROGMap A* direct plan`），但**降级也失败**——`ROGMap(fallback) Astar failed to find path`，端点诊断显示 `start(used) cell=(100,99) cost=0(free)`、**`goal cell=(181,147) cost=255(unknown)`**。
- 根因：两个"未知格是否可通行"标志来源不同。外层判定用 `smacTraversableCost()` / `goal_traversable`，取自 mode context 的 `exploration.unknown_as_occupied`（本轮已改为 `false`，故放行了降级）；而 `Astar` 内部只在 `allow_unknown` 为真时才接受 `cost == 255`（`astar.cpp:196,259`），该值由 `GlobalPathSearcher::configure()`（`global_path_searcher.cpp:174`）一次性写入，调用点在 `minco_planner.cpp:479` 传的是 **MincoPlanner 自己的 `allow_unknown_` 成员**，与 `exploration_unknown_as_occupied_` 无关联。于是"外面说未知可通行、里面仍把未知当障碍"，目标落在未观测区域时必然搜不到路径。
- 修改：`minco_planner.cpp` 的 `global_path_searcher_->configure(...)` 第三个实参由 `allow_unknown_` 改为 `!exploration_unknown_as_occupied_`，使两者**同源**，不再可能各说各话。未改动 `Astar`、`GlobalPathSearcher` 与 `MincoPlanner::allow_unknown_` 成员本身（后者保留但不再影响该处）。
- 未改动：Kino 搜索、降级逻辑、地形/方向判定、全部参数。
- 验证：`mas2027_nav_executor` Release 构建通过（1 min 8 s）；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车——远处目标能否由此成功规划未实测；`MincoPlanner::allow_unknown_` 若另有其他用途（本次未追查其赋值来源），改动后两者的语义差异未评估；降级路径给出的纯栅格路径能否被下游 MINCO/执行器正常接受仍未验证。

## 2026-09-15 — 远处目标全局规划失败：Kino 无解时降级到 ROGMap A* 直连规划

- 现象：在先验图上设置较远目标点后全局规划持续失败（`mas2027_nav_executor_node_63143_*.log`：目标 `(4.43, 2.29)`，38 次 `No acceleration-feasible route satisfies terrain and current dynamic obstacles` + 38 次 `Global path search failed; retrying`），而同一位置在 docker 内的上游/旧工程可以规划并执行到先验图最远处。现场在 RViz 确认 `HW Cost Map` 显示可通过、`HW Direction Map` 无异常，故排除地形层与方向层。
- 根因：本工程自研的全向 Kino A*（`searchOmniKinoPath`，`global_path_searcher.cpp:304`）以「实测里程计速度种子 + 速度/加速度可行性」扩展状态格点，**可行解集依赖车当前状态**，目标较远或朝向不巧时可能无解；而失败分支只是 `return false`，**不会走到紧随其后、本可用于兜底的 ROGMap A\* 直连规划**（同文件 353-371 行，仅对"目标在 ROGMap 内且可通行"生效）。上游用的是 Nav2 SMAC（对栅格搜索、不含速度可行性硬约束），因此不存在这种"看状态抽风"的行为。
- 修改：`planExploration()` 中 Kino 失败分支不再直接返回，改为打印 `falling back to ROGMap A* direct plan` 后，按与下方 `goal_traversable` 分支相同的判据（目标在 ROGMap 内、且 `value(kUnknownCost)` 时按 `explorationUnknownAsOccupied()` 取反、否则 `< kInscribedCost`）调用现成的 `makePlanOnQuery(start_rog, goal_rog, query, ...)`；成功则 `latest_global_path` 已由其填充，直接 `return true` 跳过基于 `map_path` 的填充逻辑，失败才 `return false`。降级调用传入空的 `cancel_checker`（该处 try 作用域内取不到外层 checker；`makePlanOnQuery` 内部先判空，A* 另有自身循环预算）。
- 未改动：Kino 搜索本身、地形/方向层判据、`isFree`、全部参数。
- 验证：`mas2027_nav_executor` Release 构建通过（10.1 s）；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车——远处目标是否因此可规划未实测；降级路径给出的是**纯栅格路径**（不含速度/加速度可行性），是否会被下游 MINCO/执行器接受、以及 `latest_global_path` 与调用方 `minco_planner.cpp:1318` 的期望是否完全一致，均未验证；日志中新增的 `falling back to ROGMap A* direct plan` 可用于确认降级是否被触发。

## 2026-09-15 — 修复 isFree 把「无信息」当「不可通行」导致远处目标全局规划失败

- 现象：在先验图上设置远处目标点后全局规划持续失败。日志（`mas2027_nav_executor_node_58693_*.log`，243 行）中目标为 `(4.22, 1.59)`（落在 10×10 m 滑窗内），随后 **118 次** `No acceleration-feasible route satisfies terrain and current dynamic obstacles` + 118 次 `Global path search failed; retrying`，车不动。
- 根因：`global_path_searcher.cpp:295-301` 的 ROGMap 判据在窗口内要求 `query->isFree(mx,my)`；而 `QueryAdapter::isFree()` 原实现是 `values[idx] < 253U`。按 `projection_layer.cpp::applyValueAndMask()`，`unknown_as_occupied: false` 时**未知列的取值是 255**（no information），`255 < 253` 为假，于是**未观测区域整片被判为不可通行**。远处目标必经的正是车还没看过的区域，搜索状态空间被堵死。255（无信息）与 254（`kLethalCost`/OCCUPIED）语义不同，必须区别对待。
- 修改：`query_adapter.cpp::isFree()` 改为 `cost < 253U || cost == 255U`。该改在两种配置下都自洽：`unknown_as_occupied: true` 时未知列取 254，仍判为不可通行；`false` 时取 255，判为可通行，与二维 ESDF 的 `mask=1` 以及其他消费方（`trajectory_safety_checker` 只拒绝 253/254）保持一致。同时受益的还有 `path_planner.cpp:123` 的目标准入与 `local_path_processor.cpp:26` 的种子路径裁剪——此前它们同样把未观测格当"不自由"。
- 未改动：ROGMap 与 planner 的全部参数；本次仅一处判据，无配置改动。
- 验证：`rog_map`(5.8 s) 与 `mas2027_nav_executor`(6.6 s) Release 构建通过；执行器 7 项 CTest 全部通过。
- 未验证：未跑实车——远处目标能否规划成功未实测；**未评估新放行的"未知区域可通行"带来的风险**（规划可能穿过尚未观测的区域，与 `unknown_as_occupied: false` 的既有取向一致，靠地形先验图兜底）。

## 2026-09-15 — 对齐上游：取消速度相关净空要求（消除「一卡一卡」的极限环）

- 背景：实车仍「一卡一卡」。用户指出上游（docker 内的 `/home/mas/nav_opensource/navi_minco_bit`）用 `raycasting.ray_range: [0.03, 10.0]` 也完全正常，促使改为以上游为参照逐项核对。
- **决定性发现**：在上游源码中检索 `replan_react_time`、`monitor_margin`、`requiredClearance` **全部不存在**（`grep -rn ... src/` 无结果），其碰撞校验（`minco_planner.cpp:1567 validateTrajectory` / `1638,1662 checkCollision`）使用**固定阈值** `collision_dist: 0.25`。而本工程在发布前校验与运行时监视中都引入了 `required = collision_dist + max(v * replan_react_time, monitor_margin)`，使净空要求**随速度增长**，等价于速度上限 `v < (可用净空 - collision_dist) / replan_react_time`；规划速度略超该上限时，轨迹会在最窄处差几毫米被否掉，形成「加速 → 否决 → 急停 → 再加速」的极限环。实车日志证据：19:37 次运行 6 次否决全部为毫米级（0.414 vs 0.425、0.395 vs 0.403、0.314 vs 0.340、0.342 vs 0.345、0.414 vs 0.414、0.336 vs 0.338），且 6 个失败点与 6 次起点净空落在同一区间（0.30~0.42），与位置无关。
- 改动一：`planner_params.yaml` 中 `minco_optimizer.replan_react_time` 由 0.15 改为 **0.0**。此时 `requiredClearance()` 退化为固定 `collision_dist`（0.30），与上游的固定阈值行为一致，极限环不再成立。
- 改动二（修正前一次改动引入的矛盾）：`rog_map.virtual_ground_height` 由 −1.0 改为 **−1.5**。`prob_map.cpp` 的 `getGridType()` 将 `z <= virtual_ground_height` 的体素一律判为 OCCUPIED，而 `projection.scan_z_min_abs` 为 −1.2，窗口探到地面以下 0.2 m，每个柱子凭空多出 4 个体素的虚假占据，污染分类统计。旧工程取 −1.5 正是为了让 −1.2 的窗口落在其上。
- 未改动：`collision_dist`（保持 0.30，本车内切半径标定值；上游为 0.25，属另一台车）、`monitor_margin`（0.0）、`safe_dist`（0.50）、`speed_aware_clearance`（false）、`ray_range`（[0.3, 10.0]）、两个 `unknown_as_occupied`（false）均未动；本次为配置改动，无需重新编译。
- 验证：`yaml.safe_load` 解析通过并打印确认（`replan_react_time = 0.0`、`collision_dist = 0.3`、`virtual_ground_height = -1.5` 且 `-1.2 > -1.5` 成立）。
- 未验证：未跑实车——卡顿是否消除、速度是否进一步提升均未实测；**代价是高速下不再预留制动距离**（安全余量退化为固定 0.30 m，靠 20 Hz 重规划兜底），若实车出现高速制动不及，应从 0.05 起逐步回调 `replan_react_time`。

## 2026-09-15 — 关闭 map_server 动态障碍层（对齐旧工程架构）

- 背景：实车速度已恢复，但「明显可以通过的地方仍不丝滑」。逐项对比发现**结构性差异**：旧工程 `/home/mas/mas_nav_2027/mas2027_perception/` 下只有 `Localization`、`mid360_driver`、`Odometry`、`rog_map` —— **没有 `map_server`，没有动态障碍层**；本工程多出一整套「点云对比全局静态云 → 检测动态障碍 → 按 `full_cost_radius_m 0.2` / `cutoff_radius_m 0.4` 膨胀 → 发布 `/dynamic_cost_map`」，且执行器据它刹车（`command_safety.cpp` 中 `dynamic->freeAt()` → `ExecutorStatus::DYNAMIC_BLOCKED`，日志表现为 `Braking: current dynamic obstacle intersects the MPC reference horizon`）。现场 RViz 观察：该层明显比实物厚，把路径显示得比实际窄。
- 改动：`mas2027_nav_bringup/launch/nav_executor_launch.py` 中 `terrain_map_server` 的 `bypass_dynamic_obstacle` 由 `False` 改为 **`True`**。按 `map_server_node.cpp:67,88`，该开关为真时不加载全局静态云与 KdTree、**不订阅点云**，`/dynamic_cost_map` 保持全空；`/cost_map` 与 `/direction_map` 照常发布，规划器使用的地形图不受影响。
- 安全性取舍：动态物体改由 ROGMap 的时间衰减（`decay.keep_time 0.8 s` / `clear_time 1.2 s`）承担，即旧工程既有做法；代价是失去「点云与全局静态云比对」这一层显式动态检测，快速移动物体不再有独立的急停判据。需要动态避障时把该参数改回 `False` 即可。
- 未改动：ROGMap 的全部参数、`minco_optimizer` 全部参数（`safe_dist 0.50`、`replan_react_time 0.15`、`speed_aware_clearance false` 等）均未动；本次为 launch 文件改动，无需重新编译。
- 验证：`python3 -m py_compile` 解析通过；确认文件中该参数当前为 `True`（第 119 行）。**未验证**：未跑实车——`Braking: current dynamic obstacle ...` 是否消失、卡顿是否改善均未实测；若卡顿依旧，说明主因是「规划速度超过净空允许值」形成的极限环（净空 0.396 vs 要求 0.406 一类毫米级否决），下一步应改为按沿途净空给规划速度加上界，而不是继续动地图层。

## 2026-09-15 — 抬高优化器软目标 safe_dist 至 0.50，把硬判据交叉点推到 1.33 m/s

- 目的：在不引入"降速收益"的前提下消除窄道「卡一下」。卡顿根源是优化器软目标固定 0.40、而硬判据 `collision_dist + v * replan_react_time = 0.30 + 0.15v` 在 `v > 0.67 m/s` 时反超软目标；优化器只能渐近逼近软目标，于是窄道处稳定地差几毫米被 `validateTrajectory` 否掉（实测净空 0.396 vs 要求 0.406、0.425 vs 0.427）。
- 改动：`planner_params.yaml` 中 `minco_optimizer.safe_dist` 由 0.40 改为 **0.50**。交叉点随之推到 `(0.50-0.30)/0.15 ≈ 1.33 m/s`，覆盖当前实测约 0.8 m/s 的巡航速度。
- 影响面核查（改动前逐条确认，避免重蹈 `speed_aware_clearance` 的覆辙）：
  - `minco_planner.cpp:488` 为 `safety_checker_->configure(collision_dist, ...)`，检查器阈值取自 `collision_dist` 而非 `safe_dist`，**所有硬判据不受影响**；
  - `minco_planner.cpp:289` 的 `double collision_dist = minco_config.safe_dist` 只是参数缺省值，YAML 已显式给出 `collision_dist: 0.30`，同样不受影响；
  - `magnitudeBounds(0)` 在 `minco_optimizer.cpp` 中仅用于位置罚项（`safe_dist = magnitudeBounds[0]`），未参与速度/加速度约束与时间屏障。
  - 结论：该改动**只改优化器的软惩罚目标**，不含速度项，因此不产生「降速收益」，不会像上一条记录的 `speed_aware_clearance` 那样拖慢全局。
- 未改动：`collision_dist`、`replan_react_time`、`speed_aware_clearance`（仍为 `false`）、`clearance_optimizer_margin`、`ray_range`、两个 `unknown_as_occupied`、`scan_z_min_abs` 均未动；本次不涉及代码，无需重新编译。
- 验证：`yaml.safe_load` 解析通过并打印确认（`safe_dist = 0.5`、`collision_dist = 0.3`、`replan_react_time = 0.15`、`speed_aware_clearance = False`，交叉点计算为 1.333 m/s）。
- 未验证：未跑实车——窄道是否不再卡、窄缝中优化器追不到 0.50 目标是否导致 `MINCO path generation failed` 增多，均未实测；若出现后者，应把 `safe_dist` 回调到 0.45 再试。

## 2026-09-15 — 回退速度感知净空的启用（全程变慢），保留代码与参数

- 现象：上一条记录启用 `speed_aware_clearance`（含 `clearance_optimizer_margin: 0.05`）后，实车**全程速度都变慢**，不只是窄道。
- 原因（两条叠加，均已写入配置注释）：
  1. 目标净空由固定 `safe_dist = 0.40` 变为 `0.35 + 0.15v`，在 `v > 0.33 m/s` 之后**一律高于原目标**（v=0.5 → 0.425、v=1.0 → 0.50），整体净空要求被抬高；
  2. `penalty_weight_pos = 50000` 对 `penalty_weight_time = 100`（相差 500 倍），**减速几乎不花代价**，优化器因而选择「降速」而不是「绕开」来降低位置惩罚，等效于全局刹车。这是权重比例的结构性问题，不是余量取值能调好的。
- 修改：`planner_params.yaml` 中 `speed_aware_clearance` 由 `true` 改回 **`false`**，恢复固定 `safe_dist` 的原有行为；注释完整记录上述成因与重新启用前必须先提高 `penalty_weight_time` 的前提。`clearance_optimizer_margin`（0.05）与上一条记录新增的代码路径**保留不动**，仅在开关开启时生效，关闭时不进入任何计算。
- 未改动：`safe_dist`、`collision_dist`、`replan_react_time`、`ray_range`、两个 `unknown_as_occupied`、`scan_z_min_abs` 均未动；本次不涉及代码改动，无需重新编译。
- 验证：`yaml.safe_load` 解析通过并逐项打印确认（`speed_aware_clearance = False`、`safe_dist = 0.4`、`collision_dist = 0.3`、`replan_react_time = 0.15`、`penalty_weight_time = 100.0`、`penalty_weight_pos = 50000.0`）。
- 未验证：未跑实车确认速度已恢复；「窄道卡一下」在该配置下仍会存在（优化器固定目标 0.40 与硬判据 0.30+0.15v 在 v > 0.67 m/s 处交叉的问题未解决），是否需要提高 `safe_dist` 把交叉点推高，待实测后决定。

## 2026-09-15 — 修复窄道「卡一下」：开启速度感知净空并给优化器软目标加余量

- 现象：路一变窄就卡一下，但该处明显够宽、本可快速通过。19:12 次运行（`mas2027_nav_executor_node_17539_1789470653217.log`）只剩 2 次急停，两个失败样本都是**毫米级擦边**：净空 0.396 vs 要求 0.406（反推 v≈0.71 m/s，位置 (1.44,-0.04)）、净空 0.425 vs 要求 0.427（v≈0.85 m/s，位置 (0.53,-0.02)）。
- 成因：优化器位置罚项用**固定**软目标 `safe_dist = 0.40`，检查器硬判据是 `0.30 + 0.15v`。当 `v > 0.67 m/s` 时硬判据反超软目标，优化器按 0.40 规划出的解必然差几毫米被 `validateTrajectory` 否掉 → 急停 → 卡一下。旧工程配置注释早已写明这一关系：「safe_dist 是优化器的软目标净空，collision_dist 是 validateTrajectory 的硬判据……优化器只能渐近逼近 safe_dist」。
- 修改（代码 + 配置）：
  1. `minco_optimizer.hpp`：`Config` 新增 `clearance_optimizer_margin{0.05}`，`ClearanceModel` 新增 `optimizer_margin{0.05}`，并在构造函数与 `setConfig` 的聚合初始化中一并赋值。
  2. `minco_optimizer.cpp::constraintsFunctional`：速度感知目标改为 `collision_dist + max(|v|·react_time, monitor_margin) + optimizer_margin`，使软目标恒高于硬判据。
  3. `minco_planner.cpp` / `minco_planner.hpp`：新增参数 `minco_optimizer.clearance_optimizer_margin`（默认 0.05，非法值回落 0.05），成员 `clearance_optimizer_margin_`，并在两处 `minco_config` 构建点写入。
  4. `planner_params.yaml`：`speed_aware_clearance` 由 `false` 改为 **`true`**（现场已出现该开关所针对的场景），新增 `clearance_optimizer_margin: 0.05`。
- 未改动：`collision_dist`、`safe_dist`、`replan_react_time`、`ray_range`、两个 `unknown_as_occupied`、`scan_z_min_abs` 均未动。
- 验证：`mas2027_nav_executor` Release 构建通过（1 分 40 秒）；执行器 7 项 CTest 全部通过；YAML 用 `yaml.safe_load` 解析并打印确认（`speed_aware_clearance = True`、`clearance_optimizer_margin = 0.05`、`replan_react_time = 0.15`）。
- 未验证：未跑实车——窄道是否不再卡、窄处主动减速后整体通行时间如何变化，均未实测；余量 0.05 是否合适未调参（调大更保守、更早减速）；开启 `speed_aware_clearance` 后的 L-BFGS 收敛性与轨迹质量未评估（该路径此前只验证到能正常启动）。

## 2026-09-15 — 对齐旧工程的未知区域语义与投影 Z 下限（提速）

- 背景：`/home/mas/mas_nav_2027`（旧工程）在同一场地能跑得很快。逐项对比后确认**限速公式与数值两边一致**（`safe_dist: 0.40` / `collision_dist: 0.30` / `replan_react_time` / `max_velocity: 3.0`），差别在**地图语义**：旧工程 `projection.unknown_as_occupied: false`、`scan_z_min_abs: -1.2`，本工程为 `true`、`-0.5`。
- 现场证据（19:05 次运行 `mas2027_nav_executor_node_11405_1789470310433.log`）：速度已由 0.40 提到 0.81 m/s，但**失败样本全贴在目标上** —— 目标 (2.07,-0.24) 的失败样本在 (2.06,-0.24)（1 cm）、目标 (0.74,0.07) 在 (0.76,0.02)/(0.75,-0.01)、目标 (0.46,-0.10) 在 (0.63,-0.12)，而这些点的可用净空只有 0.299~0.42 m。目标附近多为未观测区域，被判成障碍后通往目标的走廊塌缩到约 0.3 m，`required = 0.30 + 0.15v` 在 v 稍大时即无法满足。
- 改动（`planner_params.yaml` 三项，均对齐旧工程取值）：
  1. `planner.rog_map.projection.unknown_as_occupied`：`true` → **`false`**。按 `projection_layer.cpp::applyValueAndMask()`，该开关同时决定二维 ESDF 的 mask（0=障碍）与 value（254=`kLethalCost` / 255=no-information），是本工程净空读数普遍只有 0.3~0.45 m 的主要来源。
  2. `planner.exploration.unknown_as_occupied`：`true` → **`false`**。该项经 `planner_mode_context` 控制全局搜索的 `smacTraversableCost()` 与目标可通行判定（`global_path_searcher.cpp:72,349`），**必须与第 1 项一致**，否则同一格会被搜索判为不可通行、却被距离场判为自由。
  3. `planner.rog_map.projection.scan_z_min_abs`：`-0.5` → **`-1.2`**。窗口相对雷达，雷达离地超过 0.5 m 时地面会掉出窗口，地面柱子随即成为 UNKNOWN。
- 未改动：`ray_range` 保持 `[0.3, 10.0]`（0.3 m 盲区已由同日的「近距盲区清空」修复解决，退回 0.01 会重新引入车身点云写入）；`collision_dist`、`safe_dist`、`max_velocity`、`replan_react_time` 均未动。
- 验证：`yaml.safe_load` 解析通过并逐项打印确认（两个 `unknown_as_occupied` 均为 `False`、`scan_z_min_abs = -1.2`、`ray_range = [0.3, 10.0]`）；`install/` 下为符号链接，改源码即时生效，无需重新编译。
- 未验证：未跑实车，提速幅度未测；`unknown_as_occupied: false` 后**规划可能穿过尚未观测的区域**（旧工程既有行为，靠地形先验图兜底），该风险未评估；`QueryAdapter::isFree()` 判据是 `value < 253`，UNKNOWN(255) 在 `path_planner` 的目标准入（`path_planner.cpp:123`）与 `local_path_processor` 的种子路径裁剪（`local_path_processor.cpp:26`）处仍被视为"不自由"——与改前一致、无回归，但若目标附近仍失败，应优先查这两处。

## 2026-09-15 — 提速：下调制动距离预算与线速度死区，并新增速度感知净空开关

- 背景（2026-09-15 18:46 次实车日志 `mas2027_nav_executor_node_44323_1789469167367.log`）：车已能到达目标（出现 `Navigation goal reached`），但表现为「慢慢挪动」。用日志里的 `required` 反推被命令的速度只有 0.04–0.40 m/s，约为 `max_velocity: 3.0` 的 13%；37 秒内 19 次 `MINCO path generation failed`、8 次 `not published: COLLISION`、5 次急停。
- 成因：净空要求随速度增长 —— `required = collision_dist + max(v * replan_react_time, monitor_margin)`（`minco_planner.cpp:1588`），而优化器的 ESDF 位置罚项使用**固定**目标 `safe_dist`（`minco_optimizer.cpp` 原 `violaPos = safe_dist - esdf_dist`）。两者在 `v = (safe_dist - collision_dist) / replan_react_time = (0.40-0.30)/0.35 ≈ 0.29 m/s` 处交叉：超过该速度，优化器规划出的轨迹必然被检查器否决 → 急停 → 从静止重规划。现场可用净空约 0.45 m 时模型预测上限 0.43 m/s，与实测 0.40 m/s 吻合。
- 改动一（配置）：`planner_params.yaml` 中 `minco_optimizer.replan_react_time` 0.35 → **0.15**，同样净空下速度上限提到约 1.0 m/s；注释写入计算依据与回退方法。
- 改动二（配置）：`mpc_params.yaml` 中 `deadzone_speed_threshold` 0.1 → **0.02**。`path_executor.cpp:116` 中线速度低于该死区即被清零（角速度仍发出），0.1 m/s 会让低速段「给一点速度就被清掉」，表现为起步顿挫、贴障挪不动。
- 改动三（新增能力，默认关闭）：**速度感知净空** —— 让优化器用与检查器相同的口径。`minco_optimizer.hpp` 新增 `Config::speed_aware_clearance` / `clearance_collision_dist` / `clearance_react_time` / `clearance_monitor_margin` 与 `ClearanceModel` 结构（罚函数是静态成员，故按值传参）；`minco_optimizer.cpp::constraintsFunctional` 在开启时把位置罚项目标净空改为 `required(v)`，并补上 `d(cost)/dv = w · pena' · t · v̂` 一项，使优化器在窄处主动减速、一次规划出可通过检查的轨迹；`minco_planner.cpp` 声明参数 `minco_optimizer.speed_aware_clearance` 并在两处 `minco_config` 构建点写入该口径；`planner_params.yaml` 增加同名参数（`false`）。关闭时行为与改动前完全一致。
- 验证：`mas2027_nav_executor` Release 构建通过；执行器 7 项 CTest 全部通过；隔离 `ROS_DOMAIN_ID=230` 直启 `mas2027_nav_executor_node`（补 `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu`，与 launch 中的设置一致）8 秒，`[ROGMap Config]` 与 `[MincoPlanner]` 正常初始化、无异常退出，说明新参数声明与 `minco_config` 接线不破坏启动；参数路径静态核对与已生效的 `replan_react_time` 同构。
- 未验证：未在实车测速，**0.15 s 的制动距离预算是否够用未经验证**（若出现制动不及，应先回调该项）；速度感知净空只验证到「关闭时行为不变 + 开启后能正常启动」，**开启后的轨迹质量、L-BFGS 收敛性与实际提速效果均未验证**；沙箱内 `ros2 param get` 因 `~/.ros` 只读无法运行，运行时参数取值未实际读出。

## 2026-09-15 — 底盘转发桥改为默认启动（use_ros2_comm）

- 现象：`/cmd_vel` 持续有非零值（当日 18:38 次运行日志中 `required` 反推线速度约 0.45–0.55 m/s）但车不动。
- 原因：`ros2_comm_node` 未运行，而它是全仓库 `/cmd_vel` 的唯一消费者（`mas2027_utils/ros2_comm/src/ros2_comm.cpp:68`；另外两处引用分别是烟测脚本与性能分析器），不启动则执行器的速度指令没有任何接收方。`nav_executor_launch.py` 中 `use_ros2_comm` 默认 `False`（原注释 "Keep hardware output opt-in for the first on-robot test"），默认 launch 不含该节点；18:38 次运行的日志目录中确实没有 `ros2_comm_node_*.log`。
- 修改：`mas2027_nav_bringup/launch/nav_executor_launch.py` 将 `use_ros2_comm` 默认值由 `False` 改为 `True`，注释改为说明它是 `/cmd_vel` 唯一消费者、协议只发 `vx`/`vy`/`nav_state`、只看不发车时可显式 `use_ros2_comm:=False`。**未改动 `ros2_comm.cpp`，协议字段保持原样（仍不含角速度）**：按本次要求不向底盘发送角速度数值，`/cmd_vel.angular.z` 依旧不出桥。`install/` 下该 launch 为符号链接，改源码即时生效。
- 验证：单独启动 `ros2 run ros2_comm ros2_comm_node`（隔离 `ROS_DOMAIN_ID=232`，`ROS_LOG_DIR`/`ROS_HOME` 指向可写目录）通过——日志确认 `UDP initialized. Target: 127.0.0.1:8889`、`Receive thread running`、`UDP Bridge Node started. Listening on port 8888`；`ss -lunp` 确认 8888 已绑定、对端 8889 在监听；`ros2 topic info /cmd_vel` 订阅数为 1；无发布者时按设计约 20 Hz 补发零速并打印 `/cmd_vel timeout, sending zero velocity`（测试期间未发布任何非零速度，车不会动）；测试后进程已终止、8888 已释放。**未验证**：8 秒内未收到对端回传的 `referee_data`（台架上通常无裁判数据），因此「底盘确实收到并执行了指令」仍未确认；未在整机 launch 下级联验证，也未在真车确认能否起步。

## 2026-09-15 — 修复近距盲区把车体自身判成障碍（cmd_vel 恒 0、轨迹闪烁）

- 现象（2026-09-15 18:25–18:30 实车日志 `~/.ros/log/mas2027_nav_executor_node_13541_1789467929023.log`）：发出目标点后车不动，日志反复出现 `Trajectory clearance 0.302 m below required 0.303 m`、`MINCO trajectory not published: COLLISION`（60 次）、`MINCO path generation failed; retrying`（141 次）、`Committed path became unsafe; braking and replanning`（12 次）、`Publishing emergency stop`（12 次）；并出现**负净空** `start clearance -0.073 m`、`Trajectory clearance -0.020 m below required 0.000 m`，即二维距离场认为**车体自己就在障碍物内部**。
- 根因：`raycast_range_min`（当前 0.3 m）以内的体素既不会产生命中（`prob_map.cpp` 中近距点被 `continue` 跳过），也不会被射线扫到（射线自 `raycast_start = (p - cur_odom).normalized() * raycast_range_min + cur_odom` 才开始推进），因此永远停留在 UNKNOWN；而 `projection_layer.cpp::applyValueAndMask()` 在 `unknown_as_occupied: true`（`planner_params.yaml` 当前取值）下把 UNKNOWN 列写成 `mask=0`（二维 ESDF 的障碍源）与 `value=254`（`kLethalCost`），于是 ESDF 在车体自身位置为 0 或负值，`QueryAdapter::isFree()`（`value < 253`）在 `path_planner` 目标准入处亦为假。唯一清理该盲区的代码块（原注释「For the first frame, clear all unknown around the robot」）被 `static bool first` 限制为**只在首帧执行一次**：机器人一旦移动或旋转离开开机位置，就重新落进未观测区并把自己关死。
- 该结论同时解释了同日早前那次 `ray_range` 下限恢复为何没能解决问题：把下限由 0.01 调回 0.3 消除了车身点云写入，但同时把不可观测盲区从 0.01 m 撑到 0.3 m，问题以另一种机制继续存在。
- 修改：`prob_map.cpp` 将首帧一次性清理改为**传感器每移动超过半个体素就重清一次**（半径与原来一致，仍为 `raycast_range_min`）；`prob_map.h` 新增 `last_near_field_clear_pos_` 与 `near_field_cleared_` 两个成员；`ProbMap::resetLocalMap()` 中复位该状态。新增 `insideLocalMap()` 过滤——`SlidingMap::getLocalIndexHash()` 不做边界检查，反复清理后越界写入会踩内存（原实现靠开机时车在图中心侥幸规避）。
- 未改变：清理半径、`unknown_as_occupied`、`ray_range`、`collision_dist` 与近场放宽判据均未改动，「未观测视为障碍」的语义保留。
- 验证：`rog_map` 与 `mas2027_nav_executor` Release 构建通过；执行器 7 项 CTest 全部通过。**未验证**：未做离线复现（`rog_map` 包没有测试目标），未在真机确认车体所在格已变为 FREE、也未确认日志不再刷 COLLISION；需重启 `nav_executor` 后先静止观察 `/rog_map/layer_type` 在车体中心格是 `33`(FREE) 而非 `-1`(UNKNOWN)，再发目标点确认 `/opt_path` 与 `/cmd_vel` 有输出。

## 2026-09-15 — 恢复 ROGMap ray_range 近距下限，避免车身自碰锁死

- `planner_params.yaml` 中 `planner.rog_map.raycasting.ray_range` 由 `[0.01, 10.0]` 改回 `[0.3, 10.0]`（与 `rog_map` 核心默认一致）。涉及 `mas2027_nav_executor/config/planner_params.yaml`。
- 现象：RViz 发目标点后目标已入队，但车不动。日志反复出现 `Trajectory clearance ≈0.28 m below required 0.30 m at (-0.01, -0.01)`、`MINCO trajectory not published: COLLISION`、`Committed path became unsafe; braking and replanning`，失败点几乎贴在车体原点。
- 原因：`ray_range` 下限过小（0.01 m）时，近距车身点云不再被跳过，会写入 ROGMap；距离场在机体系原点附近只剩约 0.28 m，低于 `collision_dist` 0.30 m 与监控阈值，规划与执行层持续急停。`prob_map` 对 `sqr_dis < sqr_raycast_range_min` 的点会 `continue` 跳过，故下限应至少覆盖车体半径。
- 验证：YAML 已改为 `[0.3, 10.0]`，`install/` 下配置副本一并同步；`git diff --check` 预期通过。需重启 `nav_executor` 后重发目标点实车确认；未改规划/控制代码，未跑 CTest。

## 2026-09-15 — RViz 增加备份安全盒（SFC）可视化

- 新增 `/nav_executor/debug/safe_corridor` 话题（`visualization_msgs/msg/MarkerArray`，Transient Local），用青色线框立方体画出 `SimpleCorridorGenerator::generateSafeBox()` 产出的安全盒，并附带 `SFC half=… m` 的半边长文字标注。发布点在 `MincoPlanner::generateBackupTraj()` 生成 SFC 之后，因此随重规划周期刷新；`cleanup()` 中一并 reset。涉及 `minco_planner.hpp`、`minco_planner.cpp`。
- 背景说明（本次排查结论）：本仓库的「走廊」只有这一个盒子，且仅作为备份（急停）轨迹优化器的约束（`backup_opt_->setPolygons({safe_poly})`，`minco_planner.cpp`）。主轨迹优化不使用盒子走廊，`minco_optimizer.cpp` 中没有任何 polygon/halfspace/SFC 代码，只用 ESDF 罚项。本次**仅新增可视化**，未改动盒子生成逻辑、约束构造或任何规划行为。
- `nav_executor_view.rviz` 新增默认启用的 `Safe Corridor (backup SFC)` 显示项（`MarkerArray`，Transient Local + Reliable，与 `/nav_executor/debug/minco_trajectory` 保持一致）。
- 验证：`mas2027_nav_executor`、`mas2027_nav_bringup` Release 构建通过；执行器 6 项 CTest 全部通过（首次运行 `rog_map_command_safety` 因沙箱内 `~/.ros` 只读、spdlog 无法写日志而失败，将 `ROS_LOG_DIR`/`ROS_HOME` 指向可写目录后通过，与本次改动无关）；RViz YAML 可解析且新显示项的 `Enabled`/`Value` 一致；`install/` 下 RViz 配置副本已确认；`git diff --check` 通过。
- 未验证：未在真机 RViz 确认立方体与文字的实际观感；未实测话题在真实重规划循环中的发布频率，以及规划停止后最后一个盒子会因 Transient Local 保留在 RViz 中这一行为。

## 2026-09-15 — 区分 layer_type 的 UNKNOWN 配色并默认显示分类层

- `/rog_map/layer_type` 中 UNKNOWN 的发布值由 `0` 改为 `-1`：新增 `ROGMapROS::kUnknownTypeValue = 255` 作为内部哨兵，`fillLayerGrid()` 对该值透传为 `-1`，其余数值仍夹到不超过 100 后按 `int8_t` 发布。此前 UNKNOWN 发 `0`，而 OccupancyGrid 语义里 `0` 是"自由"，导致 RViz 中未知区域与 FREE 同色、看起来像已探明。涉及 `rog_map_ros2.hpp`。
- 编码现为 `-1`=UNKNOWN、`33`=FREE、`66`=PASSABLE、`100`=OCCUPIED。已核对全仓库引用：`layer_type` 只用于可视化，没有规划器、测试或其他节点依赖其数值。
- `nav_executor_view.rviz` 中 `ROGMAP/Layer Type` 与 `ROGMAP/Height Analysis` 由默认关闭改为默认开启（`Enabled` 与末尾 `Value` 同步置 `true`），Alpha 分别由 0.55/0.8 调整为 0.45/0.6 以减少对其他图层的遮挡；ROGMAP 分组其余显示项与几何参数未改动。
- `rog_map/README.md` 的话题表补充 `layer_type` 的数值编码说明。
- 验证：`rog_map`、`mas2027_nav_executor`、`mas2027_nav_bringup` Release 构建通过；RViz YAML 可解析，ROGMAP 分组 8 项的 `Enabled`/`Value` 逐项核对一致；`install/` 下 RViz 配置副本已确认为新值；`git diff --check` 通过。未在真机 RViz 确认四档配色的实际观感，也未验证 `costmap` 配色下 UNKNOWN 灰度的可辨识度。

## 2026-09-15 — 提高 ROGMap 写入与可视化的 Z 范围

- `planner_params.yaml` 中 `planner.rog_map` 的三项 Z 参数调整：`raycasting.local_update_box` 由 2.5 提到 4.0、`visualization.range` 由 1.5 提到 4.0、`map_size` 由 2.5 改回 4.5。目标是让 `/rog_map/occupied` 能显示更高的墙体（原上限约 ±0.75 m）。涉及 `mas2027_nav_executor/config/planner_params.yaml`。
- 排查结论（已写入配置注释）：Z 方向可用高度由四道限制串联决定——`raycasting.local_update_box`/2 决定点云能否写入（`prob_map.cpp` 中超出盒子的点被投影到盒壁并记 `update_hit=false`，永不成为占据体素）、`visualization.range`/2 决定能否发布、`virtual_ceil_height`/`virtual_ground_height` 是绝对上下界、`map_size[2]`/2 是容器上限（`updateLocalBox()` 与 `boundBoxByLocalMap()` 都会按局部地图边界夹取）。因此只抬高其中一两项不会生效，`map_size` 是其余各项的天花板。
- 调整后实际能力：可写入与可显示 z 半高均为 2.00 m，扣除 `map_sliding.threshold` 0.2 的跟随滞后约 1.80 m；下边界由 `virtual_ground_height` 限制在 -1.00 m。更早一版曾只改前两项而 `map_size[2]` 为 2.5，当时上限仍被夹在 ±1.25 m，故补上 `map_size` 一并调整。
- 验证：YAML 可解析，四道限制的取值用脚本逐项核对通过；`mas2027_nav_executor` 构建通过，`install/` 下配置副本已确认同步为 4.5/4.0/4.0。未在真机 RViz 观察紫色块高度变化；未评估抬高 `local_update_box` 后 raycast 清空射线的耗时增量，也未验证提升 z 范围后的建图帧耗时。

## 2026-09-15 — 为 ROGMap ROS 2 适配层补充中文注释

## 2026-09-15 — 为 ROGMap ROS 2 适配层补充中文注释

- 仅新增注释，未改动任何代码逻辑、接口、参数或话题。`rog_map_ros2.hpp` 增加文件级总览（类职责、线程模型、关键约定），并为成员变量、`getPriorMapTransform`/`bindNode`、`create*` 封装、`ROSCallback` 接力区、`odomCallback`/`cloudCallback`/`watchdogCallback`、`updateWorkerLoop`、`captureVizFrame`/`vizCallback`、`hasVisualizationSubscriber`、各 `fill*` 与 `initializeRos`、构造函数与静态 `visualize*` 辅助函数补充逐段说明。
- 注释中记录了若干阅读时易踩的既有约定：`updateMap()` 在 `ros_callback_en` 下不生效、`map_io_mutex_` 目前仅更新线程持有、`updete_lock` 为上游拼写、可视化 heavy 层限频 0.5 s、`fillLayerGrid` 会把 254/255 一并夹到 100、`/rog_map/layer_value` 实际发布二值 mask、`fillLayerHeightDeltaCloud` 对 FREE 单元会发布未定义的 z。
- 验证：`rog_map` 与 `mas2027_nav_executor` Release 构建通过，`git diff --check` 通过。注释内容为静态阅读所得，未在真机运行时逐条复核（如 `use_intra_process_comm` 在当前跨进程部署下的实际效果）。

## 2026-09-14 — 修正 ROGMap 边界标记时间戳并重建

- `rog_map_ros2.hpp` 的 Map Bound 标记直接传递 `rclcpp::Time`，不再把秒数误作纳秒；修正边界线、文字和原点标记的时间戳。未更改地图或规划逻辑。
- 验证：`interfaces`、`rog_map`、`mas2027_nav_executor` Release 构建通过，`git diff --check` 通过。当前运行的导航进程未重启，尚未在重启后的 RViz 验证 `Map Bound` 状态。

## 2026-09-14 — ROGMap 占据点改为立方体显示

- RViz 的 `ROGMAP/Occupied` 由 `Points` 改为 `Boxes`，保留 0.05 m 尺寸，与 ROGMap 当前 0.05 m 体素分辨率一致；仅影响渲染，不改地图、规划或 `/rog_map/occupied` 消息。涉及 `nav_executor_view.rviz` 与 README。
- 验证：RViz YAML 可解析，`Occupied` 的 `Boxes` 样式、0.05 m 尺寸及默认启用状态校验通过；`mas2027_nav_bringup` 构建与 `git diff --check` 通过。未在真机 RViz 检查视觉效果，上一轮日志中的 QoS 不兼容需单独确认。

## 2026-09-14 — RViz 增加 ROGMap Map Bound

- `ROGMAP` 分组新增默认启用的 `MarkerArray` 显示项 `/rog_map/map_bound`，使用发布器匹配的 Best Effort/Volatile QoS；显示可视范围、局部地图范围及更新范围，保留已有占据点、距离场和轨迹显示。涉及 `nav_executor_view.rviz` 与 README。
- 验证：RViz YAML 可解析，`Map Bound` 的类型、启用状态、话题及 QoS 校验通过；`mas2027_nav_bringup` 构建与 `git diff --check` 通过。未连接真机检查渲染效果。

## 2026-09-14 — 参考 navi_minco_bit 整理 ROGMap RViz 显示

- 将原单项 `/rog_map/occupied` 改为 `ROGMAP` 分组，参照 `navi_minco_bit` 配置动态/静态/合成投影栅格、类型图、2D 距离场、高度分析和占据点；话题与本仓库 `ROGMapVisualizer` 发布器逐项核对，统一使用 Best Effort/Volatile。占据点和低透明度距离场默认启用，诊断栅格默认关闭且绘制在背景，保留全局路径和 MINCO 轨迹可见性。涉及 `nav_executor_view.rviz` 和 README。
- 验证：`mas2027_nav_bringup` 构建通过；RViz YAML 可解析，`ROGMAP` 分组含 7 个预期话题且启用状态符合配置；`git diff --check` 通过。未在真机 RViz 渲染、TF 和实时点云上实测；ROGMap 先验图融合关闭，静态层显示项仅供诊断。

## 2026-09-14 — 将 ROGMap 接回在线规划与制动

- `nav_executor_planner` 重新在进程内构造 ROGMap，订阅 `/cloud_registered` 和 `/Odometry`；MINCO 走廊、优化与轨迹安全改用其在线占据和距离查询。全向 Kino A* 在 ROGMap 滑窗内叠加其占据判断，滑窗外继续使用静态地形与 `map_server` 当前动态代价图；MPC 下一段指令增加可调净距 `node.rog_map_clearance` 检查，缺失查询时制动，并拒绝 ROGMap/规划/里程计帧名不一致的配置。底盘 `/cmd_vel` 接口与无预测行为不变。涉及执行器规划、搜索、安全监测、参数、CMake/package 依赖。
- 将 MINCO 的同名兼容查询接口改为包含 ROGMap 原生接口，避免重复定义和 ABI 分裂；RViz 增加紫色 `/rog_map/occupied` 在线点层，README 更新链路与实车标定说明。
- 验证：`mas2027_nav_executor` 与 `mas2027_nav_bringup` Release 编译通过；执行器 6 项 CTest（含 ROGMap 净距制动回归）通过；参数/RViz YAML 与 `git diff --check` 通过；隔离 ROS 域启动执行器 10 秒，确认加载 ROGMap 配置和注入 MINCO 查询，因未接入点云出现预期的无输入警告。未连接双雷达、未验证地面投影/外参及真车闭环；启动前需静止检查 `/rog_map/occupied` 与轨迹净距。

## 2026-09-14 — 稳定当前动态障碍的短时漏检

- `map_server` 在点数与连通域筛选后、代价膨胀前，对已检出的栅格保留最近观测时间；新障碍立即进入 `/dynamic_cost_map`，短时漏检默认保留 0.3 s，持续无观测则清除。参数 `local_map.dropout_hold_seconds` 可调，设为 0 可关闭。规划器与 RViz 共用同一稳定后的地图；未把持续不匹配的静止障碍从避障图中删除。涉及 `map_server_node.cpp/.hpp`、点云烟测和 README。
- 验证：`map_server` Release 编译通过；隔离 `ROS_DOMAIN_ID=229` 的合成点云烟测覆盖首次检出、短时漏检保留和超时清除；`git diff --check` 通过。未重启当前真机导航进程、未实测先验 PCD 与实时点云配准或 RViz 闪烁改善。

## 2026-09-13 — 接入全向 Kino 搜索与弧长速度种子

- 全局搜索改为全向底盘的速度向量格点 A*：以位置、运动方向、速度档为状态，按速度/加速度可行性扩展，逐边检查静态地形方向、当前动态障碍及软代价；从实测里程计速度出发，不引入未来障碍预测。涉及 `path_planner/search/omni_kino_astar.*`、`global_path_searcher.*`、`minco_planner.cpp`、CMake。
- 在 MINCO 分段时间分配前加入弧长速度剖面：按速度平方前后向传播、角点速度上限和加速/巡航/制动时间形成时间种子；优化后的 MINCO 轨迹仍是唯一执行与安全检查时间轴。起始速度与可用刹车距离不相容时保留原保守时间种子。涉及 `path_planner/trajectory/arc_length_speed_profile.*`、`minco_planner.cpp`、README 及启动日志。
- 验证：`mas2027_nav_executor` 编译、地形/MPC/全向搜索/弧长速度剖面共五项单元测试、隔离 ROS 域双目标抢占烟测及 `git diff --check` 通过；烟测在第二目标后曾出现净距 0.490 m 低于运行监测阈值 0.500 m 的安全制动警告，需实车标定地图与余量。未连接真机验证全向加减速、贴障绕行或实地语义方向。原 HW 曲率原语与指导走廊、台阶速度窗、后置 MINCO 速度优化和腿式 LPV/FDDP 未原样迁入，当前不宣称完整 HW 等价。

## 2026-09-13 — 适配全向底盘转向与航向跨界

- 确认底盘执行 `/cmd_vel.linear.y` 后，保留 MPC 的二维全局速度到车体系 `x/y` 转换；解除配置中角速度/角加速度的零锁定，让规划航向可以由 MPC 跟踪，并对叠加 `/cmd_spin` 后的角速度施加限幅。涉及 `path_executor/mpc/mpc_solver.cpp`、`path_executor/path_executor.cpp`、`config/mpc_params.yaml`。
- MPC 参考航向逐点按最短角差展开，避免穿过 ±π 时绕远路；增加侧向、转向及正反两个跨界方向的回归检查，README 记录全向控制接口与实车限值标定要求。
- 验证：`mas2027_nav_executor` 编译、地形栅格/距离场/MPC 三项单元测试、隔离 ROS 域的双目标抢占烟测、launch 参数解析和 `git diff --check` 通过；未连接真机验证侧向、转向动力学或 `/cmd_spin` 叠加效果。HW 的 Kino A*、弧长速度剖面和 FDDP 仍未移植，不宣称与 HW 全部等价。

## 2026-09-13 — 拆分导航执行器并移除 MincoFsm

- `TaskManager` 接管目标、周期重规划、原有的限时推离恢复与执行许可；删除 `MincoFsm` 及其内部 20 Hz 定时器，`MincoPlanner` 不再管理目标生命周期。新目标抢占后以轨迹时间戳拒绝晚到旧轨迹；规划失败或轨迹失效时停止输出运动命令，里程计超时同样制动。涉及 `task_manager/`、`path_planner/`、`path_executor/`、节点与 `node_params.yaml`。
- 按功能整理 `path_planner/search`、`path_planner/trajectory`、`path_executor/mpc`、`path_executor/monitoring`、`path_executor/state`、`common/environment` 的实际源码与头文件；更新 CMake、单元测试和 README。`vendor/` 保留数值计算后端与 qpOASES。
- 验证：`mas2027_nav_executor` 编译、地形栅格/距离场/MPC 三项单元测试、隔离 ROS 域的单目标与双目标抢占烟测通过，`git diff --check` 通过；未连接真机验证闭环控制或触发贴障推离恢复。HW Kino A*、弧长速度剖面与 FDDP 尚未迁入，当前不宣称算法等价。

## 2026-09-13 — 移除先验定位的 Z 轴锁定

- 删除 `odom_localizer` 的 `update.lock_z` 参数及其变换裁剪代码；GICP 估计的完整六自由度 `map→odom` 变换现在直接进入 EMA 与 TF 发布，允许实时点云与先验 PCD 在地板、天花板高度上对齐。涉及 `params.yaml`、`odom_localizer_node.hpp/.cpp` 与 README。
- 验证：完成源码引用扫描与 `git diff --check`；尚未使用真机点云确认 GICP 的 Z 估计稳定性，启动后需在 RViz 检查先验/实时云重合情况。

## 2026-09-13 — 修复轨迹距离场热路径并记录安全拒绝原因

- `TerrainMapQuery` 不再在每次距离查询时复制整张约束图；`TerrainGrid` 为地图内容维护版本号，只有静态/动态栅格发生变化才重建二维距离场，重复动态帧仅更新新鲜时间戳。涉及 `terrain_grid.hpp/.cpp`、`terrain_map_query.hpp/.cpp` 及单元测试。
- `PathPlanner` 将原“Accepted goal”改为准确的“Queued goal”；`MincoPlanner` 在局部轨迹未发布时节流记录失败原因，安全检查器记录位置、实际净距与所需净距，便于区分规划耗时与安全拒绝。涉及 `path_planner.cpp`、`minco_planner.cpp`、`trajectory_safety_checker.cpp` 和 README。
- 实时只读诊断：RViz Goal、执行器订阅与 RViz 轨迹订阅匹配，起点和目标处于同一可通行区域；当前机器人静态地图净距约 0.255 m，小于配置的 0.30 m 轨迹碰撞距离。因此保留安全阈值，未通过降低半径强行发布轨迹。
- 验证：`mas2027_nav_executor` 重新编译通过；隔离 ROS 域同一目标的 MINCO 优化耗时从约 5.6 s 降到约 0.0005 s，生成 52 点轨迹；地图版本与距离更新测试及执行器全量 3 项单元测试通过；`git diff --check` 通过。未在真机重新发送 Goal 或验证实际墙体距离。

## 2026-09-13 — 忽略编辑器缓存、补足贴地障碍检测及目标诊断

- `.gitignore` 忽略 clangd 生成的 `.cache/`。`map_server` 缩小地面平面内点容差，并对高于地面的低矮点采用更严格的先验 PCD 匹配距离，避免 0.20 m 的原距离阈值把贴地障碍误判为先验地面；保留可调的最小离地高度与匹配距离参数。涉及 `map_server_node.cpp/.hpp`。
- `nav_executor` 在收到 RViz Goal、缺少规划里程计及接受目标时打印明确日志，便于区分“目标未送达”和“地图/TF/里程计不就绪”；涉及 `nav_executor_node.cpp`、`path_planner.cpp`。新增贴地障碍 PCD 夹具、点云回归与隔离 ROS 域的 Goal→轨迹烟测，并更新 README。
- 验证：`map_server`、`mas2027_nav_executor` 编译通过；贴地 11 cm 障碍与地面点云烟测通过；隔离 ROS 域中原 RViz 目标 `(2.56, 0.44)` 生成 52 点轨迹；执行器 3 项单元测试通过；`.cache/` 忽略规则和 `git diff --check` 通过。未连接真机复现当时的动态地图或检验实地低矮障碍，相关高度/配准阈值仍须实测标定。

## 2026-09-13 — RViz 单独显示当前动态障碍

- `nav_executor` 从 `/dynamic_cost_map` 当前帧提取硬阻塞格，发布青色点标记 `/nav_executor/debug/dynamic_obstacles`；收到空帧时发布空标记清除旧点。RViz 默认启用独立的动态障碍点层，避免静态/动态都为红色时难以区分，点层高于地形图且不会整幅遮挡轨迹。
- 验证：执行器编译通过，地形栅格/距离场/MPC 三项单元测试通过，RViz 配置可解析且 `git diff --check` 通过。此轮未重复运行 `map_server` 点云烟测，也未在真机 RViz 中检查实际渲染。

## 2026-09-13 — 移除 ROGMap 在线链路

- `nav_executor` 不再实例化、链接或声明 `rog_map`；移除其启动依赖、规划参数和 RViz 显示。`map_server` 发布的静态地形与当前动态栅格现在是唯一在线地图输入。
- 新增执行器内置 `TerrainMapQuery`：将合并后的二维约束栅格转换为 signed-distance field 与 XY 梯度，并注入 MINCO 的全局搜索、走廊、优化器和轨迹安全检查，替换原 ROGMap distance/gradient 查询。
- 验证：`mas2027_nav_executor`、`mas2027_nav_bringup` 编译通过；地形栅格、二维距离场与 MPC 单元测试通过；RViz YAML 和启动参数可解析。未进行真机闭环验证。

## 2026-09-13 — 移除动态障碍预测并收紧静态图膨胀

- 删除 `CostMaps` 消息、目标跟踪器、预测渲染器及 20 帧预测链路；`map_server` 改为仅发布 `/dynamic_cost_map` 当前帧，`nav_executor` 的全局搜索、MINCO 轨迹验收与 MPC 制动统一读取该当前动态图。
- 默认静态地形图膨胀由 0.20 m 满代价 + 0.60 m 截止调整为原始障碍硬约束 + 0.25 m 软代价，避免过厚的先验地图额外挤占可通行空间；ROGMap 的局部 ESDF 检查仍保留。
- 原因：已配准点云受底盘振动影响时，静止物体会产生虚假速度，未来预测会将其错误扩大；现阶段先验地图与实地地形厚度也未完成标定。
- 验证：`interfaces`、`map_server`、`mas2027_nav_executor` 编译通过；当前动态图烟测确认地面不占用、抬高障碍可检出；执行器地形/MPC 单元测试通过。未进行真机验证。

## 2026-09-13 — 修复规划约束图遮挡轨迹

- 默认 RViz 视图改用 `/planning_constraints_markers` 高对比度点层，并按执行器实际约束着色：红色为当前硬阻塞、橙色为预测占用、黄色为方向受限；点层抬高到地图平面上方，不再被整张不透明栅格遮住路径。
- `/planning_constraints` 完整栅格保留为默认关闭的诊断显示，且始终绘制在轨迹后方；原始地形代价图继续作为半透明背景。
- 验证：待重新编译后运行执行器单元测试与 RViz 配置解析；未连接真机或实际 RViz 渲染检查。

## 2026-09-13 — RViz 高对比度规划约束图

- `nav_executor` 新增瞬态 `/planning_constraints`：红色数值表示静态地形或当前动态层的硬阻塞，橙色表示未来 2.0 s 内的预测占用，黄色表示方向受限地形；图由执行器实际使用的地形/动态判定生成。
- RViz 将原始 `HW Cost Map` 调为半透明背景，并默认启用不透明的 `Planning Constraints` 前景层；README 记录颜色含义。
- 验证：补充地形栅格单元测试，覆盖静态阻塞、方向约束、当前动态与未来动态四种可视化值；尚未启动 RViz 实测配色。

## 2026-09-13 — 动态地图地面误检抑制

- `map_server` 在先验 PCD 差分前，对局部点云做近水平、位于车体下方且具有足够点数的地面平面剔除，避免稀疏或小幅错位的先验点云把地面投影成动态障碍；保留高于地面的障碍点与原有 21 帧预测输出。涉及 `map_server_node.cpp`、PCL 构建依赖及合成点云烟测。
- 验证：`map_server` 编译通过；合成地面单独输入时当前代价图为空，加入抬高障碍物后当前代价图非空且仍为当前帧 + 20 帧、步长 0.1 s。未用真机点云验证，也未排除实车 TF/PCD 配准偏差。

## 2026-09-13 — 静态地形约束与动态预测代价图接入原生执行器

- `map_server` 从 `/cloud_registered` 与先验 `lab3.pcd` 提取动态点，经过 ROI、体素降采样、栅格聚类、目标跟踪和预测渲染，在 `/cost_maps` 发布当前帧 + 20 个未来帧（`prediction_dt=0.1 s`）；`/cost_map`、`/direction_map` 仍是独立静态地形层。
- `mas2027_nav_executor` 新增地形栅格快照：全局 A* 融合静态地形障碍/方向与动态当前帧，局部路径稀疏化和 MINCO 轨迹验收加入地形检查，`/cmd_vel` 跟踪器对地形和预测占用执行制动保护；保留 ROGMap 的局部 ESDF 和原底盘接口，不移植腿部模式。
- MPC 参考时域扩展为 2.0 s，制动后重置求解器的上一帧控制状态；启动文件启用 `map_server` 动态检测并配置静态 PCD 与 PCL 运行库路径；README 说明现有能力及尚未移植的 HW Kino A*、速度剖面、FDDP 与预测障碍绕行优化。
- 验证：`map_server`、`mas2027_nav_executor`、`mas2027_nav_bringup` 编译通过；地形搜索、预测渲染、40 步 MPC 单元测试通过（单次空障碍求解约 2.7 ms）；合成点云烟测收到 21 帧、0.1 s 步长且当前帧有动态占用；launch 参数可解析、`git diff --check` 通过。未做真机控制或完整导航闭环测试。

## 2026-09-12 — 清理独立执行器的遗留表述

- 将 LIO、ROGMap 独立节点、轨迹消息、地形图转换脚本和 MINCO 内部注释改为独立执行器语义，避免继续暗示旧 Nav2 链路仍可运行。
- 更新 Nav2 移除记录的验证结论：`rog_map`、`mas2027_nav_executor`、`mas2027_nav_bringup` 均已编译，独立 launch 参数可解析，包清单不再声明 Nav2 依赖。
- 验证：运行 `git diff --check`、依赖扫描与 `ros2 launch mas2027_nav_bringup nav_executor_launch.py --show-args`；未启动真车硬件节点。

## 2026-09-12 — 移除 Nav2 并完成执行器解耦

- 删除 Nav2 server 启动、参数、Behavior Tree、默认视图以及 `pb_nav2_plugins`、`fake_vel_transform`、航点/BT 编辑器等旧链路。
- 删除 `minco_planner`、`minco_controller` ROS 插件包；把实际使用的 MINCO、MPC、qpOASES 源码迁入 `mas2027_nav_executor/vendor`。
- 规划器固定为 ROGMap `EXPLORATION`，移除 Nav2 Costmap、SMAC、pluginlib 与 `nav2_util` 依赖；ROGMap 参数声明改用标准 rclcpp API。
- `nav_executor_launch.py` 不再使用 `nav2_common/RewrittenYaml`，独立执行器成为唯一在线导航入口。
- 重写根 README 和导航说明，删除已失效的 Nav2 架构文档。
- 验证：`rog_map` 与 `mas2027_nav_executor` 编译通过；`mas2027_nav_bringup` 在删除空 Behavior Tree 安装项后重新验证。

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
