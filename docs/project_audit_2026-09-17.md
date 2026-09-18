# 项目问题审查报告（2026-09-17）

本报告是对 `mas_nav_2027_native` 全工作区的一次**只读**审查（读码 + 实测数据复核），目标不是复述文档，
而是给出「当前代码/配置里真实存在的问题、证据、影响与最小修复」。

- 审查范围：`src/` 下全部自有包（nav_executor、perception 各包、utils、bringup、robot_description、third_party）
  与 `src/mas2027_nav_executor/config/*.yaml`、`src/mas2027_nav_bringup/**`。`vendor/` 只看调用契约。
- 本次实测（以下结论中的"实测"均指这几项，不是引用文档）：
  1. `ctest`（`build/mas2027_nav_executor`）**9/9 通过**，0.18 s；
  2. 端到端冒烟 `test/smoke_goal.py` + 原始 `lab3_terrain.msgpack` → `goal smoke passed: 52 trajectory poses,
     48 global plan poses, marker width 0.150 m`（退出码 0）；
  3. **最新一次实车运行的数据此前未被分析过**：`~/.ros/log/mas2027_nav_executor_node_80395_1789567142727.log`
     （2026-09-16 21:59–22:04，374 行）+ 同一次运行的 `.scratch/rog_map_perf_summary.csv`（300 个 1 s 窗口）；
  4. 独立解包 `src/mas2027_nav_bringup/pcd/lab3.pcd` 复核静态地图里的"墙"是否真实。
- 所有引用行号都以本次读到的文件内容为准。凡未实测、只有代码依据的推断，都在条目里标了「较可能 / 待验证」。

> 结论摘要：工程整体成熟度很高（注释密度、失败原因分类、回归脚本都少见地完备），**没有发现必然撞车或
> 必然崩溃的缺陷**；但存在 7 条**高优先级**问题（§1）：其中 5 条正在制造现场"卡"或让安全门/诊断失效
> （四道净空门两套阈值、种子门零容差 + verdict 分类口径错、优化器看不到静态墙、`robot_state_` 竞态、
> 动态图层 0.5 s 陈旧门），另 2 条会**毁掉现场证据或让换图流程跑不通**（冒烟脚本清空实车 CSV、
> 离线建图脚本引用已删除的 launch）。此外还有一批会让"按日志/按参数调优"落空的中等问题（§2）。
> 建议的动手顺序见 §5。

---

## 0. 本次实车数据基线（后文反复引用）

最新一次运行（21:59–22:04，308 s）的成绩与瓶颈：

| 指标 | 值 | 说明 |
| --- | --- | --- |
| 目标 / 到达 | **21 / 18** | 每目标 14.7 s，明显好于 21:30 那次的 64 s/目标 |
| `MINCO trajectory not published` | 39 | 采样后可见原因 12 LOCAL_SEED_INVALID / 11 TERRAIN / 11 COLLISION / 5 OPTIMIZER |
| 失败汇总（第 100 次） | `COLLISION=28 LOCAL_SEED_INVALID=39 OPTIMIZER_FAILED=5 TERRAIN_COLLISION_OR_DIRECTION=28` |  |
| `Braking:` | 13 | **全部** `reason=clearance`；其中 9 次 threshold=0.280、value 0.262~0.279（见 §1.1） |
| 急停 / 轨迹判不安全 | 8 / 9 | |
| `Repaired a ROGMap-blocked global segment` | 50 | 局部绕行修复仍在高频触发 |
| `Live obstacle blocks ...` | 17 | verdict：**16 GEOMETRY + 1 SEED_GATE_STRICTER** |
| `Terrain rejection` | 10 | 全部 `stage=edge_static`，`cost=100` 而 `rog_clear=0.74~1.03 / rog_free=1 / rog_value=255` |
| 建图（CSV 中位） | 点云回调 **28.78 Hz** → 建图 **20.02 Hz** | 丢帧 **29.6%**；单轮周期中位 **45.7 ms**、p95 **94.0 ms**、max **150.5 ms** |
| 分阶段耗时（中位） | raycast 6.2 + projection 12.8 + prob 6.2 + decay 2.3 + query 0.87 + field 1.5 ≈ **29.9 ms** | 自报 total 37.6 ms，未计入 3.2 ms |
| 脏列增量 | `full_reason_dirty_over_ratio` 300/300 为 0，`periodic` 20/300 | **增量路径这次真的生效了**（见 §4.1） |

---

## 1. 高优先级：正在制造现场"卡"或让安全门/诊断失效

### 1.1 【高】四道净空门用了**两套**有效阈值 → "规划放行、执行刹车"

- 位置与证据：
  - 发布前校验与运行时监视：`src/mas2027_nav_executor/src/path_planner/trajectory/minco_planner.cpp:25`
    `kMonitorClearanceTolerance = 0.02`，`:1925` / `:1962` 都用
    `requiredClearance(v) - kMonitorClearanceTolerance` → 有效 **0.26**（`collision_dist 0.28`、
    `replan_react_time 0.0`、`monitor_margin 0.0`）。
  - MPC 指令门：`src/mas2027_nav_executor/src/path_executor/monitoring/command_safety.cpp:63-64,75`
    用**裸** `rog_map_clearance`（0.28），判据 `clearance.distance <= required` → 有效 **0.28**。
  - 种子门：`.../trajectory/local_path_processor.cpp:329-339` 也是裸 `collision_dist_`（0.28），判据 `<=`。
- 实测证据：最新一次运行 13 次 `Braking:` **全部**是 `reason=clearance`，其中 **9 次**
  `threshold=0.280`、实测 `value` = 0.262 / 0.269 / 0.274 / 0.276 / 0.276 / 0.278 / 0.278 / 0.279 / 0.279
  —— **全部落在 (0.26, 0.28] 区间**，即能通过规划侧发布门与 20 Hz 监视（0.26）、却被执行门（0.28）否决。
  另外 4 次执行门用的是近场下调后的阈值（0.238 / 0.255 / 0.266），属两侧一致的真实否决。
- 影响：净空落在 (0.26, 0.28] 的轨迹会被"发布→立刻刹车"，表现为周期性顿挫；
  与 `planner_params.yaml:110-116` 里"发布门与监视门必须同一条公式"的既定教训是同一类问题，
  只是这次不一致发生在**规划门 ↔ 执行门**之间。
- 建议：把这一条判据收进 `common/environment/clearance_gate.hpp` 一处（例如
  `requiredClearance(v, collision_dist, react_time, monitor_margin) - kMonitorClearanceTolerance`），
  三处调用同一个函数；或在 `command_safety` 显式减同一个容差常量。
- 置信度：确定（四处数值逐个核对 + 现场 13 条日志吻合）。

### 1.2 【高】种子门零容差 + `verdict=` 分类口径错 → **11/17 次"其实能过"被报成 GEOMETRY**

- 位置：`src/mas2027_nav_executor/src/path_planner/trajectory/local_path_processor.cpp:27-65`
  （`classifySeedReject`）与 `:329-339`（`segmentClear` 的实际判据）。
- 两处规则**不同源**：
  - `segmentClear`：只有当 `start_clearance < collision_dist_`（0.28）时才放宽为
    `start_clearance - 0.02`，否则全程用 0.28，且判据是**严格大于**（零容差）。
  - `classifySeedReject`：只要 `arc < collision_dist` 就按 `start_clearance - 0.02` 算"近场规则"，
    **没有** `start_clearance < collision_dist` 这个前置条件，也**没有**用轨迹门的 0.26 作基线。
- 实测证据（最新运行 17 条 `Live obstacle blocks` 逐条复算）：
  - **11/17** 满足 `start_clear >= 0.28` 且 `arc < 0.28`：分类器用 0.29~0.35 判 → 报 `GEOMETRY`
    （日志含义是"连近场规则都过不了，拒绝是正确行为，别动阈值"），而实际门是 0.28、轨迹门是 0.26。
    例：`start_clear=0.370 arc=0.12 clear=0.279 req=0.280` → 差 **1 mm**；
    `start_clear=0.317 arc=0.04 clear=0.275 req=0.280` → 差 5 mm。
  - 5/17 的差 ≤ 5 mm（`clear=0.133 req=0.134` 这类 1~3 mm 的抖动级差距）。
  - 只有 1/17 报出了正确的 `SEED_GATE_STRICTER`。
- 影响：操作者按日志判读会得出"几何上真过不去 → 不要动阈值"，而真实成因是
  **种子门零容差把 1~19 mm 的 ESDF 差距放大成整条种子无效**（车停），
  且这些点按轨迹门（0.26）本可通过。这条正是 `docs/refusal_triage_2026-09-16.md` §1.3 与
  `change-history.md` 2026-09-16 "结论 2" 里标注"已定位、本轮未修"的判读陷阱，**至今未修**。
- 建议（二选一，建议都做）：
  1. `classifySeedReject` 的近场基线改用轨迹门口径
     （`in_near_field ? min(collision_dist - tol, max(0, start_clear - slack)) : collision_dist - tol`），
     并补一条单测锁定"同一对数字只改 arc 得到不同 verdict"（现有测试只覆盖了部分分支）；
  2. 给种子门与执行门同样加 `kMonitorClearanceTolerance`（保持 0.26 底线不变），
     让四道门真正同源。
- 置信度：确定（分类器与门两条代码路径逐行核对 + 17 条日志逐条复算）。

### 1.3 【高】优化器看不到静态先验图 → MINCO 穿墙 → 地形门事后否决

- 位置：`minco_planner.cpp:1754-1888`（地形只在 `validateTrajectory` 事后否决）；
  `planner_params.yaml:274-278`（`unknown_as_occupied: false`：未知格按自由进 ESDF）。
- 实测证据（最新运行）：
  - 10 条 `Terrain rejection #N: stage=edge_static map=(5.97,2.11) cell=(211,200) cost=100
    prev_cost=56/66 ... | rog_clear=0.961 rog_free=1 rog_value=255 rog_status=OK`
    —— 静态图说"占据"，ROGMap 同时给出 **1 m 净空且该格是 unknown(255)**；
  - 失败汇总里 `TERRAIN_COLLISION_OR_DIRECTION=28/100`；
  - **独立复核**：解包 `lab3.pcd`，map (5.90~6.10, 2.00~2.20) 有 **19 个点，z 0.06~3.89 m（中位 2.14）**，
    放宽到 (5.7~6.3, 1.9~2.3) 有 42 点 → **是真实墙体，地形门的拒绝是对的**，
    问题在"优化器唯一看得见的 ROGMap 在那里没有观测、且未知被当自由"。
- 影响：MINCO 生成穿墙轨迹 → 被发布门/MPC 门否掉 → 车停或反复重规划。这正是 `src/README.md`
  里写明的"结构性成因"，本次数据说明它仍是最大的单一失败类别，且**无法靠调净空阈值解决**。
- 建议（按代价排序）：
  1. 让 `unknown_as_occupied` 在**静态先验已知为占据**的格子上始终为真（把地形图作为 ROGMap 二维层的先验，
     `projection.prior_map` 已有实现，只是因对齐问题被关闭）；
  2. 或把地形层直接作为走廊生成/优化器的硬约束（当前 `corridor_generator` 只吃 ESDF）；
  3. 短期至少让 `verdict`/日志把这类失败单独归类（现在是 TERRAIN_COLLISION_OR_DIRECTION）。
- 置信度：确定（日志 + 点云双向验证）。

### 1.4 【高】ROGMap `robot_state_` 被三个线程无锁/半锁读写

- 位置：`src/mas2027_perception/rog_map/include/rog_map_ros/rog_map_ros2.hpp:292-299`（odom 回调持
  `rc_.updete_lock` 写）、`:323`/`:331`/`:356-357`（cloud 回调**不持锁**读
  `robot_state_.rcv / .rcv_time / .p / .q`）、
  `src/mas2027_perception/rog_map/src/rog_map/rog_map.cpp:328` → `:910-918`
  （**更新线程**无锁写 `updateRobotState()`）。
- 证据：`RobotState` 是 `Vec3f p,v,a,j; Quatf q;`（`include/super_utils/type_utils.hpp:81-88`），
  非原子。`odom_me_cbk_group` 与 `cloud_me_cbk_group` 是两个独立的 MutuallyExclusive 组
  （`rog_map_ros2.hpp:1040-1043`），且更新线程是独立的 `std::thread`（`:1063`）——
  **该竞争与执行器类型无关，必然存在**。
- 影响：撕裂读会得到"半新半旧的位姿/四元数"→ raycast 起点错、`refreshLayers` 的 z 窗口取错、
  `rc_.pc_pose` 与点云不匹配 → 地图错位/瞬时幽灵障碍或漏检。属于"偶发、难复现"的地图错误来源。
- 建议：让 `updateRobotState()` 自持 `rc_.updete_lock`（读侧同锁），或改成"回调写输入槽、
  更新线程取走"的所有权模型；至少让 cloud 回调的读也进同一把锁。
- 置信度：确定（三处代码路径逐行核对）。

### 1.5 【高】动态图层"新鲜度"阈值 = 发布周期（0.5 s），零余量 → 目标被静默丢弃 / 指令门可能急刹

- 位置：`src/mas2027_nav_executor/src/path_planner/path_planner.cpp:70-76`
  （`std::abs(now - stamp) > 0.5` → `Ignoring goal until a fresh dynamic map is ready`）；
  `command_safety.cpp:39-42`（同一阈值 → `dynamic_stale` 拒绝整条指令）。
  发布侧：`src/mas2027_perception/map_server/src/map_server_node.cpp:94`
  `create_wall_timer(500ms, publish_maps)`，且 `bypass_dynamic_obstacle: True` 时也每 500 ms
  发一帧**全 0** 的 `/dynamic_cost_map`（`:106-111`）。
- 实测证据（全部 `~/.ros/log/mas2027_nav_executor_node_*.log`，共 14 次）：
  逐条计算"该次拒绝距该进程首条日志的秒数"：**12 次发生在运行 20 s 之后**（65 s / 74.6 s / 112.7 s /
  187 s / 232 s / 541.6 s / 578.2 s / 676.6 s / 687.1 s / 702.1 s …），只有 2 次在启动期。
  即这不是"启动没数据"，而是运行中途 map_server 定时器抖动使 age 略超 500 ms 导致的**目标静默丢弃**。
- 影响：操作者点击目标没有任何反应（`nav_executor_node` 端只有一条 2 s 节流的 WARN）；
  同一判据用在 MPC 指令门时后果更重——抖动越界会让正在行驶的车**立即发零速**。
  另外：该图层当前被刻意旁路成空图（`launch: bypass_dynamic_obstacle: True`），
  却仍是"能否接受目标/能否发指令"的**硬门**，等于让一个空图层掌握生杀权。
- 建议：阈值取发布周期的 ≥3 倍并**参数化**（例如 1.5 s），或把 map_server 发布周期降到 100 ms；
  并且"图层为空但新鲜"与"图层缺失"应区别对待（当前两者都硬拒）。
- 置信度：确定（代码 + 14 条现场日志）。

### 1.6 【高】`test/smoke_goal_motion.py` 每跑一次就会**清空实车性能 CSV**

- 位置：`src/mas2027_nav_executor/test/smoke_goal_motion.py:151-157`
  （把**真配置目录**直接喂给 nav_executor：`--params-file <config>/planner_params.yaml`）；
  `planner_params.yaml:327` 的 `summary_csv_path` 指向工作区 `.scratch/rog_map_perf_summary.csv`；
  `performance_monitor.cpp:101` 以 `std::ios::out | std::ios::trunc` 打开。
- 对比：`test/smoke_goal.py:30-66` **已经**为此写了"复制配置 + 改 CSV 路径到 `.scratch/smoke_run`"的规避，
  并在注释里记录了上次踩坑（"每跑一次冒烟都会把实车那一次的数据覆盖"，已实测踩到）；
  motion 版**没有抄这段**。
- 影响：实车 ROGMap 时序数据（当前唯一能定位"建图积压"的证据）会被下一次冒烟静默清空，
  且不可恢复。本次审查能拿到 §0 的基线，正是因为最近一次冒烟用的是已修好的 `smoke_goal.py`。
- 建议：把 `smoke_goal.py:35-66` 的 config-copy-and-rewrite 抽成共用函数供两个脚本复用；
  或在 `PerformanceMonitor` 层改成"带时间戳命名 / 追加"。
- 置信度：确定（代码路径）。**本次未实跑该脚本**（会毁掉现有实车数据）。

### 1.7 【高】README 推荐的离线换图流程当前**跑不通**

- 位置：`src/mas2027_nav_bringup/scripts/save_pcd_and_make_map.sh:34,218`
  （提示用户执行 `ros2 launch mas2027_nav_bringup rm_navigation_small_point_lio_launch.py ...`，
  而该 launch 在 `src/mas2027_nav_bringup/launch/` 下**不存在**，全仓只剩
  `install/mas2027_nav_bringup/.../__pycache__/rm_navigation_small_point_lio_launch.cpython-312.pyc`）；
  `:108`/`:130` 的兜底绝对路径少一层 `src/`
  （`/home/mas/mas_nav_2027_native/mas2027_nav_bringup`、`.../mas2027_utils/pcd_trans/pcd_tool.py`
  均不存在，已 `ls` 验证）；安装版脚本用 `dirname $0` 推 `$SRC` → 指向 `install/.../lib`。
  README:272 正是推荐用这个脚本重建地图。
- 影响：`src/README.md`「地图更新」一节给出的换图/建图流程按文档执行必然失败；
  换新场地时最需要的那条路径是坏的。
- 建议：把 `rm_navigation_*` 改为现有 `nav_executor_launch.py`；兜底路径改 `ros2 pkg prefix` /
  `get_package_share_directory`；并补 `pcd2pgm`、`nav2_map_server` 到 bringup 的 `exec_depend`。
- 置信度：确定（路径存在性已逐个验证；未实际执行脚本）。

---

## 2. 中优先级：真实缺陷，触发条件明确或正在持续损耗性能/可观测性

### 2.1 【中】建图仍然跟不上点云：输入 28.8 Hz、处理 20.0 Hz、丢帧 30%

- 证据（本次 CSV，300 窗口，中位）：`cloud_callback_hz 28.78` → `map_update_hz 20.02`，
  比值 0.704；`last_update_period_ms` 中位 **45.7**、p95 **94.0**、max **150.5**；
  60% 的窗口 `update_period > 34.7 ms`（按 28.8 Hz 输入换算的预算），45% 的窗口 > 50 ms。
- **重要更正**：此前所有分析都基于"输入 20 Hz、预算 50 ms"（例如 `planner_params.yaml:336-345`），
  而本次实测输入是 **~28.8 Hz**（`lidar_publish_time_interval` 为 0.05 s，理论上限 20 Hz，
  实际回调频率更高，具体来源待查：怀疑 LIO 侧发布速率高于驱动合帧速率）。
  在 29 Hz 下"中位 45.7 ms"意味着**每一轮都在丢帧**，而不是"贴着预算跑"。
- 分阶段（中位）：raycast 6.2 + projection 12.8 + prob_update 6.2 + decay 2.3 + query 0.87 + field 1.5
  ≈ 29.9 ms，而自报 total 37.6 ms、真实周期 45.7 ms ⇒ 还有 ~8 ms 在 `total_update_time` 之外
  （未计入 3.2 ms 中位、p95 33.6 ms）。结合 §2.16 的"每帧 14 MB reserve"，
  未计入部分很可能主要是可视化 heavy 帧与分配抖动。
- 进展（正面）：脏列增量**这次确实生效**——`full_reason_dirty_over_ratio` 在 300/300 个窗口中为 0，
  `full_reason_periodic` 占 6.7%（与 `dirty_full_period_s: 0.5` 自洽），projection 中位 12.8 ms
  （全量时代为 42.9 ms）。上一轮"未在实车确认"的悬念可以结案。
- 建议：下一步压 `prob_update`（第二大项）与 `raycasting.ray_range`（10 m 射线扫满 10 m 窗口是脏列比例高的根因）；
  同时把 `update_unaccounted_ms` 的 p95（33.6 ms）定位到具体分配点。
- 置信度：确定（300 窗口统计）。

### 2.2 【中】`detailed_csv_enable` 仍是 `true`，且 `detailed_csv_path` 未设置 → 写到 `/tmp`

- 位置：`planner_params.yaml:334` `detailed_csv_enable: true`（注释自己写着"跑完一次后改回 false"）；
  `src/mas2027_perception/rog_map/include/rog_map/rog_map_core/config.hpp:384` 默认路径
  `/tmp/rog_map_perf_detailed.csv`，YAML 未覆盖。
- 影响：每次更新写一行（实测约 20~29 行/s、30+ 列）→ 跑一次几 MB；
  且路径在 `/tmp`，正是 `change-history.md` 2026-09-16 20:45 那条"写到 /tmp 排障侧读不到"踩过的坑
  （本仓库因此把 summary 路径改到了 `.scratch/`，但 detailed 没改）。
- 建议：改回 `false`；若要保留，把 `detailed_csv_path` 也指到工作区并加日期后缀。
- 置信度：确定。

### 2.3 【中】summary CSV 以 `trunc` 打开 → 每次运行覆盖上一次实车数据

- 位置：`src/mas2027_perception/rog_map/src/rog_map/performance_monitor.cpp:101`
  `summary_csv_.open(path, std::ios::out | std::ios::trunc)`（detailed 同，`:92`）。
- 影响：21:44 那次的 300 行数据已被 21:59 那次覆盖；`change-history.md` 里多次因为
  "没有同场景 A/B"而无法定论（例如 21:44 与 21:30 的对比），这个设计是直接原因。
- 建议：文件名带 `run_id`/时间戳，或改 append 并写入表头判断。
- 置信度：确定。

### 2.4 【中】`onSetParameters` 对绝大多数参数"返回成功但不生效"

- 位置：`minco_planner.cpp:690-914`：默认 `result.successful = true`（`:693-694`），
  只有 `max_velocity`/`max_acceleration`/5 个 penalty weight/4 个 `smac_2d.*` 有分支；
  其余参数（含 `safe_dist`、`collision_dist`、`replan_react_time`、`monitor_margin`、
  `failure_log_*`、`stuck_escape.*`）既不进 configure-time 拒绝列表，也没有生效分支。
- 影响：`ros2 param set ... planner.minco_optimizer.collision_dist 0.35` 返回成功、`param get` 也显示 0.35，
  但硬判据仍是 0.28−0.02 → **"标称值"与"生效值"不一致且无任何日志**。现场"我明明改过阈值"的判断会失效。
- 建议：循环末尾加 `else { successful=false; reason="not runtime-tunable"; }`，
  或为安全阈值补 `setConfig` 同步。
- 置信度：确定。

### 2.5 【中】"沿用旧轨迹"分支返回 `true` 却不发布任何消息 → 静默急停（且不进 ROS 日志）

- 位置：`minco_planner.cpp:1273-1279`
  ```cpp
  if (has_last_traj && isTrajSafe()) {
    if (!isTrajectoryTimeExpired(...)) {
      std::cout << "... Continuing to execute it." << std::endl;   // 只打印，不 publish
      return finish(true, "NONE");
  ```
  下游：`task_manager.cpp:136-138` 把 `true` 当成"新轨迹已接受"，
  `minimum_trajectory_stamp_ns_.store(plan_started_ns)`；`task_manager.hpp:29`
  `acceptsTrajectory()` 要求 `stamp >= minimum_trajectory_stamp_ns_` ⇒
  对"正在执行的旧轨迹"恒为 false ⇒ `path_executor.cpp:60-64` 发**零速**。
- 影响：注释承诺的"继续执行旧安全轨迹"从未发生，实际是急停；该路径不计入 `replan_failure_total_`、
  没有 `RCLCPP_WARN`（只有 `std::cout`，不落进节点日志文件）→
  现场表现"车无故停一下，日志里查不到原因"。
- 建议：要么真的用新的 `header.stamp` 重发一次缓存轨迹（这样抬高准入下限才合法），
  要么返回 `false` 让 TaskManager 走 `:142` 的"未过期就继续执行"分支；同时补一条 `RCLCPP_WARN`。
- 置信度：确定。

### 2.6 【中】MPC 的加速度锚点锚在"上一帧指令"且被 7 处 `resetLastControl()` 清零；实测速度没进 QP

- 位置：`src/mas2027_nav_executor/src/path_executor/mpc/mpc_solver.cpp:179-201`（`last_u_global_`，
  注释却写"v(-1) 使用当前时刻真实全局速度"），`x0` 只用位姿，`curr.vx/vy/omega` 无引用；
  清零点：`path_executor.cpp:61,66,72,79,98,104,114`。
- 影响：任何一次否决/失败后的下一帧，k=0 约束变成 `|v0| <= a_max*dt = 0.2 m/s`；
  若车实际以 1~3 m/s 行进，等效减速度 16~60 m/s²，远超底盘能力 → 打滑、里程计跳变、再反馈成抖动。
- 建议：k=0 用实测速度作锚（或 reset 时把 `last_u` 置为实测速度）。
- 置信度：确定。

### 2.7 【中】所有"否决/陈旧/求解失败"路径都是一步到 0 速，绕过加速度约束

- 位置：`path_executor.cpp:60-82,113-116`（未通过时 `output.command` 保持默认全 0），
  `nav_executor_node.cpp:437-438` 无条件发布。
- 影响：`TRAJECTORY_STALE / TERRAIN_BLOCKED / DYNAMIC_BLOCKED / SOLVER_FAILED` 都变成阶跃停车
  （配置的限幅本是 0.2 m/s/步），且日志不区分"主动制动"与"失控停车"。
- 建议：这些分支改为跟踪由实测速度生成的减速剖面，或复用备份轨迹（见 §2.8）。
- 置信度：确定。

### 2.8 【中】`BLOCK_COMMAND` / `/backup_path` / 备份轨迹全是死路径，"平滑制动"从未生效

- 位置：发布侧 `vendor/minco/src/minco_utils.cpp:104`（NORMAL）与 `:166`（**BLOCK_COMMAND**）；
  消费侧 `path_executor.cpp:57-131` 全程不读 `command_flag`；
  `minco_planner.cpp:599-600` 创建 `/backup_path` 后**无任何 publish**；
  `minco_planner.cpp:2018-2019` 把备份轨迹发到 `/opt_path`，但急停流程先
  `allows_motion_.store(false)`（`task_manager.cpp:122`）→ 该轨迹永远不会被跟踪。
  另：备份走廊 `corridor_generator.cpp:41-46` 用当前 ESDF 距离推半边长，
  贴墙工况下退化成 5 cm 盒 → 备份优化失败 → 回退 `make_stop_traj()`（速度阶跃）。
- 影响：急停的实际效果是"零速 + 复位上一拍控制"，比设计的平滑制动更硬；
  外部若订阅 `/backup_path` 或依赖 `BLOCK_COMMAND` 判别，永远收不到。
- 建议：二选一并写进注释——要么下游读 `command_flag` 并跟踪备份轨迹，
  要么删掉 `backup_path_pub_`/BLOCK 语义、明确"急停 = allow_motion 门 + 零速"。
- 置信度：确定（字段无消费者已 grep 全仓）。

### 2.9 【中】`mpc.weights.state` 的 x/y 被静默忽略；速度限幅是方形框（对角可达 4.24 m/s）

- 位置：`mpc_solver.cpp:128-134`（Q_bar 只用 `q_along/q_cross`，默认 3.0/**12.0**，
  无任何 `declare_parameter`；`config_.Q.x()/Q.y()` 无引用）；
  `mpc_solver.cpp:155-164`（`lb/ub` 对 global `vx`、`vy` 各自独立 ±3.0）。
- 影响：(a) YAML 写 `state: [3.0, 3.0, 2.0]` 让人以为 qx=qy=3.0，实际生效 along=3 / **cross=12**，
  现场按 YAML 调 x/y 完全无效；(b) 对角方向 `|v| = 3√2 ≈ 4.24 m/s`，
  超出 `planner.max_velocity 3.0` 与底盘设计 41%，而 `replan_react_time = 0` 没有任何速度余量覆盖。
- 建议：声明 `mpc.weights.along/cross` 并写入，或直接用 `Q.x/Q.y`；
  输出前对 body 系速度做模长限幅，或把加速度约束改成模长/多面体约束。
- 置信度：确定。

### 2.10 【中】yaw 通道是死的：`ros2_comm` 只发 vx/vy/nav_state

- 位置：`src/mas2027_utils/ros2_comm/src/ros2_comm.cpp:177-186`（只取 `linear.x/y`，
  `RawControlPacket` 无角速度字段；`mas2027_nav_bringup/launch/nav_executor_launch.py:148` 注释也承认）；
  生产侧 `path_executor.cpp:127-128` 仍计算 `angular.z`，`nav_executor_node.cpp:186-191` 订阅 `/cmd_spin`
  而**全仓无发布者**；`src/README.md:144` 却写"默认允许转向（角速度 ±2 rad/s）"。
- 影响：yaw 优化（`enable_yaw_opt: true`）、轨迹 yaw 参考、`LimitLocalVel` 的转弯限速、
  spin 叠加全部不落地（对角速度的 0.1 死区、`min_turn_vel` 等参数同样无效）。
  若底盘确为全向且不需要自转，应把这条通道与相关参数在文档里标明"不生效"，否则容易误判调参结果。
- 置信度：确定（协议字段 + 唯一消费者 + 无发布者）。

### 2.11 【中】全局搜索每次新建 `TerrainMapQuery` → revision 缓存失效，每次搜索整图重烘焙

- 位置：`src/mas2027_nav_executor/src/path_planner/search/global_path_searcher.cpp:301`
  `auto terrain_query = std::make_shared<TerrainMapQuery>(terrain_);`（每次 plan 新建）；
  `src/common/environment/terrain_map_query.cpp:61-65` 的省算条件是
  `snapshot_ && snapshot_->revision == revision`，对新对象恒不成立 →
  必走 770×347 两次 `cv::distanceTransform(DIST_MASK_PRECISE)` + 两次全图循环。
- 影响：每次全局搜索（20 Hz tick、0.5 s 限流）重复烘焙整图；`TerrainMapQuery` 的每个访问器都
  `refresh()`（互斥量 + revision），SMAC 内层每格 2 次访问器 → 额外锁开销。
- 建议：把 `TerrainMapQuery` 提为 `GlobalPathSearcher` 成员，让 revision 门真正生效。
- 置信度：确定。

### 2.12 【中】末点被无条件改写到精确目标，废掉搜索侧的"直线畅通"保护

- 位置：`global_path_searcher.cpp:787-796`（`plan.poses.back()` 与 `latest_global_path.back()`
  都被直接改成 exact goal）；被废掉的保护在 `astar.cpp:184-215`（`line_clear` 才补终点）；
  SMAC 分支同理：`smac_planner_2d_simple.cpp:304-306` 在 `tolerance`(0.30 m) 内提前停机，
  末段 0.05→0.30 m 的跳跃**不经任何检查**。
- 影响：① `use_smac: false` 时 Astar 可返回"从截断点直连目标"的穿障段（契约与 SMAC 不一致：
  SMAC 明确拒绝非通行终点，`smac_planner_2d_simple.cpp:282-290`）；② SMAC 分支末段跳跃若切薄墙，
  只能靠下游 `buildSeed` 的净空判据 fail-closed 拦下 → 表现为"明明有路却一直生成失败"。
- 建议：改末点前对该段做一次直线可通行采样；`astar.cpp` 的 `calcPath` 卡死分支（`:171-172`）应返回 false。
- 置信度：确定。

### 2.13 【中】`allow_unknown` / `unknown_as_occupied` 在地形主路径上是死开关

- 位置：`terrain_map_query.cpp:91-92,183-190`（`values` 只有 254/0，**永不出现 255**）vs
  `smac_planner_2d_simple.cpp:274-280`、`global_path_searcher.cpp:75-81,391-398`（判的是 `cost == 255`）；
  上游 `terrain_grid.cpp:104-115` 把静态层 `cost < 0`（unknown）写成 **100 = 占据**，
  而动态层合并（`:116-126`）只处理 `>= 95` ⇒ 静态未知=障碍、动态未知=自由。
- 影响：YAML 写 `unknown_as_occupied: false`、日志每帧打印 `allow_unknown=true`，
  但地形主搜索里该开关永不生效；目标落在未观测区域必被拒（`goal cell is not traversable, cost=254`），
  "往没看过的地方探索"在主路径上实际是关闭的，且**日志口径与真实行为相反**。
- 建议：二值化保留三态（`cost<0 → 255`、`0..94 → 0`、`>=95 → 254`），动态合并不吞掉 -1。
- 置信度：确定。

### 2.14 【中】双雷达时间戳漂移会让 `/lidar` 永久静默（无"无输出"看门狗）

- 位置：`src/mas2027_perception/mid360_driver/src/mid360_driver.cpp:169-175`
  （`NO_SYNC` 的 delta 只在首包计算一次，此后永不更新）+
  `src/mid360_driver_node.cpp:240-258`（超窗只丢较旧队头、禁止发半帧）、
  `:276-286`（饿死告警要求某一队**为空**，而实际两队都在流动）、
  `:531-542`（健康门由"收到 UDP 包"喂饱）。
- 机制：两路设备钟相对漂移超过 `merge_max_interval_ms`(50 ms) 后，每个 tick 都丢较旧的队头，
  两队永不配对 → `/lidar` 永久静默；两个 gate 因仍在收包都判 online ⇒ **单雷达降级不触发**，
  只有每 20 次一条 "dropped stale ... frame"。50 ppm 约 17 min、20 ppm 约 42 min 必然命中
  （若现场 PTP 可用、窗口设 5 ms 则不触发，需按实际部署确认）。
- 建议：加"连续 K 个 tick 无输出/持续超窗丢帧"看门狗 → 转 `collect_single_lidar_frames()`；
  并周期性用 wall-clock 重估 `NO_SYNC` delta。
- 置信度：较可能（路径确定，触发取决于设备钟漂移；未实测）。

### 2.15 【中】驱动发布器队列深度 1000（reliable）

- 位置：`mid360_driver_node.cpp:30-31`（`create_publisher<PointCloud2>(lidar_topic, 1000)`、
  `imu_publisher ... 1000`）。
- 影响：融合后约 1 MB/帧 @20 Hz ⇒ writer 历史上限可达 ~1 GB；只要有 reliable 订阅端跟不上，
  延迟就无界增长（`small_point_lio_node.cpp:34-37` 的注释正是为这个问题把点云 QoS 改成
  `SensorDataQoS().keep_last(1)`，驱动侧却没改）。
- 建议：改 `rclcpp::SensorDataQoS()`（LIO 端本就是 SensorDataQoS，仍匹配）。
- 置信度：确定。

### 2.16 【中】每帧大内存分配（含约 14 MB 的 heavy viz reserve）

- 位置：`rog_map_ros2.hpp:355`（`rc_.pc = temp_pc;` 在锁内整帧拷贝，应为 `swap`）、
  `:681` + `prob_map.cpp:512-513`（UNKNOWN 分支 `reserve(box_size.prod())`，
  `visualization.range [10,10,1.5]` 裁剪后约 200×200×30 ≈ 1.2 M 格 → ~14 MB，每 0.5 s 一次，
  **不计入 `total_update_time`**）、`prob_map.cpp:926-927`（每帧 ~1.1 MB）、
  `rog_map.cpp:866-877`（快照拷贝 ~720 KB/帧）。
- 影响：20 Hz 下约 40 MB/s 分配 churn + 偶发 10 ms 级尖峰，是 §2.1 里
  `update_unaccounted_ms`（p95 33.6 ms）最可疑的来源。
- 建议：入槽改 `swap`；`boxSearch` 先统计再填充或给 reserve 上限；快照改双缓冲复用。
- 置信度：确定（代码事实）/ 耗时量级较可能。

### 2.17 【中】`ProbMap::initProbMap()` 的 `static bool init_once` 是进程级

- 位置：`src/mas2027_perception/rog_map/src/rog_map/prob_map.cpp:67-71`
  （对比 `SlidingMap::initSlidingMap` 用的是成员 `had_been_initialized`）。
- 影响：同进程第二次构造 `ROGMap/ROGMapROS` 会在构造函数里 `throw` → 节点启动即死
  （`nav_executor_node.cpp:226` 已有 `shutdown_planner()`，一旦加"重建 planner"或同进程起两个实例就会踩到）。
- 建议：改成成员变量。
- 置信度：确定。

### 2.18 【中】恢复状态机 latch：`ENTER_EMER_STOP` 被丢弃 ⇒ 本进程内脱困永久失效

- 位置：`task_manager.cpp:41-45`（只处理 `DO_ESCAPE`）+
  `src/task_manager/recovery/recovery_behaivor.cpp:44-49,115-119,132-139`
  （`startRecovery()` 先置 `recovery_active_ = true`，随后 `calculateEscapeVelocity` 失败返回
  `ENTER_EMER_STOP`，无人回滚该标志 → `shouldTryRecovery` 之后恒 false）。
- 影响：一次逃逸速度计算失败（要求 4 个梯度采样全 finite，而
  `trajectory_safety_checker.cpp:201` 在 query 失败时返回 NaN）之后，本进程再也不会脱困，
  只剩反复重规划。
- 建议：`ENTER_EMER_STOP` 分支显式 `finishRecovery(false, now)`（或返回前回滚标志）并打日志。
- 置信度：确定。

### 2.19 【中】map_server 侧：`pgm_to_terrain_msgpack.py` 的 `negate` 反了；地面剔除余量只有 4 cm

- 位置：`src/mas2027_perception/map_server/scripts/pgm_to_terrain_msgpack.py:60-64`
  （`negate` 时又对 `occupied` 取了一次反 → 障碍与空地互换；`lab3.yaml` 是 `negate: 0`，当前未触发）；
  `src/mas2027_perception/map_server/src/map_server_node.cpp:178-188`
  （地面剔除依赖 `height <= robot.z() - 0.04`，而 URDF 里 `base_link` 距地仅 0.1 m）。
- 影响：前者在换地图时会产生**完全相反**的规划图；后者在 LIO z 漂移 4 cm 以上时静默失效，
  残留地面点会被按 `full_cost 0.2 m` 膨胀成"墙"。
- 置信度：确定（脚本逻辑）/ 较可能（现场触发）。

### 2.20 【中】`use_odom_localizer:=False` 会让整条规划链路失效，但它被列在"常用参数"里

- 位置：`nav_executor_launch.py:56-72,151`（`odom_localizer` 受 `IfCondition` 控制，而它是 `map→odom` 的
  唯一来源）；`global_path_searcher.cpp:276-282,325-328`（查不到 `map→rog_frame` 就
  `Cannot align terrain map with planning frame` → 返回 false）。
- 影响：README:180-185 把它与 `use_ros2_comm` 并列成"常用参数"，用户会以为关掉只是"不做重定位"，
  实际是**接受目标后永远没有轨迹**，且失败原因只有一次 WARN。
- 建议：在 launch 的 arg description 里写明后果，或 off 时补一个 identity 的 `map→odom`
  静态发布并打醒目警告。

### 2.21 【中】`params.yaml` 里的 `$(find-pkg-share …)` 对 `Node` **不会**被展开

- 位置：`src/mas2027_perception/Localization/odom_localizer/config/params.yaml:28`
  （`prior_pcd_file: "$(find-pkg-share mas2027_nav_bringup)/pcd/lab3.pcd"`；代码直接
  `pcl::io::loadPCDFile` 且失败即 throw，`odom_localizer_node.cpp:206-213`）、
  `src/mas2027_utils/pcd2pgm/config/pcd.yaml:3`。
- 机制：`launch_ros` 只对**参数文件路径**做替换，YAML 内容里的替换语法仅在
  `ComposableNode`（进程内）路径才展开；`Node` 动作是 `--params-file` 原样传参。
- 影响：`ros2 launch odom_localizer odom_localizer.launch.py` 单独启动即抛异常退出；
  主链路只是因为 launch 用绝对路径覆盖了同名参数才正常（两处定义同一参数）。
- 建议：YAML 去掉替换语法，改由 launch 用 `PathJoinSubstitution` 传入。

### 2.22 【中】`output_topic` 可配，但下游 `ros2_comm` 把 `/cmd_vel` 写死

- 位置：`nav_executor_launch.py:152`（`output_topic` 默认 `/cmd_vel`，传给 `node.topics.cmd_vel_pub`）
  vs `src/mas2027_utils/ros2_comm/src/ros2_comm.cpp:68-70`（`create_subscription<Twist>("/cmd_vel", …)`，无参数）。
- 影响：按 launch 的设计意图改用别的输出话题（例如并跑两套导航）时，底盘桥收不到指令 →
  "cmd_vel 有值但车不动"，而 launch 注释恰恰把 ros2_comm 描述为"`/cmd_vel` 的唯一消费者"。
- 同理：`/opt_path`、`/backup_path` 在 `minco_planner.cpp:596-600` 写死，而订阅端
  `node.topics.trajectory_sub`（`node_params.yaml:18`）可配 —— 改名即静默无轨迹。

### 2.23 【中】同一份参数在"包内 config"与"bringup config"各存一份且**数值互相矛盾**

- 位置：`src/mas2027_perception/mid360_driver/config/params.yaml:26`（`validate_crc: true`）
  vs `src/mas2027_nav_bringup/config/small_point_lio_params.yaml:31`（`false`，实际生效的那份）；
  `.../small_point_lio/config/mid360.yaml`（`lidar_frame: livox_frame`、`min_distance 0.5`、
  `gravity [0,0,-9.810]`）vs bringup 那份（`lidar_link`、`0.38`、`gravity [-0.0017,-0.7407,-0.6653]`，**单位差 9.81 倍**）。
- 影响：误用包内 launch/config 会得到完全不同的外参/重力标度，表现为"LIO 莫名漂移"；
  `measure_lidar_mount.py:349-350` 只能靠打印提醒"同一份也要写进另一处"。
- 建议：明确唯一真源（bringup 那份），包内 config 删除或加"已废弃"抬头，
  标定脚本直接写两处。

### 2.24 【中】`install/` 空间残留已删除的消息类型 `CostMaps`

- 位置：`install/interfaces/share/interfaces/msg/CostMaps.idl`、`build/interfaces/rosidl_generator_cpp/.../cost_maps.hpp`；
  `third_party/interfaces/msg/navigation/CostMaps.msg` 已在 `1a6a008` 删除，`src/` 内无任何引用。
- 影响：`ros2 interface list` 仍会列出该类型，旧代码在未 clean 的机器上仍能编过；
  换机器 clean build 后行为突变（"在我机器上是好的"）。
- 建议：clean build 一次 `interfaces`（或整体 `rm -rf build install`）。

### 2.25 【中】README 三处安全阈值数字与生效值不符

- `src/README.md:67`：`kMonitorClearanceTolerance`（**0.05 m**）↔ 实际 `minco_planner.cpp:25` 是 **0.02**；
- `src/README.md:127`：`node.rog_map_clearance`（默认 **0.30 m**）↔ 生效值 `node_params.yaml:11` 是 **0.28**；
- `src/README.md:206`：`planner.tolerance` 默认 **0.30 m** ↔ 代码缺省 `minco_planner.cpp:265` 是 **0.5**（0.30 来自 YAML）。
- 影响：按 README 手算"有效硬阈值 = collision_dist − 容差"会得到 0.25 而非真实的 0.26；
  这正是 §1.1 里那条判据的文档版本，属于撞墙事故直接相关的文档。

---

## 3. 低优先级：可观测性、契约与工程卫生

| # | 问题 | 位置 | 说明 |
| --- | --- | --- | --- |
| 3.1 | 启动日志硬编码 `global_search=OmniKinoAstar`、`planner_mode=EXPLORATION` | `minco_planner.cpp:189-200` | 实际 `use_smac: true` 走地形图 SMAC 2D，排障会去找一个不存在的搜索器 |
| 3.2 | 参数注释与实际值脱节 | `planner_params.yaml:47-59,92-98,160-164`；`src/README.md:67` | 仍写 `0.30 + 0.15v`、`collision_dist 0.30`、容差 `0.05`，实际是 0.28 / 0.0 / 0.02 |
| 3.3 | `verdict=GEOMETRY` 语义被误用（见 §1.2）；`docs/refusal_triage_2026-09-16.md` §1.3 的"强嫌疑"至今未修 | 同 §1.2 | 建议连同分类器一起修 |
| 3.4 | 冒烟脚本未进 CTest | `mas2027_nav_executor/CMakeLists.txt:116-160` | `colcon test` 只跑 10 个纯单元测试（executor 9 + mid360_driver 1），不覆盖真实规划链路；本次手动跑 `smoke_goal.py` 才验证到端到端 |
| 3.5 | 死代码/死字段 | `MincoPlanner::makePlan`、`extractLocalPath`、`getTrajectoryRemainTime`、`omni_kino_astar`（已确认无生产调用点）、`opt_freq_`、`time_allocation_iters`、`global_frame_`、`backup_path_pub_`、`has_last_u_`、`mpc_solver` 的 `max_iterations`/`planner_freq` | 只增噪音与误判成本 |
| 3.6 | `rclcpp::Clock()`（系统钟）与节点时钟混用 | `minco_planner.cpp:1103,1130,1247,1388,1999,2167,2291`；`path_executor.cpp:76-78` | `use_sim_time:=true` 时轨迹时间戳与 odom 不同源 → 陈旧判据 fail-open（甚至 `allows_motion` 恒 false） |
| 3.7 | `build/` 残留 5 个已删除包 | `build/minco_planner`、`minco_controller`、`waypoint_editor`、`pb_nav2_plugins`、`fake_vel_transform` | 不参与构建（`install/` 干净），但占 ~500 MB 且易误导 |
| 3.8 | `odom_localizer` 的 GICP（0.04~0.21 s）与 20 Hz TF 发布定时器同处默认互斥回调组 | `odom_localizer_node.cpp:254-260` | 注册期间 TF 发布被推迟最多 0.2 s |
| 3.9 | `planner.lidar_offset_x/y` 代码有、YAML 无 | `minco_planner.cpp:292-297` | 一直用默认 (0.0, −0.2)，影响 `getCurrentSpeed()` 的杆臂补偿 |
| 3.10 | 文档漂移 | `src/docs/README.md:22-23,46` | 仍写"HW 地形图不参与轨迹优化"（实际已参与三处门）、话题名少 `/debug` |
| 3.11 | README 与 launch 默认值冲突 | `src/README.md:187` vs `nav_executor_launch.py:150` | README 建议首次上车 `use_ros2_comm:=False`，而 launch 默认 **True**（照 README 的启动命令会直接对接底盘） |
| 3.12 | 无节流 `std::cout` | `minco_planner.cpp:1275,1287,1301`、`rog_map_ros2.hpp:327,335`、`mpc_solver.cpp:302-308` | 高频 stdout 且不落进节点日志文件 |
| 3.13 | 未使用/失效的参数校验 | `nav_executor_node.cpp:135-139`（NaN 可通过、`lookahead_time` 未校验）、`config.hpp:443-449`（`logit(p)` 对 p∈{0,0.5,1} 产生 ±inf/0；当前值安全） | 一份手改 YAML 可得到 horizon=1 或必然失败的 QP，且启动无报错 |
| 3.14 | RViz 里两个 `*_updates` 话题无人发布 | `rviz/nav_executor_view.rviz:98,119` | Map 显示的更新触发话题空挂（无发布者已确认，显示影响待验证） |
| 3.15 | `build/` 里 5 个已删包 + `install/` 里 2 个旧 launch 的 `.pyc` + 已删消息 `CostMaps` | 见 §2.24、§3.7 | 一次 `rm -rf build install` 重编即可 |
| 3.16 | 一批 `/home/ros2_ws/...`、`/home/lihanchen/...` 硬编码路径（本机均不存在） | `pcd2ele.yaml:3-4`、`pcd2esdf.yaml:3-4`、`pcd_tool.py:196,202`、`pcd2pgm.cpp:47` | 离线工具默认参数即失效；`pcd2ele.yaml:4` 的 `output_folder` 还指向源码树 |
| 3.17 | `package.xml` 依赖多缺多报 | `mas2027_nav_executor`（缺 `builtin_interfaces`/`yaml-cpp`/`libdw`）、`mas2027_nav_bringup`（缺 `pcd2pgm`/`nav2_map_server`）、`map_server`（声明了未用的 `common_libs`） | `rosdep install`（README:155 推荐）装不齐，clean machine 复现困难 |
| 3.18 | `mas2027_utils/map_edit` 是无法还原的 gitlink（无 `.gitmodules`，目录为空） | `git ls-files -s` 显示 `160000 … map_edit`；`save_pcd_and_make_map.sh:315` 仍在提示用它 | clone 后该步骤无法执行 |
| 3.19 | 代码 declare 但 YAML 未提供的键（静默落代码缺省） | `planner.lidar_offset_x/y`、`time_allocation_iters`、`smooth_eps`、`integral_res`、`planner.performance.*`、`rog_map.point_filt_num`(2)、`projection.passable_cost`(50)、`debug.*` | 这些值只能靠读代码得知，改代码缺省会静默改变现场行为 |
| 3.20 | 4 MB 未被 URDF 引用的 `LakiBeam.STL` 入库；`.gitignore` 与真实布局不符（产物在工作区根） | `mas2027_robot_description/meshes/`、`src/.gitignore:9-12` | 仓库体积；`.scratch/`、`*.bak.*`、`*.csv` 未忽略 |

---

## 4. 本次核实为"正常/已改善"的部分（避免重复劳动）

1. **脏列增量确实生效**：300/300 窗口 `full_reason_dirty_over_ratio=0`，`periodic` 占 6.7%
   （与 0.5 s 周期自洽），projection 中位 12.8 ms vs 全量 42.9 ms。
   `change-history.md` 顶条"未在实车 RViz 确认"的悬念可以结案（RViz 目视仍需一次上车确认）。
2. **回归链路健康**：`ctest` 9/9 通过；`smoke_goal.py` 通过并给出文档一致的输出；
   本次实测确认冒烟脚本的 CSV 隔离**有效**（冒烟前后实车 CSV 行数/mtime 不变，
   detailed CSV 落在 `.scratch/smoke_run/`）。
3. **构建与安装一致**：`install/share/**/config/*.yaml` 是指向源码的软链（改 YAML 无需重编）；
   二进制（21:58）新于源码（21:49），无陈旧产物。
4. **关键不变量当前成立**：`collision_dist == node.rog_map_clearance == 0.28`；
   `safe_dist 0.33 > collision_dist 0.28`；`exploration.unknown_as_occupied ==
   projection.unknown_as_occupied == false`；`fill_occ_min 8 / denoise_occ_max 0` 满足
   `config.hpp:299-305` 的区间校验（不会启动即死）。
5. **查询链路线程安全设计正确**：规划侧只通过 `QueryAdapter` 的不可变快照读图
   （`query_adapter.cpp:120-132,262-273`），`copyValues` 单快照实现无悬垂；
   `TerrainGrid` 的 `snapshot()/dynamicSnapshot()` 返回 `shared_ptr<const>`。
6. **下游底盘桥有看门狗**：`ros2_comm` 在 `/cmd_vel` 超时后发零速（`ros2_comm.cpp:200-210`），
   节点崩溃不会让底盘保持旧速度。
7. **fail-close 语义完整**：`checkCommandSafety` 的所有早退（空快照/空 tf/空 query、
   查询失败、越界）都返回非 `PUBLISHED`，没有"取不到数据就当安全"的分支。
8. **静态地图不是错的**：被判"占据"的位置在先验点云里确有真实墙体（§1.3 独立复核）。
9. **launch ↔ config 键位无静默忽略**：`nav_executor_node.cpp` 的 49 处 `declare_parameter` 与
   `node_params.yaml`/`mpc_params.yaml` 逐键对应；`planner_params.yaml` 的 `planner.*` 键与
   `minco_planner.cpp:215-560` 的 declare 列表逐键核对无遗漏；`mid360_driver` 的 27 个键一一对应；
   `map_server` 的 launch 内联参数全部被读取；最新实车日志无任何 undeclared 告警。
10. **QoS 契约全链路一致**：地图链路 `QoS(1).reliable().transient_local()` 两端一致；
    点云/里程计链路 SensorDataQoS 系一致（驱动侧 depth 1000 的问题见 §2.15，属队列深度而非兼容性）。
11. **`map ↔ odom` 换算确实已实现**：`global_path_searcher.cpp:276-282,311-323` 显式做双向
    `lookupTransform`/`doTransform`，缺失时 fail-closed（与 `src/README.md:36` 一致）。
12. **`bypass_dynamic_obstacle: True` 不会饿死下游**：`map_server_node.cpp:106-111` 在 bypass 时
    仍每 500 ms 发一张同几何的全 0 动态图，因此 `global_path_searcher.cpp:285` 的就绪门槛能通过
    （但这条"空图层当硬门"本身就是 §1.5 的风险来源）。
13. **两处"必须同时改"的参数当前确实一致**：`node.rog_map_clearance == collision_dist == 0.28`；
    两处 `unknown_as_occupied == false`（见 §4.4）。

---

## 5. 建议的修复顺序（按"收益 / 代价"排序）

1. **保护现场数据**（§1.6）：把 `smoke_goal.py` 的 CSV 隔离逻辑抽成共用函数给 `smoke_goal_motion.py` 用。
   一行级的改动，但它是"下一轮排障还能不能拿到数据"的前提。（另顺手把 §2.2 的
   `detailed_csv_enable` 关掉、§2.3 的 CSV 改成不 trunc。）
2. **统一四道净空门**（§1.1 + §1.2 第 2 项）：一个常量、一处函数，预计直接消掉最新一次运行
   13/13 的 `Braking: clearance` 与一部分"种子无效"。
3. **修 `classifySeedReject` 的口径**（§1.2 第 1 项）：纯日志逻辑，零行为风险，
   但能让后续所有排障不再被误导；补单测。
4. **动态图层新鲜度阈值参数化并放大到 ≥3 倍发布周期**（§1.5）：两行改动，
   消掉运行中途"点目标没反应"和潜在的指令门急刹。
5. **`robot_state_` 加锁**（§1.4）：一处 `lock_guard`，换掉一整类偶发地图错位。
6. **修 `onSetParameters` 的静默成功**（§2.4）与"沿用旧轨迹"的静默急停（§2.5）：
   都很小，直接决定"改参数/看日志"这两条排障手段是否可信。
7. **让未知格不再无条件当自由**（§1.3）：这是"卡"的结构性根因，代价最大，但缺了它
   阈值怎么调都只能改变失败分布。
8. 其余按 §2 排期；§3 可随手清理（尤其 §3.10/§3.11/§2.25 的文档修正成本极低）。

---

## 6. 复审：**现在代码里确实存在**的 vs **之前已经改掉**的

用户要求只要"现在确实有的问题"。本节是对 §1~§3 的逐条复核结果：**每条都在当前 HEAD 上重新读过代码**
（行号即本次读到的位置），并把历史上已经修好的条目明确剔除，避免重复劳动。

### 6.1 现存（当前代码确认，且多数有本次实车数据支撑）

| 条目 | 现存证据（本次在 HEAD 上读到） | 现场状态 |
| --- | --- | --- |
| §1.1 四道门两套阈值 | `command_safety.cpp:63-64,75` 用裸 `rog_map_clearance`(0.28)；`minco_planner.cpp:25,1925,1962` 用 `-0.02`(0.26)；`local_path_processor.cpp:331-339` 用裸 0.28 | **每次运行**：本次 9/13 次刹车 value∈(0.26,0.28] |
| §1.2 种子门零容差 + verdict 口径错 | `local_path_processor.cpp:43-64`（分类器）vs `:331-339`（门）两条规则不同源 | **每次运行**：本次 11/17 误报 GEOMETRY |
| §1.3 优化器看不到静态墙 | `minco_planner.cpp:1754-1888` 只事后否决；`terrain_grid.cpp:104-108` 静态 unknown→100、`:120-126` 动态 unknown 不处理；本次日志 `rog_value=255` 与 `cost=100` 同点并存 | **每次运行**：28+10 次 |
| §1.4 `robot_state_` 竞态 | `rog_map_ros2.hpp:323,331,356`（cloud 无锁读）、`rog_map.cpp:328→910-918`（更新线程无锁写）、`:292-299`（odom 持锁写） | 每次更新都在竞争（后果偶发） |
| §1.5 动态图层陈旧门 | `path_planner.cpp:70-76`、`command_safety.cpp:39-42` 均为 `>0.5`；`map_server_node.cpp:94` 500 ms 发布 | **跨多次运行** 14 次目标被丢，12 次在运行中途 |
| §2.1 建图丢帧 30% | 本次 CSV：28.78 Hz → 20.02 Hz，周期中位 45.7/p95 94 ms | **每次运行** |
| §2.4 `onSetParameters` 静默成功 | `minco_planner.cpp:693-694` 默认 success=true，只有 4 组参数有分支 | 每次 `ros2 param set` |
| §2.5 "沿用旧轨迹"静默急停 | `minco_planner.cpp:1273-1279`（只 `std::cout`，不 publish）+ `task_manager.cpp:136-138` + `task_manager.hpp:29` | 优化器失败时 |
| §2.6 加速度锚点 | `mpc_solver.cpp:179-181,190-200`（`last_u_global_`，注释写"真实全局速度"）；`path_executor.cpp` 7 处 `resetLastControl()` | 每次否决后 |
| §2.7 一步零速 | `path_executor.cpp:60-82,113-116`；`nav_executor_node.cpp:437-438` 无条件发布 | 每次否决 |
| §2.8 `BLOCK_COMMAND`/`/backup_path` 死路径 | `vendor/minco/src/minco_utils.cpp:104,166` 写入，**全仓无读取**；`minco_planner.cpp:599` 建、`:683` 释放，无 publish | 每次急停 |
| §2.9 `mpc.weights.state` x/y 无效 | `mpc_solver.cpp:128-134` 只用 `q_along/q_cross`、只用 `Q.z()`；`mpc_types.hpp:52-53` 硬编码 3.0/12.0，无 declare | 改 YAML x/y 无效 |
| §2.10 yaw 通道死 | `ros2_comm.cpp:177-186` 只取 vx/vy；`/cmd_spin` 全仓无发布者 | 每次运行 |
| §2.11 每次搜索重建 `TerrainMapQuery` | `global_path_searcher.cpp:301` 每 plan 新建；`terrain_map_query.cpp:61-65` 的 revision 门对新对象恒不成立 | 每次全局搜索 |
| §2.12 末点被无条件改写 | `global_path_searcher.cpp:787-796`；`astar.cpp:184-215` 的 `line_clear` 保护被废 | 每次搜索 |
| §2.13 `allow_unknown` 是死开关 | `terrain_map_query.cpp:91-92`（值只有 0/254）vs `smac_planner_2d_simple.cpp:276`、`global_path_searcher.cpp:78`（判 255） | 每次搜索 |
| §2.15 驱动 depth 1000 | `mid360_driver_node.cpp:30-31` | 恒定 |
| §2.16 每帧大分配 | `prob_map.cpp:513` `reserve(box_size.prod())`（`visualization.range` 10×10×1.5）；`rog_map_ros2.hpp:355` 锁内整帧拷贝 | 每 0.5 s / 每帧 |
| §2.17 `initProbMap` 进程级 static | `prob_map.cpp:67-71` | 第二次构造即抛 |
| §2.18 恢复 latch | `recovery_behaivor.cpp:75-87`（`startRecovery` 置位后 `ENTER_EMER_STOP` 直接返回）+ `:117`（`recovery_active_` 挡住重试）+ `task_manager.cpp:41-45`（只处理 `DO_ESCAPE`） | 逃逸速度算失败一次后永久 |
| §2.22 `output_topic` 可配但 `ros2_comm` 写死 | `nav_executor_launch.py:152` vs `ros2_comm.cpp:68-70`；`/opt_path` 在 `minco_planner.cpp:596` 写死 | 改名即静默无轨迹 |
| §2.23 双份配置不一致 | `mid360_driver/config/params.yaml` vs `bringup/config/small_point_lio_params.yaml`（本次 diff 确认内容不同） | 误用包内 config 即错 |
| §2.24 `CostMaps` 残留 | `install/interfaces/share/interfaces/msg/CostMaps.idl`、`.json` 仍在（软链到 build） | 本机存在，clean build 后消失 |
| §2.25 README 阈值过期 | `README.md:67`(0.05)、`:127`(0.30)、`:206`(0.30) vs `minco_planner.cpp:25`(0.02)、`node_params.yaml:11`(0.28)、`minco_planner.cpp:265`(0.5) | 恒定 |
| §1.6 冒烟清空 CSV | `smoke_goal_motion.py:151-157` 直接喂真配置 + `performance_monitor.cpp:101` trunc | 跑一次即发生 |
| §1.7 换图脚本不可用 | `save_pcd_and_make_map.sh:34,218`（launch 不存在）、`:108,130`（路径少一层 `src/`） | 换图时 |
| §2.14 双雷达漂移静默 | `mid360_driver.cpp:169-175`（delta 只在首包算）+ `mid360_driver_node.cpp:240-258`（超窗只丢队头） | 无 PTP 时 17~42 min |
| §2.19 pgm `negate` 反了 | `pgm_to_terrain_msgpack.py:60-64` | `negate:1` 时 |
| §2.19 地面剔除 4 cm | `map_server_node.cpp:182` | z 漂移 >4 cm 时 |
| §2.20 `use_odom_localizer:=False` 致命 | `nav_executor_launch.py:62,150` + `global_path_searcher.cpp:325-328` | 关掉时 |
| §2.21 `$(find-pkg-share)` 不展开 | `odom_localizer/config/params.yaml:28`、`pcd2pgm/config/pcd.yaml:3`（全仓仅此两处） | 单独 launch 时 |

### 6.2 已剔除：**之前已经改掉**，本次不再列为问题

| 曾经的症状 | 现在的代码 | 结论 |
| --- | --- | --- |
| 发布门(0.30) 与监视门(0.30+margin) 不一致 → 刚发布就被急停 | `minco_planner.cpp:1925,1962` 两处都已是 `requiredClearance(v) - kMonitorClearanceTolerance` | 已修（**注意**：§1.1 说的是执行门与种子门没跟上这次修复，不是同一个问题） |
| 近场净空查询失败 → 整条轨迹被误杀（OUT_OF_MAP 成对日志） | `trajectory_safety_checker.cpp:133-144` 已改为"不放宽"而非 `return false` | 已修 |
| `fill_occ_min: 9` 越界 → 节点启动即死 | `planner_params.yaml:299` 为 8，满足 `config.hpp` 的 [1,8] 校验 | 已修 |
| 投影层每帧全量重算 → 建图积压 | 本次实测：`full_reason_dirty_over_ratio` 300/300 为 0，projection 中位 12.8 ms（全量时代 42.9 ms） | 已修**且本次实测确认生效** |
| 可视化快照每轮重建 | `rog_map_ros2.hpp` 已按发布频率限频（`last_viz_built/skipped` 本次实测 88/212） | 已修 |
| `smoke_goal.py` 覆盖实车 CSV | 已加 config-copy-and-rewrite（本次实测实车 CSV 未被改动） | 已修（但 `smoke_goal_motion.py` 仍未修 → §1.6） |
| 双雷达掉线后 `/lidar` 整段静默、掉线雷达回不来 | 已加单雷达降级 + 时间戳重锚定（`merge_failover.hpp`、`packet_resync_silence`） | 已修（剩下的 §2.14 是**另一个**分支：漂移导致永不配对） |
| 地形门从不打印否决点 | 已加 `Terrain rejection #N: stage=... rog_clear=...` 插桩 | 已修（诊断已可用，本次正是靠它定位 §1.3） |
| 失败原因被 2 s 节流吃掉 | 已改为按计数采样（`failure_log_first_n/every_n`） | 已修 |
| `tolerance` 语义、`safe_dist` 反复调参 | 属调参过程，不是缺陷 | 不计 |

> 说明：§6.1 中标注"每次运行"的条目都有本次实车日志/CSV 的直接计数；
> 其余为**当前代码路径确认**（读码确定"现在就在那里"），但触发频率未实测，条目内已标置信度。
> §6.2 的剔除依据是**当前代码**而不是历史记录——即"以前修的东西现在确实还是修好的状态"。

---

*本报告为只读审查产物，未修改任何代码或配置；§6 为按"只要现在确实存在的问题"要求的复审结果。*
