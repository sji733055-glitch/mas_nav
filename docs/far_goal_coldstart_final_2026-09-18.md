# 远点不出轨迹 / 反复启停"挪动"：最终结论（2026-09-18）

本文是 2026-09-17 一整天排障的收口文档，回答三件事：

1. **到底哪一版是真的解决了**——以 2026-09-17 21:52–22:07 的实车运行（下称 **run 7150**）为准；
2. **相对上一个提交 `7e19968` 真正改了哪些东西**（含每一条的证据与验证状态）；
3. **`docs/change-history.md` 最近两条条目哪里写错了**——它们互相矛盾，且各有一半结论没有证据支持。

> 状态：**实车验收通过**。远点目标（最远 `(19.54, -2.4)`）能连续到达；不再出现"车不动 / 只能一小段一小段挪"。
> 但窄口本身仍在：run 7150 里仍有 191 次 MINCO 失败、39 次急停、24 次净空制动（见 §5），
> 这些是现场通行性问题，不是这次要掩盖的部分。

---

## 1. 验收证据：run 7150（21:52:30–22:07，969 行）

日志：`~/.ros/log/mas2027_nav_executor_node_7150_1789653149789.log`
运行版本：`build/mas2027_nav_executor/mas2027_nav_executor_node`（构建于 2026-09-17 21:27:12，
`install/` 是指向它的符号链接），配置 `install/.../config/planner_params.yaml` → 源码 `src/.../config/planner_params.yaml`（21:35 版）。

### 1.1 目标账本：29 个目标，28 个打印到达

| # | 目标 (odom) | 发出 | 用时 | # | 目标 (odom) | 发出 | 用时 |
|---|---|---|---|---|---|---|---|
| 1 | (0.73, -0.68) | 21:52:39 | 12.9 s | 16 | (4.54, -3.92) | 21:57:49 | 17.1 s |
| 2 | (-0.96, -0.41) | 21:52:57 | 3.8 s | 17 | (9.90, -0.18) | 21:58:10 | 22.1 s |
| 3 | (1.92, -0.67) | 21:53:03 | 4.9 s | 18 | (5.33, 1.35) | 21:59:15 | 7.1 s |
| **4** | **(9.79, -0.05)** | 21:53:24 | **12.6 s** | 19 | (8.02, 0.09) | 21:59:31 | 5.5 s |
| 5 | (1.62, -0.71) | 21:53:49 | 20.2 s | 20 | (8.37, -0.52) | 21:59:40 | 2.8 s |
| 6 | (8.41, -0.64) | 21:54:23 | 10.7 s | 21 | (5.63, 0.97) | 21:59:47 | 6.7 s |
| 7 | (5.18, 0.94) | 21:56:01 | 5.8 s | 22 | (10.01, 0.18) | 22:00:01 | 33.7 s |
| 8 | (9.84, 0.07) | 21:56:20 | 5.8 s | 23 | (5.16, 0.98) | 22:01:14 | 60.6 s |
| 9 | (5.28, -2.70) | 21:56:32 | 12.1 s | 24 | (10.19, -0.06) | 22:02:36 | 33.8 s |
| 10 | (4.20, -4.34) | 21:56:51 | 4.5 s | 25 | (5.54, 2.32) | 22:03:29 | 17.4 s |
| 11 | (7.72, -3.95) | 21:57:00 | — | 26 | (11.70, -0.52) | 22:04:13 | 28.2 s |
| 12 | (9.00, -2.76) | 21:57:07 | 11.5 s | 27 | (5.90, 1.14) | 22:04:51 | 7.7 s |
| 13 | (7.37, 0.29) | 21:57:21 | 5.9 s | **28** | **(19.54, -2.40)** | 22:05:09 | **58.7 s** |
| 14 | (5.31, 1.21) | 21:57:31 | 4.0 s | 29 | (0.73, -0.67) | 22:06:25 | 25.8 s |
| 15 | (9.92, 0.20) | 21:57:37 | 5.9 s | | | | |

- 唯一没有 `Navigation goal reached` 的是 **#11**：它在 7 s 后被操作者的下一次点击（#12）顶替，
  不是规划失败。
- 去程 `(9.79, -0.05)`：日志第 33 行收目标、**第 59 行到达**（12.6 s）；返程 `(1.62, -0.71)` 同样到达。
  最远一次 `(19.54, -2.40)` 58.7 s 到达——"远点无法导航"这一条被这 29 个目标直接否证。

### 1.2 关键计数（run 7150 vs 当晚失败的两次）

| 指标 | run 7150（21:52，已修） | run 35251（21:07） | run 32290（21:05） |
|---|---|---|---|
| 目标 / 到达 | **29 / 28** | 4 / 1 | 8 / 4 |
| `Local seed gate switched to STRICT` | **0** | 2 | 26 |
| `LOCAL_SEED_INVALID` | 19 | 12 | 15 |
| `MINCO path generation failed` | 191 | 48 | 122 |
| `dynamic_stale` 制动 | **0** | 0 | 0 |
| `Braking` 次数（全是 `reason=clearance`） | 24（值 0.245~0.259 / 阈值 0.260） | 1（0.255） | 10（0.243~0.256） |
| `Publishing emergency stop` | 39 | 0 | 4 |
| 启动时 `loaded prior map` | **无** | 有 | 有 |
| `Ignoring goal` | **0** | 0 | 0 |

run 7150 仍有 `Local seed is tight but not blocked ... handing it to MINCO` 71 条：
**软种子门 + 轨迹级门**这一套在实车上是"能过去、偶尔刹车"，不是"锁死"。

### 1.3 当晚全部场次对照（同一台车，按时间）

| 日志 | 时间 | 二进制 | prior_map | strict 切换 | 目标/到达 | MINCO 失败 |
|---|---|---|---|---|---|---|
| `75351_…592` | 19:55 | 18:47 | 开 | 0 | 9 / 0（18 次 `Ignoring goal`） | 0 |
| `102977_…439` | 20:02 | 18:47 | 开 | 0 | 6 / 0 | 49 |
| `6166_…183` | 20:09–20:26 | 18:47 | 开 | 0 | 30 / 18 | 531 |
| `14967_…771` | 20:28–20:31 | 18:47 | 开 | 0 | 5 / 4 | 0 |
| `22956_…850` | 20:45 | 18:47 | 开 | 0 | 3 / 0 | 150 |
| `32290_…940` | 21:05 | 20:52 | 开 | **26** | 8 / 4 | 122 |
| `35251_…567` | 21:07 | 20:52 | 开 | **2** | 4 / 1 | 48 |
| **`7150_…789`** | **21:52–22:07** | **21:27** | **关** | **0** | **29 / 28** | 191 |

结论：**13:55 到 21:07 的每一次运行都加载了先验图，其中到达率为 0 的 20:02 / 20:45 与到达率
0.6 的 20:09 都在这一段里；21:52 那次没有加载先验图，29 个目标到 28 个**。
`strict` 只在 21:05/21:07 两次触发，且两次都把到达率打到 ≤ 0.5。

---

## 2. 相对 `7e19968` 真正的改动（即将提交的净差异）

工作区（`src/`）相对 `HEAD = 7e19968` 的净改动，按症状分三组。**注意**：有些实验（如
`penalty_weight_time 100→500`、`prior_map.enable true`）只存在于当天的未提交中间态，现已被回退，
因此**不在下面的净差异里**（`7e19968` 本来就是 100 / false）。

### A. 远点目标不出轨迹

| # | 改动 | 文件 | 机制 / 证据 |
|---|---|---|---|
| A1 | 种子门恢复"软"语义：折线净空不足不再否决种子，只查占据/地形/查询失效 | `local_path_processor.{hpp,cpp}`（新增 `enforce_seed_clearance`，默认 false）、`minco_planner.cpp` 传参 | 净空是**轨迹**属性：交给 MINCO 的只是折线种子，优化后的轨迹由三道门验收；旧工程 `mas_nav_2027` 的 `isLineFree` 就是这个口径。实车：run 7150 有 71 条"tight but not blocked"照样到达 |
| A2 | 四道净空门统一有效阈值 `0.28 − 0.02 = 0.26` | 新增 `common/environment/clearance_gate.hpp`（`kEsdfJitterTolerance` / `effectiveClearanceThreshold`）、`command_safety.cpp`、`local_path_processor.cpp`（种子门/绕行搜索/判读器） | 曾经：发布前校验与 20 Hz 监视用 0.26，MPC 指令门与种子门用 0.28 → "规划放行、执行刹车"。现场证据：`61759_*.log` 中 `threshold=0.280` 的 8 次否决值域 0.261~0.280，**全部落在 2 cm 缝里**；修后 run 7150 打印的就是 `threshold=0.260` |
| A3 | `smac_2d.esdf_max_cost: 0.5 → 5.0` | `config/planner_params.yaml` | `potential = min(1.0·e^(−d/0.8), 0.5)` 在 `d ≤ 0.8·ln2 = 0.5545 m` 内梯度恒为 0 → 全局搜索只剩"最短路径"，折线贴墙切内角。`max_cost ≥ weight` 时封顶永不生效。台架量测：min 净空 0.391 → 0.427 m，贴墙采样点比例 10.4% → 6.2%（路径长度 2.755 → 2.749 m） |
| A4 | 先验图保持关闭（`prior_map.enable: false`） | `config/planner_params.yaml`（仅重写注释） | 先验 PGM 与本车 `map↔odom` 有错位记录；开着时返程在 `(3.78,1.62)`、去程在 `(4.8~6.0, 0.35~0.78)` 被压到 `clear 0.244~0.250 < 0.260`。当天 13:55–21:07 的运行全部加载了先验图，到达率 0~0.6；21:52 关着，28/29 到达。**这是相关性，不是单变量证明**（同时回退的还有时间权重与硬门） |
| A5 | `minco_optimizer.strict_seed_after_failures: 0`（禁用"连续 3 次失败后把种子门切回净空硬否决"） | `planner_params.yaml`、`minco_planner.{hpp,cpp}` | 实车 21:05/21:07：切到 STRICT 后 `(3.78,1.62)` 附近以 `clear=0.244~0.245 / req=0.260` 反复 `LOCAL_SEED_INVALID`，返程走不动。⚠️ 这条的**长期正确性有争议**，见 §4 P0 |

### B. "一卡一卡 / 反复启停（被称作冷启动）"

| # | 改动 | 文件 | 机制 / 证据 |
|---|---|---|---|
| B1 | 动态层新鲜度阈值参数化：硬编码 `0.5 s` → `node.dynamic_map_timeout_s: 1.5` | `config/node_params.yaml`、`nav_executor_node.cpp`、`path_planner.{hpp,cpp}`（目标接纳）、`command_safety.{hpp,cpp}`、`path_executor.hpp` | `map_server` 在 `bypass_dynamic_obstacle:=True`（本仓库默认）下由 **500 ms 定时器**发全 0 空图，判据也是 0.5 s ⇒ 零余量。现场：`9049_*.log`（11:40）**175 条** `reason=dynamic_stale value=0.501 threshold=0.500`；改后 `83784`（13:55）/`92461`（14:02）/run 7150 全部为 **0** |
| B2 | 打断后不再把 MPC 加速度锚点清零，改锚到**实测车速** | `mpc_solver.hpp`（`setLastControl`）、`path_executor.cpp`（轨迹过期/参考失败/求解失败/门否决四条路径） | 加速度约束 `a_min·dt ≤ u_0 − anchor ≤ a_max·dt`，清零后恢复首拍被限成 `a_max·dt = 0.2 m/s`，而车可能还在 1.5 m/s → 硬刹再爬升，门控 1~2 Hz 抖动时就是"反复启停"。**不放松任何速度/加速度/净空限值**，只让约束与物理一致。单测给出：清零首拍 0.2 m/s，锚 1.5 m/s 后首拍 1.7 m/s |

### C. 其它（远端调试链路 + 可观测性）

| # | 改动 | 文件 | 机制 / 证据 |
|---|---|---|---|
| C1 | 目标时间戳归零后再查 TF | `path_planner.cpp`（两处 transform 前各归零一次） | 远端（Foxglove）用客户端时钟打戳，本机比笔记本快约 330 ms ⇒ tf2 拒绝"向未来外推"，目标被静默丢弃。现场：`75351_*.log`（19:55）18 次 `Ignoring goal: terrain TF unavailable (extrapolation into the future)`；修后 21:05 起 0 次，run 7150 的 29 个目标全部被接纳 |
| C2 | 净空/种子/失败原因日志补齐（`clearance_only`、带坐标的 WARN、`reason/value/threshold`） | `local_path_processor.{hpp,cpp}`、`command_safety.cpp` | 让"贴墙但能过"与"真堵死"在日志里可区分——这是本次能定位 A1/A2 的前提 |
| C3 | 回归用例 | `test_local_path_processor.cpp`、`test_mpc_horizon.cpp`、`test_rog_map_command_safety.cpp` | 锁住软种子门语义、锚点语义、四道门统一阈值与动态层余量 |
| C4 | 台架/实车工具与文档 | `test/bench_closed_loop.py`、`test/trace_speed.py`、`docs/{project_audit,global_plan_stale_triage,executor_velocity_triage,foxglove_remote_debug}_2026-09-17.md` | 真实 map_server + nav_executor 闭环复现；三路速度记录 |

**本机复验（2026-09-18）**：`colcon build --packages-select mas2027_nav_executor --symlink-install
--cmake-args -DCMAKE_BUILD_TYPE=Release` 通过；`colcon test` **9/9 通过**。

---

## 3. 对最近两条历史条目的纠错

`docs/change-history.md` 顶部两条（同一天）互相矛盾，用户点名"有犯错"。逐条对账如下。

### 3.1 条目《修"远处点导航失败 / 车原地不动"：软种子门连续失败后退回净空硬否决》

| 它的说法 | 实际情况 | 判定 |
|---|---|---|
| "三层兜底（ROGMap 绕行 → 停车前缀 → 脱困前缀）**根本不会被执行**，规划器每 0.5 s 重复同一个不可能成功的优化" | **错**。软种子门下兜底链照常执行：run 7150 第 39 行 `Repaired a ROGMap-blocked global segment with a local grid-search detour.`，且 28/29 目标到达。兜底链的触发源是 ROGMap 占据否决，与"净空算不算否决种子"是两条独立判据 | ❌ |
| 因此新增 `strict_seed_after_failures: 3` 是远点问题的解 | **反了**。启用它的两次运行（21:05 / 21:07）目标到达率反而掉到 8/4、4/1，`switched to STRICT` 26 / 2 次，之后长期 `LOCAL_SEED_INVALID` | ❌ |
| "台架：改前 0/1986 拍有轨迹、位移 0.00 m；改后 2804/2974 拍、位移 5.22~6.31 m" | 数据本身可查，但**台架场景是合成回放**：该条目自己承认那里的"真实净空"只有 ~0.25 m、PCD 与地形图不一致。用这种台架去决定实车判据，方向被带反了 | ⚠️ 证据不可用 |

### 3.2 条目《回归旧工程的连续规划语义：禁用严格种子切换、恢复 HOT_START 与时间权重》

方向与 run 7150 的实车结果一致（软种子门 + 时间权重 100 + 先验关 = 28/29 到达），但三处与事实不符：

| 它的说法 | 实际情况 | 判定 |
|---|---|---|
| "大位置/速度误差**继续返回 HOT_START**，不再丢弃热启动种子"——被列为行为改动 | **纯日志改动**。`git show 7e19968:.../minco_planner.cpp` 第 1574–1585 行本来就是 `return PlanningState::HOT_START;`，`COLD_START` 那行一直是注释。本次只把误导性的 `Downgrading to COLD_START` 文本改成 `keeping HOT_START` | ❌ 归因错 |
| "禁用严格种子切换（→0）"被当作远点问题的解，理由是"连续 3 次失败切硬门会在近场锁死" | 实车只能证明"**硬门在那一点也失败**"，不能推出"永久关闭硬门"。同一个条目引用的台架 A/B（条目 3.1 的表）方向恰好相反。真正让 21:52 那次成功的是**先验关 + 时间权重 100 + 硬门不触发**的组合，无法把功劳单独归给"关掉硬门" | ⚠️ 过度归因 |
| "验证：9/9 通过 + 标准冒烟 + 20 s 闭环"（条目自称"未上实车"） | 属实，但那是**台架**。真正的验收是它写完之后 15 分钟的 run 7150；条目里没有这次实车结论 | ⚠️ 缺实车验收 |

### 3.3 两条共同的错误

1. **把"净空差几毫米"当成"种子门该更严"来解决**。净空是轨迹属性，折线净空 0.244 m 完全可以被
   MINCO 优化成 0.30 m；拿折线净空否决种子，等于要求种子先满足轨迹指标，MINCO 的避障能力作废。
   正确做法是 A2（四道**轨迹**门统一 0.26）而不是在种子门上加减阈值。
2. **拿台架当实车结论**。两次条目都写了"未验证：尚未上实车"，但正文的因果表述是按"已解决"写的，
   导致后一条去回滚前一条，来回两次都没有实车数据支撑。本仓库的规矩应当是：
   **种子门/净空/速度这类判据的结论，必须由实车日志或明确标注的合成场景给出**。

---

## 4. 仍未解决 / 风险清单（按优先级）

- 🔴 **P0｜`strict_seed_after_failures: 0` 的证据只有一半**。实车证明"开着它会更糟"，
  但没证明"关着它，窄口每次都能过"。run 7150 里 191 次 MINCO 失败 + 39 次急停说明：
  窄口每次都在靠兜底链硬过。**建议保留一个有界退路**（例如连续失败 N 次 → 停车 + 报警，
  或降级蠕行），而不是把回退路径变成死代码。若下一次现场出现"唯一通路比 required 窄、
  车不动"，目前**没有任何自动恢复**，只能人工换目标。
- 🔴 **P0｜STRICT 链路成了死代码**。`planner_params.yaml` 写 0、`minco_planner.hpp` 成员默认仍是 3、
  `configure()` 默认 0、`consecutive_local_failures_` / `strict_seed_active_` / `buildSeed(..., bool)`
  及只为它存在的测试分支都还在。上一轮 21:42 已决定"整条删除"但被中断。需要一次专项提交
  （删除 或 明确保留为诊断开关 + 上述有界退路），本次不动。
- 🟠 **P1｜锚点没做可行域裁剪**：若实测速度（含里程计尖峰）使 `anchor > v_max − a_min·dt`，
  QP 首行不可行 → `solve()` 失败 → 又锚回同一实测值 → 连续失败直到车被刹停。建议按轴夹到
  `[v_min − a_max·dt, v_max − a_min·dt]`。
- 🟠 **P1｜`command_safety` 对 `dynamic_map_timeout_s ≤ 0` 是 fail-open**（等于静默关掉这道门）；
  而 `PathExecutorParams::dynamic_map_timeout_s{}` 默认就是 0。建议默认给 1.5 或改 fail-closed。
- 🟠 **P1｜"四道门统一 0.26"依赖隐藏前提**：executor 取 `rog_map_clearance`，planner 取
  `collision_dist + max(v·replan_react_time, monitor_margin)`。现在相等只因为后两项为 0；
  一旦按旧建议把 `replan_react_time` 调到 0.05~0.1，轨迹门会重新严于种子门/绕行搜索。
- 🟠 **P1｜`allow_motion == false`（等新轨迹/刚换目标）分支仍清零锚点**，这条路径上车还在动，
  仍会出现 0.2 m/s 首拍。若 odom 新鲜，这里也应锚到实测车速。
- 🟡 **P2｜测试断言被放宽**：`test_local_path_processor.cpp` 里稀疏点净空断言 `0.30 → 0.0`、
  `dense_path.front()` 容差 `1e-6 → 0.10`。新语义下可以理解，但该用例不再保护任何净空数值，
  建议补一条"轨迹级门仍按 0.26 验收"的断言。
- 🟡 **P2｜`.scratch/` 未进 `.gitignore`**（本次已补），一次性脚本/日志有被 `git add -A` 误提交的风险。
- 🟡 **P2｜窄口本身没动**：`x ≈ 4.8~5.6` 处实测净空 0.244~0.259 m（要求 0.260 m），
  来自在线层；PGM 先验图同一处是空的。这是环境问题（挪开物体或改路线），**本轮没有放松任何阈值**。

---

## 5. 复现与验证

```bash
# 构建（工作区根目录）
colcon build --packages-select mas2027_nav_executor --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select mas2027_nav_executor --event-handlers console_direct+
colcon test-result --verbose        # 期望 9/9

# 闭环台架（真实 map_server + nav_executor + lab3 地图/PCD）
python3 src/mas2027_nav_executor/test/bench_closed_loop.py \
  src/mas2027_nav_bringup/map/lab3_terrain.msgpack src/mas2027_nav_executor/config \
  /home/mas/mas_nav_2027/mas2027_nav_bringup/pcd/lab3.pcd \
  --start 1.211,-0.097 --goal 7.26,2.28 \
  --map-to-odom 0.046,0.123,0.274,0.0057,-0.0034,-0.0119,0.9999 --seconds 60
```

下次上车的观察清单（按顺序 grep run 日志）：

| 检查 | 期望 |
|---|---|
| `loaded prior map` | 不出现 |
| `switched to STRICT` | 不出现（若出现，说明又开了硬门） |
| `dynamic_stale` | 不出现（或只有真实丢帧时偶发） |
| `Ignoring goal` | 不出现（远端点击的目标必须被接纳） |
| `Navigation goal reached` / `Received goal` | 目标数基本相等 |
| `Braking ... reason=clearance` | 允许出现（窄口），但**不应**伴随长时间无轨迹 |

关键日志索引：

| 用途 | 路径 |
|---|---|
| 验收（成功） | `~/.ros/log/mas2027_nav_executor_node_7150_1789653149789.log` |
| 硬门锁死现场 | `~/.ros/log/mas2027_nav_executor_node_35251_1789650374567.log`（第 36 行切 STRICT）、`…_32290_1789650047940.log` |
| 动态层零余量现场 | `~/.ros/log/mas2027_nav_executor_node_9049_1789615395575.log`（175 条 `dynamic_stale`） |
| 四道门 2 cm 缝 | `~/.ros/log/mas2027_nav_executor_node_61759_1789622678611.log`（`threshold=0.280`，值 0.261~0.280） |
| 目标被时钟偏差丢弃 | `~/.ros/log/mas2027_nav_executor_node_75351_1789643344592.log`（18 次 `Ignoring goal`） |
