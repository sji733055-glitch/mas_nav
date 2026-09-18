# 全局折线穿过新障碍物 —— 判别流程（2026-09-17）

**现象**：运行中前方突然出现障碍物（人或箱子），RViz 里那条粗青色**全局路径**一直穿过障碍块，
看起来像"规划器要撞上去"。

**结论（本次核实）**：若车实际绕开了或停在障碍前，这是**预期行为**，不是缺陷。
"全局折线不重搜"是设计；真正负责绕障的是本地的滑窗修复。本文给出 30 秒判别流程，
用来区分"只是显示/参考线陈旧"和"车真的会穿过去"这两种完全不同的情况。

> 本文只做判别，不改任何代码或阈值。相关修复选项见文末。

---

## 1. 为什么全局折线不会变

全局搜索只在**没有全局路径时**才执行：

```cpp
// src/mas2027_nav_executor/src/task_manager/task_manager.cpp:153
if (!planner_->hasGlobalPath() && !planner_->PlanGlobalPath(pose, *goal_)) { ... }
```

`invalidateGlobalPath()`（清空折线，从而允许下一次重搜）全仓库只有 5 个调用点，都在 `task_manager.cpp`：

| 触发条件 | 行号 |
|---|---|
| 新目标到达（且与当前目标不同） | 68 |
| 到达终点 | 91 |
| 已提交轨迹被判不安全（急停 + 重规划） | 124 |
| 进入脱困 | 108 |
| 脱困结束 | 145 |
| 重规划失败**且**已提交轨迹到期 | 145 |

因此 FOLLOWING 期间障碍物出现时，只要旧轨迹还没到期、还没被 20 Hz 监视器判不安全，
**折线保持原样**，RViz 里就一直是穿过去的旧线。

注意它并不是永远不重搜：等"轨迹到期 + 局部重规划也失败"之后
（`task_manager.cpp:142` 的 `if (!expired) return;` 会先挡回大多数情形），
:145 会清空折线，下一拍重搜。

## 2. 谁负责绕障：局部四层兜底

`local_path_processor.cpp:171-229`。种子从全局折线上截取（最近顶点起、累计 `lookahead_dist`
= 10 m，并裁到 ROGMap 滑窗内），逐点按 ROGMap 净空复核；被实时占据截断时依次尝试：

1. `searchDynamicDetour`（:174）—— 滑窗内 8 邻域栅格 A\*，重新接回局部终点；
2. `buildSafeStoppingPrefix`（:182）—— 障碍前完整停车前缀，末速度强制为零；
3. `buildEscapePrefix`（:191，受 `stuck_escape.enable` 控制）—— 短距离脱困前缀（creep）；
4. 全败（:211）—— 清空种子，打印四层各自的否决点与 `verdict=`。

任一层成功，发布的 MINCO 轨迹就与那条粗青线**无关**：绕开或停在障碍前。
这就是"折线穿过去、车却绕开了"的组合。

**关键前提**：四道安全门（局部种子、MINCO 位置罚项、发布前校验、20 Hz 监视、MPC 指令门）
读的都是**同一张 ROGMap**：

```cpp
// src/mas2027_nav_executor/vendor/minco/src/components/planner_mode_context.cpp:39-41
global_query_ = raw_rog_query;
dynamic_query_ = raw_rog_query;
sparsify_query_ = raw_rog_query;
```

而**全局搜索**读的是另一张图：`TerrainMapQuery`（静态地形 + `map_server` 动态层，
`terrain_map_query.cpp:66` → `terrain_grid.cpp:97-122`）。两张图不一致是审计报告 §1.3 的既有结论。

---

## 3. 30 秒判别流程

### ① 局部层到底有没有发现被挡？

```bash
grep -E "Repaired a ROGMap-blocked|No local route around the live obstacle|escaping with a .* creep prefix|verdict=|Repair rejected" <本次运行的节点日志>
```

| 日志 | 含义 | 判读 |
|---|---|---|
| `Repaired a ROGMap-blocked global segment with a local grid-search detour.` | 走了第 1 层绕行 | ROGMap 看见了，车会绕开 —— **预期行为** |
| `No local route around the live obstacle; planning a safe stopping prefix.` | 走了第 2 层 | 车会停在障碍前 —— 预期行为 |
| `escaping with a X.XX m creep prefix.` | 走了第 3 层（脱困） | 车只挪一小段，可能表现为"原地不动" |
| `Live obstacle blocks ... verdict=...` | 四层全败 | 看 `verdict` 决定是几何真的过不去还是判据不一致 |
| `Repair rejected: seed invalid after repair` | 修复后又被复核否掉 | 与随后那条 `LOCAL_SEED_INVALID` 是**同一事件的两条日志**，别重复计数 |

### ② 优化器/校验是否被否

```bash
grep -E "MINCO trajectory not published|MINCO failure summary" <日志>
```

原因串的含义：`LOCAL_SEED_INVALID` / `LOCAL_SEED_REJECTED_AFTER_REPAIR` 表示还没进优化器；
`OPTIMIZER_FAILED` 表示 L-BFGS 不收敛；`COLLISION` 表示净空门否决；
`TERRAIN_COLLISION_OR_DIRECTION` 表示地形门否决（看插桩日志里的 `cell/cost/rog_clear`）。

### ③ ROGMap 跟不跟得上点云

```bash
column -s, -t .scratch/rog_map_perf_summary.csv | tail -20
```

看 `cloud_callback_hz`（输入）与 `valid_update_hz`（实际完成更新）的比值，以及
`projection_time` / `total_update_time`。现场实测基线：输入 ≈ 20 Hz、实际更新 ≈ 12.8 Hz，
约 35% 的帧被丢弃（终端会出现 `[ROG WARN] Unfinished frame cnt > 1`），
单帧全量投影 ≈ 43 ms（见 `config/planner_params.yaml:338` 的实测记录）。

### ④ RViz 看哪条线

| 显示 | 话题 | 含义 |
|---|---|---|
| `Global Plan (SMAC search, thick)` | `/nav_executor/debug/global_plan` | 全局折线，**不随新障碍更新** |
| `MINCO Trajectory` | `/nav_executor/debug/minco_trajectory` | 车真正要走的轨迹 |

**粗青线穿过障碍 ≠ 车穿过去**。判定必须看细彩线。

---

## 4. 什么情况下不再是"预期行为"

若 ① 里没有任何修复日志、③ 显示更新率正常，而**细彩线**也穿过障碍，
说明 ROGMap 没把它记进二维占据（四道门全绿）。已有依据的成因：

| # | 成因 | 位置 / 参数 |
|---|---|---|
| 1 | 建图滞后、丢帧（最大嫌疑，"提前看到就没事、突然冒出来就反应差"） | `planner_params.yaml:308-364` 的 `performance` 段 |
| 2 | 车前 0.3 m 是盲区（为避免车身点云写入） | `raycasting.ray_range: [0.3, 10.0]`，`planner_params.yaml:233` |
| 3 | 投影高度带/分类阈值把障碍归为可通行 | `scan_z_min_abs: -1.2` / `scan_z_max_abs: 2.75`、`min_observed_voxels`、`wall_height_delta_min`（:272-285） |
| 4 | 监视器只看未来 1.2 s，高速下反应距离不足 | `safety_lookahead_time: 1.2`，`minco_planner.cpp:1928` |
| 5 | 重规划失败但轨迹未到期时继续执行旧轨迹 | `task_manager.cpp:142` |
| 6 | 两张图不一致（全局搜索看 map_server 图，净空看 ROGMap） | 审计 `project_audit_2026-09-17.md` §1.3 |
| 7 | ROGMap `robot_state_` 三线程竞态导致偶发错位/漏检 | 审计 `project_audit_2026-09-17.md` §1.4 |

判据：先按 §3 ① ③ 取证，能区分的就不要凭观感改阈值。

---

## 5. 相关代码位置索引

- 全局搜索触发：`src/task_manager/task_manager.cpp:153`；失效点：同文件 68/91/108/124/145
- 全局搜索实现：`src/path_planner/search/global_path_searcher.cpp:243`（`planExploration`）、`:557`（`makePlanOnQuery`）
- 局部种子与四层兜底：`src/path_planner/trajectory/local_path_processor.cpp:93`（`buildSeed`）、`:365`（`searchDynamicDetour`）
- 折线缓存与失效：`src/path_planner/trajectory/minco_planner.cpp:2146`（`hasGlobalPath`）、`:2159`（`invalidateGlobalPath`）
- 净空判据（三处共用）：`src/path_planner/trajectory/minco_planner.cpp:1893`（`requiredClearance`）、
  `include/mas2027_nav_executor/common/environment/clearance_gate.hpp`
- 监视器视界：`src/path_planner/trajectory/minco_planner.cpp:1919-1931`
- 三张 query 同一对象：`vendor/minco/src/components/planner_mode_context.cpp:39-41`

## 6. 后续可选修复（本次**未**实施）

| 方案 | 内容 | 代价 |
|---|---|---|
| A | 局部种子一走绕行/停车前缀/四层全败就触发 `invalidateGlobalPath()` | 小；对未进 map_server 动态层的障碍无效 |
| B | 种子被实时占据否决时立即停发运动许可（不等轨迹到期） | 小；急停会更早 |
| C | 监视视界改为制动距离 `v²/(2·a_max) + 余量` | 小；检查耗时随视界增长 |
| D | 感知侧：修 `robot_state_` 竞态、压投影耗时、评估 0.3 m 盲区与高度带 | 中～大，审计已排期 |
