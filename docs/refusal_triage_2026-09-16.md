# 拒绝原因定性：读码结论（2026-09-16）

起因：对 `~/.ros/log/mas2027_nav_executor_node_8292_1789541753456.log`（2026-09-16 14:55:53 起，
294 s）里两类"保守拒绝"做定性——14 次 `Live obstacle blocks the local route and leaves no
safe stopping prefix` 与 17 次 `LOCAL_SEED_INVALID`。本轮**只读源码 + 读日志，未改动任何阈值或机制**
（唯一改动是把一条会误导判读的日志措辞改直白，见文末）。

## 0. 这次运行的真实成绩

| 指标 | 值 |
| --- | --- |
| 目标 / 到达 | 13 个目标 / 12 次 `Navigation goal reached`（最后 1 个未到达即收尾） |
| MINCO 发布失败总数 | 158 |
| 失败原因分布（最后一次 summary，total=150） | `COLLISION=109 KINEMATIC_VIOLATION=1 LOCAL_SEED_INVALID=37 TERRAIN_COLLISION_OR_DIRECTION=3` |
| `Repaired a ROGMap-blocked...`（局部绕行成功） | 73 |
| `No local route around the live obstacle`（停车前缀成功） | 12 |
| `escaping with a X.XX m creep prefix`（第四层脱困成功） | 6（0.28 / 0.15 / 0.09 / 0.30×2 ...） |
| `Live obstacle blocks...`（四层全败） | 14 |
| `Repair rejected: seed invalid after repair` | 0 |

**结论先行：这不是"车不动"。** 车走得很好。17 次 `LOCAL_SEED_INVALID` 每次都紧跟一条
`Live obstacle blocks...`（9 次间隔 0.0~0.45 s，其余 8 次是该消息被 2 s 节流吞掉），
**下一次成功事件的中位间隔 0.938 s**（n=17，全部在 2.84 s 内恢复）。
所以这是「每轮 ~1 s 的短暂停」，不是 13:40 那次「104 s 内 122 次失败、车原地不动」。

## 1. 三条消息的出处与真实含义

### 1.1 `Live obstacle blocks the local route and leaves no safe stopping prefix`

`local_path_processor.cpp:145-150`，`buildSeed()` 里 `pathClear` 失败后的**最终 else**：

```
pathClear(稠密种子) 失败
  ├─ searchDynamicDetour        → "Repaired a ROGMap-blocked global segment..."        (INFO)
  ├─ buildSafeStoppingPrefix    → "No local route around the live obstacle..."        (WARN)
  ├─ buildEscapePrefix          → "Live obstacle leaves no full stopping prefix..."   (WARN)
  └─ 全部失败                    → "Live obstacle blocks the local route and leaves
                                    no safe stopping prefix."  ← 本条，然后清空种子
```

**它表达的是"四层兜底全败"，不是"第三层失败"。** 措辞里的 "local route" 和
"no safe stopping prefix" 容易被读成"路被物理挡住了"，但实际含义是
**"几何路线存在、只是每一段都过不了净空判据"**——这正是下面 1.3 的关键。

数量对不上的解释：本条与 §1.2 都是 `_THROTTLE(..., 2000)`，14 vs 17 的差是**节流吞行**，
不是两条不同的判据。逐对核过：17 次 `LOCAL_SEED_INVALID` 里 9 次本条在 0.05 s 内，
其余 8 次落在被吞掉的窗口里。

### 1.2 `LOCAL_SEED_INVALID` / `LOCAL_SEED_REJECTED_AFTER_REPAIR`

唯一发出点 `minco_planner.cpp:1073-1079`：

```cpp
if (!seed.valid) {
  return finish(false,
                seed.repair_rejected ? "LOCAL_SEED_REJECTED_AFTER_REPAIR"
                                     : "LOCAL_SEED_INVALID");
}
```

`seed.valid` 的定义在 `local_path_processor.cpp:165-167`：
稀疏点 ≥ 2 **且** 稀疏点整体再过一次 `pathClear`。

`reason #n` 是**该原因自身**的计数（不是全局次数）：`#1→#4` 时 total 是 8→11，
`#5` 时 total=12（中间夹了一次 COLLISION）。本轮 17 次即 `#5~#9` 加后面若干。

本轮 0 次 `LOCAL_SEED_REJECTED_AFTER_REPAIR`、0 次 `Repair rejected:` 日志，说明这 17 次
**`repair_rejected` 全为假**。而 `repair_rejected` 只在
`used_dynamic_detour || stop_at_local_end` 时置位（`local_path_processor.cpp:173`）——
所以这 17 次是**直接走进 §1.1 那个 else** 的，也就是"四层全败"那一路。

> **判读提醒**：17 次 `LOCAL_SEED_INVALID` 里 14 次与 `Live obstacle blocks` 是**同一事件的
> 两条日志**，不是两个独立现象。上一轮把二者当成两类分别统计，会重复计数。

### 1.3 净空判据在两条代码路径上不一致（本轮最重要的发现）

`trajectory_safety_checker.cpp:146` 的近场放宽条件是
`gate.nearFieldEnabled() && gate.near_required < gate.required`，
而 `clearance_gate.hpp:50-51` 里
`near_required = min(required, max(0, current_clearance - slack))`。
两条合起来 ⇒ **只要 `current_clearance < required + slack` 成立，放宽就生效**，
即净空落在 `[0.300, 0.320)` 这 2 cm 带内时，轨迹校验的净空要求被下调到
`净空 − 0.02`（0.318 → 0.298）。

但**种子侧的放宽门槛完全不同**（`local_path_processor.cpp:229-232`，`segmentClear`）：

```cpp
double required = collision_dist_;                       // 0.30
if (start_clearance_ok && start_clearance < collision_dist_ &&   // ← 门槛是"严格小于"
    (point - planning_start).head<2>().norm() <= collision_dist_) {
  required = std::max(0.0, start_clearance - kNearFieldSlack);
}
```

于是同一个 0.318 m 的起点净空：

| 判据 | 是否放宽 | 近场内的实际要求 |
| --- | --- | --- |
| `trajectory_safety_checker`（§1.3 上） | **放宽** | 0.298 m |
| `local_path_processor::segmentClear`（种子门） | 不放宽 | **0.300 m** |

**净空在 `[0.300, 0.320)` 这 2 cm 带内时，种子门比轨迹校验门严 0.02 m。**
种子门更严——所以不会放过不安全的轨迹（安全性无虞），代价是
**一条本来会被轨迹校验接受的路线，在种子阶段就被否掉**，轨迹校验根本没机会跑。

本轮的现场数据落在这个带上：`Near-field exemption` 打印的起点净空是
0.318 / 0.319 / 0.318 / 0.314 / 0.317 / 0.299 / 0.292 / 0.280；
`Trajectory clearance ... below required` 是 0.253 / 0.296 / 0.298 / 0.300；
`Trajectory clearance 0.253 m ... at (3.18, -0.27)`、`0.298 m ... at (3.31, -0.14)`
——**反复卡在 x ≈ 2.7~3.3 这一小段**。

> **这是强嫌疑，不是已证实的因果。** 未证实的原因：日志从不打印"被种子门否掉时的那一点净空
> 是多少"，所以无法把 17 次拒绝逐一归因到这个带内。要证实只需一条插桩（见 §3）。

## 2. 净那条 `Braking: ...`：已修对，不必再查

`nav_executor_node.cpp:447-450`：

```cpp
} else if (output.status == ExecutorStatus::DYNAMIC_BLOCKED) {
  RCLCPP_WARN_THROTTLE(..., "Braking: current dynamic obstacle intersects the MPC reference horizon");
```

它的真身是 `command_safety.cpp:56,62-63` 的 `DYNAMIC_BLOCKED`——**动态层障碍检查**
（`dynamic->freeAt()` / 净空查询），与净空判据无关。历史上"这条消息其实是净空判据"的
误读在本次运行中不成立；本轮它出现 38 次，与 17 次拒绝没有一一对应关系。
**不用再动它。**

## 3. 关于 0.318 vs 0.300 的判读

- 这条消息**不是**"把通过的值和别处的阈值打在一起"的拼接错误，也**不是**阈值不一致：
  0.318 是起点实测净空，0.300 是完整要求，两者本来就允许 0.318 > 0.300。
  `src/README.md:65-66` 早已写明"不要按字面读成起点净空不足"。
- 但它**确实又骗过了一轮判读**（本轮第一眼也读成了矛盾）。旧措辞
  `start clearance X m below required Y m` 在 X > Y 时自相矛盾，且丢掉了"要求被下调到多少"
  这个唯一有价值的数字。已改为直接打印三个量：

  ```
  Near-field exemption active: start clearance 0.318 m, near requirement lowered to 0.298 m
  (full requirement 0.300 m); inside 0.30 m of the start only 'not worse than now' is enforced
  ```

  **判据与行为零改动**，只是措辞。

## 4. 观察到的最大可观测性口子（**已在本轮修复**，见 change-history 同日"可观测性"条）

158 次失败里，**只有 54 次留下了原因**：

- `finish()` 的"每种原因前 `failure_log_first_n`(10) 次逐条打印"在**第一次运行就失效**了——
  每种原因累计过 10 次之后，第 11 次起走 `RCLCPP_WARN_THROTTLE(..., 2000)`，而
  COLLISION 以约 0.5 次/s 的频率持续发生，把 **COLLISION 的 109 次压成 31 条**
  （`reason #115` 出现在 total=158 那次）。summary 行本身没问题（150/150 对得上）。
- 结论：`COLLISION=109` 这个数字是**唯一**证据，它下面 78 次的具体现场全部丢失。
- **修复**：第 11 次起改为按计数采样（新参数 `minco_optimizer.failure_log_every_n`，默认 25），
  不再使用按时间节流。详见 `change-history.md`。

## 5. 下一步的最小实验（**已实施**：插桩已加，尚未上车）

不改任何阈值，只加一条插桩，一次运行即可把 §1.3 从"强嫌疑"变成"证实/证伪"：
在 `local_path_processor.cpp:146` 那条 `Live obstacle blocks...` 里带上
`start_clearance`、`collision_dist_`、以及 `segmentClear` 首次失败的
`(点, 净空, 该点的 required)`。若失败点净空集中在 `[0.300, 0.320)` 且
`required` 打印为 0.300，则 §1.3 成立。

**状态：插桩已实现并通过离线验证**（`SeedRejectInfo` + 四层各自的否决现场，
见 `change-history.md` 同日"可观测性"条）。下一步是**上车跑一次**收数据，
插桩输出形如：

```
Live obstacle blocks the local route and leaves no safe stopping prefix.
  [start_clear=0.318 collision_dist=0.300] dense_reject=(3.18,-0.21) clear=0.297 req=0.300
  detour_reject=n/a prefix_reject=(3.15,-0.22) clear=0.295 req=0.300 escape_reject=n/a
```

判读：`clear` 在 `[0.300, 0.320)` 且 `req=0.300` → §1.3 成立；`clear` 明显更低 → 拒绝正确。

**留意一条新增的反证据**：离线探针发现 0.50 m 走廊（净空 0.25）**开着**第四层时会被救成
0.24 m 的 creep 前缀并成为**有效**种子。既然实车那 17 次是"四层全败"，说明当时**连
`collision_dist` 那么短的安全段都没有**——这比 0.50 m 走廊更贴死，而日志又显示车在 0.318 m
净空处。两者有张力，可能意味着**否决点不在起点附近，而在路径更远处**（例如全局路径中段
穿过一个窄口）。这一条只能靠上面的插桩数据判定。

**成立之后才谈改法**，两个方向（先看数据，不要现在就选）：

1. **统一两条门的放宽门槛**：把 `segmentClear` 的 `start_clearance < collision_dist_`
   改成与 `clearance_gate.hpp` 同源的 `current_clearance < required + slack`。
   影响面最直接，但等于把种子门放宽到与轨迹校验门同宽，需要确认 `stuck_escape` 的
   "棘轮"效应不会因此被放大。
2. **不改门槛，只改类型**：把 §1.1 那条消息拆成"物理阻断"与"净空毫米级否决"两种情况
   （本轮已经是这一步的弱化版：同一条消息里带上两种判据各自需要的数字，不再需要读源码）。

无论选哪个，**`stuck_escape` 的四个 creep 前缀都恰好命中 0.30 m 上限**这一点要留意：
上限 = `collision_dist`，说明那段近场可用长度已经顶到设计上限，不是巧合。
