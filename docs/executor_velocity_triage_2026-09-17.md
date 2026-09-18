# 「MPC 输出的速度让车慢且卡顿」排查（2026-09-17）

用户报的现象：*"经过 MPC 输出的速度，会使得频繁地进入冷启动；行驶速度慢且卡顿。
/home/mas/mas_nav_2027 里是正常的，设 3 m/s 上限就跑得很快，应该不是上下位机传输的问题。"*

结论先说：

1. **MPC 不是限速环节。** 台架里规划峰值 2.5~2.9 m/s，MPC 输出跟着到 2.4~2.6 m/s，
   odom 峰值 2.2~2.6 m/s。慢不是"MPC 输出被压住"，而是**规划侧反复拒绝发布轨迹**：
   `/opt_path` 一断，执行器只能发 0，车停下；下一次规划成功再重新起步。
2. **"频繁冷启动"是症状不是原因。** 规划器的 `Large velocity error → COLD_START` 打印，
   是在车被刹停、而上一条轨迹还以为自己在 1.5 m/s 时触发的自愈动作；停车的根因在规划侧。
3. 本次改了两处**代码**（都与旧工程对齐、都不放松任何净空/速度阈值），见 §3。
4. 配置层面的可疑项（rho、safe_dist、ROGMap 高度窗口）在台架上**双峰、单次无法判定**，
   因此**没有**动配置，只在 §5 给出实车验证方法。

---

## 1. 台架与量法

`test/bench_closed_loop.py`（本次固化进仓库）：真实 `map_server` + 真实 `nav_executor`
+ 真实 `lab3` 地图，底盘换成一阶速度模型（τ=0.10 s），点云用 `lab3.pcd` 按当前位姿回放，
记录四路：

| 列 | 含义 |
|---|---|
| `plan_peak` / `plan_near` | `/opt_path` 的峰值速度 / 离车最近点的速度（与 `PathExecutor::buildReference` 同口径） |
| `cmd` | `/cmd_vel` 线速度（MPC 实际发出的） |
| `odom` / `x,y` | 底盘实际速度与位置 |
| 节点 stdout | `COLD_START` / 停车前缀 / `MINCO path generation failed` / `Publishing emergency stop` |

判读用三个量：**移动拍占比**（`odom>0.05` 的采样比例）、**走完前 5 m 的用时**、
**`MINCO path generation failed` 次数**（= 这一轮有多少拍根本没有轨迹可执行）。

## 2. 证据

### 2.1 台架（同一起点/目标，40 s 单目标；`—` 表示没走到）

| 运行 | 变更 | 移动拍 | `cmd` 中位 | 位移 | 前 5 m 用时 | MINCO 失败 |
|---|---|---|---|---|---|---|
| A2 | 原版（种子门硬否决，rho=500） | 41% | 0.00 | 8.3 m/30 s | 11.2 s | 4 |
| B1 | +种子门修复 | 40% | 0.00 | 8.7 m/30 s | 10.2 s | 11 |
| G1 | +ROGMap 高度窗口对齐旧工程 | 30% | 0.00 | 8.5 m/30 s | 18.1 s | 19 |
| H1 | +rho=100 | 70% | 0.33 | 15.1 m/30 s | 7.0 s | 9 |
| R100a | 种子修复 + rho=100 | 78% | 0.40 | 19.5 m/40 s | 17.2 s | 11 |
| R500a | 种子修复 + rho=500 | 11% | 0.00 | 4.4 m/40 s | 卡住 | 62 |
| R100b | 种子修复 + rho=100 | 19% | 0.00 | 4.6 m/40 s | 卡住 | 57 |
| R500b | 种子修复 + rho=500 | 73% | 0.41 | 19.2 m/40 s | 17.5 s | 8 |
| FIXa | 种子修复 + 锚点修复 | **84%** | 0.43 | 19.8 m/40 s | 12.4 s | 7 |
| FIXb | 同上 | 30% | 0.00 | 9.5 m/40 s | 9.5 s | 11 |

**机制相关性成立**：`MINCO 失败 <= 11` 的那些运行，移动拍 70~84%、位移 15~20 m；
`MINCO 失败 >= 57` 的运行，移动拍 11~19%、位移 4.4~4.6 m。也就是说"慢"= 轨迹发布率掉下去了。

**但配置 A/B 是双峰的**：rho=100 与 rho=500 各自都出现过"好"和"坏"的两次
（R100a 78% / R100b 19%；R500b 73% / R500a 11%），所以**不能**用单次台架结果决定配置。
双峰来自起点：odom(0,0) 在 map 系里净空只有 **0.158 m**（贴墙起步），头 1 秒能不能从
near-field 放宽里挤出来基本是掷硬币；挤不出来就一直在原地重规划。

### 2.2 实车日志（2026-09-17 14:02 那次，3 个目标全到）

- 第 3 段 (6.45,2.58)→(0.72,0.19)，约 6.8 m 用了 12.8 s ⇒ **平均 0.53 m/s**，
  而**全程没有任何 Braking / 停车前缀 / 急停**（日志 28 行，全是正常规划）。
- 同一轮的 `.scratch/speed_trace.csv`：`cmd` 1.14~1.73 m/s，`odom` 0.55~0.91 m/s，
  且 `odom` 与位姿位移一致（0.66 m/s vs 0.62~0.75），**不是里程计虚报**。
  ⇒ 这一轮"慢"不在规划也不在 MPC，而在**底盘只跟到指令的一半**，或 `/cmd_vel` 时序。
- 台架里量到的 `/cmd_vel` 间隔：中位 45~52 ms（标称 50 ms），p90 61~80 ms，**max 94~100 ms**。
  旧工程链路末端是 nav2 `velocity_smoother` 定频重发，本工程由执行器定时器直接发；
  底盘若有指令超时/每条指令限加速度，这种抖动就会吃掉平均速度。

### 2.3 环境本身的余量（lab3 地图，实车活动区 odom 附近）

- 自由格里只有 **35%** 的格子净空 ≥ 0.26 m（`collision_dist 0.28 − kEsdfJitterTolerance 0.02`
  = 有效硬阈值）；该区域自由格的平均净空只有 **0.21 m**。
- 净空 ≥0.26 m 的最长连续直线段只有 **3.45 m**。
- 台架/实车日志里的轨迹否决绝大多数是**毫米级**：`clearance 0.258 vs required 0.260`、
  `0.250 vs 0.260`、`0.226 vs 0.232`（近场内）。这正是 `planner_params.yaml` 里
  反复记录过的"加速 → 差几毫米被否 → 急停 → 再加速"极限环。

## 3. 本次改动（只改代码，不动阈值）

### 3.1 种子门不再用净空硬否决（`local_path_processor.cpp` / `.hpp`）

交给 MINCO 的是**折线种子**，真正执行的是**优化后的轨迹**；MINCO 的位置罚项
（`safe_dist 0.33`）会把轨迹推离障碍，"折线净空 0.244"完全可以优化成"轨迹净空 0.30"。
旧工程 `mas_nav_2027` 的 `isLineFree` 只查占据
（`minco_core/components/local_path_processor.cpp:9-31`），它的轨迹级门槛反而更严
（`collision_dist 0.30`、无抖动容差），实车却能跑快。本工程多出来的这道"折线净空门"
把 MINCO 的避障能力作废了，代价是 `LOCAL_SEED_INVALID` → 停车前缀 → 0.28 m 蠕行。

改动：
- `segmentClear(..., enforce_clearance)`：**占据 / 地形 / 查询无效仍是硬否决**；
  净空不足只记录现场（`SeedRejectInfo::clearance_only`）并继续采样。
- 停车/脱困前缀（`buildStoppingPrefixCore`）传 `enforce_clearance=true`：
  前缀末端就是"车要停在哪"，它本身必须满足净空，否则连停车轨迹都会被发布前校验拒掉。
- 新增一条可查的 INFO：`Local seed is tight but not blocked (closest clear=... req=...)`，
  与 `Live obstacle blocks the local route`（真堵死）区分开。
- `note_reject` 允许"硬否决覆盖先前的软记录"，避免"前面贴墙、后面堵死"被记成只是贴墙。
- 净空要求仍然由轨迹级三道门把关（发布前校验 / 20 Hz 监视 / MPC 指令门），**一处都没放松**。

### 3.2 MPC 加速度锚点：打断时锚到实测车速（`mpc_solver.hpp` / `path_executor.cpp`）

加速度约束是 `a_min·dt <= u_0 - anchor <= a_max·dt`。此前 `resetLastControl()` 把 anchor
清零 ⇒ 恢复后的**第一拍**被限成 `|u_0| <= a_max·dt = 0.2 m/s`。车还在 1.5 m/s 跑着，
指令却被砍到 0.2 ⇒ 底盘急刹，再按每拍 0.2 m/s 爬回去；轨迹过期 / 参考失败 / 求解失败 /
任一道门否决都会走这条路。门控 1~2 Hz 一抖，就是现场听到的"反复启停"，也就是用户说的
"频繁进入冷启动"。

现在这几条分支改为 `setLastControl({vx_odom, vy_odom, ω_odom})`：指令从车真正所在的速度
接着走。速度上下限、加速度上下限、三道净空门全部不变，只是把约束从"相对上一条指令"改成
"相对车实际速度"——后者才是真正的加速度。没有里程计的分支仍然清零（保守）。

## 4. 验证

1. `colcon build`（Release）通过；`ctest` **9/9 通过**。
2. `test_mpc_horizon` 新增锚点用例：`anchor semantics: reset cap 0.2 m/s,
   measured 1.5 m/s -> first step 1.7 m/s`（锁住"清零会把 1.5 m/s 砍到 0.2"这条回归）。
3. `test_local_path_processor` 按新语义改写并保留反例：
   - `0.50 m 宽走廊（中线净空 0.25 < collision_dist 0.30）` ⇒ **种子有效、原样交给 MINCO**，
     `dense_reject.clearance_only == true`；
   - `走廊中段整列封死 + 车离封口 0.10 m` ⇒ 四层兜底全败、`dense_path` 清空、
     `clearance_only == false`、`verdict=GEOMETRY`（真堵死仍然拒绝）。
4. 标准冒烟：
   - `smoke_goal.py`（隔离域）→ `goal smoke passed: 40 trajectory poses, 48 global plan poses, marker width 0.150 m`；
   - `smoke_goal_motion.py` → `cmd_vel: 240 msgs, moving 239, max speed 0.941 m/s` + `goal motion smoke passed`。
5. **未上实车**。台架里两处改动的两次运行：移动拍 84% / 30%、前 5 m 用时 12.4 s / 9.5 s，
   好于改前的 41% / 11.2 s，但受 §2.1 的双峰影响，**不能**当作定量结论。

## 5. 上车继续验证（建议顺序）

1. **先看轨迹发布率**：跑一段，数 `MINCO path generation failed; retrying`、
   `Publishing emergency stop`、`Live obstacle blocks the local route`、
   新增的 `Local seed is tight but not blocked` 四条各多少次。前三条应明显下降。
2. **量三路速度**（务必先 `source install/setup.bash`，否则 `plan` 两列恒为 0）：
   `python3 src/mas2027_nav_executor/test/trace_speed.py .scratch/speed_trace.csv 120`
   - 若 `cmd ≈ plan_near` 而 `odom ≈ cmd/2` ⇒ 继续查底盘/时序（第 3 步），规划侧已无嫌疑；
   - 若 `cmd` 大段为 0 而 `plan_peak` 有值 ⇒ 回到规划侧，看 §5.1 的日志计数。
3. **量 `/cmd_vel` 时序**：录一段 `/cmd_vel` 的到达间隔（中位/p90/max），并在下位机侧确认
   指令超时与每条指令的加速度限制。台架里 p90 61~80 ms、max ~100 ms；
   若实车更差，考虑把控制节拍与可视化回调解耦（`nav_executor_node` 的
   `/cost_map`、`/direction_map`、`/dynamic_cost_map`、`/opt_path` 四个订阅回调与
   20 Hz 控制定时器在同一个默认回调组里，可视化重建占用的时间会直接顶掉控制拍）。
4. **配置候选（必须成组、单变量、至少 2~3 次运行）**——台架单次不可判定，仅列出与旧工程的差异：
   | 项 | 本工程 | 旧工程 | 备注 |
   |---|---|---|---|
   | `minco_optimizer.penalty_weight_time` | 500（今天由 100 提上来） | 100 | 计划速度更快，但 `MINCO 失败` 次数在两次对照里 62/8 与 11/57 反向，未定论 |
   | `minco_optimizer.safe_dist` | 0.33 | 0.40 | 软目标离硬阈值只有 0.07 m；旧工程 0.10 m |
   | `rog_map.projection.scan_z_max_abs` | 2.75 | 0.75 | 本工程把车顶上方 2.75 m 内的点也投影成二维障碍；台架上该差异未单独复现出速度收益，但会污染 ESDF |
   | `rog_map.projection.fill_occ_min` | 8 | 4 | 旧工程更"补洞" |
   | `rog_map.raycasting.ray_range` | [0.3, 10.0] | [0.01, 10.0] | 本工程刻意抬下限避免车体自点，属有意为之 |
   | `rog_map.map_size`（z） | 2.5 | 1.5 | 与高度窗口配套 |

## 6. 未做的事

- 没动任何净空/速度数值（`collision_dist 0.28`、`kEsdfJitterTolerance 0.02`、`max_velocity 3.0` 全部原样）。
- 没动 `penalty_weight_time`（今天的 100→500 是上一次"想更快"的改动；台架证据不足以推翻它）。
- 没动 ROGMap 高度窗口等配置。
- 没有实车数据；台架的双峰特性意味着**所有配置类结论都必须上车复测**。
