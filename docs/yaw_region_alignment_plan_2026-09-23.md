# 定向区域偏航方案（坡道/台阶：关小陀螺 + 正对坡）

日期：2026-09-23　状态：**方案（未实现，无代码改动）**

目标行为：车导航到特定区域（最先要的是坡道）时，底盘不再自转（关小陀螺），并以指定
姿态（yaw 对准坡的轴向）通过；离开区域后恢复原行为。

---

## 0. 结论摘要

1. **要求本身是对的，而且现在的链路一定做不到**：`/cmd_vel.angular.z` 根本不出桥，底盘侧
   也没有"自转模式"输入。小陀螺由下位机内部产生，导航侧目前**没有任何通道**能关掉它或
   指定朝向。
2. 所以要动两处：**导航侧新增"定向区域 → 偏航意图"**，**下位机侧新增一个 yaw 权限/模式
   入口**。只改导航侧无法满足"正对坡"；只改下位机则它不知道该对哪个方向。
3. 导航侧可以**先做**，做到"有意图、可观测、可单测、可回放"；等下位机给通道后只差
   填字段（约 5 行）。反之不成立。
4. 区域语义不必新造：地图格式里**已经有** `SLOPE / STEP_L1 / STEP_L2 / FLY_SLOPE /
   STEP_HIGH` 与每格方向角（`direction` 通道），建图控制台也**已经能标注**。
   现状是两个地图都没画（见 §1.3），第一步是画出来。
5. 上游 HW 的同构实现（`/home/mas/下载/out_flat/`）已经把"自转是模式交接、不是 yaw 指令"
   和"地形区驱动 ω 残差 + run_up 提前激活"做出来了，本方案在命名与协议上与之对齐，
   避免以后合并两份语义。

---

## 1. 现状取证

### 1.1 三个断点

| # | 断点 | 证据 |
|---|---|---|
| ① | 导航侧**算得出** yaw 指令 | `mpc_params.yaml:13` yaw 状态权重 2.0；`mpc_solver.cpp:111` `U_ref(ω)=yaw_rate`；`path_executor.cpp:134-135` `angular.z = clamp(control.omega + spin_speed, ±2.0)` |
| ② | 但 `angular.z` **不出桥** | `mas2027_utils/ros2_comm/src/ros2_comm.cpp:49-54` `RawControlPacket{float vx; float vy; uint8_t nav_state;}`；`:180-186` 只按 vx/vy 写 `nav_state`；README:99、launch:190 均注明"协议只发 vx/vy/nav_state，不含角速度" |
| ③ | 自转由下位机自己产生，导航侧**无输入通道** | `/cmd_spin` 是 `example_interfaces/Float32`（`node_params.yaml:22`、`nav_executor_node.cpp:179-184`），**全仓无发布者**；`spin_speed_` 恒 0，`path_executor.cpp:135` 的叠加项是死代码。上游对应位置是 `/decision/spin_cmd`（`interfaces::msg::SpinCmd{spin_mode, high_priority}`），本仓未移植 |

补充：现有"坡道联动"也是半死的——`minco_planner.cpp:1267-1300` 读当前 odom 的 pitch，
超过 `constexpr 0.1` rad(≈5.7°) 才把 `goal_yaw` 改成行进方向。两个问题：**触发太晚**
（已经在坡上，yaw 早该在坡前摆好）、**只改时域末端目标 yaw**，且其产物同样卡在断点 ②。

### 1.2 方向语义已经存在（可直接复用）

- 地图格标签：`FLAT=0, OBSTACLE=1, SLOPE=2, STEP_L1=3, STEP_L2=4, FLY_SLOPE=5,
  STEP_HIGH=6`（`map_server/include/map_server/utils.hpp:12-20`；`is_directional_label` :82-89）。
- 方向通道每格 3 字节 `{angle, magnitude, label}`：`angle/255*2π` 是该格的**允许通行轴向**，
  `magnitude==255` 为硬约束体、否则是膨胀晕圈（软）。
- 规划侧已在用：`terrain_grid.cpp:105-113` `permitted()` 要求
  `|travel·dir| ≥ 0.85`，而 `label==6` 还要求 `> 0`（单向）。
  注意 **0.85 只等于 ±31.8°**：路径允许斜着切上坡，所以"路径方向 = 坡轴向"并不成立，
  必须另做 yaw 约束（这正是本方案的价值）。
- 每格角度与地图同源、同在 `map` 系；执行器查询地形时已有 `map←odom` 的 TF 处理
  （`command_safety.cpp:44-58`），照抄即可。

### 1.3 实测：当前两张地图都没有方向内容

用一次性解码脚本读 terrain msgpack：

| 地图 | 尺寸 | `terrain` 标签分布 | `direction` |
|---|---|---|---|
| `lab3_terrain.msgpack` | 770×347 @0.05 | 仅 FLAT 0 / OBSTACLE 1 | label 全 0，magnitude 全 0 |
| `lab_map_20260921_211523_terrain.msgpack` | 772×308 @0.05 | 仅 FLAT 0 / OBSTACLE 1 | label 全 0，magnitude 全 0 |

即：**坡/台阶一次都没画过**，现场的上坡在地图里就是平地。所以第一步是标注，
而标注工具已有：`/home/mas/桌面/mapping_web_ui`（README：PGM/YAML → HW map_server
terrain msgpack，并可标注平地/障碍/斜坡/各级台阶/飞坡**及其方向**）。

### 1.4 上游参考（HW）做了什么，没做什么

参考实现：`/home/mas/下载/out_flat/`（扁平化转储）。

- 下发：`/nav_executor/chassis_cmd` → `interfaces::msg::ChassisCmd{velocity, omega, mode,
  step_dist}`。**含 `omega`，也含 `mode`**；`mode: 0=NORMAL, 1=SPIN_SLOW, 2=SPIN_FAST,
  3..11=地形模式`（`chassis_defs.hpp:8-18` + `terrain_traversal.yaml`）。
- 自转 = **模式交接**：`execute_spin` 只发 `mode`，ω 留 0，由下位机自己转
  （`path_executor.cpp:647-652`）。
- 仲裁：`ControlOwner{IDLE, NAVIGATION, LOW_PRIORITY_SPIN, HIGH_PRIORITY_SPIN}`
  （`control_arbitration.hpp:9-46`），并有 `PREPARE_SPIN` 交接互锁：先刹到
  `|v_cmd|<0.5 且 |v_meas|<0.8` 才允许交权（`state_machine.cpp:115-120`,
  `config__path_executor.yaml:39-41`）。
- 地形区行为：`label → up/down → {chassis_mode, 速度窗, run_up}`；up/down 只由
  `sign(tangent · cell_direction)` 决定（`traversal_annotator.cpp:117-120`）；
  `run_up` 是**距离**，"约束锚点距物理边缘的上游距离"，把模式激活点提前
  （`traversal_annotator.cpp:71-73`）。
- 偏航：**上游没有任何绝对 yaw 设定点**。区域内的朝向对齐是 MPC 软残差——
  `cross = cosθ·dir.y − sinθ·dir.x`（`follow_problem.cpp:134-139`）+ 区域内
  `residual(17)(18)` 把 ω 压向 0（:149-150，权重 0.6/1.2）。

**结论**：上游证明了 (a) 通道必须带 `omega` 与 `mode`；(b) 区域朝向用 ω 层解决、
不引入绝对 yaw；(c) 交接要有互锁与提前量。这三条本方案直接继承。

---

## 2. 目标行为（三阶段）

以坡道为例，设坡轴方向角为 `θ_slope`（方向层该格角度，符号按行进方向取）：

| 阶段 | 触发 | 期望输出 |
|---|---|---|
| 接近（align） | 当前位置沿路径前方 `run_up`(默认 0.8 m) 内出现定向格 | 关小陀螺；yaw 以限斜率转向 `θ_slope`；线速度上限降到 `align_vmax`(默认 0.4 m/s)；允许原地先摆正 |
| 通过（cross） | 车体已在定向格内 | 保持 yaw 误差 `≤ tol`(默认 10°)；ω 阻尼（不要边转边上坡）；不因路径曲率而扭 |
| 离开（release） | 离开定向格并满足滞回（越过 ≥0.5 m 或 ≥1.0 s） | 交还权限：允许小陀螺；yaw 恢复由 MPC 跟踪路径 |

失败降级：**拿不到 yaw 权限就停在区域外**（`on_unavailable: hold`，默认），
绝不"边自转边爬坡"。可配 `on_unavailable: proceed` 保留旧行为。

---

## 3. 方案设计（四层）

### L1 区域与方向语义：方向层是唯一真源

- 区域定义继续用 terrain msgpack 的 `SLOPE/STEP_*/FLY_SLOPE` 格 + 每格 `angle`：
  一处标注，规划侧（`permitted()` 方向门）与执行侧（yaw 意图）共用，不会出现
  "地图说能斜着走、执行器却要正着走"的两套事实。
- 行为参数放独立 YAML（新增 `config/terrain_traversal.yaml`），按 label 配置，
  与上游同名同义：

```yaml
terrain_traversal:
  directional_labels:
    slope:      { yaw_lock: true,  align_dist: 0.8, yaw_tol: 0.17, omega_max: 1.5, align_vmax: 0.4, cross_omega_damp: 0.6, on_unavailable: hold }
    step_l1:    { yaw_lock: true,  align_dist: 1.2, yaw_tol: 0.10, omega_max: 1.2, align_vmax: 0.2, cross_omega_damp: 1.0, on_unavailable: hold }
    step_l2:    { yaw_lock: true,  align_dist: 1.5, yaw_tol: 0.10, omega_max: 1.2, align_vmax: 0.2, cross_omega_damp: 1.0, on_unavailable: hold }
    step_high:  { yaw_lock: true,  align_dist: 2.0, yaw_tol: 0.08, omega_max: 1.0, align_vmax: 0.2, cross_omega_damp: 1.2, on_unavailable: hold }
    fly_slope:  { yaw_lock: true,  align_dist: 1.0, yaw_tol: 0.10, omega_max: 1.5, align_vmax: 0.4, cross_omega_damp: 0.8, on_unavailable: hold }
  exit_hysteresis: { distance: 0.5, time: 1.0 }
  lookahead: { min: 0.3, max: 2.5 }   # 前瞻窗，别超出 MPC 时域
```

- `align_dist` 就是上游 `run_up`；`yaw_tol` 比方向门的 ±31.8° 严得多，是"正对坡"的量化。
- 兜底：若某 label 只画了区域没给方向（magnitude 全 0），策略表里显式
  `yaw_lock: false` 或按"锁当前 yaw"处理，不要拿 0 弧度当真值。

### L2 偏航意图生成（新增 `TraversalDirector`）

位置：`include/mas2027_nav_executor/common/environment/traversal_director.hpp`（+ src）。

输入：当前位姿（odom 系）、当前轨迹/全局折线、`TerrainGrid` 快照、时间戳。
输出：`YawDirective`

```cpp
enum class YawAuthority : uint8_t { CHASSIS_SPIN = 0, NAV_ALIGN = 1 };
struct YawDirective {
  YawAuthority authority{YawAuthority::CHASSIS_SPIN};
  bool active{false};         // 是否处于定向区（含提前量）
  uint8_t label{0};           // 2..6
  double yaw_ref{0.0};        // 已限斜率的期望 yaw（odom 系）
  double omega_cap{2.0};      // 本区 ω 上限
  double vmax{3.0};           // 本区线速度上限
  double cross_damp{0.0};     // 通过段 ω 阻尼权重
  bool yaw_valid{false};      // 方向层是否真的给了角
};
```

算法（每个控制周期 20 Hz）：

1. 取车体点沿路径向前 `[0, align_dist]` 采样（用已有轨迹/全局折线，等弧长取 5~10 点），
   换到 `map` 系（`map←odom` TF，与 `command_safety.cpp:44-58` 同款）。
2. 命中第一个 `label∈[2,6] 且 magnitude==255` 的格 → `active`。
3. `θ_cell = angle/255*2π`；`tangent = ` 该段路径切向；`up = (tangent·dir) > 0`；
   `θ_target = up ? θ_cell : θ_cell + π`（归一化）。这就是"上坡面朝上、下坡面朝下"。
4. 限斜率：`yaw_ref` 从当前值朝 `θ_target` 以 `rate_max`(默认 1.2 rad/s) 与
   `alpha_max`(默认 3.0 rad/s²) 推进；**每周期变化 ≤ 3~5°**，避免 MPC 的
   `buildStepModel(ref.yaw)` 线性化点跳变（`mpc_solver.cpp:70、89` 用参考 yaw 做 LTV）。
5. 滞回：`active` 退出需同时满足"前瞻窗内无定向格"且"离开 ≥ exit_hysteresis"，
   防边界抖动导致权限来回交接。
6. `authority = active ? NAV_ALIGN : CHASSIS_SPIN`；`yaw_valid=false`（无方向数据）
   时 `authority` 保守取 `NAV_ALIGN` 且 `yaw_ref` 锁当前 yaw（不猜方向）。

### L3 执行器侧：权限与融合

`ExecutorInput` 增加 `YawDirective directive`（`execution_state.hpp:23-30`）；
`computeCommand`（`path_executor.cpp:57-138`）按权限分两路：

- **CHASSIS_SPIN（默认，保持今天行为）**：线速度照旧；`angular.z` 不下发
  （`angular_z_valid=false`），自转由下位机负责；`/cmd_spin` 前馈按上游语义
  改为"下位机自转模式请求"（见 L4），不再当数值叠加。
- **NAV_ALIGN**：
  1. `spin_speed := 0`，且**禁止**下发自转模式（`mode=0`）。
  2. `buildReference()` 之后、`solver_->solve()` 之前，对参考序列做
     `applyYawDirective(reference, directive)`：把 `reference[i].yaw` 覆盖为
     `yaw_ref`（同一值即可，或按 `yaw_ref + 0` 常量段），`yaw_rate` 置 0，
     让 MPC 自己产生跟踪 ω；同时把 `control.omega` 限到 `directive.omega_cap`。
  3. 通过段加 ω 阻尼：对解出的 `control.omega` 加一项
     `-cross_damp * sin(yaw_meas − yaw_ref)`（或直接在代价里加残差，见"取舍"）。
  4. 线速度限 `directive.vmax`；`|yaw_err| > yaw_tol` 且尚未进入区域时可
     只转不走（`hypot(vx,vy) < deadzone_speed` 时现有逻辑本就只发角速度，
     `path_executor.cpp:125-133`，无需新开关）。
  5. 安全门（`checkCommandSafety`）保持原样：它只看平移与控制量，不读 yaw，
     区域限速只会让它更容易通过。

**为什么先覆盖参考、而不是只加上游那种软残差**：覆盖式可预测、可单测、
"正对坡"是硬要求；软残差在路径本身斜切时会被 `tracking` 项拉回去。建议覆盖式为主、
`cross_omega_damp` 作为通过段的附加阻尼（两者叠加不冲突）。若实车发现转向太猛，
把 `omega_cap` 和斜率上限调小即可，不必改结构。

### L4 控制权与协议

**权限语义**（谁拥有 yaw）：

| 状态 | yaw 拥有者 | 导航侧行为 | 下位机行为 |
|---|---|---|---|
| CHASSIS_SPIN | 下位机 | 不发 ω；只发平移 | 小陀螺/自持朝向，忽略 ω |
| NAV_ALIGN | 导航侧 | 发 ω（限幅） | 关小陀螺，按 ω 闭环 |

交接规则（借上游 `PREPARE_SPIN` 互锁的反向版）：

- **进入**：`active` 由 L2 在区域前 `align_dist` 处拉起 → 导航侧先请求
  `mode=NAV`，**等下位机回执** `yaw_follow_active`；收到回执前速度限
  `align_vmax`；若在区域边界前 `align_timeout`(默认 0.5 s) 内仍无回执且策略为
  `hold` → 停在区域外。
- **退出**：离开 + 滞回后，导航侧先 `angular.z` 归零、请求 `mode=0`（或
  `mode=SPIN_SLOW/FAST` 若决策层有自转请求），确认后才恢复自转。
- **抢断**：决策层/遥控若要小陀螺（上游 `/decision/spin_cmd` 语义，
  `high_priority` 可撤销导航），在 `NAV_ALIGN` 区内的自转请求一律拒绝并记录；
  或按策略停到区域外再允许。

**下行协议 v2**（下位机最小改动，按上游 `ChassisCmd` 对齐）：

```c
// ROS2 -> 宿主机，帧头 0xBB / 帧尾 0x5B，frame[1] 已是长度 ⇒ 可做版本判别
struct __attribute__((packed)) RawControlPacketV2 {
    float   vx;          // 车体系，与 v1 同义
    float   vy;
    uint8_t nav_state;   // 0=停车 1=导航（与 v1 同义）
    float   omega;       // rad/s，车体系，NAV 模式下有效
    uint8_t mode;        // 0=NORMAL(ω 有效, 自转关闭) 1=SPIN_SLOW 2=SPIN_FAST
                         // 3..11 预留地形模式（与上游 terrain_traversal.yaml 同名同号）
};
```

下位机侧要求（一句话可交付）：

1. `mode==0` 时**关闭小陀螺**，按 `omega` 闭环转动（限幅 ±2 rad/s、限斜率）；
   `omega==0` 即保持当前朝向。
2. `mode∈{1,2}` 时按下位机自己的自转逻辑，**忽略 ω**（= 今天的行为，向后兼容）。
3. 超时（沿用现有 0.3 s，`ros2_comm.cpp:120`）或 `nav_state==0` → `omega=0`，
   且**回到 mode=0 保持朝向**，不得自动恢复自转。
4. 收到 `mode` 变化时回执状态位。

**上行协议 v2**（回执，必须，否则导航侧无法做"拿不到权限就停"）：

```c
struct __attribute__((packed)) RawRefereePacketV2 {   // 0xAA / 0x5A
    /* v1 六个字段不变 */
    uint8_t chassis_status;   // bit0 spin_active, bit1 omega_follow_active, bit2 mode_rejected
};
```

**为什么发 ω 而不是绝对 yaw**：导航侧的外环用的是 `/Odometry`（LIO，装在车体上，
与底盘 yaw 刚性同源），闭环误差自洽；绝对 yaw 需要下位机与 LIO 在同一角坐标系，
存在安装/零偏标定与漂移问题。等 ω 通道稳了，若要"静止时也能保持某个绝对方位"，
再加一个 `float yaw_ref` 字段作为可选增强，语义仍是"仅当 mode=NAV 且该字段有效"。

---

## 4. 改动清单（本仓）

| # | 文件 | 改动 |
|---|---|---|
| 1 | `mas2027_nav_executor/include/.../common/environment/terrain_grid.hpp` + `src/.../terrain_grid.cpp` | 新增公开查询 `std::optional<CellInfo> directionAt(const Eigen::Vector2d&)`（`{angle_rad, magnitude, label}`）；把 `permitted()` 的解码抽成共用私有函数，不改判据 |
| 2 | 新增 `common/environment/traversal_director.{hpp,cpp}` | L2 全部逻辑：前瞻采样、命中判定、up/down、限斜率、滞回、策略表 |
| 3 | `config/terrain_traversal.yaml`（新） | L1 策略与参数 |
| 4 | `path_executor/state/execution_state.hpp` | `ExecutorInput` 加 `YawDirective`；`ExecutorOutput` 加 `bool angular_valid{false}; uint8_t mode{0};` |
| 5 | `path_executor/path_executor.cpp` | 权限分路、`applyYawDirective()`、ω 限幅与阻尼、`spin_speed` 语义修正 |
| 6 | `nav_executor_node.cpp` | `control_tick()` 里构造 directive；发布 debug 话题（`/nav_executor/debug/yaw_directive`，含 `authority/yaw_ref/label`）与 RViz 箭头；读策略 YAML |
| 7 | `third_party/interfaces/msg/navigation/ChassisCmd.msg`（新） | `std_msgs/Header header; float32 velocity; float32 omega; uint8 mode; uint8 step_dist;`（GLOB 自动纳入，与上游同名） |
| 8 | `nav_executor_node.cpp` + `ros2_comm/src/ros2_comm.cpp` | 新增 `/nav_executor/chassis_cmd` 发布/订阅（`/cmd_vel` 保留给测试与向后兼容）；ros2_comm 填 v2 包并解析上行回执 |
| 9 | `test/test_traversal_director.cpp`（新） | 合成 cost+direction 图（一条 30° 的坡带）→ 断言提前量、符号、限幅、滞回 |
| 10 | `test/bench_closed_loop.py` / `smoke_goal_motion.py` | 加"坡带"场景，断言 `/cmd_vel.angular.z` 时序与 mode 序列 |
| 11 | `README.md` + `docs/change-history.md` | 话题表、协议口径、本次变更记录 |

第 7、8 项在下位机改好前可以先合入（发布出去没人消费，但能观测、能回放）。

---

## 5. 分期落地与验证

**P0（只改导航侧，今天就能做）**
1. 标注：在 `mapping_web_ui` 里给现场坡道画 SLOPE 格 + 方向角，生成新的 terrain msgpack。
2. 实现 #1~#6、#9：`/cmd_vel.angular.z` 在区域内出现、区域外为 0；debug 话题给出
   `authority/yaw_ref`。
3. 验证：单测（#9）+ 离线回放（#10，无底盘，`use_ros2_comm:=False`）+
   **架空车轮**看 ω 方向与提前量。
   此时仍**不能**改变实车行为——自转照旧，这是预期。

**P1（下位机给通道）**
4. 协议 v2 下行 + 上行回执；`mode` 与 `omega` 真正到达底盘。先只发 `mode=0/ω=0`
   验证"关自转"这一半，再验证 ω 跟踪。
5. 验证：架空 → 低速平地（人为画一条假坡带，验证交接与退出）→ 真坡道，
   策略先用 `on_unavailable: hold`，确认能停在坡前。

**P2（完善）**
6. 决策层自转请求接入（`SpinCmd` 语义 + `high_priority` 抢断/拒绝）、地形模式 3..11
   的台阶/飞坡速度窗、与原 `minco_planner.cpp:1267` 坡道块的合并（改为按前瞻地形触发，
   而不是按当前 pitch）。

---

## 6. 取舍与备选

- **若下位机完全不能改**：唯一可做的是"看到定向区就停"（P0 的 `hold` 策略，
  靠发零速停在坡前），因为 yaw 执行器不在导航侧。**"正对着坡上去"无法在不改下位机
  的前提下实现**——这不是软件技巧问题，是权限问题。要把这句话明确同步给下位机那边。
- **不改协议、只蹭 `nav_state` 空闲值**（`nav_state=2` 表示"关自转+保持当前朝向"）：
  线格式不用动，但底盘只能"锁住进区那一刻的朝向"，导航侧无法把它转到坡轴向；
  且 `nav_state` 是 uint8 已被语义占用，扩展性差。只建议作为**临时验证关自转**的手段。
- **绝对 yaw 设定点**：见 L4 末尾，作为后续增强而非首选。
- **软残差 vs 参考覆盖**：上游选软残差（`follow_problem.cpp:134-139`），因为它同时
  服务台阶/飞坡的姿态与速度窗；本仓 MPC 结构更简单，覆盖式更直接。若以后要移植
  上游的整套 `traversal`，L2 的输出（label/方向/up-down/run_up）可直接复用，
  只需换 L3 的融合方式。

## 7. 风险与注意

1. **ω 限幅是 ±2 rad/s**（`mpc_params.yaml:19`）：区域内的 `omega_cap` 必须 ≤ 该值，
   否则覆盖值会被外层 clamp 吃掉，表现为"转不到位"。
2. **MPC 线性化**：参考 yaw 每周期跳太多会让 LTV 线性化失真（`mpc_solver.cpp:70、89`），
   所以 L2 必须限斜率（≤3~5°/周期）。
3. **方向层晕圈**：`magnitude != 255` 的膨胀格角度是插值来的，只用于软判据；
   取区域方向时只认 `magnitude==255`，否则会在边界读到不可信的角。
4. **地图与 odom 对齐**：区域查询必须在 `map` 系做（同 `command_safety.cpp:44-58`）；
   若 `map→odom` 缺失/滞后，directive 应保守 `authority=CHASSIS_SPIN` 并告警，
   不要拿错位的区域去锁 yaw。
5. **交接抖动**：区域边界 + 权限来回切换会表现为"转到一半又放开"，滞回参数要实车调；
   建议同时记录每次交接的时间戳与原因，便于回看。
6. **斜切路径**：方向门只保证 ±31.8°，覆盖式 yaw 覆盖可能与路径切向有几十度差；
   `align_vmax` 要足够低，必要时先原地摆正再进区域（这也是"停在坡前摆正"的由来）。
7. **原有 `/cmd_spin` 语义要改口径**：现在是"数值叠加到 ω"，上游是"模式请求"。
   改口径时同步改 `node_params.yaml` 注释与 README，避免下一个人又按数值接。
