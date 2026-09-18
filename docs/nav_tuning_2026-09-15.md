# 导航调参与修复全过程（2026-09-15）

> 记录从「车根本不动 / 一卡一卡」到「行驶流畅」，以及从「只能近处规划」到「可规划远处目标」的
> 完整因果链与改动清单。所有结论均来自当日实车日志与 `mas_nav_2027_native` 源码核对。
> 逐条改动的时间顺序与验证边界见 `change-history.md`，本文按**因果**重组。

## 0. 两个问题的最终状态

| 问题 | 起点 | 终点 |
|---|---|---|
| 行驶 | 发目标点后车不动；后来能走但一卡一卡 | 正常行驶，窄道不再卡顿 |
| 规划范围 | 只能规划车附近的目标；远目标全局搜索失败 | 全局搜索建在整张地形图上（770×347），远处目标可规划 |

---

## 一、从「卡顿 / 不动」到「流畅」

### 1.1 车完全不动：近距盲区把车体自己判成了障碍

**现象**：日志反复出现 `Trajectory clearance ≈0.28 m below required 0.30 m at (-0.01, -0.01)`、
`MINCO trajectory not published: COLLISION`，失败点几乎贴在车体原点；后期更出现**负净空**
（`start clearance -0.073 m`），即二维距离场认为车体自身在障碍物内部。

**根因（三处代码叠加成死锁）**：

1. `prob_map.cpp` 的射线推进从 `raycast_range_min` 之外才开始：
   `raycast_start = (p - cur_odom).normalized() * cfg_.raycast_range_min + cur_odom`；
2. 同文件把 `raycast_range_min` 以内的点直接 `continue` 跳过（为挡车身点云）；
3. `projection_layer.cpp::applyValueAndMask()` 在 `unknown_as_occupied: true` 时把 UNKNOWN 列写成
   `mask=0`（二维 ESDF 的障碍源）与 `value=254`（`kLethalCost`）。

⇒ **传感器 0.3 m 内的体素永远不会被标成空闲**，只能停在 UNKNOWN → 被判成障碍 → 车体所在的一圈
成了幻影障碍。而唯一清理它的代码块被 `static bool first` 限制为**只跑首帧**，车一移动/旋转就重新
落进未观测区。

**改动**：
- `prob_map.cpp`：首帧一次性清理改为「**传感器每移动超过半个体素就重清一次**」，半径仍为
  `raycast_range_min`；
- `prob_map.h`：新增 `last_near_field_clear_pos_` / `near_field_cleared_`；
- `prob_map.cpp::resetLocalMap()`：复位该状态；
- 新增 `insideLocalMap()` 过滤——`SlidingMap::getLocalIndexHash()` **不做边界检查**，反复清理后
  越界写入会踩内存（原实现靠开机时车在图中心侥幸规避）。

**验证**：负净空出现次数 **25 → 0**；`COLLISION` 60 次 → 12 次。

> 旁注：`raycasting.ray_range` 下限由 0.01 改回 0.3 是更早的一次修复，它消除了车身点云写入，
> 但同时把盲区从 0.01 m 撑到 0.3 m——**这就是上述死锁的直接来源**。两者必须一起看。

### 1.2 走走停停 / 一卡一卡：净空要求随速度增长形成的极限环

**现象**：能到目标，但一路走走停停；日志里失败样本**全是毫米级**（`0.414 vs 0.425`、
`0.395 vs 0.403`、`0.414 vs 0.414`），且失败点与起点净空落在同一区间（0.30~0.42）、与位置无关。

**根因**：检查器的净空要求随速度增长

```
required = collision_dist + max(v * replan_react_time, monitor_margin)
```

等价于速度上限 `v < (可用净空 − collision_dist) / replan_react_time`。规划速度略超该上限，轨迹就在
最窄处差几毫米被否 → **加速 → 否决 → 急停 → 再加速**。

**决定性对照**：上游 `/home/mas/nav_opensource/navi_minco_bit`（docker 内）源码中检索
`replan_react_time`、`monitor_margin`、`requiredClearance` **全部不存在**——它的碰撞校验
（`validateTrajectory` / `checkCollision`）用的是**固定阈值** `collision_dist`。**这是"上游丝滑、
本工程一卡一卡"的结构性根因。**

**改动**：`planner_params.yaml` 的 `minco_optimizer.replan_react_time` 由 0.35 → 0.15 → **0.0**，
`required` 退化为固定的 `collision_dist`(0.30)，与上游一致。

### 1.3 低速顿挫：线速度死区

`path_executor.cpp` 中线速度低于 `deadzone_speed_threshold` 即被清零（角速度仍发出）。
0.1 m/s 会让低速段「给一点速度就被清掉」。

**改动**：`mpc_params.yaml` 的 `deadzone_speed_threshold` 0.1 → **0.02**。

### 1.4 未观测区域被当成障碍：净空读数普遍偏小

**改动**（对齐上游/旧工程）：
- `planner.rog_map.projection.unknown_as_occupied`：`true` → **`false`**
  （同时决定二维 ESDF 的 mask 与 value：254=`kLethalCost` 还是 255=no-information）；
- `planner.exploration.unknown_as_occupied`：`true` → **`false`**（经 `planner_mode_context` 控制
  全局搜索的 `smacTraversableCost()` 与目标可通行判定，**必须与上一项一致**）。

### 1.5 投影窗口探到了"地面以下"：每列凭空多出虚假占据

`prob_map.cpp::getGridType()` 把 `z <= virtual_ground_height` 的体素一律判为 OCCUPIED。而
`projection.scan_z_min_abs` 曾被改为 −1.2，低于当时的 `virtual_ground_height`(−1.0)，
**每个柱子凭空多出 4 个体素（0.2/0.05）的虚假占据**，污染 `occupied_count` / 高度差 / 占据率。

**改动**：`virtual_ground_height` −1.0 → **−1.5**（旧工程取值，使 −1.2 的窗口稳稳落在其上）。

### 1.6 动态障碍层：多出来的一层刹车

旧工程 `/home/mas/mas_nav_2027/mas2027_perception/` 下**没有 `map_server`，没有动态障碍层**；
本工程多出「点云对比全局静态云 → 检测 → 按 `full_cost 0.2 / cutoff 0.4 m` 膨胀 → 执行器据此刹车」，
RViz 里明显比实物厚，并触发 `Braking: current dynamic obstacle intersects the MPC reference horizon`。

**改动**：launch 中 `terrain_map_server` 的 `bypass_dynamic_obstacle` `False` → **`True`**
（该开关为真时不订阅点云，`/dynamic_cost_map` 保持全空，`/cost_map` 与 `/direction_map` 照常发布）。
动态物体改由 ROGMap 的时间衰减（`keep_time 0.8 s` / `clear_time 1.2 s`）承担，即旧工程做法。

### 1.7 两次失败尝试（保留记录，避免重踩）

| 尝试 | 结果 | 原因 |
|---|---|---|
| **速度感知净空**（优化器位置罚项改用 `required(v)`，代码保留、开关默认关） | **全程变慢**，已回退 | ① 目标净空在 v > 0.33 m/s 后一律高于原固定值；② `penalty_weight_pos(50000)` 远大于 `penalty_weight_time(100)`，**减速几乎不花代价**，优化器选择降速而非绕开来降低惩罚 |
| **`safe_dist` 0.40 → 0.50** | MINCO 出现 **93 次** `path generation failed`，已回退到 0.40 | 降级路径要穿过未观测区，那些地方几何紧，0.50 成了**够不到的软目标**，罚项恒不满足 → L-BFGS 不收敛 |

### 1.8 仍存在但未再处理的一处不一致

`minco_planner.cpp` 中**运行时监视**用 `requiredClearance(v) - kMonitorClearanceTolerance`，
而**发布前校验**用不减容差的严格值——尽管其上方注释写着「阈值必须与运行时监视一致」。
现场表现为净空停在 `0.296~0.300` 的轨迹被毫米级否决（`0.300/0.299/0.298 below required 0.300`）。
曾改为一致后该日志消失，但随后出现的「无法规划」经日志核实瓶颈在更上游（全局搜索失败），
故**按用户要求回退**。**该不一致仍然成立**，待现场数据干净后再单独评估。

---

## 二、从「只能近处规划」到「可规划远处目标」

### 2.1 `isFree` 把「无信息」当成「不可通行」

`global_path_searcher.cpp` 在 ROGMap 滑窗内逐格调用 `query->isFree(mx, my)`；
而 `QueryAdapter::isFree()` 原实现是 `values[idx] < 253U`。按 `applyValueAndMask()`，
`unknown_as_occupied: false` 时**未知列的取值是 255**（no-information），`255 < 253` 为假
⇒ **未观测区域整片被判为不可通行**，而远处目标必经的正是车还没看过的区域。
现场：目标 `(4.22, 1.59)`，**118 次** `No acceleration-feasible route ...` + 118 次
`Global path search failed; retrying`。

**改动**：`query_adapter.cpp::isFree()` 改为 `cost < 253U || cost == 255U`。
该改法在两种配置下都自洽（`unknown_as_occupied: true` 时未知列取 254，仍判不可通行）。
同时受益：`path_planner.cpp:123` 的目标准入、`local_path_processor.cpp:26` 的种子路径裁剪。

### 2.2 Kino 无解时不再直接放弃

全向 Kino A\*（`searchOmniKinoPath`）以「实测速度种子 + 速度/加速度可行性」扩展状态格点，
**可行解集依赖车当前状态**，目标较远或朝向不巧时可能无解；而失败分支只是 `return false`，
**不会走到紧随其后、本可用于兜底的 A\* 直连规划**（同文件 353-371 行）。

**改动**：`global_path_searcher.cpp` 的降级分支改为调用 `makePlanOnQuery(...)`，成功则
`latest_global_path` 已填好、直接 `return true`，失败才 `return false`。

### 2.3 `allow_unknown` 与探索口径不同源

`Astar` 只在 `allow_unknown` 为真时才接受 `cost == 255`（`astar.cpp:196,259`），该值由
`GlobalPathSearcher::configure()` 一次性写入，而调用点传的是 **`MincoPlanner::allow_unknown_`
成员**，与 `exploration_unknown_as_occupied_` 无关联 ⇒ **外层说未知可通行、内层仍把未知当障碍**。
现场端点诊断：`start cost=0(free)`、**`goal cost=255(unknown)`**、`Astar failed to find path`。

**改动**：`minco_planner.cpp` 的 `configure(...)` 第三实参改为
`!exploration_unknown_as_occupied_`，两者**同源**。

### 2.4 降级路径改用地形图查询（关键一步）

原先降级 A\* 在 **ROGMAP 查询**上做纯栅格搜索，不看 HW 地形/方向层，于是 MINCO 沿该种子路径优化出
的轨迹被 `validateTrajectory` 判 `TERRAIN_COLLISION_OR_DIRECTION`（一次运行 10 次）。

**改动**：
- `global_path_searcher.cpp`：降级分支构造 `mas2027_nav_executor::TerrainMapQuery(terrain_)` 作查询，
  **起终点改用已算好的 map 系 `start_map.pose` / `goal_map.pose`**（地形查询坐标系是 map），
  日志前缀改为 `Terrain(fallback)`；新增 `terrain_map_query.hpp` 的 include；
- `CMakeLists.txt`：把 `src/common/environment/terrain_map_query.cpp` 加入 `${PROJECT_NAME}_node`
  目标——该实现此前**只出现在 `test_terrain_map_query`** 中，不加会报
  `undefined reference to TerrainMapQuery::TerrainMapQuery / vtable`。

**效果（实测日志）**：

```
[MincoPlanner] Terrain global search input: planner=Astar frame=odom
               map=770x347 origin=(-4.600,-7.940) res=0.050
               start_cost=0(free)  goal_cost=0(free)
```

搜索图由 **200×200 的 ROGMap 滑窗**变为 **770×347 的整张地形图**（origin −4.6/−7.94 即 lab3 坐标），
起终点均为 `free`、无 `Astar failed` ⇒ **远处目标可规划**。这在功能上等价于旧工程
`mas_nav_2027` 的「Nav2 SMAC 跑在 StaticLayer 整张先验图上」。

### 2.5 先验图融合：启用后又关闭

按上游 `prior_map: {enable, yaml_path, pgm_path, frame_id}` 键名启用 lab3 先验图融合
（`prior_map.cpp` 与上游**逐字节相同**，变换方向语义也一致）。实测在 `(3.2, −0.03)` 一带
`layer_value_static` 有墙、`layer_value_dynamic` 为空，该处净空由约 0.4 m 掉到 0.272~0.285 m
（低于 `collision_dist` 0.30），**返程 26 秒中有 20 秒耗在该点**。

**改动**：`prior_map.enable` 改回 **`false`**（`yaml_path`/`pgm_path`/`frame_id` 保留，便于恢复）。
⚠️ 后续经用户确认「docker 用的也是这张图/这个 pcd」，**该错位判断本身需重新验证**——当时看到的两层
不一致，可能只是 1.8 节所述毫米级现象在 RViz 上的表现。

---

## 三、与上游 / 旧工程的对齐结果

| 项 | `mas_nav_2027`（旧）/ `navi_minco_bit`（上游） | 本工程现状 | 是否对齐 |
|---|---|---|---|
| 全局图 | 整张先验图（StaticLayer + SMAC） | 整张地形图（A\*，770×347） | ✅ 功能等价 |
| 碰撞校验阈值 | **固定** `collision_dist` | 固定（`replan_react_time = 0`） | ✅ |
| `safe_dist` / `collision_dist` | 0.40 / 0.30 | 0.40 / 0.30 | ✅ |
| `projection.unknown_as_occupied` | false | false | ✅ |
| `scan_z_min_abs` / `virtual_ground_height` | −1.2 / −1.5 | −1.2 / −1.5 | ✅ |
| `center_offset` | 关闭 | 关闭（默认） | ✅ |
| 动态障碍层 | **无** | 已关闭（`bypass_dynamic_obstacle: True`） | ✅ |
| **方向约束层** | **无** | **有**（`transition()` 逐边判方向） | ❌ **唯一实质差异** |
| `exploration.unknown_as_occupied` | true | false | ⚠️ 有意偏离，见下 |

**唯一剩下的实质差异是方向约束层**：`TerrainMapQuery` 的值是二值的（254/0）、不含方向信息，
而方向约束是逐边判定的（Kino 用 `transition(from,to)`，朴素 A\* 没有该概念）。因此降级路径仍可能
**逆向穿过方向受限格**，被否为 `TERRAIN_COLLISION_OR_DIRECTION`（现为 10 次/次运行）。
两条出路：① 让搜索**边感知方向**；② **放开方向约束**（对齐旧工程，它没有这一层）。

**有意保留的本工程取值**（均来自实车数据，不是随意偏离）：

| 参数 | 旧/上游 | 本工程 | 原因 |
|---|---|---|---|
| `replan_react_time` | 0.35 | 0.0 | 0.35 会形成「加速→净空要求反超→否决→急停」的极限环 |
| `deadzone_speed_threshold` | 0.1 | 0.02 | 0.1 导致低速段顿挫 |

---

## 四、仍未解决 / 未验证

1. **方向约束**（见第三节）——当前 `TERRAIN_COLLISION_OR_DIRECTION` 的 10 次来源；
2. **发布前校验与监视器阈值不一致**（1.8 节）——曾修后按要求回退，事实仍成立；
3. **先验图错位**（2.5 节）——结论需重新验证；对齐修复前，先验图融合与「放大 `map_size`」保持暂停；
4. **`MincoPlanner::allow_unknown_` 成员**——2.3 节改动后该成员不再影响 searcher，但其原本的赋值
   来源与其它用途未追查；
5. **`speed_aware_clearance` 代码路径**——机制保留、开关关闭；若重新启用，需**先把
   `penalty_weight_time` 提到与 `penalty_weight_pos` 同量级**，否则仍会全局拖慢；
6. **`safe_dist` 最优值**——0.40 恢复了收敛性，是否介于 0.40~0.45 更优未调参。

---

## 五、改动清单（按文件）

| 文件 | 改动 | 类型 |
|---|---|---|
| `mas2027_perception/rog_map/src/rog_map/prob_map.cpp` | 近距盲区随移动持续清空；`resetLocalMap` 复位；边界过滤 | 代码 |
| `mas2027_perception/rog_map/include/rog_map/prob_map.h` | 新增清理记账成员 | 代码 |
| `mas2027_perception/rog_map/src/rog_map/query_adapter.cpp` | `isFree` 放行 255（无信息） | 代码 |
| `mas2027_nav_executor/src/path_planner/search/global_path_searcher.cpp` | Kino 失败降级 A\*；降级改用地形图查询（map 系起终点）；include | 代码 |
| `mas2027_nav_executor/src/path_planner/trajectory/minco_planner.cpp` | `allow_unknown` 与探索口径同源；新增 `clearance_optimizer_margin` 参数与接线 | 代码 |
| `mas2027_nav_executor/include/.../minco_planner.hpp` | 新增 `speed_aware_clearance_` / `clearance_optimizer_margin_` | 代码 |
| `mas2027_nav_executor/include/.../minco_optimizer.hpp`、`src/.../minco_optimizer.cpp` | `ClearanceModel`、速度感知净空与余量（默认关） | 代码 |
| `mas2027_nav_executor/CMakeLists.txt` | `terrain_map_query.cpp` 编入节点目标 | 构建 |
| `mas2027_nav_executor/config/planner_params.yaml` | `replan_react_time 0.0`、`safe_dist 0.40`、两个 `unknown_as_occupied false`、`scan_z_min_abs −1.2`、`virtual_ground_height −1.5`、`prior_map.enable false`、`speed_aware_clearance false` | 配置 |
| `mas2027_nav_executor/config/mpc_params.yaml` | `deadzone_speed_threshold 0.02` | 配置 |
| `mas2027_nav_bringup/launch/nav_executor_launch.py` | `use_ros2_comm True`、`bypass_dynamic_obstacle True` | 配置 |

> 底盘链路（不属于规划，但同期修复）：`ros2_comm` 是 `/cmd_vel` 的**唯一消费者**，launch 中
> `use_ros2_comm` 原默认 `False`，导致「`cmd_vel` 有值但车不动」；已改为默认 `True`。
