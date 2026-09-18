结论：**会出现你描述的“全局路径已经规划出来，但路径与 ROGMap 动态占据冲突后，不发布可执行轨迹、底盘保持不动”的问题。**

不过需要区分两件事：

- 代码里没有一条简单的逻辑是“检测到全局路径上有 ROGMap occupied，就直接禁止调用 MINCO”。
- 实际情况更接近：**全局规划器看不到 ROGMap 动态障碍，给出一条穿过障碍的路径；局部阶段又没有真正绕障搜索能力，只能让 MINCO 从碰撞种子上尝试优化；优化失败或硬碰撞校验失败后轨迹不发布，而状态机持续拿同一条全局路径重试，所以 `/cmd_vel` 一直为零。**

这条故障链在当前代码中是成立的。

## 1. 全局规划与局部优化使用了不同的动态障碍来源

全局 SMAC/A* 目前运行在：

```text
静态 terrain map + /dynamic_cost_map
```

相关代码在 [global_path_searcher.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/path_planner/search/global_path_searcher.cpp:267)。这里创建的是 `TerrainMapQuery`：

```cpp
auto terrain_query =
  std::make_shared<mas2027_nav_executor::TerrainMapQuery>(terrain_);
```

随后全局搜索完全基于该 query：

```cpp
makePlanOnQuery(..., terrain_query, ...);
```

但当前 launch 明确设置了：

```python
"bypass_dynamic_obstacle": True,
```

见 [nav_executor_launch.py](/home/mas/mas_nav_2027_native/src/mas2027_nav_bringup/launch/nav_executor_launch.py:112)。

这意味着：

- `map_server` 不进行动态障碍检测；
- `/dynamic_cost_map` 基本保持全空；
- 全局搜索只能看到静态地形；
- 进程内 ROGMap 看到的实时占据并没有进入全局拓扑搜索。

所以全局路径穿过 `/rog_map/occupied` 是当前架构下的预期结果，不是 RViz 显示误差。

README 中写着“ROGMap 也检查全局搜索落在其局部滑窗内的部分”，见 [README.md](/home/mas/mas_nav_2027_native/src/README.md:30)，但代码实际只是在局部路径稀疏化阶段做线段检查，并没有让全局搜索根据 ROGMap 重新绕路。

## 2. 局部路径阶段只检查线段，不能围绕动态障碍搜索绕行路径

局部路径生成位于 [local_path_processor.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/path_planner/trajectory/local_path_processor.cpp:52)。

它先从全局路径截取一段，然后调用：

```cpp
utils::getSparseWaypoints(...)
```

其中的碰撞条件是：

```cpp
isLineFree(mode_context.sparsifyQuery(), a, b)
```

这里的 `sparsifyQuery()` 就是 ROGMap。

但 `isLineFree()` 仅沿线采样并检查：

```cpp
map->isFree(mx, my)
```

也就是说，它只有两个能力：

- 如果两个全局路径点之间可以直连，就删掉中间点；
- 如果不可以直连，就保留更多原始全局路径点。

它没有能力：

- 在障碍左侧或右侧搜索；
- 找到一个新的绕障拓扑；
- 将全局路径改道；
- 选择障碍后的重新接入点。

更严重的是，`getSparseWaypoints()` 最后会无条件把局部终点补回去：

```cpp
// Ensure goal is included
if (...) {
  sparse.push_back(goal);
}
```

见 [minco_utils.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/vendor/minco/src/minco_utils.cpp:440)。

所以即使最后一段与 ROGMap occupied 相交，也可能得到：

```text
seed.valid == true
```

然后把一个仍然穿过障碍的种子交给 MINCO。

此外，这里的 `isFree()` 只拒绝占据格，却没有检查机器人需要的 `0.30 m` 净空。后续轨迹校验要求：

```cpp
esdf_dist > check_dist
```

见 [trajectory_safety_checker.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/path_planner/trajectory/trajectory_safety_checker.cpp:41)。

因此还有一处判据不一致：

```text
局部种子：只要没落入 occupied 格就认为可用
轨迹发布：距离 occupied 必须大于 collision_dist，目前是 0.30 m
```

这会导致全局路径即使没有直接压在 occupied 上，只要贴得太近，MINCO 结果仍可能被拒绝。

## 3. MINCO 的障碍项是软约束，不能代替局部绕障搜索

MINCO 确实会查询 ROGMap ESDF，并施加位置罚项，见 [minco_optimizer.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/path_planner/trajectory/minco_optimizer.cpp:313)。

但这个障碍约束是软代价：

```cpp
tmp_cost += weightPos * violaPosPena;
```

它不是一个负责改变路径拓扑的搜索器。

如果初始路径直接穿过动态障碍，MINCO 是否能把轨迹推到障碍旁边，取决于：

- 障碍形状；
- 初始点的位置；
- ESDF 梯度方向；
- 障碍两边是否有空间；
- waypoint 吸引项；
- 优化是否陷入局部极值；
- 多项式自由度是否足够。

对于需要明确“从左侧或者右侧绕过去”的情况，不能指望连续优化器稳定完成离散拓扑选择。

优化完成后，还有硬碰撞校验：

```cpp
const bool validation_ok =
  validateTrajectory(opt_traj, end_state.col(0));
```

见 [minco_planner.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/path_planner/trajectory/minco_planner.cpp:1207)。

只要轨迹经过 ROGMap 的 lethal/inscribed 格，或者 ESDF 净空小于 `collision_dist`，轨迹就会被拒绝，不会发布到 `/opt_path`。

因此典型流程是：

```text
全局路径穿过 ROGMap occupied
        ↓
局部稀疏化无法生成绕障路径
        ↓
MINCO 从碰撞种子开始优化
        ↓
优化失败，或者结果仍然碰撞
        ↓
validateTrajectory() 返回 false
        ↓
不发布 /opt_path
        ↓
TaskManager 不允许底盘运动
        ↓
/cmd_vel 为零
```

## 4. “没有执行轨迹优化”可能只是日志表现

MINCO 的实际调用在 [minco_planner.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/path_planner/trajectory/minco_planner.cpp:1156)：

```cpp
double final_cost =
  minco_optimizer_->optimize(...);
```

但 `"Minco optimization time"` 只在 `final_cost` 为有限值时才打印。

如果优化器返回 `INFINITY`，代码直接走 `OPTIMIZER_FAILED` 分支，不打印那条耗时信息。因此现场可能看起来像“没有进入轨迹优化”，但实际已经调用过优化器。

真正会在优化前退出的情况主要是：

- 局部路径被 ROGMap 边界裁剪后不足两个点；
- `buildSeed()` 返回 invalid；
- 地形图或 TF 不可用；
- 局部种子生成失败。

动态障碍本身通常不会稳定地让 `seed.valid` 变为 false，因为 `getSparseWaypoints()` 最后还会强行补终点。

所以更准确的判断方法是：

- 有 `Minco optimization time`，随后出现 `COLLISION`：优化器运行了，但硬校验拒绝了轨迹。
- 没有 `Minco optimization time`，出现 `OPTIMIZER_FAILED`：优化器被调用但没有产生有限解。
- 没有 `Minco optimization time`，出现 `COLLISION`：大概率在 `buildSeed()` 阶段提前返回。
- 只有 `MINCO path generation failed; retrying`，且全局路径一直不变：进入了下面的缓存死循环。

## 5. 当前状态机会反复使用同一条被阻塞的全局路径

这是导致问题持续存在的核心。

`TaskManager` 在规划状态下只在“没有全局路径”时重新运行全局搜索：

```cpp
if (!planner_->hasGlobalPath() &&
    !planner_->PlanGlobalPath(...)) {
  ...
}
```

见 [task_manager.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/task_manager/task_manager.cpp:150)。

局部规划失败后：

```cpp
if (!planner_->ReplanLocal(pose)) {
  RCLCPP_WARN(..., "MINCO path generation failed; retrying");
  handleFailure(...);
  return;
}
```

这里没有：

```cpp
planner_->invalidateGlobalPath();
```

所以状态机会每 0.5 秒执行一次：

```text
使用同一条穿过 ROGMap occupied 的全局路径
→ 局部优化失败
→ 保留这条全局路径
→ 0.5 秒后再次用相同路径优化
```

如果动态障碍因为衰减被清除，系统可能随后恢复；但只要障碍持续被点云观测到，或者属于持续误检，这个循环就可以一直持续。

即使简单地在局部失败后调用 `invalidateGlobalPath()`，目前也不能彻底解决，因为重新运行的全局搜索仍然看不到 ROGMap，仍可能生成同一条路径。

## 6. 为什么底盘会完全不动

新目标提交时，`allows_motion_` 被置为 false。只有全局规划和 `ReplanLocal()` 都成功后，状态才会切换到 `FOLLOWING` 并重新允许运动：

```cpp
state_ = State::FOLLOWING;
...
activateCurrentGoal();
```

见 [task_manager.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/task_manager/task_manager.cpp:158)。

执行侧还要求轨迹时间戳被 TaskManager 接受：

```cpp
input.allow_motion =
  input.trajectory &&
  path_planner_->acceptsTrajectory(...);
```

见 [nav_executor_node.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/nav_executor_node.cpp:424)。

因此只要局部轨迹没有成功发布：

- `allows_motion_` 仍然是 false；
- 旧轨迹也不能被新目标重新使用；
- `PathExecutor` 返回 `WAITING_INPUT`；
- 发布的 `/cmd_vel` 是零。

这与现场“全局线能看到，但底盘不走”完全吻合。

## 7. 还有一个会放大急停问题的状态变量错误

候选轨迹校验失败时，代码会执行：

```cpp
is_traj_safe_.store(false);
```

见 [minco_planner.cpp](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/src/path_planner/trajectory/minco_planner.cpp:1216)。

但这里失败的是“新候选轨迹”，`is_traj_safe_` 表示的却是“当前已提交轨迹是否安全”。

这两个状态不应该共用一个变量。

在 FOLLOWING 状态下，可能出现：

```text
旧的已提交轨迹仍然安全、仍未到期
        ↓
周期性局部重规划生成了一个失败的候选轨迹
        ↓
候选失败把 is_traj_safe_ 置为 false
        ↓
TaskManager 下一周期认为已提交轨迹不安全
        ↓
急停、清除全局路径、重新规划
```

20 Hz 的异步安全定时器可能随后重新检查旧轨迹并把标志恢复为 true，但 TaskManager 本身也是 20 Hz，二者存在竞争窗口，所以这种错误赋值会放大“走一下就急停”或者“候选失败后立刻停住”的现象。

## 推荐解决方案

我建议分两层处理，不能只调参数。

### 第一优先级：在 ROGMap 滑窗内增加真正的局部绕障搜索

ROGMap 是 10 m × 10 m 的滚动局部地图，更合理的职责是局部动态绕障，而不是把整个全局地图都塞进 ROGMap。

推荐流程：

```text
全局 SMAC 路径
      ↓
截取 ROGMap 滑窗内的局部段
      ↓
按 collision_dist 膨胀后的 ROGMap 检查局部段
      ↓
无碰撞：直接送 MINCO
有碰撞：在 ROGMap 上做局部 A*/Theta* 搜索
      ↓
从机器人当前位置绕过障碍并接回全局路径
      ↓
将绕障结果作为 MINCO 种子
      ↓
MINCO 只负责平滑、速度和加速度优化
```

实现上可以复用现有 A*，但需要：

1. 起点使用机器人当前位置；
2. 终点选择障碍后方、仍在 ROGMap 滑窗内的全局路径重接点；
3. 搜索栅格必须按 `collision_dist` 膨胀，不能只检查 `isFree()`；
4. 找到局部绕障路径后，与全局路径剩余部分拼接；
5. MINCO 接收的是无碰撞拓扑路径，而不是穿障路径。

这比单纯提高 MINCO 的 `penalty_weight_pos` 稳定得多。连续优化器适合平滑路径，不适合独立决定从障碍哪侧绕行。

### 第二优先级：把 ROGMap 动态占据融合进全局规划查询

如果希望全局路径在 RViz 中本身就绕开当前动态障碍，可以构建一个复合查询：

```text
MergedPlanningQuery =
  TerrainMapQuery
  + 当前 ROGMap 滑窗 occupied
  + collision_dist 膨胀
```

注意 ROGMap 是 `odom` 坐标系，地形图是 `map` 坐标系，所以每次规划必须：

1. 获取同一时刻的 `map ↔ odom` TF；
2. 将 ROGMap 滑窗内的 occupied 投影到 terrain 栅格；
3. 只覆盖 ROGMap 当前有效窗口；
4. 将机器人安全半径纳入占据膨胀；
5. 对融合后的二值图重新计算距离场，供 SMAC ESDF 软代价使用；
6. 让 SMAC 的障碍判定和 ESDF 距离使用同一份融合快照。

不能只修改 `SMAC::setESDFQuery()` 而仍让搜索栅格来自 terrain，因为那样只会让路径“倾向远离”动态障碍，却仍可能把动态障碍格当作可穿越格。

一个快速但不够理想的验证方式，是把：

```python
bypass_dynamic_obstacle = False
```

临时打开，让 `/dynamic_cost_map` 进入全局搜索。如果问题明显消失，就可以进一步确认是动态地图源不一致导致的。

但仓库注释已经记录了这一层有约 0.4 m 膨胀和误制动问题，因此它更适合作为诊断手段，不建议直接作为最终方案。最终最好只保留一个动态障碍真源，优先复用已经参与 MINCO 和 MPC 安全检查的 ROGMap。

### 第三优先级：局部无绕行解时发布“安全前缀停车轨迹”

如果障碍在局部路径较远处、暂时没有可绕路线，当前代码直接拒绝整条局部轨迹，因此即使机器人前方还有数米自由空间，它也完全不动。

可以增加安全前缀策略：

1. 找到局部路径上第一个不满足 ROGMap 净空的位置；
2. 向前回退：

```text
collision_dist
+ 当前速度对应的制动距离
+ 额外保护余量
```

3. 将该位置设为局部临时终点；
4. 终点速度和加速度设为零；
5. 如果从当前位置到临时终点仍有足够距离，则发布一条“接近障碍并安全停车”的轨迹；
6. 如果障碍已经太近，则保持急停并等待地图清除或绕行路径出现。

这样可以实现：

- 动态障碍很远：底盘向前行驶，并停在安全距离外；
- 动态障碍移开：重新规划后继续；
- 动态障碍完全封死窄道：最终安全停车；
- 障碍就在车前：不错误移动。

这可以解决“路径上只要出现一个动态占据，整车从起点就不动”的体验问题，但必须严格计算制动距离，不能简单截掉 occupied 后面的点。

### 第四优先级：修正状态机，不要无限重试同一条失败路径

建议把 `ReplanLocal()` 的 bool 返回值改成带原因的枚举，例如：

```cpp
enum class LocalPlanResult {
  SUCCESS,
  ROG_PATH_BLOCKED,
  NO_LOCAL_DETOUR,
  OPTIMIZER_FAILED,
  CANDIDATE_COLLISION,
  TERRAIN_UNAVAILABLE,
  TF_UNAVAILABLE
};
```

状态机可以据此采取不同动作：

- `ROG_PATH_BLOCKED`：触发局部绕障，必要时刷新融合全局路径；
- `NO_LOCAL_DETOUR`：发布安全前缀停车轨迹或原地等待；
- `OPTIMIZER_FAILED`：有限次数重试，必要时降低热启动依赖；
- `TERRAIN_UNAVAILABLE` / `TF_UNAVAILABLE`：保持停止，不盲目重规划；
- `CANDIDATE_COLLISION`：不得污染当前已提交轨迹的安全状态。

同时应给全局路径加一个动态地图 revision：

```text
global_path_generation
rog_map_revision_used_for_plan
```

当 ROGMap 的障碍状态发生有意义变化，或者同一路径连续局部失败超过阈值时，将全局路径标记为 stale，而不是永远认为 `hasGlobalPath() == true` 就不重算。

### 第五优先级：分离候选轨迹状态和已提交轨迹状态

应该改成类似：

```text
candidate_validation_ok
committed_trajectory_safe
```

规则是：

- 候选轨迹验证失败：丢弃候选，但不要修改 `committed_trajectory_safe`；
- 当前已提交轨迹的安全状态：只能由运行时安全监视器更新；
- 新候选验证并发布成功：原子地替换 committed trajectory，然后把 committed 状态设为 true；
- 若旧轨迹尚未过期且仍安全，候选失败时继续执行旧轨迹；
- 只有已提交轨迹本身被 ROGMap 判定不安全，才触发急停。

这能消除“新候选失败导致旧安全轨迹也被判死”的竞争问题。

### 第六优先级：修复稀疏路径的假安全行为

`getSparseWaypoints()` 不应该在最后无条件追加终点。至少应做到：

```text
当前安全点 → 终点线段安全：追加终点
当前安全点 → 终点不安全：返回 blocked，并报告第一个冲突段
```

另外，线段安全判据应从：

```cpp
map->isFree(mx, my)
```

改为与轨迹发布门一致的净空判据：

```cpp
query.ok &&
query.distance > collision_dist
```

或者更高效地在规划用栅格上提前做 `collision_dist` 膨胀，然后搜索和稀疏化统一使用膨胀后的栅格。

## 建议的最终行为

修复后，应根据场景得到以下结果：

| 场景 | 期望行为 |
|---|---|
| ROGMap 障碍旁边存在绕行空间 | 局部 A* 绕障，再由 MINCO 平滑并执行 |
| 障碍封死整个走廊 | 原地等待或行驶到安全停车点，不发布碰撞轨迹 |
| 临时障碍随后移开 | ROGMap 清除后自动重新规划并继续 |
| 新候选轨迹失败，但旧轨迹仍安全 | 继续执行旧轨迹，不急停 |
| 障碍突然进入当前已提交轨迹 | 立即制动，同时重新搜索 |
| ROGMap 把车体/雷达盲区误判成 occupied | 保持安全停止，并通过车体过滤、外参和投影参数修正，不能绕过安全检查 |

## 建议增加的测试

目前 `mas2027_nav_executor` 的 8 个现有测试全部通过；我执行了：

```text
ctest --test-dir build/mas2027_nav_executor --output-on-failure
```

结果为：

```text
100% tests passed, 0 tests failed out of 8
```

但现有测试列表里没有 `TaskManager + LocalPathProcessor + ROGMap 动态障碍` 的端到端测试，见 [CMakeLists.txt](/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/CMakeLists.txt:112)。因此当前测试通过并不能覆盖这个问题。

建议至少新增：

1. 静态地形全自由、ROGMap 直线路径上有障碍、侧面有通道：必须生成绕障轨迹。
2. ROGMap 障碍完全封死：不得发布碰撞轨迹，底盘安全停止。
3. 动态障碍在 FOLLOWING 状态突然出现：必须制动和重规划。
4. 动态障碍清除：必须从等待状态恢复。
5. 新候选轨迹碰撞、旧轨迹仍安全：旧轨迹继续执行。
6. 全局路径只离障碍 0.2 m、没有直接压 occupied：局部种子必须按 `collision_dist=0.30` 判为不可用。
7. `map ↔ odom` 存在平移和旋转时，融合后的障碍位置仍正确。
8. 局部稀疏化不能无条件追加一个碰撞终点。

综合来看，最值得先做的不是调低 `collision_dist` 或关闭 ROGMap 检查，而是：

```text
ROGMap 局部 A* 绕障
+ 安全前缀停车轨迹
+ 局部失败不再无限复用同一条路径
+ 候选轨迹与已提交轨迹安全状态分离
```

这四项一起才能既避免“有 occupied 就完全不走”，又不牺牲动态避障安全性。本次只做了代码审查和现有测试验证，没有修改仓库文件。