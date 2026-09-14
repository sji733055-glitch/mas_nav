# 修改历史

本文件记录由开发任务产生的代码、配置、脚本、资源和文档变更。新记录追加在最上方，不改写旧记录。

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
