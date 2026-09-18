#ifndef MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
#define MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_

#include "minco_core/header.hpp"
#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"

#include <string>

namespace minco_planner {

class PlannerModeContext;

/// 【排障用】种子净空判据第一次否决的那一点。`Live obstacle blocks the local route and leaves no
/// safe stopping prefix.` 这条消息以前只说"全败了"，分不清是"路被物理挡住"还是"几何路线存在、
/// 只是每段都差几毫米过不了净空判据"。带上这一组量之后，一次实车运行即可判定：
///   - clearance ∈ [collision_dist, collision_dist + slack] 且 required == collision_dist
///     → 门槛问题（种子门的放宽门槛比轨迹校验门严，见 docs/refusal_triage_2026-09-16.md §1.3）；
///   - clearance 明显低于 collision_dist 且 required 已被放宽
///     → 环境真的过不去，拒绝是正确行为。
struct SeedRejectInfo
{
  bool valid{false};           ///< 是否真的捕获到了一次否决
  Eigen::Vector3d point{};     ///< 被否的采样点（map/odom 帧，与 buildSeed 入参同帧）
  double clearance{0.0};       ///< 该点实测净空
  double required{0.0};        ///< 该点被判据要求的净空
  double arc_from_start{0.0};  ///< 该点离规划起点的平面距离（近场是按弧长划分的）
  bool near_field_relaxed{false};  ///< required 是否来自近场放宽（而非完整 collision_dist）
  bool terrain_blocked{false};     ///< 是否死在 terrain_segment_free 而不是净空
  /// 【2026-09-17】"只差净空"：该点**本身是自由格**（isFree 通过、不在 terrain 拒绝里），
  /// 只是净空小于 required。段门（segmentClear）不再因为这一类失败而否决整条种子——见
  /// 该函数头部注释：净空是**轨迹**的属性，原始折线的净空不代表 MINCO 优化后轨迹的净空。
  /// 这个标志只用于日志（把"贴墙但能过"与"真的堵死"分开）。
  bool clearance_only{false};
  /// 停车/脱困前缀专用：true 表示"第一个净空不足的采样点之前的可用安全段，减去收尾余量后
  /// 不足最小前缀长度"。此时否决的定义是"前方太短"而不是"净空不够"，`clearance` 仍会照实
  /// 记录那一点的净空（通常仍大于 required）。
  bool length_limited{false};
};

/// 自动判读"这条被否的路径到底说明了什么"。写进日志是为了不让人再手算两套判据的差异。
///
/// 【2026-09-17 之后的口径】种子门（`segmentClear`）已经**不再**用净空否决（只查占据/地形，
/// 见 `enforce_clearance` 的说明），因此 `Live obstacle blocks the local route` 这条消息现在
/// 只会出现在"路真的被堵死"时；`SEED_GATE_STRICTER` 随之变成**自检分支**——只有当某处又出现
/// "某道门拿净空去否决种子/前缀"的回归时才会被报出来。判读器保留原样，正是为了那种回归。
///
/// 返回值一览（`verdict=` 字段）：
///   - `GEOMETRY`          该点连"近场规则"都过不了 → 几何真的过不去，拒绝是正确行为，
///                         别动阈值。机器人本来就贴死时也落在这里（原因见实现处注释）。
///   - `SEED_GATE_STRICTER`该点靠近场规则能过、靠更严的那套要求过不了，且机器人起点
///                         净空 ≥ collision_dist（没有"贴死"作为放宽理由）
///                         → **判据不一致**，该改判据。
///   - `PREFIX_TOO_SHORT`  该点其实满足判据，否决来自"安全段太短、前缀凑不够最小长度"
///                         （`length_limited`）→ 不是净空问题，是前方可用距离问题。
///   - `TERRAIN`           死在 terrain/动态层，与净空无关。
///   - `NONE`              没有捕获到否决点。
std::string classifySeedReject(const SeedRejectInfo & info,
  double start_clearance, double collision_dist, double near_field_slack);

struct LocalPathSeed
{
  bool valid{false};
  bool local_end_is_goal{false};
  // 原全局路径局部段被实时 ROGMap 阻挡，已替换为滚动窗口内栅格搜索得到的绕行路径。
  bool used_dynamic_detour{false};
  // 当前 ROGMap 窗口内无路可绕，因此种子停在障碍前的安全点；
  // 即使该点不是任务终点，也必须使用零末速度。
  bool stop_at_local_end{false};
  // 【排障用】已经生成了绕行路径或安全停车前缀（即"修复成功"），但种子最终仍未通过校验。
  // 实车日志里表现为 "Repaired a ROGMap-blocked global segment..." 之后几毫秒就
  // "MINCO path generation failed"，光靠日志分不清死在哪一步；该标志让调用方把失败原因
  // 区分为"修复后被否决"，而不是笼统的 COLLISION。
  bool repair_rejected{false};
  // 正常路径、局部绕行、完整停车前缀三层都失败后，退化使用的短距离脱困前缀（creep）。
  // 它整段落在近场豁免半径（collision_dist）以内、末速度为零，只用于把车从"贴住障碍停死"
  // 的状态里挪出来；实车表现是原地不动，见 docs/change-history.md 2026-09-16 13:40 那次运行。
  bool used_escape_prefix{false};
  // 【排障用】稠密种子判据首次否决的那一点。四层兜底全败（used_* 全为假、dense_path 为空）
  // 时它说明"为什么全败"；【2026-09-17】种子门否决后又被绕行/前缀修复成功时同样会记录，
  // 用于回答"这次为什么走了兜底"（配合 strict_seed_after_failures 的切换日志一起读）。
  SeedRejectInfo dense_reject;
  std::vector<Eigen::Vector3d> dense_path;
  std::vector<Eigen::Vector3d> sparse_waypoints;
  std::vector<double> local_magnitudes;
};

class LocalPathProcessor
{
public:
  void configure(double lookahead_dist,
    double max_vel,
    double max_acc,
    double traj_goal_tolerance,
    double collision_dist,
    rclcpp::Logger logger);

  void updateLimits(double max_vel, double max_acc, double traj_goal_tolerance);

  /// 短距离脱困前缀（第四层兜底）参数：enable=false 即完全恢复"三层兜底"行为。
  /// min_length/buffer 是障碍前至少保留的净空与收尾余量，长度上限固定为 collision_dist
  /// （即整段都落在近场豁免半径内，不引入任何新的安全阈值）。
  void setEscapeOptions(bool enable, double min_length, double buffer);

  /// 折线净空不足**不否决**种子（净空是轨迹的属性，交给 MINCO 与轨迹级三道门），
  /// 只有占据 / 地形 / 查询失效才硬否决；净空不足会记进 `seed.dense_reject.clearance_only`
  /// 供日志与判读器使用。2026-09-17 曾有一个"连续失败后把本函数切回净空硬否决"的调用侧
  /// 开关（`enforce_seed_clearance`），实车证明它会在近场把种子永久拒掉，2026-09-18 已删除。
  LocalPathSeed buildSeed(const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const PlannerModeContext & mode_context,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free = {}) const;

private:
  /// 种子 / 局部绕行 / 停车前缀三类判据共用的「完整净空要求」。
  /// 取 collision_dist_ 减 ESDF 抖动余量（clearance_gate.hpp 的 effectiveClearanceThreshold），
  /// 与发布前校验、20 Hz 监视、MPC 指令门**必须是同一个有效阈值**——四道门不一致时，
  /// 规划放行、执行刹车（2026-09-17 实车 7 次否决全部落在 0.26~0.28 这条缝里）。
  /// 近场半径仍用 collision_dist_（车体安全半径），与本值无关。
  double fullClearanceRequirement() const
  {
    return mas2027_nav_executor::effectiveClearanceThreshold(collision_dist_);
  }

  std::vector<Eigen::Vector3d> extractLocalPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const Eigen::Vector3d & cur_pos) const;

  bool clipLocalPathByRogBoundary(
    std::vector<Eigen::Vector3d> & path, const PlannerModeContext & mode_context) const;

  /// 最后一个参数是【排障用】可选输出：传入非空指针时，在返回 false 之前填入"第一次否决"
  /// 的那一点（净空 / 要求 / 是否近场放宽 / 是否死在 terrain）。nullptr 表示不需要，零开销。
  ///
  /// 【2026-09-17】`enforce_clearance` 决定净空不足算不算否决：
  ///   - false（种子/绕行/稀疏复核用）：只有占据、地形、查询无效才算否决。净空是**轨迹**的
  ///     属性，交给 MINCO 与轨迹级三道门；这也是旧工程 mas_nav_2027 `isLineFree` 的口径。
  ///   - true（停车前缀用）：净空不足即否决。前缀末端是"车要停在哪里"，必须本身满足
  ///     净空，否则连停车轨迹都会被发布前校验拒掉，车反而失去可执行指令。
  bool segmentClear(const std::shared_ptr<rog_map::MapQueryInterface> & query,
    const Eigen::Vector3d & from,
    const Eigen::Vector3d & to,
    const Eigen::Vector3d & planning_start,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    SeedRejectInfo * reject_info = nullptr,
    bool enforce_clearance = false) const;

  /// 始终按"软"口径逐段检查（`segmentClear(..., enforce_clearance=false)`）：
  /// 只有占据 / 地形 / 查询失效才否决。停车前缀需要硬口径时直接调 `segmentClear`。
  bool pathClear(const std::vector<Eigen::Vector3d> & path,
    const Eigen::Vector3d & planning_start,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    SeedRejectInfo * reject_info = nullptr) const;

  bool searchDynamicDetour(const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    std::vector<Eigen::Vector3d> & detour,
    SeedRejectInfo * reject_info = nullptr) const;

  bool buildSafeStoppingPrefix(const Eigen::Vector3d & start,
    const std::vector<Eigen::Vector3d> & path,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    std::vector<Eigen::Vector3d> & prefix,
    SeedRejectInfo * reject_info = nullptr) const;

  /// buildSafeStoppingPrefix / buildEscapePrefix 共用的核心：沿 path 前进，在第一个净空不足的
  /// 采样点之前收尾，得到「障碍前 buffer 米、至少 min_length 米、最多 max_length 米
  /// （max_length <= 0 表示不限）」的停车前缀。
  bool buildStoppingPrefixCore(const Eigen::Vector3d & start,
    const std::vector<Eigen::Vector3d> & path,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    double buffer,
    double min_length,
    double max_length,
    std::vector<Eigen::Vector3d> & prefix,
    SeedRejectInfo * reject_info = nullptr) const;

  /// 第四层兜底：三层都失败时，用"不比当前净空更差"的短前缀把车挪出贴死状态。
  bool buildEscapePrefix(const Eigen::Vector3d & start,
    const std::vector<Eigen::Vector3d> & path,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double start_clearance,
    bool start_clearance_ok,
    const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> &
      terrain_segment_free,
    std::vector<Eigen::Vector3d> & prefix,
    SeedRejectInfo * reject_info = nullptr) const;

  double lookahead_dist_{5.0};
  double max_vel_{2.0};
  double max_acc_{4.0};
  double traj_goal_tolerance_{0.5};
  double collision_dist_{0.30};
  bool escape_enable_{true};
  double escape_min_length_{0.08};
  double escape_buffer_{0.05};
  rclcpp::Logger logger_{rclcpp::get_logger("LocalPathProcessor")};
  // buildSeed() 是 const，但"修复后被否决"这条日志需要计数：前 N 次逐条打出（排障最需要的
  // 就是第一次发生时的现场），之后退回 2 s 节流，避免窄道里按重规划频率刷屏。
  mutable uint64_t repair_reject_logged_{0};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
