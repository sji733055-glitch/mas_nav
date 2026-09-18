#include "mas2027_nav_executor/path_planner/trajectory/trajectory_safety_checker.hpp"

#include "data_structure/base/trajectory.h"
#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace minco_planner {

void TrajectorySafetyChecker::configure(double safe_dist, double sample_dt, rclcpp::Logger logger)
{
  safe_dist_ = safe_dist;
  sample_dt_ = sample_dt > 1e-6 ? sample_dt : 0.05;
  logger_ = logger;
}

void TrajectorySafetyChecker::setQuery(std::shared_ptr<rog_map::MapQueryInterface> dynamic_query)
{
  dynamic_query_ = std::move(dynamic_query);
}

bool TrajectorySafetyChecker::ensureQueryAvailable() const
{
  if (dynamic_query_) {
    return true;
  }
  RCLCPP_ERROR_THROTTLE(logger_,
    *rclcpp::Clock::make_shared(),
    1000,
    "[MincoPlanner] ROGMap dynamic query is unavailable for trajectory safety check.");
  return false;
}

bool TrajectorySafetyChecker::checkPoint(const Eigen::Vector3d & pos) const
{
  return checkPoint(pos, safe_dist_);
}

bool TrajectorySafetyChecker::checkPoint(const Eigen::Vector3d & pos, double check_dist) const
{
  // 安全检查先拒绝二维 layer 中的致命/膨胀代价值，再用 ROGMap distance field 与 check_dist 比较。
  // ROGMap field 已扣除 field.inflation_radius，check_dist 会继续增加规划安全余量。
  if (!ensureQueryAvailable()) {
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!dynamic_query_->worldToMap(pos.x(), pos.y(), mx, my)) {
    return false;
  }
  const unsigned char cost = dynamic_query_->value(mx, my);
  if (cost == kLethalCost || cost == kInscribedCost) {
    return false;
  }
  double esdf_dist = 0.0;
  Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query_->query(pos);
  if (!query.ok) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      1000,
      "[MincoPlanner] Trajectory safety query failed: %s",
      rog_map::queryStatusName(query.status));
    return false;
  }
  esdf_dist = query.distance;
  esdf_grad = query.gradient;
  (void)esdf_grad;
  if (std::isfinite(esdf_dist) && esdf_dist <= check_dist) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "Trajectory clearance %.3f m below required %.3f m at (%.2f, %.2f)",
      esdf_dist, check_dist, pos.x(), pos.y());
  }
  return std::isfinite(esdf_dist) && esdf_dist > check_dist;
}

bool TrajectorySafetyChecker::checkTrajectory(const traj_opt::Trajectory & traj) const
{
  CheckOptions options;
  options.check_dist = safe_dist_;
  return checkTrajectory(traj, options);
}

bool TrajectorySafetyChecker::checkTrajectory(
  const traj_opt::Trajectory & traj, double t_start, double check_dist) const
{
  CheckOptions options;
  options.t_start = t_start;
  options.check_dist = check_dist;
  return checkTrajectory(traj, options);
}

bool TrajectorySafetyChecker::checkTrajectory(
  const traj_opt::Trajectory & traj, double t_start, double check_dist, double horizon) const
{
  CheckOptions options;
  options.t_start = t_start;
  options.check_dist = check_dist;
  options.horizon = horizon;
  return checkTrajectory(traj, options);
}

bool TrajectorySafetyChecker::checkTrajectory(
  const traj_opt::Trajectory & traj, const CheckOptions & options) const
{
  if (!ensureQueryAvailable()) {
    return false;
  }

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  const double t0 = std::max(0.0, std::min(options.t_start, dur));
  const double span = (options.horizon > 0.0 && std::isfinite(options.horizon)) ?
    options.horizon : dur;
  const double t1 = std::min(dur, t0 + span);

  // 近场判据：t0 处的实测净空是「机器人现在到底有多近」的权威值。拿不到就不放宽。
  const Eigen::Vector3d start_pos = traj.getPos(t0);
  double start_clearance = 0.0;
  bool start_clearance_ok = false;
  if (options.near_field > 1e-6) {
    const auto start_query = dynamic_query_->query(start_pos);
    if (start_query.ok) {
      start_clearance = start_query.distance;
      start_clearance_ok = true;
    } else {
      // 起点查不到净空（现场典型是 OUT_OF_MAP：车在滑窗边缘或栅格之外）**不等于轨迹不安全**。
      // 原实现在这里直接 return false，会把整条轨迹误杀——现场表现为连续
      //   "Near-field clearance query failed: OUT_OF_MAP" + "Trajectory collision detected"（各 30 次）。
      // 按 clearance_gate.hpp 的既定语义降级：拿不到当前净空就不启用近场放宽，
      // 全程按完整 required 判（makeClearanceRequirement 在 current_clearance_ok=false 时正是如此）。
      // 这只是取消"近场豁免"这一放宽项，逐个采样点的完整净空判据仍然生效，不放松安全底线。
      RCLCPP_WARN_THROTTLE(logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "[MincoPlanner] Near-field clearance query failed: %s; near-field relaxation disabled for this check",
        rog_map::queryStatusName(start_query.status));
    }
  }
  const mas2027_nav_executor::ClearanceRequirement gate =
    mas2027_nav_executor::makeClearanceRequirement(options.check_dist,
      options.near_field,
      start_clearance,
      start_clearance_ok,
      options.near_field_slack);
  if (gate.nearFieldEnabled() && gate.near_required < gate.required) {
    // 明确记录「近场要求被下调」：真车贴着墙停车时靠这条规则才能起步，
    // 排查「车不动」时先看这里有没有刷，再看是不是连近场外的要求也满足不了。
    // 措辞注意：本分支的条件是「近场要求被下调」，它会在起点净空**高于** required 时成立
    // （净空落在 [required, required + slack) 区间内就会成立）。旧措辞写作
    // "start clearance X below required Y" 会把 X = 0.318、Y = 0.300 这种 X > Y 的数字
    // 摆在一起，读起来自相矛盾，已两次导致现场把「近场规则正常工作」误判成日志 bug 或
    // 阈值不一致。这里改成直接打印三个量：实测净空、下调后的近场要求、完整要求。
    RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "[MincoPlanner] Near-field exemption active: start clearance %.3f m, "
      "near requirement lowered to %.3f m (full requirement %.3f m); "
      "inside %.2f m of the start only 'not worse than now' is enforced",
      start_clearance, gate.near_required, gate.required, gate.near_field);
  }

  if (t1 <= t0 + 1e-9) {
    return checkPoint(start_pos, gate.requiredAt(0.0));
  }

  // 沿轨迹累积弧长：近场是按「离起点多远」划分的，不是按时间。
  double arc = 0.0;
  Eigen::Vector3d previous = start_pos;
  if (!checkPoint(previous, gate.requiredAt(arc))) {
    return false;
  }
  for (double t = t0 + sample_dt_; t <= t1; t += sample_dt_) {
    const Eigen::Vector3d pos = traj.getPos(t);
    arc += (pos - previous).norm();
    previous = pos;
    if (!checkPoint(pos, gate.requiredAt(arc))) {
      return false;
    }
  }
  const Eigen::Vector3d last = traj.getPos(t1);
  arc += (last - previous).norm();
  return checkPoint(last, gate.requiredAt(arc));
}

double TrajectorySafetyChecker::getDistance(const Eigen::Vector3d & pos) const
{
  if (!dynamic_query_) {
    return 0.0;
  }
  double dist = 0.0;
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query_->query(pos);
  dist = query.distance;
  grad = query.gradient;
  (void)grad;
  return query.ok ? dist : std::numeric_limits<double>::quiet_NaN();
}

bool TrajectorySafetyChecker::projectOutOfObstacle(Eigen::Vector3d & pos, double margin) const
{
  if (!dynamic_query_) {
    return false;
  }
  double esdf_dist = 0.0;
  Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query_->query(pos);
  if (!query.ok) {
    return false;
  }
  esdf_dist = query.distance;
  esdf_grad = query.gradient;
  if (esdf_dist < 0.0 && esdf_grad.norm() > 1e-6) {
    pos += (margin - esdf_dist) * esdf_grad.normalized();
    return true;
  }
  return false;
}

}  // namespace minco_planner
