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
  // 先按 layer 致命/膨胀代价否决，再比 ROGMap field 净空与 check_dist（已扣除 inflation_radius）。
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
      // 起点查不到净空（如 OUT_OF_MAP：车在滑窗边缘或栅格之外）不等于轨迹不安全：
      // 直接判 false 会误杀整条轨迹，按 clearance_gate 语义只关闭近场放宽，逐点完整判据仍生效。
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
    // 本分支指「近场要求被下调」（起点净空可高于 required），故并列打印三个量以免读成自相矛盾。
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
