#include "minco_core/components/trajectory_safety_checker.hpp"

#include "data_structure/base/trajectory.h"

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
  return std::isfinite(esdf_dist) && esdf_dist > check_dist;
}

bool TrajectorySafetyChecker::checkTrajectory(const traj_opt::Trajectory & traj) const
{
  return checkTrajectory(traj, 0.0, safe_dist_, traj.getTotalDuration());
}

bool TrajectorySafetyChecker::checkTrajectory(
  const traj_opt::Trajectory & traj, double t_start, double check_dist) const
{
  return checkTrajectory(traj, t_start, check_dist, traj.getTotalDuration());
}

bool TrajectorySafetyChecker::checkTrajectory(
  const traj_opt::Trajectory & traj, double t_start, double check_dist, double horizon) const
{
  if (!ensureQueryAvailable()) {
    return false;
  }

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  const double t0 = std::max(0.0, std::min(t_start, dur));
  const double span = (horizon > 0.0 && std::isfinite(horizon)) ? horizon : dur;
  const double t1 = std::min(dur, t0 + span);
  if (t1 <= t0 + 1e-9) {
    return checkPoint(traj.getPos(std::min(dur, t0)), check_dist);
  }
  for (double t = t0; t <= t1; t += sample_dt_) {
    if (!checkPoint(traj.getPos(t), check_dist)) {
      return false;
    }
  }
  return checkPoint(traj.getPos(t1), check_dist);
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
