#ifndef MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_
#define MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class TrajectorySafetyChecker
{
public:
  void configure(double safe_dist, double sample_dt, rclcpp::Logger logger);
  void setQuery(std::shared_ptr<rog_map::MapQueryInterface> dynamic_query);

  /// 轨迹安全检查参数。近场（沿轨迹弧长 near_field 以内）是机器人当前已经占住的一段，
  /// 那里不可能满足完整净空要求，因此只要求「不比当前实测净空更差」，判据见
  /// mas2027_nav_executor/common/environment/clearance_gate.hpp。
  /// 发布前校验与运行时监视必须用同一套参数，否则会出现「刚发布就被监视器判不安全」的
  /// 急停 + 重规划死循环（cmd_vel 大部分时间为 0、RViz 轨迹来回闪）。
  struct CheckOptions
  {
    double t_start{0.0};            ///< 从轨迹的这个时刻开始检查（0 = 轨迹起点）
    double horizon{0.0};            ///< 检查时长；<= 0 表示检查到轨迹结束
    double check_dist{0.0};         ///< 近场之外要求的最小净空
    double near_field{0.0};         ///< 近场弧长半径；<= 0 表示关闭近场放宽
    double near_field_slack{0.02};  ///< 近场内允许比当前净空再少多少（吸收 ESDF 抖动）
  };

  bool checkPoint(const Eigen::Vector3d & pos) const;
  bool checkPoint(const Eigen::Vector3d & pos, double check_dist) const;
  bool checkTrajectory(const traj_opt::Trajectory & traj) const;
  bool checkTrajectory(const traj_opt::Trajectory & traj, const CheckOptions & options) const;
  bool checkTrajectory(
    const traj_opt::Trajectory & traj, double t_start, double check_dist) const;
  bool checkTrajectory(
    const traj_opt::Trajectory & traj, double t_start, double check_dist, double horizon) const;
  double getDistance(const Eigen::Vector3d & pos) const;
  bool projectOutOfObstacle(Eigen::Vector3d & pos, double margin) const;

private:
  bool ensureQueryAvailable() const;

  std::shared_ptr<rog_map::MapQueryInterface> dynamic_query_;
  double safe_dist_{0.0};
  double sample_dt_{0.05};
  rclcpp::Logger logger_{rclcpp::get_logger("TrajectorySafetyChecker")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_
