#ifndef MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
#define MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_

#include "minco_core/header.hpp"
#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"

namespace minco_planner {

class PlannerModeContext;

class GlobalPathSearcher
{
public:
  void configure(std::shared_ptr<tf2_ros::Buffer> tf,
    Astar * astar,
    bool allow_unknown,
    double tolerance,
    rclcpp::Logger logger);

  void setQuery(const std::shared_ptr<rog_map::MapQueryInterface> & global_query);
  void setTerrainGrid(std::shared_ptr<mas2027_nav_executor::TerrainGrid> terrain)
  { terrain_ = std::move(terrain); }

  bool plan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const Eigen::Vector2d & start_velocity,
    double max_speed, double max_acceleration,
    const PlannerModeContext & mode_context,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  bool makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    const PlannerModeContext & mode_context,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

private:
  bool normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
    const std::string & fallback_frame,
    const std::string & target_frame,
    const std::string & context,
    geometry_msgs::msg::PoseStamped & out) const;

  bool planExploration(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const Eigen::Vector2d & start_velocity,
    double max_speed, double max_acceleration,
    const PlannerModeContext & mode_context,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  bool makePlanOnQuery(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    const std::string & output_frame,
    const std::string & failure_source,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  Astar * astar_{nullptr};
  std::shared_ptr<rog_map::MapQueryInterface> global_query_;
  std::shared_ptr<mas2027_nav_executor::TerrainGrid> terrain_;
  bool allow_unknown_{true};
  double tolerance_{0.5};
  rclcpp::Logger logger_{rclcpp::get_logger("GlobalPathSearcher")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
