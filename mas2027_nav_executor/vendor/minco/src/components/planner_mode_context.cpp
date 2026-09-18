#include "minco_core/components/planner_mode_context.hpp"

#include <cctype>

namespace minco_planner {

void PlannerModeContext::configure(const PlannerModeParams & params,
  const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const rclcpp::Logger & logger)
{
  params_ = params;
  map_frame_ = params_.map_frame.empty() ? "map" : params_.map_frame;
  rog_frame_ = params_.rog_frame.empty() ? "camera_init" : params_.rog_frame;

  std::string mode_upper = params_.planner_mode;
  std::transform(mode_upper.begin(), mode_upper.end(), mode_upper.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });

  mode_ = PlannerMode::EXPLORATION;
  if (mode_upper != "EXPLORATION") {
    RCLCPP_WARN(logger,
      "[MincoPlanner] planner_mode='%s' is no longer supported; using EXPLORATION.",
      params_.planner_mode.c_str());
  }
  planning_frame_ = rog_frame_;
  output_frame_ = rog_frame_;
  direct_odom_pose_ = true;
  rebuildQueries(raw_rog_query, tf, logger);
}

void PlannerModeContext::rebuildQueries(const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const rclcpp::Logger & logger)
{
  (void)tf;
  (void)logger;
  global_query_ = raw_rog_query;
  dynamic_query_ = raw_rog_query;
  sparsify_query_ = raw_rog_query;
}

}  // namespace minco_planner
