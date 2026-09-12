#include "mas2027_nav_executor/path_planner.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rog_map/map_registry.hpp"
#include "rog_map/rog_map_core/config.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mas2027_nav_executor {

PathPlanner::PathPlanner()
{
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("nav_executor_planner");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  rog_map::Config config;
  config.loadFromRosNode(node_, "planner.rog_map");
  rog_map_ = std::make_shared<rog_map::ROGMapROS>(node_, config, tf_buffer_);
  const auto query = rog_map_->queryInterface();
  rog_map::MapRegistry::set(query);
  map_query_ = std::make_shared<RogMapQueryAdapter>(query);
  planner_ = std::make_shared<minco_planner::MincoPlanner>();
  planner_->configure(node_, "planner", tf_buffer_);
}

PathPlanner::~PathPlanner()
{
  if (planner_) planner_->cleanup();
}

bool PathPlanner::acceptGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  std::string rog_frame{"odom"};
  if (node_->has_parameter("planner.frames.rog_frame")) {
    node_->get_parameter("planner.frames.rog_frame", rog_frame);
  }
  if (rog_frame.empty()) {
    rog_frame = "odom";
  }

  geometry_msgs::msg::PoseStamped goal_in_rog = goal;
  const std::string source = goal.header.frame_id.empty() ? rog_frame : goal.header.frame_id;
  if (source != rog_frame) {
    try {
      goal_in_rog = tf_buffer_->transform(goal, rog_frame, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(),
        "Ignoring goal: cannot transform from %s to %s (%s)",
        source.c_str(), rog_frame.c_str(), ex.what());
      return false;
    }
  } else {
    goal_in_rog.header.frame_id = rog_frame;
  }

  if (!map_query_->isFree({goal_in_rog.pose.position.x, goal_in_rog.pose.position.y})) {
    RCLCPP_WARN(node_->get_logger(),
      "Ignoring goal at (%.2f, %.2f) in %s: not free in ROGMap",
      goal_in_rog.pose.position.x, goal_in_rog.pose.position.y, rog_frame.c_str());
    return false;
  }
  geometry_msgs::msg::PoseStamped start;
  if (!planner_->getRobotPose(start)) return false;
  planner_->createPlan(start, goal_in_rog, []() { return !rclcpp::ok(); });
  return true;
}

}  // namespace mas2027_nav_executor
