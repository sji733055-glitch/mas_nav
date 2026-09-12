#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "minco_core/minco_planner.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "mas2027_nav_executor/rog_map_query_adapter.hpp"

namespace rog_map { class ROGMapROS; }

namespace mas2027_nav_executor {

class PathPlanner final
{
public:
  PathPlanner();
  ~PathPlanner();
  bool acceptGoal(const geometry_msgs::msg::PoseStamped & goal);
  rclcpp_lifecycle::LifecycleNode::SharedPtr node() const { return node_; }
  std::shared_ptr<RogMapQueryAdapter> mapQuery() const { return map_query_; }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<rog_map::ROGMapROS> rog_map_;
  std::shared_ptr<RogMapQueryAdapter> map_query_;
  minco_planner::MincoPlanner::Ptr planner_;
};

}  // namespace mas2027_nav_executor
