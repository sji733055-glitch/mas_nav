#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace minco_planner
{

// Keep a one-shot ComputePath subtree running until the navigation goal
// changes. A new goal halts the running FollowPath action, allowing the child
// sequence to execute ZeroGoalStamp and ComputePathToPose again.
class GoalUpdatedController : public BT::DecoratorNode
{
public:
  GoalUpdatedController(const std::string & name, const BT::NodeConfig & config)
  : BT::DecoratorNode(name, config),
    has_goal_port_(config.input_ports.count("goal") != 0),
    has_goals_port_(config.input_ports.count("goals") != 0)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (has_goal_port_) {
      geometry_msgs::msg::PoseStamped goal;
      if (getInput("goal", goal)) {
        if (goal_snapshot_valid_ && !samePose(goal, goal_snapshot_)) {
          haltChild();
        }
        goal_snapshot_ = goal;
        goal_snapshot_valid_ = true;
      }
    }

    if (has_goals_port_) {
      std::vector<geometry_msgs::msg::PoseStamped> goals;
      if (getInput("goals", goals)) {
        if (goals_snapshot_valid_ && !samePoses(goals, goals_snapshot_)) {
          haltChild();
        }
        goals_snapshot_ = goals;
        goals_snapshot_valid_ = true;
      }
    }

    return child_node_->executeTick();
  }

  void halt() override
  {
    goal_snapshot_valid_ = false;
    goals_snapshot_valid_ = false;
    BT::DecoratorNode::halt();
  }

private:
  static bool samePose(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs)
  {
    constexpr double epsilon = 1.0e-6;
    return lhs.header.frame_id == rhs.header.frame_id &&
           std::abs(lhs.pose.position.x - rhs.pose.position.x) <= epsilon &&
           std::abs(lhs.pose.position.y - rhs.pose.position.y) <= epsilon &&
           std::abs(lhs.pose.position.z - rhs.pose.position.z) <= epsilon &&
           std::abs(lhs.pose.orientation.x - rhs.pose.orientation.x) <= epsilon &&
           std::abs(lhs.pose.orientation.y - rhs.pose.orientation.y) <= epsilon &&
           std::abs(lhs.pose.orientation.z - rhs.pose.orientation.z) <= epsilon &&
           std::abs(lhs.pose.orientation.w - rhs.pose.orientation.w) <= epsilon;
  }

  static bool samePoses(
    const std::vector<geometry_msgs::msg::PoseStamped> & lhs,
    const std::vector<geometry_msgs::msg::PoseStamped> & rhs)
  {
    if (lhs.size() != rhs.size()) {
      return false;
    }
    for (size_t i = 0; i < lhs.size(); ++i) {
      if (!samePose(lhs[i], rhs[i])) {
        return false;
      }
    }
    return true;
  }

  const bool has_goal_port_;
  const bool has_goals_port_;
  bool goal_snapshot_valid_{false};
  bool goals_snapshot_valid_{false};
  geometry_msgs::msg::PoseStamped goal_snapshot_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_snapshot_;
};

}  // namespace minco_planner

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<minco_planner::GoalUpdatedController>("GoalUpdatedController");
}
