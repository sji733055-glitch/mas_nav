#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include <vector>

namespace minco_planner
{

// planner_server transforms start/goal with the PoseStamped stamp. RViz /
// NavigateToPose copy the click time onto {goal}; BT ComputePathToPose then
// reuses that stamp. After ~10 s it falls out of the TF cache (odom→map
// extrapolation) and every far goal aborts. Zero the stamp so lookups use
// the latest transform.
class ZeroGoalStamp : public BT::SyncActionNode
{
public:
  ZeroGoalStamp(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal;
    if (getInput("goal", goal)) {
      goal.header.stamp.sec = 0;
      goal.header.stamp.nanosec = 0;
      setOutput("goal", goal);
    }

    std::vector<geometry_msgs::msg::PoseStamped> goals;
    if (getInput("goals", goals)) {
      for (auto & g : goals) {
        g.header.stamp.sec = 0;
        g.header.stamp.nanosec = 0;
      }
      setOutput("goals", goals);
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace minco_planner

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<minco_planner::ZeroGoalStamp>("ZeroGoalStamp");
}
