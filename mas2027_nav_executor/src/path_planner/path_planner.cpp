#include "mas2027_nav_executor/path_planner/path_planner.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rog_map/rog_map_core/config.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mas2027_nav_executor {

PathPlanner::PathPlanner(std::shared_ptr<TerrainGrid> terrain, double odom_timeout_s,
  const std::string & odom_frame, double rog_map_timeout_s)
: terrain_(std::move(terrain)), odom_timeout_s_(odom_timeout_s),
  rog_map_timeout_s_(rog_map_timeout_s)
{
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("nav_executor_planner");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  rog_map::Config config;
  config.loadFromRosNode(node_, "planner.rog_map");
  rog_map_ = std::make_shared<rog_map::ROGMapROS>(node_, config, tf_buffer_);
  // ROGMap supplies local online distance/gradient; terrain remains the global semantic map.
  planner_ = std::make_shared<minco_planner::MincoPlanner>();
  planner_->setMap(rog_map_->queryInterface());
  planner_->configure(node_, "planner", tf_buffer_);
  std::string planning_frame;
  node_->get_parameter("planner.frames.rog_frame", planning_frame);
  if (config.frame_id != odom_frame || planning_frame != odom_frame) {
    throw std::invalid_argument(
      "ROGMap frame, planner.frames.rog_frame and node.frames.odom must match");
  }
  planner_->setTerrainGrid(terrain_);
  task_manager_ = std::make_unique<TaskManager>(planner_, node_->get_logger(), odom_timeout_s_);
  task_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
    task_manager_->tick();
  });
}

PathPlanner::~PathPlanner()
{
  task_timer_.reset();
  task_manager_.reset();
  if (planner_) planner_->cleanup();
}

std::shared_ptr<rog_map::MapQueryInterface> PathPlanner::mapQuery() const
{
  return rog_map_->queryInterface();
}

bool PathPlanner::copyLatestGlobalPath(std::vector<geometry_msgs::msg::PoseStamped> & out) const
{
  if (!planner_) {
    out.clear();
    return false;
  }
  return planner_->copyLatestGlobalPath(out);
}

bool PathPlanner::acceptGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  const auto terrain = terrain_->snapshot();
  if (!terrain) {
    RCLCPP_WARN(node_->get_logger(),
      "Ignoring goal until static terrain cost and direction maps are ready");
    return false;
  }
  const auto query = rog_map_->queryInterface();
  if (!query || query->sizeX() == 0U || query->sizeY() == 0U) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal until ROGMap has an online snapshot");
    return false;
  }
  const double rog_stamp_s = query->snapshotStampSeconds();
  const double rog_age_s = std::isfinite(rog_stamp_s) ?
    std::abs(node_->now().seconds() - rog_stamp_s) : std::numeric_limits<double>::infinity();
  if (!std::isfinite(rog_age_s) || rog_age_s > rog_map_timeout_s_) {
    RCLCPP_WARN(node_->get_logger(),
      "Ignoring goal until ROGMap is fresh (age %.3f s, limit %.3f s)",
      rog_age_s, rog_map_timeout_s_);
    return false;
  }

  // Planning frame name kept for config compatibility; default odom.
  std::string planning_frame{"odom"};
  if (node_->has_parameter("planner.frames.rog_frame")) {
    node_->get_parameter("planner.frames.rog_frame", planning_frame);
  }
  if (planning_frame.empty()) {
    planning_frame = "odom";
  }

  // 远端操作者（Foxglove 等）发布目标时用的是客户端自己的时钟，与本机存在毫秒级偏差时，
  // tf2 会因"请求时间在未来"直接抛 extrapolation into the future 而丢弃目标
  // （2026-09-17 实测：笔记本快约 330 ms，Foxglove 点击的目标 100% 被静默忽略）。
  // 人工点击的目标"什么时候点的"对规划没有意义，所以查询前把时间戳归零：
  // tf2 把 0 视为"取最新可用 TF"，从此免疫两端时钟偏差。
  geometry_msgs::msg::PoseStamped goal_query = goal;
  goal_query.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  geometry_msgs::msg::PoseStamped goal_in_planning = goal_query;
  const std::string source = goal.header.frame_id.empty() ? planning_frame : goal.header.frame_id;
  if (source != planning_frame) {
    try {
      goal_in_planning = tf_buffer_->transform(goal_query, planning_frame, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(),
        "Ignoring goal: cannot transform from %s to %s (%s)",
        source.c_str(), planning_frame.c_str(), ex.what());
      return false;
    }
  } else {
    goal_in_planning.header.frame_id = planning_frame;
  }

  // tf2 的 transform() 是否把输出时间戳改写成查询时刻取决于实现，这里显式再归零一次，
  // 保证第二次（转到地形图坐标系）查询同样走"最新 TF"。
  goal_in_planning.header.stamp = goal_query.header.stamp;

  geometry_msgs::msg::PoseStamped goal_in_map;
  try {
    goal_in_map = tf_buffer_->transform(goal_in_planning, terrain->cost.header.frame_id,
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal: terrain TF unavailable (%s)", ex.what());
    return false;
  }
  const Eigen::Vector2d target(goal_in_map.pose.position.x, goal_in_map.pose.position.y);
  if (!terrain->traversable(target)) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal: occupied in static terrain");
    return false;
  }
  geometry_msgs::msg::PoseStamped start;
  if (!planner_->getRobotPose(start)) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal: no /Odometry received by planner yet");
    return false;
  }
  if (std::abs(node_->now().seconds() - rclcpp::Time(start.header.stamp).seconds()) >
    odom_timeout_s_) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal: odometry is stale");
    return false;
  }
  unsigned int mx = 0, my = 0;
  if (!query->worldToMap(start.pose.position.x, start.pose.position.y, mx, my) ||
    !query->isFree(mx, my) ||
    !query->query({start.pose.position.x, start.pose.position.y, 0.0}).ok) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal: ROGMap is not clear at the robot pose");
    return false;
  }
  task_manager_->submitGoal(goal_in_planning);
  RCLCPP_INFO(node_->get_logger(), "Queued goal (%.2f, %.2f) in %s; waiting for global search and MINCO",
    goal_in_planning.pose.position.x, goal_in_planning.pose.position.y,
    planning_frame.c_str());
  return true;
}

}  // namespace mas2027_nav_executor
