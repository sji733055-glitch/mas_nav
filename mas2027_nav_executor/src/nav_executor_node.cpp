#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "interfaces/msg/mpc_position_command.hpp"
#include "minco_controller/mpc_solver.hpp"
#include "mas2027_nav_executor/path_planner.hpp"
#include "mas2027_nav_executor/path_executor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace mas2027_nav_executor {
using minco_controller::MPCConfig;
namespace {

std_msgs::msg::ColorRGBA velocity_color(
  const double velocity, const double minimum, const double maximum)
{
  const double normalized = std::isfinite(velocity) && maximum > minimum ?
    std::clamp((velocity - minimum) / (maximum - minimum), 0.0, 1.0) : 0.0;
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(std::clamp(1.5 - std::abs(4.0 * normalized - 3.0), 0.0, 1.0));
  color.g = static_cast<float>(std::clamp(1.5 - std::abs(4.0 * normalized - 2.0), 0.0, 1.0));
  color.b = static_cast<float>(std::clamp(1.5 - std::abs(4.0 * normalized - 1.0), 0.0, 1.0));
  color.a = 1.0F;
  return color;
}

}  // namespace

// A deliberately small standalone navigation loop. The node only wires ROS up:
// PathPlanner owns ROGMap and MINCO, PathExecutor owns the MPC tracking step,
// and this class subscribes, ticks the control loop and publishes.
class NavExecutorNode final : public rclcpp::Node
{
public:
  NavExecutorNode()
  : Node("nav_executor")
  {
    control_rate_hz_ = declare_parameter<double>("node.control_rate_hz");
    planner_frequency_ = declare_parameter<double>("node.planner_frequency");
    odom_topic_ = declare_parameter<std::string>("node.topics.odom_sub");
    trajectory_topic_ = declare_parameter<std::string>("node.topics.trajectory_sub");
    odom_frame_ = declare_parameter<std::string>("node.frames.odom");
    output_topic_ = declare_parameter<std::string>("node.topics.cmd_vel_pub");
    cmd_spin_topic_ = declare_parameter<std::string>("node.topics.spin_cmd_sub");
    goal_topic_ = declare_parameter<std::string>("node.topics.goal_sub");
    global_path_topic_ = declare_parameter<std::string>("node.topics.global_path_pub");
    minco_trajectory_topic_ = declare_parameter<std::string>("node.topics.minco_trajectory_pub");
    velocity_color_min_ = declare_parameter<double>("node.visualization.velocity_color_min");
    velocity_color_max_ = declare_parameter<double>("node.visualization.velocity_color_max");

    PathExecutorParams executor_params;
    executor_params.planner_frequency = planner_frequency_;
    executor_params.trajectory_timeout_s = declare_parameter<double>("node.trajectory_timeout_s");
    executor_params.control_delay_s = declare_parameter<double>("mpc.control_delay_compensation");
    executor_params.deadzone_speed = declare_parameter<double>("mpc.deadzone_speed_threshold");
    executor_params.output_in_body_frame = declare_parameter<bool>("node.output_in_body_frame");
    executor_params.odom_frame = odom_frame_;

    MPCConfig config;
    config.dt = declare_parameter<double>("mpc.dt");
    config.lookahead_time = declare_parameter<double>("mpc.lookahead_time");
    config.horizon = std::max(1, static_cast<int>(std::ceil(config.lookahead_time / config.dt)));
    config.planner_freq = planner_frequency_;
    config.vx_min = declare_parameter<double>("mpc.constraints.velocity.x.min");
    config.vx_max = declare_parameter<double>("mpc.constraints.velocity.x.max");
    config.vy_min = declare_parameter<double>("mpc.constraints.velocity.y.min");
    config.vy_max = declare_parameter<double>("mpc.constraints.velocity.y.max");
    config.omega_min = declare_parameter<double>("mpc.constraints.velocity.omega.min");
    config.omega_max = declare_parameter<double>("mpc.constraints.velocity.omega.max");
    config.use_acc_constraints = declare_parameter<bool>("mpc.constraints.acceleration.enable");
    config.ax_min = declare_parameter<double>("mpc.constraints.acceleration.x.min");
    config.ax_max = declare_parameter<double>("mpc.constraints.acceleration.x.max");
    config.ay_min = declare_parameter<double>("mpc.constraints.acceleration.y.min");
    config.ay_max = declare_parameter<double>("mpc.constraints.acceleration.y.max");
    config.alpha_min = declare_parameter<double>("mpc.constraints.acceleration.omega.min");
    config.alpha_max = declare_parameter<double>("mpc.constraints.acceleration.omega.max");

    const auto q = declare_parameter<std::vector<double>>("mpc.weights.state");
    const auto r = declare_parameter<std::vector<double>>("mpc.weights.command");
    if (q.size() != 3U || r.size() != 3U) {
      throw std::invalid_argument("mpc.weights.state and command must each contain 3 values");
    }
    config.Q = Eigen::Vector3d(q[0], q[1], q[2]);
    config.R = Eigen::Vector3d(r[0], r[1], r[2]);

    if (control_rate_hz_ <= 0.0 || planner_frequency_ <= 0.0 || config.dt <= 0.0) {
      throw std::invalid_argument("control_rate_hz, planner_frequency and dt must be positive");
    }
    if (!std::isfinite(velocity_color_min_) || !std::isfinite(velocity_color_max_) ||
      velocity_color_min_ >= velocity_color_max_) {
      throw std::invalid_argument("visualization velocity color range must be finite and ordered");
    }

    path_planner_ = std::make_shared<PathPlanner>();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    path_executor_ = std::make_unique<PathExecutor>(config, executor_params, tf_buffer_);

    trajectory_sub_ = create_subscription<interfaces::msg::MpcPositionCommand>(
      trajectory_topic_, rclcpp::QoS(1),
      [this](const interfaces::msg::MpcPositionCommand::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          trajectory_ = msg;
        }
        publish_trajectory_visualization(*msg);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        odom_ = msg;
      });
    spin_sub_ = create_subscription<example_interfaces::msg::Float32>(
      cmd_spin_topic_, rclcpp::QoS(1),
      [this](const example_interfaces::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        spin_speed_ = std::isfinite(msg->data) ? msg->data : 0.0;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal) { accept_goal(goal); });
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(1));
    global_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      global_path_topic_, rclcpp::QoS(1).transient_local());
    minco_trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      minco_trajectory_topic_, rclcpp::QoS(1).transient_local());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&NavExecutorNode::control_tick, this));
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr planner_node() const
  {
    return path_planner_->node();
  }

  void shutdown_planner()
  {
    path_planner_.reset();
  }

private:
  void publish_trajectory_visualization(const interfaces::msg::MpcPositionCommand & trajectory)
  {
    if (trajectory.cmds.empty()) return;

    nav_msgs::msg::Path path;
    path.header = trajectory.header;
    if (path.header.frame_id.empty()) path.header.frame_id = odom_frame_;
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "minco_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.07;
    path.poses.reserve(trajectory.cmds.size());
    marker.points.reserve(trajectory.cmds.size());
    marker.colors.reserve(trajectory.cmds.size());
    for (const auto & command : trajectory.cmds) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = command.position;
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
        0.0, 0.0, std::sin(command.yaw * 0.5), std::cos(command.yaw * 0.5)));
      path.poses.push_back(pose);
      marker.points.push_back(command.position);
      marker.colors.push_back(
        velocity_color(command.vel_norm, velocity_color_min_, velocity_color_max_));
    }
    global_path_pub_->publish(path);
    minco_trajectory_pub_->publish(marker);
  }

  void accept_goal(const geometry_msgs::msg::PoseStamped::SharedPtr & goal)
  {
    if (!goal || !std::isfinite(goal->pose.position.x) ||
      !std::isfinite(goal->pose.position.y) || !std::isfinite(goal->pose.position.z)) {
      RCLCPP_WARN(get_logger(), "Ignoring an invalid navigation goal");
      return;
    }
    if (!path_planner_->acceptGoal(*goal)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ignoring goal until ROGMap and %s are ready", odom_topic_.c_str());
    }
  }

  void control_tick()
  {
    ExecutorInput input;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      input.trajectory = trajectory_;
      input.odom = odom_;
      input.spin_speed = spin_speed_;
    }
    input.stamp = now();

    const ExecutorOutput output = path_executor_->computeCommand(input);
    command_pub_->publish(output.command);
    if (output.status == ExecutorStatus::REFERENCE_FAILED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Braking: trajectory cannot be expressed in %s", odom_frame_.c_str());
    } else if (output.status == ExecutorStatus::SOLVER_FAILED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Braking: MPC solve failed");
    }
  }

  std::unique_ptr<PathExecutor> path_executor_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<PathPlanner> path_planner_;
  std::mutex data_mutex_;
  interfaces::msg::MpcPositionCommand::SharedPtr trajectory_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  rclcpp::Subscription<interfaces::msg::MpcPositionCommand>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr spin_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr minco_trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double control_rate_hz_{};
  double planner_frequency_{};
  double spin_speed_{};
  std::string odom_topic_;
  std::string trajectory_topic_;
  std::string odom_frame_;
  std::string output_topic_;
  std::string global_path_topic_;
  std::string minco_trajectory_topic_;
  std::string cmd_spin_topic_;
  std::string goal_topic_;
  double velocity_color_min_{};
  double velocity_color_max_{};
};

}  // namespace mas2027_nav_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mas2027_nav_executor::NavExecutorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->planner_node()->get_node_base_interface());
  executor.spin();
  node->shutdown_planner();
  rclcpp::shutdown();
  return 0;
}
