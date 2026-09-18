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
#include "mas2027_nav_executor/path_executor/mpc/mpc_solver.hpp"
#include "mas2027_nav_executor/path_planner/path_planner.hpp"
#include "mas2027_nav_executor/path_executor/path_executor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
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
// PathPlanner owns the map-server distance query and MINCO; PathExecutor owns the MPC tracking step,
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
    planning_constraints_topic_ = declare_parameter<std::string>(
      "node.topics.planning_constraints_pub", "/planning_constraints");
    planning_constraints_marker_topic_ = declare_parameter<std::string>(
      "node.topics.planning_constraints_marker_pub", "/planning_constraints_markers");
    dynamic_obstacle_marker_topic_ = declare_parameter<std::string>(
      "node.topics.dynamic_obstacle_marker_pub", "/nav_executor/debug/dynamic_obstacles");
    const auto cost_map_topic = declare_parameter<std::string>("node.topics.terrain_cost_sub", "/cost_map");
    const auto direction_map_topic = declare_parameter<std::string>("node.topics.terrain_direction_sub", "/direction_map");
    const auto dynamic_map_topic = declare_parameter<std::string>("node.topics.dynamic_cost_map_sub", "/dynamic_cost_map");
    velocity_color_min_ = declare_parameter<double>("node.visualization.velocity_color_min");
    velocity_color_max_ = declare_parameter<double>("node.visualization.velocity_color_max");
    // 全局搜索折线（SMAC 2D / Astar 的输出）。注意这与上面 global_path_pub 的语义不同：
    // global_path_pub 发的是 MINCO 轨迹（名字是历史遗留，smoke_goal.py 依赖它数点数），
    // 下面这两个话题才是真正的全局折线，专门给 RViz 看。默认发布周期 5 Hz。
    global_plan_topic_ = declare_parameter<std::string>(
      "node.topics.global_plan_pub", "/nav_executor/global_plan");
    global_plan_marker_topic_ = declare_parameter<std::string>(
      "node.topics.global_plan_marker_pub", "/nav_executor/debug/global_plan");
    global_plan_publish_hz_ = declare_parameter<double>("node.visualization.global_plan_publish_hz", 5.0);
    global_plan_line_width_ = declare_parameter<double>("node.visualization.global_plan_line_width", 0.15);
    global_plan_color_r_ = declare_parameter<double>("node.visualization.global_plan_color_r", 0.0);
    global_plan_color_g_ = declare_parameter<double>("node.visualization.global_plan_color_g", 1.0);
    global_plan_color_b_ = declare_parameter<double>("node.visualization.global_plan_color_b", 1.0);
    if (!(std::isfinite(global_plan_publish_hz_) && global_plan_publish_hz_ > 0.0)) {
      global_plan_publish_hz_ = 5.0;
    }
    if (!(std::isfinite(global_plan_line_width_) && global_plan_line_width_ > 0.0)) {
      global_plan_line_width_ = 0.15;
    }

    PathExecutorParams executor_params;
    executor_params.planner_frequency = planner_frequency_;
    executor_params.trajectory_timeout_s = declare_parameter<double>("node.trajectory_timeout_s");
    executor_params.odom_timeout_s = declare_parameter<double>("node.odom_timeout_s", 0.5);
    executor_params.rog_map_clearance = declare_parameter<double>("node.rog_map_clearance", 0.30);
    // 动态层新鲜度上限。默认 1.5 s = map_server 旁路心跳（500 ms 一帧全 0 空图）的 3 倍：
    // 原来两处都硬编码 0.5 s，与发布周期零余量，实测 age 恒在 0.509~0.521 → 每个周期末尾
    // 必然有一拍把整条速度指令清零并丢掉 MPC 热启动，车表现为"一卡一卡"。
    executor_params.dynamic_map_timeout_s =
      declare_parameter<double>("node.dynamic_map_timeout_s", 1.5);
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

    if (control_rate_hz_ <= 0.0 || planner_frequency_ <= 0.0 || config.dt <= 0.0 ||
      !std::isfinite(executor_params.odom_timeout_s) || executor_params.odom_timeout_s <= 0.0 ||
      !std::isfinite(executor_params.dynamic_map_timeout_s) ||
      executor_params.dynamic_map_timeout_s <= 0.0 ||
      !std::isfinite(executor_params.rog_map_clearance) || executor_params.rog_map_clearance < 0.0) {
      throw std::invalid_argument(
        "control_rate_hz, planner_frequency, dt, odom_timeout_s and dynamic_map_timeout_s "
        "must be positive");
    }
    if (!std::isfinite(velocity_color_min_) || !std::isfinite(velocity_color_max_) ||
      velocity_color_min_ >= velocity_color_max_) {
      throw std::invalid_argument("visualization velocity color range must be finite and ordered");
    }

    terrain_grid_ = std::make_shared<TerrainGrid>();
    path_planner_ = std::make_shared<PathPlanner>(
      terrain_grid_, executor_params.odom_timeout_s, odom_frame_,
      executor_params.dynamic_map_timeout_s);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    path_executor_ = std::make_unique<PathExecutor>(
      config, executor_params, tf_buffer_, terrain_grid_, path_planner_->mapQuery());

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    terrain_cost_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      cost_map_topic, map_qos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        terrain_grid_->updateCost(*msg);
        publish_planning_constraints();
      });
    terrain_direction_sub_ = create_subscription<sensor_msgs::msg::Image>(
      direction_map_topic, map_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        terrain_grid_->updateDirection(*msg);
        publish_planning_constraints();
      });
    dynamic_cost_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      dynamic_map_topic, map_qos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        terrain_grid_->updateDynamic(*msg);
        publish_planning_constraints();
        publish_dynamic_obstacles(*msg);
      });

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
    global_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
      global_plan_topic_, rclcpp::QoS(1).transient_local());
    global_plan_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      // depth 要 > 1：transient_local 的 durability 缓存只保留最后 depth 条，而折线与
      // 终点球是两条独立消息，depth=1 时后打开 RViz 只能拿到终点球、看不到折线。
      global_plan_marker_topic_, rclcpp::QoS(10).transient_local());
    minco_trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      minco_trajectory_topic_, rclcpp::QoS(1).transient_local());
    planning_constraints_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      planning_constraints_topic_, rclcpp::QoS(1).reliable().transient_local());
    planning_constraints_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      planning_constraints_marker_topic_, rclcpp::QoS(1).reliable().transient_local());
    dynamic_obstacle_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      dynamic_obstacle_marker_topic_, rclcpp::QoS(1).reliable().transient_local());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&NavExecutorNode::control_tick, this));
    global_plan_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / global_plan_publish_hz_),
      std::bind(&NavExecutorNode::publish_global_plan, this));
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
  void publish_dynamic_obstacles(const nav_msgs::msg::OccupancyGrid & grid)
  {
    if (grid.info.width == 0 || grid.info.resolution <= 0.0F ||
      grid.data.size() != static_cast<size_t>(grid.info.width) * grid.info.height) return;
    visualization_msgs::msg::Marker marker;
    marker.header = grid.header;
    marker.ns = "dynamic_obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.06F, grid.info.resolution);
    marker.scale.y = marker.scale.x;
    marker.color.r = 0.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    for (size_t index = 0; index < grid.data.size(); ++index) {
      if (grid.data[index] < 95) continue;
      geometry_msgs::msg::Point point;
      point.x = grid.info.origin.position.x +
        (static_cast<double>(index % grid.info.width) + 0.5) * grid.info.resolution;
      point.y = grid.info.origin.position.y +
        (static_cast<double>(index / grid.info.width) + 0.5) * grid.info.resolution;
      point.z = 0.12;
      marker.points.push_back(point);
    }
    dynamic_obstacle_marker_pub_->publish(marker);
  }

  void publish_planning_constraints()
  {
    auto constraints = terrain_grid_->planningConstraints();
    if (!constraints) return;
    constraints->header.stamp = now();
    planning_constraints_pub_->publish(*constraints);

    visualization_msgs::msg::Marker markers;
    markers.header = constraints->header;
    markers.ns = "planning_constraints";
    markers.id = 0;
    markers.type = visualization_msgs::msg::Marker::POINTS;
    markers.action = visualization_msgs::msg::Marker::ADD;
    markers.pose.orientation.w = 1.0;
    markers.scale.x = std::max(0.04F, constraints->info.resolution * 0.9F);
    markers.scale.y = markers.scale.x;
    for (size_t index = 0; index < constraints->data.size(); ++index) {
      const int8_t value = constraints->data[index];
      if (value <= 0) continue;
      geometry_msgs::msg::Point point;
      point.x = constraints->info.origin.position.x +
        (static_cast<double>(index % constraints->info.width) + 0.5) * constraints->info.resolution;
      point.y = constraints->info.origin.position.y +
        (static_cast<double>(index / constraints->info.width) + 0.5) * constraints->info.resolution;
      point.z = 0.06;
      std_msgs::msg::ColorRGBA color;
      color.a = 1.0F;
      if (value >= 100) {
        color.r = 1.0F; color.g = 0.05F; color.b = 0.05F;
      } else {
        color.r = 1.0F; color.g = 1.0F; color.b = 0.0F;
      }
      markers.points.push_back(point);
      markers.colors.push_back(color);
    }
    planning_constraints_marker_pub_->publish(markers);
  }

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

  // 全局搜索折线（SMAC 2D / Astar 的输出，odom 系）。与上面的 MINCO 轨迹分开显示：
  // 轨迹是「车准备怎么走」，这条线是「搜索给出的拓扑引导」，两者对不上时一眼就能看出来。
  // 按定时器周期重发（见函数末尾说明），这样 RViz 后开也能拿到完整折线。
  void publish_global_plan()
  {
    if (!path_planner_ || !global_plan_pub_ || !global_plan_marker_pub_) return;

    std::vector<geometry_msgs::msg::PoseStamped> plan;
    const bool has_plan = path_planner_->copyLatestGlobalPath(plan) && plan.size() >= 2U;

    if (!has_plan) {
      // 目标被 invalidate / 搜索失败时清掉 RViz 上的旧线，否则会看着像还有一条可用路径。
      if (!global_plan_published_) return;
      global_plan_published_ = false;
      nav_msgs::msg::Path empty;
      empty.header.frame_id = odom_frame_;
      empty.header.stamp = now();
      global_plan_pub_->publish(empty);
      for (int id = 0; id <= 1; ++id) {
        visualization_msgs::msg::Marker clear;
        clear.header = empty.header;
        clear.ns = "global_plan";
        clear.id = id;
        clear.action = visualization_msgs::msg::Marker::DELETE;
        global_plan_marker_pub_->publish(clear);
      }
      return;
    }

    const auto & last = plan.back();

    nav_msgs::msg::Path path;
    path.header.frame_id = plan.front().header.frame_id.empty()
      ? odom_frame_ : plan.front().header.frame_id;
    path.header.stamp = now();
    path.poses = plan;

    // 抬高 3 cm，避免与 z=0 的代价图/规划约束栅格打架闪面。
    const double z_offset = 0.03;
    visualization_msgs::msg::Marker line;
    line.header = path.header;
    line.ns = "global_plan";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.pose.position.z = z_offset;
    line.scale.x = global_plan_line_width_;
    line.color.r = static_cast<float>(std::clamp(global_plan_color_r_, 0.0, 1.0));
    line.color.g = static_cast<float>(std::clamp(global_plan_color_g_, 0.0, 1.0));
    line.color.b = static_cast<float>(std::clamp(global_plan_color_b_, 0.0, 1.0));
    line.color.a = 1.0F;
    line.points.reserve(plan.size());
    for (const auto & pose : plan) {
      line.points.push_back(pose.pose.position);
    }

    // 终点球：让「全局搜索最后停在哪」也一目了然（到点容差内可能不是精确目标点）。
    visualization_msgs::msg::Marker goal_dot = line;
    goal_dot.id = 1;
    goal_dot.type = visualization_msgs::msg::Marker::SPHERE;
    goal_dot.points.clear();
    goal_dot.pose.position = last.pose.position;
    goal_dot.pose.position.z += z_offset;
    goal_dot.scale.x = global_plan_line_width_ * 2.5;
    goal_dot.scale.y = global_plan_line_width_ * 2.5;
    goal_dot.scale.z = global_plan_line_width_ * 2.5;

    global_plan_pub_->publish(path);
    global_plan_marker_pub_->publish(line);
    global_plan_marker_pub_->publish(goal_dot);
    // 每个 tick 都重发（不是只在变化时发）：RViz 常常在本节点之后才打开，靠的是
    // transient_local 的 durability 缓存，定期重发才能保证它一打开就看到完整折线，
    // 也避免 Marker 因缓存被后续消息挤出而消失。5 Hz × 一条折线，开销可以忽略。
    global_plan_published_ = true;
  }

  void accept_goal(const geometry_msgs::msg::PoseStamped::SharedPtr & goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal on %s", goal_topic_.c_str());
    if (!goal || !std::isfinite(goal->pose.position.x) ||
      !std::isfinite(goal->pose.position.y) || !std::isfinite(goal->pose.position.z)) {
      RCLCPP_WARN(get_logger(), "Ignoring an invalid navigation goal");
      return;
    }
    if (!path_planner_->acceptGoal(*goal)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ignoring goal until odometry and terrain/dynamic maps are ready (%s)", odom_topic_.c_str());
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
    input.allow_motion = input.trajectory &&
      path_planner_->acceptsTrajectory(rclcpp::Time(input.trajectory->header.stamp));

    const ExecutorOutput output = path_executor_->computeCommand(input);
    command_pub_->publish(output.command);
    if (output.status == ExecutorStatus::REFERENCE_FAILED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Braking: trajectory cannot be expressed in %s", odom_frame_.c_str());
    } else if (output.status == ExecutorStatus::SOLVER_FAILED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Braking: MPC solve failed");
    } else if (output.status == ExecutorStatus::TERRAIN_BLOCKED ||
               output.status == ExecutorStatus::DYNAMIC_BLOCKED) {
      // 这一类以前四种完全不同的成因共用一句话（动态层缺帧 / 地形层 transition 拒绝 /
      // TF 一时拿不到 / 净空或动态走廊违规），现场 43 条 Braking 里 28 条属于这类却
      // 分不出是哪一种。这里把命令门带出来的判据名与实测值打进日志。
      const auto & d = path_executor_->lastSafetyDetail();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Braking: %s (reason=%s value=%.3f threshold=%.3f)",
        output.status == ExecutorStatus::TERRAIN_BLOCKED ?
          "terrain layer or map transform unavailable, or next command violates terrain" :
          "current dynamic obstacle intersects the MPC reference horizon",
        d.reason, d.value, d.threshold);
    }
  }

  std::unique_ptr<PathExecutor> path_executor_;
  std::shared_ptr<TerrainGrid> terrain_grid_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_cost_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr terrain_direction_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_cost_map_sub_;
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
  // 真正的全局搜索折线（global_path_pub_ 发的是 MINCO 轨迹，名字是历史遗留）。
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_plan_marker_pub_;
  rclcpp::TimerBase::SharedPtr global_plan_timer_;
  bool global_plan_published_{false};
  double global_plan_publish_hz_{5.0};
  double global_plan_line_width_{0.15};
  double global_plan_color_r_{0.0};
  double global_plan_color_g_{1.0};
  double global_plan_color_b_{1.0};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr minco_trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_constraints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planning_constraints_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dynamic_obstacle_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double control_rate_hz_{};
  double planner_frequency_{};
  double spin_speed_{};
  std::string odom_topic_;
  std::string trajectory_topic_;
  std::string odom_frame_;
  std::string output_topic_;
  std::string global_path_topic_;
  std::string global_plan_topic_;
  std::string global_plan_marker_topic_;
  std::string minco_trajectory_topic_;
  std::string planning_constraints_topic_;
  std::string planning_constraints_marker_topic_;
  std::string dynamic_obstacle_marker_topic_;
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
