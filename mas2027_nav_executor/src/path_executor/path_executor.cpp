#include "mas2027_nav_executor/path_executor/path_executor.hpp"
#include "mas2027_nav_executor/path_executor/monitoring/command_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mas2027_nav_executor {
using minco_controller::Control;
using minco_controller::MPCConfig;
using minco_controller::MpcSolver;
using minco_controller::ReferencePoint;
using minco_controller::State;

namespace {

double normalize_yaw(const double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

template<typename T>
bool transform_vector(
  const T & input,
  T & output,
  const geometry_msgs::msg::TransformStamped & transform)
{
  try {
    tf2::doTransform(input, output, transform);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

}  // namespace

PathExecutor::PathExecutor(
  const MPCConfig & config,
  const PathExecutorParams & params,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<TerrainGrid> terrain,
  std::shared_ptr<rog_map::MapQueryInterface> rog_query)
: config_(config),
  params_(params),
  tf_buffer_(std::move(tf_buffer)),
  terrain_(std::move(terrain)),
  rog_query_(std::move(rog_query)),
  solver_(std::make_unique<MpcSolver>(config)) {}

ExecutorOutput PathExecutor::computeCommand(const ExecutorInput & input)
{
  ExecutorOutput output;
  if (!input.allow_motion) {
    solver_->resetLastControl();
    output.status = ExecutorStatus::WAITING_INPUT;
    return output;
  }
  if (!input.trajectory || !input.odom || input.trajectory->cmds.empty()) {
    solver_->resetLastControl();
    output.status = ExecutorStatus::WAITING_INPUT;
    return output;
  }
  if (std::abs((input.stamp - rclcpp::Time(input.odom->header.stamp)).seconds()) >
    params_.odom_timeout_s) {
    solver_->resetLastControl();
    output.status = ExecutorStatus::WAITING_INPUT;
    return output;
  }

  const double yaw = tf2::getYaw(input.odom->pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  State current;
  current.x = input.odom->pose.pose.position.x;
  current.y = input.odom->pose.pose.position.y;
  current.yaw = yaw;
  current.vx = c * input.odom->twist.twist.linear.x - s * input.odom->twist.twist.linear.y;
  current.vy = s * input.odom->twist.twist.linear.x + c * input.odom->twist.twist.linear.y;
  current.omega = input.odom->twist.twist.angular.z;

  // 【2026-09-17】有实测车速之后，被打断**不再清零**加速度锚点，而是锚到实测车速。
  //
  // 清零的后果：恢复后的第一拍被加速度约束限成 |u_0| <= a_max·dt = 0.2 m/s，而车可能还在
  // 1.5 m/s —— 指令被硬砍到 0.2，底盘跟着急刹，再按每拍 0.2 m/s 爬回去。轨迹过期、参考
  // 失败、求解失败、任一道门否决都会走这条路，门控只要以 1~2 Hz 抖动，现场就是"反复启停、
  // 频繁冷启动"（用户报的现象）。锚到实测车速后，指令从车真正所在的速度接着走；速度上下限、
  // 加速度上下限、三道净空门全部不变，只是把约束从"相对上一条指令"改成"相对车实际速度"
  // —— 后者才是真正的加速度 (v_cmd - v_actual)/dt。
  const auto anchor_to_measured = [this, &current]() {
    solver_->setLastControl(Eigen::Vector3d(current.vx, current.vy, current.omega));
  };

  if ((input.stamp - rclcpp::Time(input.trajectory->header.stamp)).seconds() >
    params_.trajectory_timeout_s)
  {
    anchor_to_measured();
    output.status = ExecutorStatus::TRAJECTORY_STALE;
    return output;
  }

  std::vector<ReferencePoint> reference;
  if (!buildReference(*input.trajectory, current, reference)) {
    anchor_to_measured();
    output.status = ExecutorStatus::REFERENCE_FAILED;
    return output;
  }
  Control control;
  if (!solver_->solve(current, reference, control)) {
    anchor_to_measured();
    output.status = ExecutorStatus::SOLVER_FAILED;
    return output;
  }

  CommandSafetyDetail safety_detail;
  output.status = checkCommandSafety(
    terrain_, rog_query_, tf_buffer_, params_.odom_frame, params_.rog_map_clearance,
    params_.rog_map_timeout_s, config_.dt, current, control, input.stamp, &safety_detail);
  last_safety_detail_ = safety_detail;
  if (output.status != ExecutorStatus::PUBLISHED) {
    anchor_to_measured();
    return output;
  }

  if (std::hypot(control.vx, control.vy) >= params_.deadzone_speed) {
    if (params_.output_in_body_frame) {
      output.command.linear.x = c * control.vx + s * control.vy;
      output.command.linear.y = -s * control.vx + c * control.vy;
    } else {
      output.command.linear.x = control.vx;
      output.command.linear.y = control.vy;
    }
  }
  output.command.angular.z = std::clamp(
    control.omega + input.spin_speed, config_.omega_min, config_.omega_max);
  output.status = ExecutorStatus::PUBLISHED;
  return output;
}

bool PathExecutor::transformedCommands(
  const interfaces::msg::MpcPositionCommand & trajectory,
  std::vector<interfaces::msg::PositionCommand> & commands) const
{
  commands = trajectory.cmds;
  const std::string source_frame = trajectory.header.frame_id.empty() ?
    params_.odom_frame : trajectory.header.frame_id;
  if (source_frame == params_.odom_frame) return true;
  if (!tf_buffer_) return false;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(params_.odom_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }
  const double yaw_offset = tf2::getYaw(transform.transform.rotation);
  for (auto & command : commands) {
    geometry_msgs::msg::PointStamped point_in;
    geometry_msgs::msg::PointStamped point_out;
    geometry_msgs::msg::Vector3Stamped velocity_in;
    geometry_msgs::msg::Vector3Stamped velocity_out;
    geometry_msgs::msg::Vector3Stamped acceleration_in;
    geometry_msgs::msg::Vector3Stamped acceleration_out;
    geometry_msgs::msg::Vector3Stamped jerk_in;
    geometry_msgs::msg::Vector3Stamped jerk_out;
    point_in.header.frame_id = source_frame;
    point_in.point = command.position;
    velocity_in.header.frame_id = source_frame;
    velocity_in.vector = command.velocity;
    acceleration_in.header.frame_id = source_frame;
    acceleration_in.vector = command.acceleration;
    jerk_in.header.frame_id = source_frame;
    jerk_in.vector = command.jerk;
    if (!transform_vector(point_in, point_out, transform) ||
      !transform_vector(velocity_in, velocity_out, transform) ||
      !transform_vector(acceleration_in, acceleration_out, transform) ||
      !transform_vector(jerk_in, jerk_out, transform)) {
      return false;
    }
    command.position = point_out.point;
    command.velocity = velocity_out.vector;
    command.acceleration = acceleration_out.vector;
    command.jerk = jerk_out.vector;
    command.yaw = normalize_yaw(command.yaw + yaw_offset);
  }
  return true;
}

bool PathExecutor::buildReference(
  const interfaces::msg::MpcPositionCommand & trajectory,
  const State & current,
  std::vector<ReferencePoint> & reference) const
{
  std::vector<interfaces::msg::PositionCommand> commands;
  if (!transformedCommands(trajectory, commands) || commands.empty()) return false;

  size_t nearest = 0;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < commands.size(); ++index) {
    const double dx = commands[index].position.x - current.x;
    const double dy = commands[index].position.y - current.y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      nearest = index;
    }
  }

  const double planner_dt = 1.0 / params_.planner_frequency;
  const double current_time = static_cast<double>(nearest) * planner_dt + params_.control_delay_s;
  reference.clear();
  reference.reserve(static_cast<size_t>(config_.horizon));
  for (int step = 0; step < config_.horizon; ++step) {
    const double target_time = current_time + static_cast<double>(step) * config_.dt;
    const double index_float = std::clamp(
      target_time / planner_dt, 0.0, static_cast<double>(commands.size() - 1U));
    const size_t index = static_cast<size_t>(std::floor(index_float));
    const size_t next = std::min(index + 1U, commands.size() - 1U);
    const double alpha = index_float - static_cast<double>(index);
    const double dt = alpha * planner_dt;
    const auto & command = commands[index];
    ReferencePoint point;
    point.pos = Eigen::Vector2d(command.position.x, command.position.y) +
      Eigen::Vector2d(command.velocity.x, command.velocity.y) * dt +
      Eigen::Vector2d(command.acceleration.x, command.acceleration.y) * (0.5 * dt * dt) +
      Eigen::Vector2d(command.jerk.x, command.jerk.y) * (dt * dt * dt / 6.0);
    point.vel = Eigen::Vector2d(command.velocity.x, command.velocity.y) +
      Eigen::Vector2d(command.acceleration.x, command.acceleration.y) * dt +
      Eigen::Vector2d(command.jerk.x, command.jerk.y) * (0.5 * dt * dt);
    const double yaw_delta = normalize_yaw(commands[next].yaw - command.yaw);
    point.yaw = command.yaw + alpha * yaw_delta;
    point.yaw_rate = command.yaw_dot + alpha * (commands[next].yaw_dot - command.yaw_dot);
    reference.push_back(point);
  }
  return true;
}

}  // namespace mas2027_nav_executor
