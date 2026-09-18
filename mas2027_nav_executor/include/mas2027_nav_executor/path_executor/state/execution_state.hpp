#pragma once

#include <cstdint>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/mpc_position_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"

namespace mas2027_nav_executor {

enum class ExecutorStatus : uint8_t
{
  PUBLISHED,
  WAITING_INPUT,
  TRAJECTORY_STALE,
  REFERENCE_FAILED,
  SOLVER_FAILED,
  TERRAIN_BLOCKED,
  DYNAMIC_BLOCKED,
};

struct ExecutorInput
{
  interfaces::msg::MpcPositionCommand::ConstSharedPtr trajectory;
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  double spin_speed{};
  bool allow_motion{false};
  rclcpp::Time stamp;
};

struct ExecutorOutput
{
  geometry_msgs::msg::Twist command;
  ExecutorStatus status{ExecutorStatus::WAITING_INPUT};
};

}  // namespace mas2027_nav_executor
