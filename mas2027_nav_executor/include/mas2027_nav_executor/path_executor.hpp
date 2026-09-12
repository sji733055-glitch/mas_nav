#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/mpc_position_command.hpp"
#include "minco_controller/mpc_solver.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"

namespace mas2027_nav_executor {

struct PathExecutorParams
{
  double planner_frequency{};
  double trajectory_timeout_s{};
  double control_delay_s{};
  double deadzone_speed{};
  bool output_in_body_frame{};
  std::string odom_frame;
};

// Why the tracking loop produced (or refused to produce) a velocity. The node
// only uses this to log; every status still carries a publishable command.
enum class ExecutorStatus : uint8_t
{
  PUBLISHED,          // MPC solved and the command is its output
  WAITING_INPUT,      // no trajectory or no odometry yet
  TRAJECTORY_STALE,   // trajectory older than trajectory_timeout_s
  REFERENCE_FAILED,   // trajectory could not be expressed in the odom frame
  SOLVER_FAILED,
};

struct ExecutorInput
{
  interfaces::msg::MpcPositionCommand::ConstSharedPtr trajectory;
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  double spin_speed{};
  rclcpp::Time stamp;
};

struct ExecutorOutput
{
  // Zero twist on every status other than PUBLISHED, so the caller can publish
  // it unconditionally and the robot brakes when tracking is not possible.
  geometry_msgs::msg::Twist command;
  ExecutorStatus status{ExecutorStatus::WAITING_INPUT};
};

// Owns the MPC and the whole trajectory-tracking step: staleness check, frame
// alignment, reference sampling, solve, and body-frame velocity conversion.
class PathExecutor final
{
public:
  PathExecutor(
    const minco_controller::MPCConfig & config,
    const PathExecutorParams & params,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  ExecutorOutput computeCommand(const ExecutorInput & input);

private:
  bool transformedCommands(
    const interfaces::msg::MpcPositionCommand & trajectory,
    std::vector<interfaces::msg::PositionCommand> & commands) const;

  bool buildReference(
    const interfaces::msg::MpcPositionCommand & trajectory,
    const minco_controller::State & current,
    std::vector<minco_controller::ReferencePoint> & reference) const;

  minco_controller::MPCConfig config_;
  PathExecutorParams params_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<minco_controller::MpcSolver> solver_;
};

}  // namespace mas2027_nav_executor
