#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mas2027_nav_executor/path_executor/mpc/mpc_solver.hpp"
#include "mas2027_nav_executor/path_executor/state/execution_state.hpp"
#include "tf2_ros/buffer.h"
#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "rog_map/map_query_interface.hpp"

namespace mas2027_nav_executor {

struct PathExecutorParams
{
  double planner_frequency{};
  double trajectory_timeout_s{};
  double odom_timeout_s{};
  double control_delay_s{};
  double deadzone_speed{};
  bool output_in_body_frame{};
  std::string odom_frame;
  double rog_map_clearance{};
};

// Owns the MPC and the whole trajectory-tracking step: staleness check, frame
// alignment, reference sampling, solve, and body-frame velocity conversion.
class PathExecutor final
{
public:
  PathExecutor(
    const minco_controller::MPCConfig & config,
    const PathExecutorParams & params,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<TerrainGrid> terrain,
    std::shared_ptr<rog_map::MapQueryInterface> rog_query);

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
  std::shared_ptr<TerrainGrid> terrain_;
  std::shared_ptr<rog_map::MapQueryInterface> rog_query_;
  std::unique_ptr<minco_controller::MpcSolver> solver_;
};

}  // namespace mas2027_nav_executor
