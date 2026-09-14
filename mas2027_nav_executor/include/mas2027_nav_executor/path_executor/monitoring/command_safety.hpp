#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "mas2027_nav_executor/path_executor/mpc/mpc_types.hpp"
#include "mas2027_nav_executor/path_executor/state/execution_state.hpp"
#include "rog_map/map_query_interface.hpp"
#include "tf2_ros/buffer.h"

namespace mas2027_nav_executor {

ExecutorStatus checkCommandSafety(
  const std::shared_ptr<TerrainGrid> & terrain,
  const std::shared_ptr<rog_map::MapQueryInterface> & rog_query,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & odom_frame,
  double rog_map_clearance,
  double dt,
  const minco_controller::State & current,
  const minco_controller::Control & control,
  const std::vector<minco_controller::ReferencePoint> & reference,
  const rclcpp::Time & stamp);

}  // namespace mas2027_nav_executor
