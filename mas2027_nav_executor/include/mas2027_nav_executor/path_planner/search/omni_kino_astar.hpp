#pragma once

#include <functional>
#include <vector>

#include <Eigen/Core>

#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"

namespace mas2027_nav_executor {

// Velocity-vector lattice: travel direction is independent of chassis yaw.
// This is the holonomic counterpart of HW's heading/speed Kino A*.
bool searchOmniKinoPath(
  const TerrainGrid::Snapshot & terrain,
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal,
  const Eigen::Vector2d & start_velocity,
  double max_speed,
  double max_acceleration,
  const std::function<bool(const Eigen::Vector2d &)> & dynamic_free,
  std::vector<Eigen::Vector2d> & path);

}  // namespace mas2027_nav_executor
