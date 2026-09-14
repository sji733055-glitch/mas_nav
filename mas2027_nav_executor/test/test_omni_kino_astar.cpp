#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <vector>

#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "mas2027_nav_executor/path_planner/search/omni_kino_astar.hpp"

int main()
{
  using mas2027_nav_executor::TerrainGrid;
  nav_msgs::msg::OccupancyGrid cost;
  cost.header.frame_id = "map";
  cost.info.width = 40;
  cost.info.height = 20;
  cost.info.resolution = 0.1F;
  cost.info.origin.orientation.w = 1.0;
  cost.data.assign(800, 0);
  for (int y = 4; y <= 6; ++y) cost.data[static_cast<size_t>(y * 40 + 14)] = 100;
  sensor_msgs::msg::Image direction;
  direction.header.frame_id = "map";
  direction.width = 40;
  direction.height = 20;
  direction.encoding = "bgr8";
  direction.step = 120;
  direction.data.assign(2400, 0);
  TerrainGrid grid;
  grid.updateCost(cost);
  grid.updateDirection(direction);
  const auto terrain = grid.snapshot();
  assert(terrain);
  std::vector<Eigen::Vector2d> path;
  assert(mas2027_nav_executor::searchOmniKinoPath(
    *terrain, {0.5, 0.5}, {2.5, 0.5}, {0.0, 0.0}, 2.0, 4.0, {}, path));
  assert(path.size() > 2U);
  assert((path.front() - Eigen::Vector2d(0.5, 0.5)).norm() < 1e-6);
  assert((path.back() - Eigen::Vector2d(2.5, 0.5)).norm() < 1e-6);
  bool detoured = false;
  for (size_t i = 1; i < path.size(); ++i) {
    assert(terrain->transition(path[i - 1], path[i]));
    if (std::abs(path[i].y() - 0.5) > 0.1) detoured = true;
  }
  assert(detoured);
  assert(!mas2027_nav_executor::searchOmniKinoPath(
    *terrain, {0.5, 0.5}, {2.5, 0.5}, {0.0, 0.0}, 2.0, 4.0,
    [](const Eigen::Vector2d & point) { return point.x() < 1.0; }, path));
}
