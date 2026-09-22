#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"

int main()
{
  using mas2027_nav_executor::TerrainGrid;
  nav_msgs::msg::OccupancyGrid cost;
  cost.header.frame_id = "map";
  cost.info.width = 5;
  cost.info.height = 5;
  cost.info.resolution = 1.0F;
  cost.info.origin.orientation.w = 1.0;
  cost.data.assign(25, 0);
  cost.data[2 + 2 * 5] = 100;
  sensor_msgs::msg::Image direction;
  direction.header.frame_id = "map";
  direction.width = 5;
  direction.height = 5;
  direction.encoding = "bgr8";
  direction.step = 15;
  direction.data.assign(75, 0);

  TerrainGrid grid;
  grid.updateCost(cost);
  assert(!grid.snapshot());
  grid.updateDirection(direction);
  auto snapshot = grid.snapshot();
  assert(snapshot);
  std::vector<Eigen::Vector2d> path;
  assert(snapshot->search({0.5, 2.5}, {4.5, 2.5}, path));
  assert(path.size() >= 4);
  assert(!snapshot->transition({1.5, 2.5}, {3.5, 2.5}));

  // Direction body: horizontal traversal allowed, vertical traversal rejected.
  direction.data[(2 * 5 + 1) * 3 + 1] = 255;
  direction.data[(2 * 5 + 1) * 3 + 2] = 2;
  grid.updateDirection(direction);
  snapshot = grid.snapshot();
  assert(snapshot->transition({0.5, 2.5}, {1.5, 2.5}));
  assert(!snapshot->transition({1.5, 1.5}, {1.5, 2.5}));

  // HW's high step has an up-only default when leg modes are omitted.
  direction.data[(2 * 5 + 1) * 3 + 2] = 6;
  grid.updateDirection(direction);
  snapshot = grid.snapshot();
  assert(snapshot->transition({0.5, 2.5}, {1.5, 2.5}));
  assert(!snapshot->transition({1.5, 2.5}, {0.5, 2.5}));
  auto constraints = grid.planningConstraints();
  assert(constraints);
  assert(constraints->data[2 + 2 * 5] == 100);
  assert(constraints->data[1 + 2 * 5] == 50);

  constraints = grid.planningConstraints();
  assert(constraints && constraints->data[2 + 2 * 5] == 100);
  assert(constraints->data[3 + 2 * 5] == 0);
}
