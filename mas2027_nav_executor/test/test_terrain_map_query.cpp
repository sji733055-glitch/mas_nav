#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

#include "mas2027_nav_executor/common/environment/terrain_map_query.hpp"

int main()
{
  using mas2027_nav_executor::TerrainGrid;
  nav_msgs::msg::OccupancyGrid cost;
  cost.header.frame_id = "map";
  cost.header.stamp.sec = 1;
  cost.info.width = 5;
  cost.info.height = 5;
  cost.info.resolution = 1.0F;
  cost.info.origin.orientation.w = 1.0;
  cost.data.assign(25, 0);
  cost.data[2 + 2 * 5] = 100;
  sensor_msgs::msg::Image direction;
  direction.header = cost.header;
  direction.width = 5;
  direction.height = 5;
  direction.encoding = "bgr8";
  direction.step = 15;
  direction.data.assign(75, 0);

  auto terrain = std::make_shared<TerrainGrid>();
  terrain->updateCost(cost);
  terrain->updateDirection(direction);
  mas2027_nav_executor::TerrainMapQuery query(terrain);
  const auto free = query.query({1.5, 2.5, 0.0});
  const auto occupied = query.query({2.5, 2.5, 0.0});
  assert(free.ok && free.distance > 0.0 && free.gradient.x() < 0.0);
  assert(occupied.ok && occupied.distance < 0.0);

  nav_msgs::msg::OccupancyGrid dynamic = cost;
  dynamic.header.stamp.nanosec = 1;
  dynamic.data.assign(25, 0);
  dynamic.data[3 + 2 * 5] = 100;
  terrain->updateDynamic(dynamic);
  assert(query.query({3.5, 2.5, 0.0}).distance < 0.0);
  const auto revision = terrain->revision();
  dynamic.header.stamp.nanosec = 2;
  terrain->updateDynamic(dynamic);
  assert(terrain->revision() == revision);  // New timestamp, same obstacles: reuse the field.
  dynamic.data[3 + 2 * 5] = 0;
  terrain->updateDynamic(dynamic);
  assert(terrain->revision() > revision);
  assert(query.query({3.5, 2.5, 0.0}).distance > 0.0);
}
