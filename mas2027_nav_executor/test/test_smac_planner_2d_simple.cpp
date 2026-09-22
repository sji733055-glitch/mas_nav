#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "mas2027_nav_executor/common/environment/terrain_map_query.hpp"
#include "mas2027_nav_executor/path_planner/search/smac/smac_planner_2d_simple.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

constexpr unsigned int kWidth = 40U;
constexpr unsigned int kHeight = 20U;
constexpr float kResolution = 0.1F;

using mas2027_nav_executor::TerrainGrid;
using mas2027_nav_executor::TerrainMapQuery;
using mas2027_nav_executor::smac::SmacPlanner2DSimple;
using Cell = std::pair<unsigned int, unsigned int>;
using Path = std::vector<SmacPlanner2DSimple::Coordinates>;

std::shared_ptr<TerrainGrid> makeTerrain(const std::vector<Cell> & blocked)
{
  nav_msgs::msg::OccupancyGrid cost;
  cost.header.frame_id = "map";
  cost.header.stamp.sec = 1;
  cost.info.width = kWidth;
  cost.info.height = kHeight;
  cost.info.resolution = kResolution;
  cost.info.origin.orientation.w = 1.0;
  cost.data.assign(static_cast<size_t>(kWidth) * kHeight, 0);
  for (const auto & cell : blocked) {
    cost.data[static_cast<size_t>(cell.second) * kWidth + cell.first] = 100;
  }

  sensor_msgs::msg::Image direction;
  direction.header = cost.header;
  direction.width = kWidth;
  direction.height = kHeight;
  direction.encoding = "bgr8";
  direction.step = kWidth * 3U;
  direction.data.assign(static_cast<size_t>(kWidth) * kHeight * 3U, 0);

  auto terrain = std::make_shared<TerrainGrid>();
  terrain->updateCost(cost);
  terrain->updateDirection(direction);
  return terrain;
}

// tolerance 取一个栅格，使得到点判定正好落在 goal 或与 goal 相邻的格子上。
void configureSearch(
  SmacPlanner2DSimple & smac, const std::shared_ptr<TerrainMapQuery> & query,
  bool use_esdf, double esdf_weight, double esdf_max_cost)
{
  smac.configure(rclcpp::get_logger("test_smac"));
  smac.setParameters(false, 1000000, kResolution);
  smac.setESDFParameters(use_esdf, esdf_weight, 0.8, esdf_max_cost);
  smac.setMap(query);
  smac.setESDFQuery(query);
}

// 返回 true 时 path 已翻转成 start -> goal（SMAC 内部输出是 goal -> start）。
bool search(
  SmacPlanner2DSimple & smac, unsigned int sx, unsigned int sy,
  unsigned int gx, unsigned int gy, Path & path)
{
  SmacPlanner2DSimple::CoordinateVector raw;
  if (!smac.createPath(sx, sy, gx, gy, raw) || raw.size() < 2U) {
    return false;
  }
  path.assign(raw.rbegin(), raw.rend());
  return true;
}

double meanClearance(
  const std::shared_ptr<TerrainMapQuery> & query, const Path & path)
{
  if (path.empty()) {
    return -1.0;
  }
  double sum = 0.0;
  for (const auto & point : path) {
    double wx = 0.0;
    double wy = 0.0;
    query->mapToWorld(static_cast<unsigned int>(point.x), static_cast<unsigned int>(point.y), wx, wy);
    const auto result = query->query(Eigen::Vector3d(wx, wy, 0.0));
    if (!result.ok || !std::isfinite(result.distance)) {
      return -1.0;
    }
    sum += std::max(0.0, result.distance);
  }
  return sum / static_cast<double>(path.size());
}

// 第 20 列是墙，留出 y=10..13 的缺口。
std::vector<Cell> wallWithGap()
{
  std::vector<Cell> blocked;
  for (unsigned int y = 0; y < kHeight; ++y) {
    if (y >= 10U && y <= 13U) {
      continue;
    }
    blocked.emplace_back(20U, y);
  }
  return blocked;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // --- 1. 有缺口时必须绕行通过缺口 -------------------------------------------------
  {
    auto terrain = makeTerrain(wallWithGap());
    auto query = std::make_shared<TerrainMapQuery>(terrain);
    SmacPlanner2DSimple smac;
    configureSearch(smac, query, false, 1.0, 0.5);

    Path path;
    assert(search(smac, 5U, 5U, 35U, 5U, path));

    assert(static_cast<unsigned int>(path.front().x) == 5U);
    assert(static_cast<unsigned int>(path.front().y) == 5U);
    assert(std::hypot(path.back().x - 35.0F, path.back().y - 5.0F) <= 1.5F);

    // 路径上不能有障碍格。
    float max_y = 0.0F;
    for (const auto & point : path) {
      const auto mx = static_cast<unsigned int>(point.x);
      const auto my = static_cast<unsigned int>(point.y);
      assert(query->value(mx, my) == 0U);
      max_y = std::max(max_y, point.y);
    }
    // 只能从缺口翻过去：y 必须爬到缺口所在的 10..13 行。
    assert(max_y >= 10.0F);
  }

  // --- 2. 目标被墙封死时必须失败（且不自动回退 Astar） -------------------------------
  {
    std::vector<Cell> sealed;
    for (unsigned int y = 0; y < kHeight; ++y) {
      sealed.emplace_back(20U, y);
    }
    auto terrain = makeTerrain(sealed);
    auto query = std::make_shared<TerrainMapQuery>(terrain);
    SmacPlanner2DSimple smac;
    configureSearch(smac, query, false, 1.0, 0.5);

    Path path;
    assert(!search(smac, 5U, 5U, 35U, 5U, path));
    assert(path.empty());
  }

  // --- 3. ESDF 势场软代价必须真的把路径推离墙面 -------------------------------------
  // 上下两行是墙夹出 1.8 m 走廊，起终点同贴下侧一行：不开 ESDF 的最短路贴墙，
  // 开启后平均净空须明显抬高；权重取远大于出厂 1.0/0.8/0.5，否则软偏置不会改变最短路拓扑。
  {
    std::vector<Cell> blocked;
    for (unsigned int x = 0; x < kWidth; ++x) {
      blocked.emplace_back(x, 0U);
      blocked.emplace_back(x, kHeight - 1U);
    }
    auto terrain = makeTerrain(blocked);
    auto query = std::make_shared<TerrainMapQuery>(terrain);

    SmacPlanner2DSimple without_esdf;
    configureSearch(without_esdf, query, false, 1.0, 0.5);
    Path straight;
    assert(search(without_esdf, 5U, 5U, 35U, 5U, straight));
    const double clearance_without = meanClearance(query, straight);
    assert(clearance_without > 0.0);

    SmacPlanner2DSimple with_esdf;
    configureSearch(with_esdf, query, true, 100.0, 0.0);
    Path bowed;
    assert(search(with_esdf, 5U, 5U, 35U, 5U, bowed));
    const double clearance_with = meanClearance(query, bowed);

    assert(clearance_with > clearance_without + 0.1);
  }

  // --- 4. 地图变化后必须基于新地图搜索，不能返回上一轮的陈旧路径 ---------------------
  // 回归：setMap() 会把 planning_id_ 归零，下一次 createPath 自增得到 1，与上一轮留在
  // visited_/closed_/parent_ 里的标记撞号，从而跳过 goal 判定并顺着旧 parent_ 返回旧路径。
  {
    auto open_terrain = makeTerrain(wallWithGap());
    auto open_query = std::make_shared<TerrainMapQuery>(open_terrain);
    SmacPlanner2DSimple smac;
    configureSearch(smac, open_query, false, 1.0, 0.5);

    Path first;
    assert(search(smac, 5U, 5U, 35U, 5U, first));
    assert(!first.empty());

    // 同一张图、同一个实例再搜一次：不能因为复用了上一轮的标记而出错。
    smac.setMap(open_query);
    Path repeat;
    assert(search(smac, 5U, 5U, 35U, 5U, repeat));
    assert(first.size() == repeat.size());

    // 缺口封死后目标不可达（栅格尺寸不变，所以搜索缓冲不会被重新分配，撞号条件成立）。
    std::vector<Cell> sealed;
    for (unsigned int y = 0; y < kHeight; ++y) {
      sealed.emplace_back(20U, y);
    }
    auto sealed_terrain = makeTerrain(sealed);
    auto sealed_query = std::make_shared<TerrainMapQuery>(sealed_terrain);
    smac.setMap(sealed_query);
    smac.setESDFQuery(sealed_query);

    Path after_seal;
    assert(!search(smac, 5U, 5U, 35U, 5U, after_seal));
    assert(after_seal.empty());
  }

  rclcpp::shutdown();
  return 0;
}
