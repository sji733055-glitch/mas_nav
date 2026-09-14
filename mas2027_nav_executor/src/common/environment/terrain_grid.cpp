#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace mas2027_nav_executor {
namespace {
constexpr int kOccupiedCost = 95;
constexpr double kMinAlignment = 0.85;
constexpr double kPi = 3.14159265358979323846;
}

void TerrainGrid::updateCost(const nav_msgs::msg::OccupancyGrid & grid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cost_ = grid;
  refresh();
  revision_.fetch_add(1, std::memory_order_release);
}

void TerrainGrid::updateDirection(const sensor_msgs::msg::Image & image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  direction_ = image;
  refresh();
  revision_.fetch_add(1, std::memory_order_release);
}

void TerrainGrid::updateDynamic(const nav_msgs::msg::OccupancyGrid & grid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto previous = dynamic_snapshot_;
  dynamic_snapshot_.reset();
  if (!snapshot_ || grid.header.frame_id != snapshot_->cost.header.frame_id ||
    grid.info.width != snapshot_->cost.info.width || grid.info.height != snapshot_->cost.info.height ||
    std::abs(grid.info.resolution - snapshot_->cost.info.resolution) > 1e-6 ||
    std::abs(grid.info.origin.position.x - snapshot_->cost.info.origin.position.x) > 1e-6 ||
    std::abs(grid.info.origin.position.y - snapshot_->cost.info.origin.position.y) > 1e-6 ||
    grid.data.size() != snapshot_->cost.data.size()) {
    if (previous) revision_.fetch_add(1, std::memory_order_release);
    return;
  }
  dynamic_snapshot_ = std::make_shared<DynamicSnapshot>(DynamicSnapshot{grid});
  if (!previous || previous->grid.data != grid.data) {
    revision_.fetch_add(1, std::memory_order_release);
  }
}

std::shared_ptr<const TerrainGrid::DynamicSnapshot> TerrainGrid::dynamicSnapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dynamic_snapshot_;
}

bool TerrainGrid::DynamicSnapshot::freeAt(const Eigen::Vector2d & point) const
{
  if (!point.allFinite()) return false;
  const double x = (point.x() - grid.info.origin.position.x) / grid.info.resolution;
  const double y = (point.y() - grid.info.origin.position.y) / grid.info.resolution;
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0 ||
    x >= grid.info.width || y >= grid.info.height) return false;
  const size_t index = static_cast<size_t>(std::floor(y)) * grid.info.width +
    static_cast<size_t>(std::floor(x));
  return grid.data[index] >= 0 && grid.data[index] < kOccupiedCost;
}

void TerrainGrid::refresh()
{
  snapshot_.reset();
  const auto width = cost_.info.width;
  const auto height = cost_.info.height;
  if (width == 0 || height == 0 || !std::isfinite(cost_.info.resolution) ||
    cost_.info.resolution <= 0.0F ||
    cost_.data.size() != static_cast<size_t>(width) * height ||
    direction_.width != width || direction_.height != height ||
    direction_.encoding != "bgr8" || direction_.step < width * 3U ||
    direction_.data.size() < static_cast<size_t>(direction_.step) * height ||
    cost_.header.frame_id.empty() || cost_.header.frame_id != direction_.header.frame_id ||
    std::abs(cost_.info.origin.orientation.x) > 1e-6 ||
    std::abs(cost_.info.origin.orientation.y) > 1e-6 ||
    std::abs(cost_.info.origin.orientation.z) > 1e-6 ||
    std::abs(cost_.info.origin.orientation.w - 1.0) > 1e-6) {
    return;
  }
  snapshot_ = std::make_shared<Snapshot>(Snapshot{cost_, direction_});
}

std::shared_ptr<const TerrainGrid::Snapshot> TerrainGrid::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

std::optional<nav_msgs::msg::OccupancyGrid> TerrainGrid::planningConstraints() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!snapshot_) return std::nullopt;

  nav_msgs::msg::OccupancyGrid result = snapshot_->cost;
  const auto & direction = snapshot_->direction;
  for (size_t index = 0; index < result.data.size(); ++index) {
    const int8_t cost = snapshot_->cost.data[index];
    if (cost < 0 || cost >= kOccupiedCost) {
      result.data[index] = 100;
      continue;
    }
    const size_t pixel = (index / result.info.width) * direction.step +
      (index % result.info.width) * 3U;
    const uint8_t magnitude = direction.data[pixel + 1U];
    const uint8_t label = direction.data[pixel + 2U];
    result.data[index] = magnitude == 255U && label >= 2U && label <= 6U ? 50 : 0;
  }
  if (!dynamic_snapshot_) return result;
  // Merge the current dynamic layer with static terrain for the shared 2D distance field.
  const auto & dynamic = dynamic_snapshot_->grid;
  result.header.stamp = dynamic.header.stamp;
  if (dynamic.data.size() == result.data.size()) {
    for (size_t index = 0; index < result.data.size(); ++index) {
      if (dynamic.data[index] >= kOccupiedCost) {
        result.data[index] = 100;
      }
    }
  }
  return result;
}

bool TerrainGrid::Snapshot::cell(const Eigen::Vector2d & point, int & x, int & y) const
{
  if (!point.allFinite()) return false;
  const double res = cost.info.resolution;
  const double fx = (point.x() - cost.info.origin.position.x) / res;
  const double fy = (point.y() - cost.info.origin.position.y) / res;
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx < 0.0 || fy < 0.0 ||
    fx >= cost.info.width || fy >= cost.info.height) return false;
  x = static_cast<int>(std::floor(fx));
  y = static_cast<int>(std::floor(fy));
  return true;
}

Eigen::Vector2d TerrainGrid::Snapshot::center(int x, int y) const
{
  const double res = cost.info.resolution;
  return {cost.info.origin.position.x + (x + 0.5) * res,
    cost.info.origin.position.y + (y + 0.5) * res};
}

bool TerrainGrid::Snapshot::permitted(int x, int y, const Eigen::Vector2d & travel) const
{
  const size_t index = static_cast<size_t>(y) * cost.info.width + x;
  if (cost.data[index] < 0 || cost.data[index] >= kOccupiedCost) return false;
  const size_t pixel = static_cast<size_t>(y) * direction.step + static_cast<size_t>(x) * 3U;
  const uint8_t angle = direction.data[pixel];
  const uint8_t magnitude = direction.data[pixel + 1U];
  const uint8_t label = direction.data[pixel + 2U];
  if (magnitude != 255U) return true;  // Inflated direction halo is soft, not a terrain body.
  if (label < 2U || label > 6U || travel.squaredNorm() < 1e-12) return false;
  const double radians = static_cast<double>(angle) * 2.0 * kPi / 255.0;
  const double alignment = travel.normalized().dot(Eigen::Vector2d(std::cos(radians), std::sin(radians)));
  return std::abs(alignment) >= kMinAlignment && (label != 6U || alignment > 0.0);
}

bool TerrainGrid::Snapshot::contains(const Eigen::Vector2d & point) const
{
  int x = 0, y = 0;
  return cell(point, x, y);
}

bool TerrainGrid::Snapshot::traversable(const Eigen::Vector2d & point) const
{
  int x = 0, y = 0;
  if (!cell(point, x, y)) return false;
  const size_t index = static_cast<size_t>(y) * cost.info.width + x;
  return cost.data[index] >= 0 && cost.data[index] < kOccupiedCost;
}

bool TerrainGrid::Snapshot::transition(
  const Eigen::Vector2d & from, const Eigen::Vector2d & to) const
{
  const Eigen::Vector2d delta = to - from;
  const double length = delta.norm();
  if (!std::isfinite(length) || length < 1e-9) return traversable(from);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (0.5 * cost.info.resolution))));
  for (int i = 0; i <= steps; ++i) {
    const Eigen::Vector2d point = from + delta * (static_cast<double>(i) / steps);
    int x = 0, y = 0;
    if (!cell(point, x, y) || !permitted(x, y, delta)) return false;
  }
  return true;
}

bool TerrainGrid::Snapshot::search(const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal, std::vector<Eigen::Vector2d> & path,
  const std::function<bool(const Eigen::Vector2d &)> & dynamic_free) const
{
  path.clear();
  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!cell(start, sx, sy) || !cell(goal, gx, gy) ||
    !traversable(start) || !traversable(goal) ||
    (dynamic_free && (!dynamic_free(start) || !dynamic_free(goal)))) return false;
  if (sx == gx && sy == gy) {
    if (!transition(start, goal)) return false;
    path = {start, goal};
    return true;
  }
  const int width = static_cast<int>(cost.info.width);
  const int height = static_cast<int>(cost.info.height);
  const int count = width * height;
  const auto index = [width](int x, int y) { return y * width + x; };
  const int source = index(sx, sy), target = index(gx, gy);
  const auto edge_free = [&](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
    if (!transition(a, b)) return false;
    if (!dynamic_free) return true;
    const int steps = std::max(1, static_cast<int>(std::ceil(
      (b - a).norm() / (0.5 * cost.info.resolution))));
    for (int i = 0; i <= steps; ++i) {
      if (!dynamic_free(a + (b - a) * (static_cast<double>(i) / steps))) return false;
    }
    return true;
  };
  const auto heuristic = [gx, gy](int x, int y) {
    const int dx = std::abs(x - gx), dy = std::abs(y - gy);
    return static_cast<double>(std::max(dx, dy)) + 0.4142135623730951 * std::min(dx, dy);
  };
  using Candidate = std::pair<double, int>;
  std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> open;
  std::vector<double> g(count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(count, -1);
  std::vector<uint8_t> closed(count, 0);
  g[source] = 0.0;
  open.emplace(heuristic(sx, sy), source);
  constexpr std::array<std::pair<int, int>, 8> neighbors{{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();
    if (closed[current]) continue;
    if (current == target) break;
    closed[current] = 1;
    const int x = current % width, y = current / width;
    for (const auto & [dx, dy] : neighbors) {
      const int nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
      const int next = index(nx, ny);
      if (closed[next] || !edge_free(center(x, y), center(nx, ny))) continue;
      // No diagonal corner cutting, even if the center-to-center ray misses the corner cells.
      if (dx != 0 && dy != 0 &&
        (!edge_free(center(x, y), center(x + dx, y)) ||
        !edge_free(center(x, y), center(x, y + dy)))) continue;
      const double step = std::hypot(dx, dy);
      const double cost_penalty = std::max(0, static_cast<int>(cost.data[next])) / 100.0;
      const double next_g = g[current] + step * (1.0 + cost_penalty);
      if (next_g >= g[next]) continue;
      g[next] = next_g;
      parent[next] = current;
      open.emplace(next_g + heuristic(nx, ny), next);
    }
  }
  if (parent[target] < 0) return false;
  for (int cursor = target; cursor != source; cursor = parent[cursor]) {
    if (cursor < 0) return false;
    path.push_back(center(cursor % width, cursor / width));
  }
  path.push_back(center(sx, sy));
  std::reverse(path.begin(), path.end());
  if (!edge_free(start, path.front()) || !edge_free(path.back(), goal)) {
    path.clear();
    return false;
  }
  path.front() = start;
  path.back() = goal;
  return true;
}

}  // namespace mas2027_nav_executor
