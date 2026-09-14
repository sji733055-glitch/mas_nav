#include "mas2027_nav_executor/path_planner/search/omni_kino_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

namespace mas2027_nav_executor {
namespace {

constexpr std::array<std::pair<int, int>, 8> kMoves{{
  {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}}};

struct Key
{
  int x, y, heading, speed;
  bool operator==(const Key & other) const {
    return x == other.x && y == other.y && heading == other.heading && speed == other.speed;
  }
};

struct KeyHash
{
  size_t operator()(const Key & key) const {
    size_t hash = 0;
    for (const int value : {key.x, key.y, key.heading, key.speed}) {
      hash ^= std::hash<int>{}(value) + 0x9e3779b9U + (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

struct Node
{
  Key key;
  double cost;
  int parent;
  bool closed{false};
};

}  // namespace

bool searchOmniKinoPath(
  const TerrainGrid::Snapshot & terrain,
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal,
  const Eigen::Vector2d & start_velocity,
  double max_speed,
  double max_acceleration,
  const std::function<bool(const Eigen::Vector2d &)> & dynamic_free,
  std::vector<Eigen::Vector2d> & path)
{
  path.clear();
  if (!start.allFinite() || !goal.allFinite() || !start_velocity.allFinite() ||
    !std::isfinite(max_speed) || !std::isfinite(max_acceleration) ||
    max_speed <= 0.0 || max_acceleration <= 0.0 || !terrain.traversable(start) ||
    !terrain.traversable(goal) || (dynamic_free && (!dynamic_free(start) || !dynamic_free(goal)))) {
    return false;
  }

  // ponytail: fixed 0.1 m lattice and 0.1 m/s bins keep the search bounded;
  // refine both only if narrow-passage maps demonstrably require it.
  const double spacing = std::max(0.1, static_cast<double>(terrain.cost.info.resolution));
  constexpr double speed_step = 0.1;
  const int speed_bins = std::max(1, static_cast<int>(std::floor(max_speed / speed_step)));
  const auto position = [&](const Key & key) {
    return (start + spacing * Eigen::Vector2d(key.x, key.y)).eval();
  };
  const auto velocity = [&](const Key & key) {
    if (key.speed == 0) return Eigen::Vector2d::Zero().eval();
    const auto & move = kMoves[static_cast<size_t>(key.heading)];
    return (key.speed * speed_step / std::hypot(move.first, move.second) *
      Eigen::Vector2d(move.first, move.second)).eval();
  };
  const auto edge_free = [&](const Eigen::Vector2d & from, const Eigen::Vector2d & to) {
    if (!terrain.transition(from, to)) return false;
    if (!dynamic_free) return true;
    const int samples = std::max(1, static_cast<int>(std::ceil(
      (to - from).norm() / (0.5 * terrain.cost.info.resolution))));
    for (int i = 0; i <= samples; ++i) {
      if (!dynamic_free(from + (to - from) * (static_cast<double>(i) / samples))) return false;
    }
    return true;
  };

  using Candidate = std::pair<double, int>;
  std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> open;
  std::vector<Node> nodes;
  std::unordered_map<Key, int, KeyHash> indices;
  nodes.push_back({{0, 0, 0, 0}, 0.0, -1});
  indices.emplace(nodes.front().key, 0);
  open.emplace((goal - start).norm() / max_speed, 0);
  int expansions = 0;
  constexpr int max_expansions = 50000;
  while (!open.empty() && expansions < max_expansions) {
    const int current_index = open.top().second;
    open.pop();
    if (nodes[static_cast<size_t>(current_index)].closed) continue;
    nodes[static_cast<size_t>(current_index)].closed = true;
    ++expansions;
    const Key current = nodes[static_cast<size_t>(current_index)].key;
    const Eigen::Vector2d here = position(current);
    const Eigen::Vector2d previous_velocity = current_index == 0 ? start_velocity : velocity(current);
    const double remaining = (goal - here).norm();
    if (remaining <= spacing * 1.5 && edge_free(here, goal) &&
      previous_velocity.squaredNorm() <= 2.0 * max_acceleration * remaining + 1e-9) {
      for (int index = current_index; index >= 0; index = nodes[static_cast<size_t>(index)].parent) {
        path.push_back(position(nodes[static_cast<size_t>(index)].key));
      }
      std::reverse(path.begin(), path.end());
      if ((path.back() - goal).norm() > 1e-9) path.push_back(goal);
      return path.size() >= 2U;
    }
    for (int heading = 0; heading < 8; ++heading) {
      const auto [dx, dy] = kMoves[static_cast<size_t>(heading)];
      const Key next_base{current.x + dx, current.y + dy, heading, 0};
      const Eigen::Vector2d next_position = position(next_base);
      if (!terrain.contains(next_position) || !edge_free(here, next_position)) continue;
      // A diagonal may graze an occupied corner even when its center ray is free.
      if (dx != 0 && dy != 0 &&
        (!edge_free(here, here + spacing * Eigen::Vector2d(dx, 0)) ||
        !edge_free(here, here + spacing * Eigen::Vector2d(0, dy)))) continue;
      const int cell_x = static_cast<int>(std::floor(
        (next_position.x() - terrain.cost.info.origin.position.x) / terrain.cost.info.resolution));
      const int cell_y = static_cast<int>(std::floor(
        (next_position.y() - terrain.cost.info.origin.position.y) / terrain.cost.info.resolution));
      const int cell_cost = terrain.cost.data[static_cast<size_t>(cell_y) *
        terrain.cost.info.width + static_cast<size_t>(cell_x)];
      const double cost_factor = 1.0 + std::max(0, cell_cost) / 100.0;
      const double distance = (next_position - here).norm();
      for (int speed = 0; speed <= speed_bins; ++speed) {
        const Key next{next_base.x, next_base.y, heading, speed};
        const double next_speed = speed * speed_step;
        const double duration = 2.0 * distance / (previous_velocity.norm() + next_speed);
        if (!std::isfinite(duration) ||
          (velocity(next) - previous_velocity).norm() > max_acceleration * duration + 1e-9) {
          continue;
        }
        const double next_cost = nodes[static_cast<size_t>(current_index)].cost +
          duration * cost_factor;
        const auto existing = indices.find(next);
        int next_index;
        if (existing == indices.end()) {
          next_index = static_cast<int>(nodes.size());
          indices.emplace(next, next_index);
          nodes.push_back({next, next_cost, current_index});
        } else {
          next_index = existing->second;
          Node & node = nodes[static_cast<size_t>(next_index)];
          if (node.closed || next_cost >= node.cost) continue;
          node.cost = next_cost;
          node.parent = current_index;
        }
        open.emplace(next_cost + (goal - next_position).norm() / max_speed, next_index);
      }
    }
  }
  return false;
}

}  // namespace mas2027_nav_executor
