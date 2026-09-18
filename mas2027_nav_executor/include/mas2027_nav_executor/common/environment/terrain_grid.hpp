#pragma once

#include <memory>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace mas2027_nav_executor {

// Immutable snapshots keep the planning thread independent of map callbacks.
class TerrainGrid final
{
public:
  void updateCost(const nav_msgs::msg::OccupancyGrid & grid);
  void updateDirection(const sensor_msgs::msg::Image & image);
  void updateDynamic(const nav_msgs::msg::OccupancyGrid & grid);

  struct DynamicSnapshot
  {
    nav_msgs::msg::OccupancyGrid grid;
    bool freeAt(const Eigen::Vector2d & point) const;
  };
  std::shared_ptr<const DynamicSnapshot> dynamicSnapshot() const;
  uint64_t revision() const noexcept { return revision_.load(std::memory_order_acquire); }

  struct Snapshot
  {
    nav_msgs::msg::OccupancyGrid cost;
    sensor_msgs::msg::Image direction;

    bool contains(const Eigen::Vector2d & point) const;
    bool traversable(const Eigen::Vector2d & point) const;
    bool transition(const Eigen::Vector2d & from, const Eigen::Vector2d & to) const;
    bool search(const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
      std::vector<Eigen::Vector2d> & path,
      const std::function<bool(const Eigen::Vector2d &)> & dynamic_free = {}) const;

  private:
    bool cell(const Eigen::Vector2d & point, int & x, int & y) const;
    Eigen::Vector2d center(int x, int y) const;
    bool permitted(int x, int y, const Eigen::Vector2d & travel) const;
  };

  std::shared_ptr<const Snapshot> snapshot() const;

  // RViz-facing map of the exact hard/directional constraints used by the planner.
  // 100: currently blocked; 50: direction-limited terrain.
  std::optional<nav_msgs::msg::OccupancyGrid> planningConstraints() const;

private:
  void refresh();
  mutable std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid cost_;
  sensor_msgs::msg::Image direction_;
  std::shared_ptr<const Snapshot> snapshot_;
  std::shared_ptr<const DynamicSnapshot> dynamic_snapshot_;
  std::atomic<uint64_t> revision_{0};
};

}  // namespace mas2027_nav_executor
