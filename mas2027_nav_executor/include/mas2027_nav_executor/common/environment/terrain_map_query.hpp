#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>

#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "minco_core/map_query_interface.hpp"

namespace mas2027_nav_executor {

// MapQueryInterface backed by the static cost/direction terrain in TerrainGrid.
// Builds a 2D signed distance field from planningConstraints() for global search.
class TerrainMapQuery final : public rog_map::MapQueryInterface
{
public:
  explicit TerrainMapQuery(std::shared_ptr<TerrainGrid> terrain);

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;

  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;

  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;

  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

private:
  struct FieldSnapshot
  {
    unsigned int width{0};
    unsigned int height{0};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    uint64_t revision{0};
    std::vector<unsigned char> values;
    std::vector<double> distances;
  };

  bool refresh() const;
  std::shared_ptr<const FieldSnapshot> snapshot() const;

  std::shared_ptr<TerrainGrid> terrain_;
  mutable std::mutex mutex_;
  mutable std::shared_ptr<const FieldSnapshot> snapshot_;

  static constexpr int kOccupiedCost = 95;
  static constexpr uint8_t kLethalCost = 254U;
  static constexpr uint8_t kFreeCost = 0U;
  static constexpr double kMaxDistance = 3.0;
  static constexpr double kMinDistance = -1.0;
};

}  // namespace mas2027_nav_executor
