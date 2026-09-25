#include <rog_map/projection_layer.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr int kSize = 21;
constexpr double kResolution = 0.05;

rog_map::ColumnStats ground(double z)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 10;
  stats.occupied_count = 1;
  stats.occupied_z_index_min = static_cast<int>(std::round(z / kResolution));
  stats.occupied_z_index_max = stats.occupied_z_index_min;
  stats.occupied_z_min_abs = z;
  stats.occupied_z_max_abs = z;
  return stats;
}

std::vector<uint8_t> labelsFor(const rog_map::ProjectionLayer::ColumnScanner & scanner)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
    config, scanner);
  return layer.terrainLabels();
}

TEST(ProjectionTerrain, FlatGroundIsNotSlope)
{
  const auto labels = labelsFor([](int, int) { return ground(1.0); });
  EXPECT_EQ(labels[10 * kSize + 10], 0U);
}

TEST(ProjectionTerrain, ContinuousInclineIsSlope)
{
  const auto labels = labelsFor([](int x, int) {
    return ground(1.0 + 0.2 * x * kResolution);
  });
  EXPECT_EQ(labels[10 * kSize + 10], 5U);
}

TEST(ProjectionTerrain, ObservedGapWithNeighborSupportIsTunnel)
{
  const auto labels = labelsFor([](int, int) {
    auto stats = ground(0.0);
    stats.observed_count = 21;
    stats.occupied_count = 2;
    stats.occupied_z_index_min = 0;
    stats.occupied_z_index_max = 20;
    stats.occupied_z_max_abs = 1.0;
    stats.free_between_occupied_count = 19;
    return stats;
  });
  EXPECT_EQ(labels[10 * kSize + 10], 6U);
}

TEST(ProjectionTerrain, UnknownGapIsNotTunnel)
{
  const auto labels = labelsFor([](int, int) {
    auto stats = ground(0.0);
    stats.observed_count = 2;
    stats.occupied_count = 2;
    stats.occupied_z_index_min = 0;
    stats.occupied_z_index_max = 20;
    stats.occupied_z_max_abs = 1.0;
    return stats;
  });
  EXPECT_NE(labels[10 * kSize + 10], 6U);
}

}  // namespace
