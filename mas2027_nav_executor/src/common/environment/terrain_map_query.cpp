#include "mas2027_nav_executor/common/environment/terrain_map_query.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace mas2027_nav_executor {
namespace {

bool sampleBilinear(
  const std::vector<double> & distances, unsigned int width, unsigned int height, double resolution,
  int ix, int iy, double fx, double fy, double & dist, Eigen::Vector3d & grad)
{
  if (ix < 0 || iy < 0 || ix + 1 >= static_cast<int>(width) || iy + 1 >= static_cast<int>(height) ||
    resolution <= 0.0)
  {
    return false;
  }
  const size_t w = static_cast<size_t>(width);
  const size_t idx00 = static_cast<size_t>(iy) * w + static_cast<size_t>(ix);
  const size_t idx10 = static_cast<size_t>(iy) * w + static_cast<size_t>(ix + 1);
  const size_t idx01 = static_cast<size_t>(iy + 1) * w + static_cast<size_t>(ix);
  const size_t idx11 = static_cast<size_t>(iy + 1) * w + static_cast<size_t>(ix + 1);
  const double d00 = distances[idx00];
  const double d10 = distances[idx10];
  const double d01 = distances[idx01];
  const double d11 = distances[idx11];
  if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) || !std::isfinite(d11)) {
    return false;
  }
  const double y0 = (1.0 - fx) * d00 + fx * d10;
  const double y1 = (1.0 - fx) * d01 + fx * d11;
  dist = (1.0 - fy) * y0 + fy * y1;
  grad.x() = ((1.0 - fy) * (d10 - d00) + fy * (d11 - d01)) / resolution;
  grad.y() = ((1.0 - fx) * (d01 - d00) + fx * (d11 - d10)) / resolution;
  grad.z() = 0.0;
  return std::isfinite(dist) && grad.allFinite();
}

}  // namespace

TerrainMapQuery::TerrainMapQuery(std::shared_ptr<TerrainGrid> terrain)
: terrain_(std::move(terrain))
{
}

std::shared_ptr<const TerrainMapQuery::FieldSnapshot> TerrainMapQuery::snapshot() const
{
  refresh();
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

bool TerrainMapQuery::refresh() const
{
  if (!terrain_) {
    return false;
  }
  const uint64_t revision = terrain_->revision();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot_ && snapshot_->revision == revision) return true;
  }
  const auto grid = terrain_->planningConstraints();
  if (!grid || grid->info.width == 0U || grid->info.height == 0U ||
    grid->info.resolution <= 0.0f ||
    grid->data.size() != static_cast<size_t>(grid->info.width) * grid->info.height)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.reset();
    return false;
  }

  auto next = std::make_shared<FieldSnapshot>();
  next->width = grid->info.width;
  next->height = grid->info.height;
  next->resolution = grid->info.resolution;
  next->origin_x = grid->info.origin.position.x;
  next->origin_y = grid->info.origin.position.y;
  next->revision = revision;
  next->values.assign(static_cast<size_t>(next->width) * next->height, kLethalCost);
  next->distances.assign(next->values.size(), kMaxDistance);

  cv::Mat free_mask(static_cast<int>(next->height), static_cast<int>(next->width), CV_8UC1);
  for (unsigned int y = 0; y < next->height; ++y) {
    for (unsigned int x = 0; x < next->width; ++x) {
      const size_t idx = static_cast<size_t>(y) * next->width + x;
      const int8_t cost = grid->data[idx];
      const bool occupied = cost < 0 || cost >= kOccupiedCost;
      next->values[idx] = occupied ? kLethalCost : kFreeCost;
      free_mask.at<uint8_t>(static_cast<int>(y), static_cast<int>(x)) = occupied ? 0U : 255U;
    }
  }

  cv::Mat occ_mask;
  cv::bitwise_not(free_mask, occ_mask);
  cv::Mat dist_pos;
  cv::Mat dist_neg;
  cv::distanceTransform(free_mask, dist_pos, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  cv::distanceTransform(occ_mask, dist_neg, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  for (unsigned int y = 0; y < next->height; ++y) {
    for (unsigned int x = 0; x < next->width; ++x) {
      const size_t idx = static_cast<size_t>(y) * next->width + x;
      const float pos = dist_pos.at<float>(static_cast<int>(y), static_cast<int>(x));
      const float neg = dist_neg.at<float>(static_cast<int>(y), static_cast<int>(x));
      double distance = next->values[idx] == kFreeCost
        ? static_cast<double>(pos) * next->resolution
        : -static_cast<double>(neg) * next->resolution;
      distance = std::clamp(distance, kMinDistance, kMaxDistance);
      next->distances[idx] = distance;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!snapshot_ || next->revision >= snapshot_->revision) snapshot_ = std::move(next);
  return true;
}

bool TerrainMapQuery::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  const auto snap = snapshot();
  if (!snap || snap->resolution <= 0.0) {
    return false;
  }
  if (wx < snap->origin_x || wy < snap->origin_y) {
    return false;
  }
  const int ix = static_cast<int>(std::floor((wx - snap->origin_x) / snap->resolution));
  const int iy = static_cast<int>(std::floor((wy - snap->origin_y) / snap->resolution));
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(snap->width) || iy >= static_cast<int>(snap->height)) {
    return false;
  }
  mx = static_cast<unsigned int>(ix);
  my = static_cast<unsigned int>(iy);
  return true;
}

void TerrainMapQuery::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  const auto snap = snapshot();
  if (!snap || snap->resolution <= 0.0) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  wx = snap->origin_x + (static_cast<double>(mx) + 0.5) * snap->resolution;
  wy = snap->origin_y + (static_cast<double>(my) + 0.5) * snap->resolution;
}

unsigned int TerrainMapQuery::sizeX() const
{
  const auto snap = snapshot();
  return snap ? snap->width : 0U;
}

unsigned int TerrainMapQuery::sizeY() const
{
  const auto snap = snapshot();
  return snap ? snap->height : 0U;
}

double TerrainMapQuery::resolution() const
{
  const auto snap = snapshot();
  return snap ? snap->resolution : 0.0;
}

double TerrainMapQuery::originX() const
{
  const auto snap = snapshot();
  return snap ? snap->origin_x : 0.0;
}

double TerrainMapQuery::originY() const
{
  const auto snap = snapshot();
  return snap ? snap->origin_y : 0.0;
}

uint8_t TerrainMapQuery::value(unsigned int mx, unsigned int my) const
{
  const auto snap = snapshot();
  if (!snap || !isValid(mx, my)) {
    return kLethalCost;
  }
  return snap->values[static_cast<size_t>(my) * snap->width + mx];
}

const unsigned char * TerrainMapQuery::values() const
{
  const auto snap = snapshot();
  return snap && !snap->values.empty() ? snap->values.data() : nullptr;
}

bool TerrainMapQuery::isValid(unsigned int mx, unsigned int my) const
{
  const auto snap = snapshot();
  return snap && mx < snap->width && my < snap->height;
}

bool TerrainMapQuery::isFree(unsigned int mx, unsigned int my) const
{
  return isValid(mx, my) && value(mx, my) < 253U;
}

rog_map::QueryResult TerrainMapQuery::query(const Eigen::Vector3d & pos) const
{
  rog_map::QueryResult result;
  if (!pos.allFinite()) {
    result.status = rog_map::QueryStatus::NONFINITE_INPUT;
    return result;
  }
  const auto snap = snapshot();
  if (!snap || snap->width <= 1U || snap->height <= 1U || snap->resolution <= 0.0 ||
    snap->distances.size() != static_cast<size_t>(snap->width) * snap->height)
  {
    result.status = snap ? rog_map::QueryStatus::FIELD_UNINITIALIZED
                         : rog_map::QueryStatus::SNAPSHOT_INVALID;
    return result;
  }

  // OpenCV's distance samples belong to cell centers, not cell corners.
  const double px = (pos.x() - snap->origin_x) / snap->resolution - 0.5;
  const double py = (pos.y() - snap->origin_y) / snap->resolution - 0.5;
  const int ix = static_cast<int>(std::floor(px));
  const int iy = static_cast<int>(std::floor(py));
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(snap->width) - 1 ||
    iy >= static_cast<int>(snap->height) - 1)
  {
    result.status = rog_map::QueryStatus::OUT_OF_MAP;
    return result;
  }

  double dist = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  if (!sampleBilinear(
      snap->distances, snap->width, snap->height, snap->resolution, ix, iy,
      px - static_cast<double>(ix), py - static_cast<double>(iy), dist, grad))
  {
    result.status = rog_map::QueryStatus::INTERPOLATION_FAILED;
    return result;
  }

  const double unclamped = dist;
  dist = std::clamp(dist, kMinDistance, kMaxDistance);
  if (unclamped != dist) {
    grad.setZero();
  }
  if (!std::isfinite(dist) || !grad.allFinite()) {
    result.status = rog_map::QueryStatus::NONFINITE_OUTPUT;
    return result;
  }

  result.ok = true;
  result.status = rog_map::QueryStatus::OK;
  result.distance = dist;
  result.gradient = grad;
  return result;
}

bool TerrainMapQuery::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  const auto result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

}  // namespace mas2027_nav_executor
