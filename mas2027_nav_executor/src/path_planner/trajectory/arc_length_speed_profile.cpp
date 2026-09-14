#include "mas2027_nav_executor/path_planner/trajectory/arc_length_speed_profile.hpp"

#include <algorithm>
#include <cmath>

namespace mas2027_nav_executor {

bool buildArcLengthSpeedProfile(
  const std::vector<double> & lengths,
  const std::vector<double> & segment_speed_limits,
  double start_speed,
  bool stop_at_end,
  double max_acceleration,
  std::vector<double> & node_speeds,
  std::vector<double> & segment_times)
{
  node_speeds.clear();
  segment_times.clear();
  const size_t count = lengths.size();
  if (count == 0 || count != segment_speed_limits.size() || !std::isfinite(start_speed) ||
    start_speed < 0.0 || !std::isfinite(max_acceleration) || max_acceleration <= 0.0) return false;
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(lengths[i]) || lengths[i] < 0.0 ||
      !std::isfinite(segment_speed_limits[i]) || segment_speed_limits[i] <= 0.0) return false;
  }

  std::vector<double> cap(count + 1U);
  cap[0] = segment_speed_limits[0];
  for (size_t i = 1; i < count; ++i) {
    cap[i] = std::min(segment_speed_limits[i - 1], segment_speed_limits[i]);
  }
  cap[count] = stop_at_end ? 0.0 : segment_speed_limits.back();
  for (size_t i = count; i-- > 0;) {
    cap[i] = std::min(cap[i], std::sqrt(cap[i + 1] * cap[i + 1] +
      2.0 * max_acceleration * lengths[i]));
  }
  if (start_speed > cap[0] + 1e-6) return false;

  node_speeds.resize(count + 1U);
  segment_times.resize(count);
  node_speeds[0] = start_speed;
  for (size_t i = 0; i < count; ++i) {
    node_speeds[i + 1] = std::min(cap[i + 1], std::sqrt(
      node_speeds[i] * node_speeds[i] + 2.0 * max_acceleration * lengths[i]));
    if (lengths[i] <= 1e-9) {
      segment_times[i] = 0.1;
      continue;
    }
    const double v0 = node_speeds[i], v1 = node_speeds[i + 1];
    const double peak = std::min(segment_speed_limits[i], std::sqrt(
      max_acceleration * lengths[i] + 0.5 * (v0 * v0 + v1 * v1)));
    if (peak + 1e-6 < std::max(v0, v1)) return false;
    const double accelerate = std::max(0.0, (peak * peak - v0 * v0) /
      (2.0 * max_acceleration));
    const double decelerate = std::max(0.0, (peak * peak - v1 * v1) /
      (2.0 * max_acceleration));
    const double cruise = std::max(0.0, lengths[i] - accelerate - decelerate);
    segment_times[i] = std::max(0.1, (peak - v0) / max_acceleration +
      (peak - v1) / max_acceleration + cruise / std::max(peak, 1e-6));
    if (!std::isfinite(segment_times[i])) return false;
  }
  return true;
}

}  // namespace mas2027_nav_executor
