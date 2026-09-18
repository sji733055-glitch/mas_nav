#pragma once

#include <vector>

namespace mas2027_nav_executor {

// Arc-length v² envelope for the MINCO time seed. The optimized polynomial
// remains the authoritative executable trajectory and is checked afterwards.
bool buildArcLengthSpeedProfile(
  const std::vector<double> & lengths,
  const std::vector<double> & segment_speed_limits,
  double start_speed,
  bool stop_at_end,
  double max_acceleration,
  std::vector<double> & node_speeds,
  std::vector<double> & segment_times);

}  // namespace mas2027_nav_executor
