#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <vector>

#include "mas2027_nav_executor/path_planner/trajectory/arc_length_speed_profile.hpp"

int main()
{
  using mas2027_nav_executor::buildArcLengthSpeedProfile;
  std::vector<double> speeds, times;
  assert(buildArcLengthSpeedProfile({1.0, 1.0}, {2.0, 2.0}, 0.0, true, 1.0,
    speeds, times));
  assert(speeds.size() == 3U && times.size() == 2U);
  assert(std::abs(speeds[1] - std::sqrt(2.0)) < 1e-6);
  assert(speeds.back() == 0.0);
  assert(std::abs(times[0] - times[1]) < 1e-6);

  assert(buildArcLengthSpeedProfile({2.0}, {1.0}, 0.0, true, 1.0,
    speeds, times));
  assert(std::abs(times[0] - 3.0) < 1e-6);  // accelerate, cruise, brake
  assert(!buildArcLengthSpeedProfile({0.1}, {3.0}, 2.0, true, 1.0,
    speeds, times));  // insufficient stopping distance
  assert(!buildArcLengthSpeedProfile({-1.0}, {1.0}, 0.0, true, 1.0,
    speeds, times));
}
