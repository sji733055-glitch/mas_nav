#pragma once

#include <array>
#include <cmath>
#include <rog_map/terrain_config.hpp>

namespace rog_map {

// Find a measured monotonic run containing the queried sample. Flat ground,
// missing support and a reversal terminate the run; they never add length.
struct SlopeRun {
  int first;
  int last;
  double length;
  double height;
};

template <std::size_t N>
SlopeRun slopeRunAt(const std::array<double, N> &z, int center, double spacing,
                    double min_gradient, double max_gradient) {
  const auto sign = [&](int i) {
    if (i < 0 || i + 1 >= static_cast<int>(N) ||
        !std::isfinite(z[i]) || !std::isfinite(z[i + 1])) return 0;
    const double gradient = (z[i + 1] - z[i]) / spacing;
    if (std::abs(gradient) < min_gradient ||
        std::abs(gradient) > max_gradient) return 0;
    return gradient > 0 ? 1 : -1;
  };
  SlopeRun best{center, center, 0., 0.};
  for (int direction : {-1, 1}) {
    const int edge = direction < 0 ? center - 1 : center;
    const int grade = sign(edge);
    if (!grade) continue;
    int first = edge, last = edge + 1;
    while (first > 0 && sign(first - 1) == grade) --first;
    while (last + 1 < static_cast<int>(N) && sign(last) == grade) ++last;
    const double length = (last - first) * spacing;
    if (length > best.length)
      best = {first, last, length, std::abs(z[last] - z[first])};
  }
  return best;
}

template <std::size_t N>
bool isShortSlopeEntry(const std::array<double, N> &z, int center,
                       double spacing, const UndulatingRecognitionConfig &config) {
  const auto run = slopeRunAt(z, center, spacing, config.entry_min_gradient,
                              config.entry_max_gradient);
  // Both ends must be observed to measure a short slope. This does not require
  // a reverse slope: an observed level continuation also bounds the length.
  return config.entry_window_enable && run.last - run.first >= 2 &&
         run.first > 0 && run.last + 1 < static_cast<int>(N) &&
         std::isfinite(z[run.first - 1]) && std::isfinite(z[run.last + 1]) &&
         run.length <= config.entry_max_slope_length_m + 1e-9 &&
         run.height >= config.min_peak_to_trough_m &&
         run.height <= config.max_peak_to_trough_m;
}

} // namespace rog_map
