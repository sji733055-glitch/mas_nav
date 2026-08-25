#include <rog_map/esdf_utils.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace rog_map {
namespace {
constexpr double kInf = 1.0e20;

// 一次局部地图尺度的 EDT 只有 0.3~0.8 ms 的实际计算量，展开很宽的 OpenMP team
// 反而是净亏：fork/join 必须等最后一个 worker 被调度到才能结束，CPU 一旦被占满，
// 墙上时间跟的是调度延迟而不是计算量。在 20 核机器满载下实测（201x201，
// 一次 field 重建 = 两遍 EDT）：
//   1 线程 ~3.3 ms | 2 线程 ~1.9 ms | 4 线程 ~21 ms | 20 线程 ~58 ms
// 因此把 team 宽度压住，让这一段的耗时在 CPU 竞争时仍然有界。
constexpr int kMaxEdtThreads = 2;
constexpr int kMinCellsPerEdtThread = 16384;

inline int edtThreadCount(int width, int height)
{
  const long cells = static_cast<long>(width) * static_cast<long>(height);
  const int by_size = static_cast<int>(cells / kMinCellsPerEdtThread);
  return std::clamp(by_size, 1, kMaxEdtThreads);
}

inline void computeEDT1DNoAlloc(
  const std::vector<double> & f, std::vector<double> & d, std::vector<int> & v, std::vector<double> & z)
{
  const int n = static_cast<int>(f.size());
  std::fill(d.begin(), d.end(), kInf);
  if (n <= 0) {
    return;
  }

  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;

  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (true) {
      const int p = v[static_cast<size_t>(k)];
      const double fq = f[static_cast<size_t>(q)];
      const double fp = f[static_cast<size_t>(p)];
      if (fq >= kInf && fp >= kInf) {
        s = kInf;
      } else {
        s = ((fq + static_cast<double>(q * q)) - (fp + static_cast<double>(p * p))) /
            (2.0 * static_cast<double>(q - p));
      }
      if (k <= 0 || s > z[static_cast<size_t>(k)]) {
        break;
      }
      --k;
    }
    ++k;
    v[static_cast<size_t>(k)] = q;
    z[static_cast<size_t>(k)] = s;
    z[static_cast<size_t>(k) + 1] = kInf;
  }

  int kk = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<size_t>(kk) + 1] < static_cast<double>(q)) {
      ++kk;
    }
    const int p = v[static_cast<size_t>(kk)];
    d[static_cast<size_t>(q)] = static_cast<double>((q - p) * (q - p)) + f[static_cast<size_t>(p)];
  }
}
}  // namespace

void ESDFUtils::computeEDT1D(const std::vector<double> & f, std::vector<double> & d)
{
  const int n = static_cast<int>(f.size());
  d.assign(static_cast<size_t>(n), kInf);
  if (n <= 0) {
    return;
  }
  std::vector<int> v(static_cast<size_t>(n));
  std::vector<double> z(static_cast<size_t>(n) + 1);
  computeEDT1DNoAlloc(f, d, v, z);
}

void ESDFUtils::computeEDT2D(
  int width, int height, const std::vector<uint8_t> & mask, std::vector<double> & dist_sq_out)
{
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("ESDFUtils::computeEDT2D: invalid width/height");
  }
  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (mask.size() != expected) {
    throw std::invalid_argument("ESDFUtils::computeEDT2D: mask size mismatch");
  }

  std::vector<double> tmp(expected, kInf);
  [[maybe_unused]] const int threads = edtThreadCount(width, height);

#ifdef _OPENMP
#pragma omp parallel num_threads(threads) if (threads > 1)
#endif
  {
    std::vector<double> f_row(static_cast<size_t>(width));
    std::vector<double> d_row(static_cast<size_t>(width));
    std::vector<int> v_row(static_cast<size_t>(width));
    std::vector<double> z_row(static_cast<size_t>(width) + 1);

#ifdef _OPENMP
#pragma omp for
#endif
    for (int y = 0; y < height; ++y) {
      const size_t row = static_cast<size_t>(y) * static_cast<size_t>(width);
      for (int x = 0; x < width; ++x) {
        f_row[static_cast<size_t>(x)] = (mask[row + static_cast<size_t>(x)] == 0U) ? 0.0 : kInf;
      }
      computeEDT1DNoAlloc(f_row, d_row, v_row, z_row);
      for (int x = 0; x < width; ++x) {
        tmp[row + static_cast<size_t>(x)] = d_row[static_cast<size_t>(x)];
      }
    }
  }

  dist_sq_out.assign(expected, kInf);

#ifdef _OPENMP
#pragma omp parallel num_threads(threads) if (threads > 1)
#endif
  {
    std::vector<double> f_col(static_cast<size_t>(height));
    std::vector<double> d_col(static_cast<size_t>(height));
    std::vector<int> v_col(static_cast<size_t>(height));
    std::vector<double> z_col(static_cast<size_t>(height) + 1);

#ifdef _OPENMP
#pragma omp for
#endif
    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
        f_col[static_cast<size_t>(y)] =
          tmp[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)];
      }
      computeEDT1DNoAlloc(f_col, d_col, v_col, z_col);
      for (int y = 0; y < height; ++y) {
        dist_sq_out[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] =
          d_col[static_cast<size_t>(y)];
      }
    }
  }
}

}  // namespace rog_map
