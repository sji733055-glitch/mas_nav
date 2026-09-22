#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>

#include "mas2027_nav_executor/path_planner/trajectory/local_path_processor.hpp"
#include "minco_core/components/planner_mode_context.hpp"
#include "minco_core/minco_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

class GridQuery final : public rog_map::MapQueryInterface {
public:
  GridQuery(unsigned int width, unsigned int height, double resolution)
      : width_(width), height_(height), resolution_(resolution),
        values_(static_cast<size_t>(width) * height, 0U) {}

  void block(unsigned int x, unsigned int y) { values_[index(x, y)] = 254U; }

  bool worldToMap(double wx, double wy, unsigned int &mx,
                  unsigned int &my) const override {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < 0.0 || wy < 0.0)
      return false;
    const int ix = static_cast<int>(std::floor(wx / resolution_));
    const int iy = static_cast<int>(std::floor(wy / resolution_));
    if (ix < 0 || iy < 0 || ix >= static_cast<int>(width_) ||
        iy >= static_cast<int>(height_)) {
      return false;
    }
    mx = static_cast<unsigned int>(ix);
    my = static_cast<unsigned int>(iy);
    return true;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double &wx,
                  double &wy) const override {
    wx = (static_cast<double>(mx) + 0.5) * resolution_;
    wy = (static_cast<double>(my) + 0.5) * resolution_;
  }

  unsigned int sizeX() const override { return width_; }
  unsigned int sizeY() const override { return height_; }
  double resolution() const override { return resolution_; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int mx, unsigned int my) const override {
    return isValid(mx, my) ? values_[index(mx, my)] : 254U;
  }
  const unsigned char *values() const override { return values_.data(); }
  bool isValid(unsigned int mx, unsigned int my) const override {
    return mx < width_ && my < height_;
  }
  bool isFree(unsigned int mx, unsigned int my) const override {
    return isValid(mx, my) && value(mx, my) < 253U;
  }

  rog_map::QueryResult query(const Eigen::Vector3d &pos) const override {
    rog_map::QueryResult result;
    unsigned int mx = 0U, my = 0U;
    if (!worldToMap(pos.x(), pos.y(), mx, my)) {
      result.status = rog_map::QueryStatus::OUT_OF_MAP;
      return result;
    }
    double best = std::numeric_limits<double>::infinity();
    for (unsigned int y = 0U; y < height_; ++y) {
      for (unsigned int x = 0U; x < width_; ++x) {
        if (isFree(x, y))
          continue;
        double wx = 0.0, wy = 0.0;
        mapToWorld(x, y, wx, wy);
        best = std::min(best, std::hypot(pos.x() - wx, pos.y() - wy));
      }
    }
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = std::isfinite(best) ? best : 100.0;
    if (!isFree(mx, my))
      result.distance = -0.5 * resolution_;
    return result;
  }

  bool evaluate(const Eigen::Vector3d &pos, double &dist,
                Eigen::Vector3d &grad) const override {
    const auto result = query(pos);
    dist = result.distance;
    grad.setZero();
    return result.ok;
  }

private:
  size_t index(unsigned int x, unsigned int y) const {
    return static_cast<size_t>(y) * width_ + x;
  }

  unsigned int width_;
  unsigned int height_;
  double resolution_;
  std::vector<unsigned char> values_;
};

std::vector<geometry_msgs::msg::PoseStamped>
straightGlobalPath(double start_x, double end_x, double y, double step) {
  std::vector<geometry_msgs::msg::PoseStamped> path;
  for (double x = start_x; x <= end_x + 1e-9; x += step) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = std::min(x, end_x);
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.push_back(pose);
  }
  return path;
}

minco_planner::PlannerModeContext
makeContext(const std::shared_ptr<rog_map::MapQueryInterface> &query) {
  minco_planner::PlannerModeParams params;
  params.planner_mode = "EXPLORATION";
  params.rog_frame = "odom";
  params.map_frame = "map";
  params.exploration_boundary_margin = 0.1;
  minco_planner::PlannerModeContext context;
  context.configure(params, query, nullptr,
                    rclcpp::get_logger("test_local_path_processor"));
  return context;
}

bool segmentHasClearance(const std::shared_ptr<GridQuery> &query,
                         const Eigen::Vector3d &from, const Eigen::Vector3d &to,
                         double clearance) {
  const int samples =
      std::max(1, static_cast<int>(std::ceil((to - from).norm() / 0.025)));
  for (int i = 0; i <= samples; ++i) {
    const Eigen::Vector3d p =
        from +
        (static_cast<double>(i) / static_cast<double>(samples)) * (to - from);
    const auto result = query->query(p);
    if (!result.ok || result.distance <= clearance)
      return false;
  }
  return true;
}

/// 绕矩形障碍的判据：线段上任一采样点落进障碍(含 margin 膨胀)即碰撞；
/// 独立于 GridQuery，用于检验 getSparseWaypoints 契约。
bool segmentClearOfRect(const Eigen::Vector3d &from, const Eigen::Vector3d &to,
                        double margin) {
  const double length = (to - from).head<2>().norm();
  const int samples =
      std::max(1, static_cast<int>(std::ceil(length / 0.02)));
  for (int i = 0; i <= samples; ++i) {
    const Eigen::Vector3d p =
        from +
        (static_cast<double>(i) / static_cast<double>(samples)) * (to - from);
    if (p.x() > 1.00 - margin && p.x() < 1.40 + margin &&
        p.y() > -0.80 - margin && p.y() < 0.30 + margin) {
      return false;
    }
  }
  return true;
}

/// 到轴对齐矩形的**解析**距离（矩形外为正，内部为负）。
struct Rect
{
  double x0{0.0};
  double x1{0.0};
  double y0{0.0};
  double y1{0.0};
};

double distanceToRect(const Eigen::Vector3d &p, const Rect &r) {
  const double dx =
      std::max(std::max(r.x0 - p.x(), p.x() - r.x1), 0.0);
  const double dy =
      std::max(std::max(r.y0 - p.y(), p.y() - r.y1), 0.0);
  if (dx > 0.0 || dy > 0.0) {
    return std::hypot(dx, dy);
  }
  const double inside = std::min(std::min(p.x() - r.x0, r.x1 - p.x()),
                                 std::min(p.y() - r.y0, r.y1 - p.y()));
  return -inside;
}

/// 用解析距离场代替栅格距离：净空落在阈值带内的用例可精确构造，不受格心取距抖动影响。
class AnalyticQuery final : public rog_map::MapQueryInterface {
public:
  AnalyticQuery(unsigned int width, unsigned int height, double resolution,
                std::vector<Rect> rects)
      : grid_(width, height, resolution), rects_(std::move(rects)) {}

  bool worldToMap(double wx, double wy, unsigned int &mx,
                  unsigned int &my) const override {
    return grid_.worldToMap(wx, wy, mx, my);
  }
  void mapToWorld(unsigned int mx, unsigned int my, double &wx,
                  double &wy) const override {
    grid_.mapToWorld(mx, my, wx, wy);
  }
  unsigned int sizeX() const override { return grid_.sizeX(); }
  unsigned int sizeY() const override { return grid_.sizeY(); }
  double resolution() const override { return grid_.resolution(); }
  double originX() const override { return grid_.originX(); }
  double originY() const override { return grid_.originY(); }
  uint8_t value(unsigned int mx, unsigned int my) const override {
    return grid_.value(mx, my);
  }
  const unsigned char *values() const override { return grid_.values(); }
  bool isValid(unsigned int mx, unsigned int my) const override {
    return grid_.isValid(mx, my);
  }
  bool isFree(unsigned int mx, unsigned int my) const override {
    return grid_.isFree(mx, my);
  }

  rog_map::QueryResult query(const Eigen::Vector3d &pos) const override {
    rog_map::QueryResult result;
    unsigned int mx = 0U, my = 0U;
    if (!grid_.worldToMap(pos.x(), pos.y(), mx, my)) {
      result.status = rog_map::QueryStatus::OUT_OF_MAP;
      return result;
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto &r : rects_) {
      best = std::min(best, distanceToRect(pos, r));
    }
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = std::isfinite(best) ? best : 100.0;
    return result;
  }

  bool evaluate(const Eigen::Vector3d &pos, double &dist,
                Eigen::Vector3d &grad) const override {
    const auto result = query(pos);
    dist = result.distance;
    grad.setZero();
    return result.ok;
  }

private:
  GridQuery grid_;
  std::vector<Rect> rects_;
};

/// 把折线按 step 加密，模拟 A* 输出的稠密栅格路径。
std::vector<Eigen::Vector3d>
densifyPolyline(const std::vector<Eigen::Vector3d> &corners, double step) {
  std::vector<Eigen::Vector3d> dense;
  for (size_t i = 1U; i < corners.size(); ++i) {
    const Eigen::Vector3d a = corners[i - 1U];
    const Eigen::Vector3d b = corners[i];
    const int steps =
        std::max(1, static_cast<int>(std::ceil((b - a).head<2>().norm() / step)));
    for (int k = (i == 1U ? 0 : 1); k <= steps; ++k) {
      dense.push_back(a + (static_cast<double>(k) / static_cast<double>(steps)) *
                              (b - a));
    }
  }
  return dense;
}

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  constexpr double resolution = 0.10;
  constexpr double clearance = 0.15;
  const auto global = straightGlobalPath(0.55, 5.45, 1.05, resolution);
  geometry_msgs::msg::PoseStamped current;
  current.header.frame_id = "odom";
  current.pose.position.x = 0.55;
  current.pose.position.y = 1.05;
  current.pose.orientation.w = 1.0;

  // 竖墙截断全局直线，但 y≈2.5 m 附近留有足够宽的缺口，按碰撞距离膨胀后的机器人仍能通过。
  {
    auto query = std::make_shared<GridQuery>(60U, 40U, resolution);
    for (unsigned int y = 0U; y < 40U; ++y) {
      if (y >= 23U && y <= 29U)
        continue;
      query->block(30U, y);
    }
    auto context = makeContext(query);
    minco_planner::LocalPathProcessor processor;
    processor.configure(10.0, 2.0, 4.0, 0.15, clearance,
                        rclcpp::get_logger("test_local_path_processor"));
    const auto seed = processor.buildSeed(global, current, context);
    assert(seed.valid);
    assert(seed.used_dynamic_detour);
    assert(!seed.repair_rejected);
    assert(!seed.stop_at_local_end);
    assert(seed.sparse_waypoints.size() >= 2U);
    double max_y = -std::numeric_limits<double>::infinity();
    for (size_t i = 0U; i < seed.sparse_waypoints.size(); ++i) {
      max_y = std::max(max_y, seed.sparse_waypoints[i].y());
      // 种子/稀疏化只保证"不穿障碍"（净空 > 0 ⇔ 不在占据格，同旧工程 isLineFree 口径）；
      // 净空是轨迹级要求：折线由 MINCO 拉开，再由发布前校验、20 Hz 监视与 MPC 门验收。
      if (i > 0U) {
        assert(segmentHasClearance(query, seed.sparse_waypoints[i - 1U],
                                   seed.sparse_waypoints[i], 0.0));
      }
    }
    assert(max_y > 2.2);
    assert((seed.sparse_waypoints.back() - Eigen::Vector3d(5.45, 1.05, 0.0))
               .norm() < 0.15);
  }

  // 封死的墙没有绕行通路：必须返回障碍前零末速度的停车前缀，不能把穿墙路径交给 MINCO。
  {
    auto query = std::make_shared<GridQuery>(60U, 40U, resolution);
    for (unsigned int y = 0U; y < 40U; ++y)
      query->block(30U, y);
    auto context = makeContext(query);
    minco_planner::LocalPathProcessor processor;
    processor.configure(10.0, 2.0, 4.0, 0.15, clearance,
                        rclcpp::get_logger("test_local_path_processor"));
    const auto seed = processor.buildSeed(global, current, context);
    assert(seed.valid);
    assert(!seed.used_dynamic_detour);
    assert(seed.stop_at_local_end);
    assert(seed.sparse_waypoints.size() >= 2U);
    assert(seed.sparse_waypoints.back().x() < 2.8);
    assert(seed.sparse_waypoints.back().x() > current.pose.position.x + 0.3);
    for (size_t i = 1U; i < seed.sparse_waypoints.size(); ++i) {
      assert(segmentHasClearance(query, seed.sparse_waypoints[i - 1U],
                                 seed.sparse_waypoints[i], clearance));
    }
  }

  // 稠密绕行路径逐段合格时，getSparseWaypoints 不得整体返回空：旧实现按"离 a->b 直线最远"
  // 挑拐点且插入时不校验净空，末尾复核发现碰撞就 `return {}`，丢掉本来可用的绕行路径。
  {
    constexpr double margin = 0.10;
    const auto rect_free = [margin](const Eigen::Vector3d &a,
                                    const Eigen::Vector3d &b) {
      return segmentClearOfRect(a, b, margin);
    };
    const std::vector<Eigen::Vector3d> corners{
        {0.00, 0.00, 0.0}, {0.90, 0.00, 0.0}, {0.90, 0.50, 0.0},
        {1.50, 0.50, 0.0}, {1.50, -0.60, 0.0}, {2.10, 0.00, 0.0}};
    const auto dense = densifyPolyline(corners, 0.05);
    assert(dense.size() >= 3U);
    for (size_t i = 1U; i < dense.size(); ++i) {
      assert(rect_free(dense[i - 1U], dense[i]));
    }

    const auto sparse =
        minco_planner::utils::getSparseWaypoints(dense, 2.0, 4.0, true, rect_free);
    assert(!sparse.empty());
    assert(sparse.size() >= 2U);
    assert((sparse.front() - dense.front()).norm() < 1e-9);
    assert((sparse.back() - dense.back()).norm() < 1e-9);
    for (size_t i = 1U; i < sparse.size(); ++i) {
      assert(rect_free(sparse[i - 1U], sparse[i]));
    }
  }

  // 种子门只查"堵没堵死"、不查净空：0.50 m 走廊（中线净空 0.25 m < collision_dist 0.30 m）的
  // 折线必须原样交给 MINCO —— isFree 全过即放行，净空由 MINCO 与轨迹级三道门负责。
  {
    constexpr double res = 0.05;
    auto query = std::make_shared<GridQuery>(80U, 60U, res);
    for (unsigned int y = 0U; y < 60U; ++y) {
      if (y >= 18U && y <= 27U)
        continue;
      for (unsigned int x = 0U; x < 80U; ++x)
        query->block(x, y);
    }
    auto context = makeContext(query);
    const auto corridor = straightGlobalPath(0.20, 3.80, 1.15, res);
    geometry_msgs::msg::PoseStamped current;
    current.header.frame_id = "odom";
    current.pose.position.x = 0.23;
    current.pose.position.y = 1.15;
    current.pose.orientation.w = 1.0;

    minco_planner::LocalPathProcessor processor;
    processor.configure(10.0, 2.0, 4.0, 0.15, 0.30,
                        rclcpp::get_logger("test_local_path_processor"));
    processor.setEscapeOptions(true, 0.08, 0.05);

    const auto seed = processor.buildSeed(corridor, current, context);
    assert(seed.valid);
    assert(!seed.used_escape_prefix);
    assert(!seed.stop_at_local_end);
    assert(!seed.used_dynamic_detour);
    assert(!seed.repair_rejected);
    assert(seed.sparse_waypoints.size() >= 2U);
    // 折线未被改写：首点仍是全局折线的局部起点（最近栅格顶点），不是停车/脱困前缀的实测位置。
    assert(!seed.dense_path.empty());
    assert((seed.dense_path.front() - Eigen::Vector3d(0.23, 1.15, 0.0)).norm() < 0.10);
    // 种子门把"贴墙但没堵死"记进 dense_reject 留证：clearance_only 为真，且非 terrain 否决。
    assert(seed.dense_reject.valid);
    assert(seed.dense_reject.clearance_only);
    assert(!seed.dense_reject.terrain_blocked);
    assert(seed.dense_reject.clearance <= 0.30 + 1e-9);
    for (size_t i = 1U; i < seed.sparse_waypoints.size(); ++i) {
      assert(segmentHasClearance(query, seed.sparse_waypoints[i - 1U],
                                 seed.sparse_waypoints[i], 0.0));
    }

    // 停车前缀仍用硬口径（segmentClear 的 enforce_clearance），由"真的堵死"用例覆盖。
  }

  // 路真的被堵死时修复链必须照旧生效：整列占据格封死走廊中段 + 车离封口仅 0.10 m ⇒
  // 绕行与完整停车前缀逐层失败，第四层关闭时"四层全败"并清空种子。
  {
    constexpr double res = 0.05;
    auto query = std::make_shared<GridQuery>(80U, 60U, res);
    // 走廊：cell y ∈ [18, 28] 自由（0.55 m 宽），其余全为障碍。
    for (unsigned int y = 0U; y < 60U; ++y) {
      if (y >= 18U && y <= 28U)
        continue;
      for (unsigned int x = 0U; x < 80U; ++x)
        query->block(x, y);
    }
    // 走廊中段整列封死：cell x ∈ [12, 16] ⇒ 世界 x ∈ [0.60, 0.85]。
    for (unsigned int x = 12U; x <= 16U; ++x) {
      for (unsigned int y = 18U; y <= 28U; ++y)
        query->block(x, y);
    }
    auto context = makeContext(query);
    const double center_y = (23.0 + 0.5) * res;   // 1.175
    const auto corridor = straightGlobalPath(0.20, 3.80, center_y, res);
    geometry_msgs::msg::PoseStamped current;
    current.header.frame_id = "odom";
    current.pose.position.x = 0.50;
    current.pose.position.y = center_y;
    current.pose.orientation.w = 1.0;

    minco_planner::LocalPathProcessor processor;
    processor.configure(10.0, 2.0, 4.0, 0.15, 0.30,
                        rclcpp::get_logger("test_local_path_processor"));
    processor.setEscapeOptions(false, 0.08, 0.05);

    const auto seed = processor.buildSeed(corridor, current, context);
    assert(!seed.valid);
    assert(!seed.used_dynamic_detour);
    assert(!seed.stop_at_local_end);
    assert(!seed.used_escape_prefix);
    assert(seed.dense_path.empty());

    const auto &reject = seed.dense_reject;
    assert(reject.valid);
    // 硬否决＝占据格（判据把 clearance 记为 0），而非"净空差几毫米"，判读器必须把两类分开；
    // 本断言同时锁住 note_reject 的优先级：先遇到的"贴墙点"不能盖掉后面的"堵死点"。
    assert(!reject.clearance_only);
    assert(!reject.terrain_blocked);
    assert(reject.clearance <= 1e-9);
    assert(minco_planner::classifySeedReject(reject, 0.125, 0.30, 0.02) ==
           "GEOMETRY");
  }

  // dense_reject 必须报出"第一个净空不足点"的净空、要求与是否放宽，该点只记录、不否决种子。
  {
    constexpr double res = 0.05;
    const double center_y = 1.5;
    const auto query = std::make_shared<AnalyticQuery>(
        80U, 60U, res,
        std::vector<Rect>{{-10.0, 10.0, -10.0, 1.25}, {-10.0, 10.0, 1.75, 10.0}});
    auto context = makeContext(query);
    const auto corridor = straightGlobalPath(0.20, 3.80, center_y, res);
    geometry_msgs::msg::PoseStamped current;
    current.header.frame_id = "odom";
    current.pose.position.x = 0.50;
    current.pose.position.y = center_y;
    current.pose.orientation.w = 1.0;

    minco_planner::LocalPathProcessor processor;
    processor.configure(10.0, 2.0, 4.0, 0.15, 0.30,
                        rclcpp::get_logger("test_local_path_processor"));

    const auto start_query =
        query->query(Eigen::Vector3d(current.pose.position.x, center_y, 0.0));
    assert(start_query.ok);
    assert(std::abs(start_query.distance - 0.25) < 1e-6);

    processor.setEscapeOptions(false, 0.08, 0.05);
    const auto seed = processor.buildSeed(corridor, current, context);
    assert(seed.valid);
    assert(!seed.used_dynamic_detour);
    assert(!seed.stop_at_local_end);
    assert(!seed.used_escape_prefix);
    assert(!seed.dense_path.empty());

    const auto &reject = seed.dense_reject;
    assert(reject.valid);
    assert(!reject.terrain_blocked);
    assert(reject.clearance_only);
    // 要求必须与发布前校验 / 20 Hz 监视 / MPC 指令门同一有效阈值：collision_dist 0.30
    // 减抖动余量 0.02 = 0.28；漂移的后果是规划放行、执行刹车。
    assert(std::abs(reject.required -
                    mas2027_nav_executor::effectiveClearanceThreshold(0.30)) < 1e-9);
    assert(std::abs(reject.required - 0.28) < 1e-9);
    // 近场之外（弧长 > 0.30 m），所以不是被放宽过的值（放宽后是 起点净空 - 0.02，明显更小）。
    assert(!reject.near_field_relaxed);
    assert(std::abs(reject.clearance - 0.25) < 0.02);
    const Eigen::Vector3d start_pos(current.pose.position.x, center_y, 0.0);
    assert((reject.point - start_pos).head<2>().norm() > 0.30);
    assert(std::abs(reject.point.y() - center_y) < 0.02);
  }

  // `verdict=` 判读必须把"环境过不去"与"判据不一致"分开：前者不该动阈值
  // （动了就是拿安全换通畅），后者才是判据 bug；长度/terrain 类否决与净空无关。
  {
    constexpr double collision_dist = 0.30;
    constexpr double slack = 0.02;

    minco_planner::SeedRejectInfo info;
    assert(minco_planner::classifySeedReject(info, 0.30, collision_dist, slack) ==
           "NONE");

    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.20;
    info.required = 0.30;
    info.arc_from_start = 0.50;   // 近场之外
    assert(minco_planner::classifySeedReject(info, 0.20, collision_dist, slack) ==
           "GEOMETRY");

    // 关键用例：点落在近场之内（arc 0.10 < collision_dist 0.30），近场规则放宽成 起点净空 - slack
    // = 0.29 能过、按完整要求 0.295 过不了 → 两套判据结论相反；统一四道门后留作漂移探针。
    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.295;
    info.required = 0.30;
    info.arc_from_start = 0.10;
    assert(minco_planner::classifySeedReject(info, 0.31, collision_dist, slack) ==
           "SEED_GATE_STRICTER");

    // 同一对数字、arc 在近场之外：有效阈值 0.30 - 0.02 = 0.28 时 0.295 能过，
    // 故否决只可能来自"某道门比统一判据更严"，这一格正是判据漂移的探针。
    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.295;
    info.required = 0.30;   // 某道门仍用完整 0.30 时才会记下这个值
    info.arc_from_start = 0.50;
    assert(minco_planner::classifySeedReject(info, 0.31, collision_dist, slack) ==
           "SEED_GATE_STRICTER");

    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.27;
    info.required = 0.28;
    info.arc_from_start = 0.50;
    assert(minco_planner::classifySeedReject(info, 0.31, collision_dist, slack) ==
           "GEOMETRY");

    // 起点本就贴死（0.25 < 0.30）时种子门记下放宽要求 0.23，净空 0.20 仍过不了 → GEOMETRY；
    // 也因记不下"起点贴死但规则没问题"的组合，才没有单独的 "ROBOT_TOO_CLOSE" 类。
    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.20;
    info.required = 0.23;
    info.arc_from_start = 0.10;
    info.near_field_relaxed = true;
    assert(minco_planner::classifySeedReject(info, 0.25, collision_dist, slack) ==
           "GEOMETRY");

    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.26;
    info.required = 0.30;
    info.arc_from_start = 0.35;
    info.length_limited = true;
    assert(minco_planner::classifySeedReject(info, 0.31, collision_dist, slack) ==
           "PREFIX_TOO_SHORT");

    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.terrain_blocked = true;
    assert(minco_planner::classifySeedReject(info, 0.31, collision_dist, slack) ==
           "TERRAIN");
  }

  rclcpp::shutdown();
  return 0;
}
