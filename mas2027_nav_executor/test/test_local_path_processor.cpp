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

/// 绕一个矩形障碍的绕行判据：线段上任意采样点落进障碍(含 margin 膨胀)即视为碰撞。
/// 与 GridQuery 无关，用来单独检验 getSparseWaypoints 的输入输出契约。
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
  // 点在矩形内部：返回到最近边的负距离。
  const double inside = std::min(std::min(p.x() - r.x0, r.x1 - p.x()),
                                 std::min(p.y() - r.y0, r.y1 - p.y()));
  return -inside;
}

/// 用解析距离场代替栅格距离，这样"净空恰好落在某个阈值带内"可以精确构造，
/// 不受 GridQuery 按格心取距带来的抖动影响。栅格部分只用来回答"这一格是否被占用"。
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
    // 走廊只由 rects_ 描述：矩形之外一律返回到矩形的正距离。
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

}  // 匿名命名空间

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

  // 竖墙截断全局直线，但 y=2.5 m
  // 附近留有足够宽的缺口，按碰撞距离膨胀后的机器人仍能通过。
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
      if (i > 0U) {
        assert(segmentHasClearance(query, seed.sparse_waypoints[i - 1U],
                                   seed.sparse_waypoints[i], clearance));
      }
    }
    assert(max_y > 2.2);
    assert((seed.sparse_waypoints.back() - Eigen::Vector3d(5.45, 1.05, 0.0))
               .norm() < 0.15);
  }

  // 封死的墙没有绕行通路。处理器必须返回障碍前零末速度的停车前缀，
  // 不能把穿墙路径交给 MINCO。
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

  // 回归（2026-09-16）：稠密绕行路径逐段合格时，getSparseWaypoints 不允许整体返回空。
  //
  // 旧实现用"离 a->b 直线最远"的启发式挑拐点，插入时不校验净空；末尾统一复核一旦发现
  // 该拐点段碰撞，就 `return {}` 把一条本来可用的绕行路径整个丢掉。在 LocalPathProcessor
  // 里这等价于 seed.valid=false -> finish(false, "COLLISION")，实车表现为局部绕行刚修好
  // 0.3 ms 后就是 "MINCO path generation failed; retrying"。
  //
  // 下面这条绕矩形障碍的 80 点折线（随机检索 2250 个紧凑几何 + 199 条随机障碍场绕行路径里
  // 挑出的最小反例）在旧实现下返回空，修复后必须返回一条每段都合格的稀疏路径。
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
    assert(!sparse.empty());          // 旧实现这里是空
    assert(sparse.size() >= 2U);
    assert((sparse.front() - dense.front()).norm() < 1e-9);
    assert((sparse.back() - dense.back()).norm() < 1e-9);
    for (size_t i = 1U; i < sparse.size(); ++i) {
      assert(rect_free(sparse[i - 1U], sparse[i]));
    }
  }

  // 回归（2026-09-16 13:40 实车）：车停在比要求净空更窄的通道里时，三层兜底
  // （正常路径 / 局部绕行 / 完整停车前缀）会全部失败，车原地不动。
  //
  // 场景：0.50 m 宽走廊（中线净空 0.275 m < collision_dist 0.30 m），全局折线沿走廊。
  //   - pathClear 失败：离开起点 0.30 m 之后必须满足完整 0.30 m；
  //   - 绕行 A* 失败：走廊里没有任何一格净空 > 0.30 m；
  //   - 完整停车前缀失败：安全段约 0.30 m，减去 0.15 m 收尾余量后不足 0.30 m。
  // 第四层兜底必须给出一个"不比当前净空更差"的短前缀，让车能挪出去。
  {
    constexpr double res = 0.05;
    auto query = std::make_shared<GridQuery>(80U, 60U, res);
    for (unsigned int y = 0U; y < 60U; ++y) {
      if (y >= 18U && y <= 27U)
        continue;   // 走廊：0.50 m 宽
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

    // 先确认"三层兜底"在这个场景下确实全部失败：关掉第四层，种子必须无效。
    processor.setEscapeOptions(false, 0.08, 0.05);
    const auto without_escape = processor.buildSeed(corridor, current, context);
    assert(!without_escape.valid);

    // 打开第四层：必须给出短前缀，且它是能用的种子。
    processor.setEscapeOptions(true, 0.08, 0.05);
    const auto seed = processor.buildSeed(corridor, current, context);
    assert(seed.used_escape_prefix);
    assert(seed.stop_at_local_end);
    assert(seed.valid);
    assert(seed.sparse_waypoints.size() >= 2U);
    double escape_length = 0.0;
    for (size_t i = 1U; i < seed.dense_path.size(); ++i) {
      escape_length += (seed.dense_path[i] - seed.dense_path[i - 1U]).head<2>().norm();
    }
    // 长度上限就是一个车体半径：整段都落在近场豁免半径以内，不引入新的安全阈值。
    assert(escape_length > 0.0);
    assert(escape_length <= 0.30 + 1e-9);
    // 起点仍是机器人实测位置，且末速度为零（stop_at_local_end）。
    assert((seed.dense_path.front() - Eigen::Vector3d(0.23, 1.15, 0.0)).norm() < 1e-6);
    assert((seed.dense_path.back() - seed.dense_path.front()).norm() > 0.0);
    // 脱困前缀的每一段只需满足近场判据：不比机器人当前实测净空更差（减 0.02 m 抖动余量）。
    const auto start_query = query->query(Eigen::Vector3d(0.23, 1.15, 0.0));
    assert(start_query.ok);
    const double near_required = std::max(0.0, start_query.distance - 0.02);
    for (size_t i = 1U; i < seed.sparse_waypoints.size(); ++i) {
      assert(segmentHasClearance(query, seed.sparse_waypoints[i - 1U],
                                 seed.sparse_waypoints[i], near_required));
    }
  }

  // 回归（2026-09-16 排障插桩）：`Live obstacle blocks the local route...` 必须能报出
  // "四层各自死在哪一点"。
  //
  // 背景：这条消息以前只说"全败了"，实车排障时无法区分两种完全不同的情形——
  //   (a) 几何路线存在、只是每段都差几毫米过不了净空判据（门槛问题，该改判据）；
  //   (b) 路本来就过不去，拒绝是正确行为（环境问题，不该动阈值）。
  // 现在带上首个否决点的 (净空, 要求, 是否近场放宽) 之后可以直接分辨。
  //
  // 本用例构造 (b) 类现场：0.50 m 宽走廊、中线净空 0.25 m < collision_dist 0.30 m。
  // 机器人起点净空 0.25 m < 0.30 m，因此起点 0.30 m 弧长内走"不比现在更差"的近场放宽
  // （0.25 - 0.02 = 0.23 m）；一旦离开近场就必须满足完整 0.30 m，而那一段只有 0.25 m。
  // 于是首个否决点必须出现在**近场之外**，且 reported required 必须是完整的 0.300，
  // 不能是放宽后的 0.23。
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

    // 关掉第四层脱困前缀才能复现"四层全败"。开着它时 0.50 m 走廊反而会被救活成
    // 一个 0.24 m 的 creep 前缀（见本文件上方那条用例）——也就是说实车日志里
    // `Live obstacle blocks` 只在连脱困前缀都建不出来时才会出现，
    // 那说明当时前方连 collision_dist 那么短的安全段都没有，比这条用例更贴死。
    processor.setEscapeOptions(false, 0.08, 0.05);
    const auto seed = processor.buildSeed(corridor, current, context);
    // 走廊太窄：绕行、停车前缀都必须失败。
    assert(!seed.valid);
    assert(!seed.used_dynamic_detour);
    assert(!seed.stop_at_local_end);
    assert(!seed.used_escape_prefix);
    assert(seed.dense_path.empty());

    const auto &reject = seed.dense_reject;
    assert(reject.valid);
    assert(!reject.terrain_blocked);
    // 首个否决点必须报出完整的 0.300 m 要求，而不是被近场放宽过的 0.23 m。
    assert(std::abs(reject.required - 0.30) < 1e-9);
    assert(!reject.near_field_relaxed);
    // 该点净空只有走廊中线那 0.25 m。
    assert(std::abs(reject.clearance - 0.25) < 0.02);
    // 必须落在近场弧长（0.30 m）之外，否则说明放宽规则用错了地方。
    const Eigen::Vector3d start_pos(current.pose.position.x, center_y, 0.0);
    assert((reject.point - start_pos).head<2>().norm() > 0.30);
    // 沿走廊方向、且仍在走廊内。
    assert(std::abs(reject.point.y() - center_y) < 0.02);
  }

  // 回归（2026-09-16）：`verdict=` 自动判读必须把"环境过不去"与"判据不一致"分开。
  //
  // 这两类的处置完全相反：前者不该动阈值（动了就是拿安全换通畅），后者才是判据 bug。
  // 现场靠人读日志区分要同时手算两套判据（种子门 vs 近场规则），很容易算错，所以固定下来。
  {
    constexpr double collision_dist = 0.30;
    constexpr double slack = 0.02;

    minco_planner::SeedRejectInfo info;
    // 没有捕获到否决点。
    assert(minco_planner::classifySeedReject(info, 0.30, collision_dist, slack) ==
           "NONE");

    // 该点净空 0.20，连放宽后的近场规则都过不了 → 环境真的过不去。
    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.20;
    info.required = 0.30;
    info.arc_from_start = 0.50;   // 近场之外
    assert(minco_planner::classifySeedReject(info, 0.20, collision_dist, slack) ==
           "GEOMETRY");

    // 关键用例：该点落在**近场之内**（arc 0.10 < collision_dist 0.30），净空 0.295。
    //   近场规则：要求放宽成 起点净空 - slack = 0.31 - 0.02 = 0.29 → 0.295 能过；
    //   种子门  ：起点 0.31 不 < 0.30，不放宽，仍按完整的 0.30 判 → 0.295 过不了。
    // 同一个点、两套判据给出相反结论，而机器人起点并不贴死 → 这是判据不一致。
    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.295;
    info.required = 0.30;
    info.arc_from_start = 0.10;
    assert(minco_planner::classifySeedReject(info, 0.31, collision_dist, slack) ==
           "SEED_GATE_STRICTER");

    // 同一对数字、不同的 arc：近场之外时"近场规则"退化为完整 0.30，0.295 过不了
    // → 环境过不去，不是判据问题。这条同时锁住"近场放宽只作用于起点附近"。
    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.295;
    info.required = 0.30;
    info.arc_from_start = 0.50;
    assert(minco_planner::classifySeedReject(info, 0.31, collision_dist, slack) ==
           "GEOMETRY");

    // 机器人本来就贴死（起点 0.25 < 0.30）：种子门此时会把该点记成**放宽后的**要求
    // （0.25 - 0.02 = 0.23）。该点净空 0.20 连这个都过不了 → 仍是 GEOMETRY。
    // 这条同时说明为什么不存在单独的 "ROBOT_TOO_CLOSE" 类：种子门记不下
    // "起点贴死但规则本身没问题"这种组合。
    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.20;
    info.required = 0.23;
    info.arc_from_start = 0.10;
    info.near_field_relaxed = true;
    assert(minco_planner::classifySeedReject(info, 0.25, collision_dist, slack) ==
           "GEOMETRY");

    // 安全段太短导致的否决：该点其实满足判据，与净空无关。
    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.clearance = 0.26;
    info.required = 0.30;
    info.arc_from_start = 0.35;
    info.length_limited = true;
    assert(minco_planner::classifySeedReject(info, 0.31, collision_dist, slack) ==
           "PREFIX_TOO_SHORT");

    // 死在 terrain/动态层，与净空无关。
    info = minco_planner::SeedRejectInfo{};
    info.valid = true;
    info.terrain_blocked = true;
    assert(minco_planner::classifySeedReject(info, 0.31, collision_dist, slack) ==
           "TERRAIN");
  }

  rclcpp::shutdown();
  return 0;
}
