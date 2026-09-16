#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>

#include "mas2027_nav_executor/path_planner/trajectory/trajectory_safety_checker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

/// 用解析场代替 ROGMap：净空由测试按位置给出，便于构造「贴着墙起步」的场景。
class ShapedFieldQuery final : public rog_map::MapQueryInterface
{
public:
  std::function<double(const Eigen::Vector3d &)> field = [](const Eigen::Vector3d &) {
      return 1.0;
    };

  bool worldToMap(double, double, unsigned int & mx, unsigned int & my) const override
  {
    mx = 0;
    my = 0;
    return true;
  }
  void mapToWorld(unsigned int, unsigned int, double & wx, double & wy) const override
  {
    wx = 0.0;
    wy = 0.0;
  }
  unsigned int sizeX() const override { return 1; }
  unsigned int sizeY() const override { return 1; }
  double resolution() const override { return 0.05; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int, unsigned int) const override { return 0; }
  const unsigned char * values() const override { return nullptr; }
  bool isValid(unsigned int, unsigned int) const override { return true; }
  bool isFree(unsigned int, unsigned int) const override { return true; }
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = field(pos);
    return result;
  }
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override
  {
    dist = field(pos);
    grad.setZero();
    return true;
  }
};

/// 沿 +x 以 1 m/s 匀速直线运动的单段轨迹：x(t) = t。
traj_opt::Trajectory straightTrajectory(double duration)
{
  Eigen::MatrixXd coefficients(3, 6);
  coefficients.setZero();
  coefficients(0, 4) = 1.0;  // Piece::getPos 把最后一列当常数项，col(4) 是 t^1 的系数。
  traj_opt::Trajectory traj;
  traj.emplace_back(duration, coefficients);
  return traj;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto query = std::make_shared<ShapedFieldQuery>();
  minco_planner::TrajectorySafetyChecker checker;
  checker.configure(0.30, 0.05, rclcpp::get_logger("test_trajectory_safety_checker"));
  checker.setQuery(query);

  const auto trajectory = straightTrajectory(2.0);
  minco_planner::TrajectorySafetyChecker::CheckOptions options;
  options.check_dist = 0.30;
  options.near_field = 0.30;

  // 1) 起点净空 0.28 m（比要求低），但轨迹一路远离障碍 → 必须放行。
  //    这正是真车「停在离墙 0.28 m 处发目标点，车不动」的场景。
  query->field = [](const Eigen::Vector3d & p) { return 0.28 + p.x(); };
  assert(checker.checkTrajectory(trajectory, options));
  // 关闭近场放宽后同一条轨迹必须被拒，证明失败原因就是起点净空。
  auto strict = options;
  strict.near_field = 0.0;
  assert(!checker.checkTrajectory(trajectory, strict));

  // 2) 近场不是免检区：起步就往障碍上贴必须拒绝。
  query->field = [](const Eigen::Vector3d & p) { return 0.28 - 0.5 * p.x(); };
  assert(!checker.checkTrajectory(trajectory, options));

  // 3) 近场之外必须满足完整要求：离开 0.30 m 后净空只有 0.277 m 仍要拒绝，
  //    避免实现退化成「跳过轨迹前 N 米」。
  query->field = [](const Eigen::Vector3d & p) { return 0.28 - 0.01 * p.x(); };
  assert(!checker.checkTrajectory(trajectory, options));

  // 4) 运行时监视：只从 t_start 开始看，起点锚在 t_start 对应的位置。
  //    t < 0.5 s 那段贴着障碍不影响监视结果，t >= 0.5 s 净空充足则通过。
  query->field = [](const Eigen::Vector3d & p) {
      return p.x() < 0.5 ? 0.10 : 0.60;
    };
  auto monitor = options;
  monitor.t_start = 0.5;
  monitor.horizon = 1.0;
  assert(checker.checkTrajectory(trajectory, monitor));
  auto from_start = options;
  from_start.horizon = 1.0;
  assert(!checker.checkTrajectory(trajectory, from_start));

  // 5) 旧接口（不带近场）行为不变：全程用给定阈值。
  query->field = [](const Eigen::Vector3d & p) { return 0.35 + p.x(); };
  assert(checker.checkTrajectory(trajectory, 0.0, 0.30));
  query->field = [](const Eigen::Vector3d & p) { return 0.25 + p.x(); };
  assert(!checker.checkTrajectory(trajectory, 0.0, 0.30));

  std::printf("trajectory safety checker near-field checks passed\n");
  rclcpp::shutdown();
  return 0;
}
