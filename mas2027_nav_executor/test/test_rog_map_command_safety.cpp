#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <memory>

#include "mas2027_nav_executor/path_executor/monitoring/command_safety.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
class FakeQuery final : public rog_map::MapQueryInterface
{
public:
  double distance{1.0};
  bool worldToMap(double, double, unsigned int &, unsigned int &) const override { return false; }
  void mapToWorld(unsigned int, unsigned int, double &, double &) const override {}
  unsigned int sizeX() const override { return 0; }
  unsigned int sizeY() const override { return 0; }
  double resolution() const override { return 1.0; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int, unsigned int) const override { return 0; }
  const unsigned char * values() const override { return nullptr; }
  bool isValid(unsigned int, unsigned int) const override { return false; }
  bool isFree(unsigned int, unsigned int) const override { return false; }
  rog_map::QueryResult query(const Eigen::Vector3d &) const override {
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = distance;
    return result;
  }
  bool evaluate(const Eigen::Vector3d &, double & dist, Eigen::Vector3d & grad) const override {
    dist = distance;
    grad.setZero();
    return true;
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto terrain = std::make_shared<mas2027_nav_executor::TerrainGrid>();
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.header.stamp.sec = 1;
  grid.info.width = grid.info.height = 10;
  grid.info.resolution = 1.0F;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(100, 0);
  sensor_msgs::msg::Image direction;
  direction.header = grid.header;
  direction.width = direction.height = 10;
  direction.encoding = "bgr8";
  direction.step = 30;
  direction.data.assign(300, 0);
  terrain->updateCost(grid);
  terrain->updateDirection(direction);
  terrain->updateDynamic(grid);

  auto tf = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.rotation.w = 1.0;
  assert(tf->setTransform(transform, "test", true));

  auto query = std::make_shared<FakeQuery>();
  minco_controller::State current;
  current.x = current.y = 5.5;
  minco_controller::Control command;
  const std::vector<minco_controller::ReferencePoint> reference;
  const rclcpp::Time stamp(grid.header.stamp);
  using mas2027_nav_executor::ExecutorStatus;
  const auto check = [&]() {
    return mas2027_nav_executor::checkCommandSafety(
      terrain, query, tf, "odom", 0.30, 0.05, current, command, reference, stamp);
  };
  assert(check() == ExecutorStatus::PUBLISHED);
  query->distance = 0.20;
  assert(check() == ExecutorStatus::DYNAMIC_BLOCKED);
  rclcpp::shutdown();
}
