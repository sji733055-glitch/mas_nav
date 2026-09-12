#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace tf_maintainer {

class TFMaintainerNode final : public rclcpp::Node
{
public:
  TFMaintainerNode()
  : Node("tf_maintainer"), broadcaster_(*this)
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    map_odom_topic_ = declare_parameter<std::string>("map_odom_topic", "/tf_maintainer/map_to_odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    publish_odom_to_base_ = declare_parameter<bool>("publish_odom_to_base", false);
    publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", false);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odometry(msg); });
    map_odom_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      map_odom_topic_, rclcpp::QoS(1).transient_local(),
      [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg) { on_map_to_odom(msg); });
  }

private:
  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    if (!publish_odom_to_base_ || !msg ||
      msg->header.frame_id != odom_frame_ || msg->child_frame_id != base_frame_) {
      return;
    }
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid odometry TF");
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = p.x;
    transform.transform.translation.y = p.y;
    transform.transform.translation.z = p.z;
    transform.transform.rotation = q;
    broadcaster_.sendTransform(transform);
  }

  void on_map_to_odom(const geometry_msgs::msg::TransformStamped::SharedPtr & msg)
  {
    if (!publish_map_to_odom_ || !msg ||
      msg->header.frame_id != map_frame_ || msg->child_frame_id != odom_frame_) {
      return;
    }
    broadcaster_.sendTransform(*msg);
  }

  std::string odom_topic_;
  std::string map_odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string map_frame_;
  bool publish_odom_to_base_{};
  bool publish_map_to_odom_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr map_odom_sub_;
  tf2_ros::TransformBroadcaster broadcaster_;
};

}  // namespace tf_maintainer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_maintainer::TFMaintainerNode>());
  rclcpp::shutdown();
  return 0;
}
