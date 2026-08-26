// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_vel_transform/fake_vel_transform.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_odom_topic", "");
  this->declare_parameter<float>("init_spin_speed", 0.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("output_odom_topic", output_odom_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);
  if (!output_odom_topic_.empty()) {
    odom_fake_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
  }

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  // odom 必须显式指定 best_effort：上游 small_point_lio 以 QoS(10).best_effort() 发布 /Odometry，
  // 而 message_filters::Subscriber::subscribe(node, topic) 默认走 rmw_qos_profile_default（reliable），
  // QoS 不兼容会让本节点一条 odom 都收不到，current_robot_base_angle_ 永远停在 0。
  rmw_qos_profile_t odom_qos_profile = rmw_qos_profile_sensor_data;
  odom_qos_profile.depth = 10;  // 与发布端 depth 对齐，避免 odom 高频时在 rmw 队列里丢样本
  odom_sub_filter_.subscribe(this, odom_topic_, odom_qos_profile);
  // local_plan 保持 reliable 默认值：minco_controller 也用默认 QoS 发布，
  // 且它是控制链的一部分（同步 + 控制器激活判定），丢包会把下游推进 CONTROLLER_TIMEOUT 分支。
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  publishFakeOdometry(msg);

  // NOTE: Haven't synced with local_plan
  if ((rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
  }
}

// Nav2 把 odom twist 直接当作 `robot_base_frame`（即 fake_robot_base_frame）中的当前速度使用，
// 不做任何坐标变换。而上游里程计按 REP-105 把 twist 表达在随车体转动的 robot_base_frame 中，
// 两者相差一个车体 yaw。若不修正，DWB 的可达速度窗口会被锚在错误的方向上，
// 表现为速度越高、横向偏移越大的持续跑偏。这里把 twist 旋到 fake_robot_base_frame 再发给 Nav2。
void FakeVelTransform::publishFakeOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  if (!odom_fake_pub_) {
    return;
  }
  const double yaw = tf2::getYaw(msg->pose.pose.orientation);
  nav_msgs::msg::Odometry out = *msg;
  out.child_frame_id = fake_robot_base_frame_;
  // fake_robot_base_frame 与 robot_base_frame 相差 Rz(-yaw)，故 v_fake = Rz(+yaw) * v_base
  out.twist.twist.linear.x =
    msg->twist.twist.linear.x * cos(yaw) - msg->twist.twist.linear.y * sin(yaw);
  out.twist.twist.linear.y =
    msg->twist.twist.linear.x * sin(yaw) + msg->twist.twist.linear.y * cos(yaw);
  // fake_robot_base_frame 的 yaw 恒定，相对 odom 没有角速度
  out.twist.twist.angular.x = 0.0;
  out.twist.twist.angular.y = 0.0;
  out.twist.twist.angular.z = 0.0;
  odom_fake_pub_->publish(out);
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (
    is_zero_vel ||
    (rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // If received velocity cannot be synchronized, publish it directly
    auto aft_tf_vel = transformVelocity(msg, current_robot_base_angle_);
    cmd_vel_chassis_pub_->publish(aft_tf_vel);
  } else {
    latest_cmd_vel_ = msg;
  }
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = rclcpp::Clock().now();
}

void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel;
  {
    if (!latest_cmd_vel_) {
      return;
    }
    current_cmd_vel = latest_cmd_vel_;
  }

  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  float yaw_diff = current_robot_base_angle_;
  geometry_msgs::msg::Twist aft_tf_vel = transformVelocity(current_cmd_vel, yaw_diff);

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
