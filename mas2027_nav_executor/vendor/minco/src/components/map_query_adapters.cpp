#include "minco_core/components/map_query_adapters.hpp"

#include "tf2/time.h"

#include <Eigen/Geometry>

#include <cmath>

namespace minco_planner {

namespace {

Eigen::Vector3d transformPoint(const geometry_msgs::msg::TransformStamped & tf, const Eigen::Vector3d & p)
{
  const auto & q_msg = tf.transform.rotation;
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  const Eigen::Vector3d t(
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
  return q * p + t;
}

Eigen::Vector3d rotateVector(const geometry_msgs::msg::TransformStamped & tf, const Eigen::Vector3d & v)
{
  const auto & q_msg = tf.transform.rotation;
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  return q * v;
}

}  // namespace

FrameAwareRogQuery::FrameAwareRogQuery(std::shared_ptr<rog_map::MapQueryInterface> raw,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::string planning_frame,
  std::string rog_frame,
  rclcpp::Logger logger)
: raw_(std::move(raw)), tf_(std::move(tf)), planning_frame_(std::move(planning_frame)),
  rog_frame_(std::move(rog_frame)), logger_(logger)
{
}

bool FrameAwareRogQuery::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (!raw_) {
    return false;
  }
  Eigen::Vector3d p(wx, wy, 0.0);
  if (!transformPlanningToRog(p, p)) {
    return false;
  }
  return raw_->worldToMap(p.x(), p.y(), mx, my);
}

void FrameAwareRogQuery::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  wx = 0.0;
  wy = 0.0;
  if (!raw_) {
    return;
  }
  double rwx = 0.0;
  double rwy = 0.0;
  raw_->mapToWorld(mx, my, rwx, rwy);
  Eigen::Vector3d p(rwx, rwy, 0.0);
  if (!transformRogToPlanning(p, p)) {
    return;
  }
  wx = p.x();
  wy = p.y();
}

unsigned int FrameAwareRogQuery::sizeX() const
{
  return raw_ ? raw_->sizeX() : 0U;
}
unsigned int FrameAwareRogQuery::sizeY() const
{
  return raw_ ? raw_->sizeY() : 0U;
}
double FrameAwareRogQuery::resolution() const
{
  return raw_ ? raw_->resolution() : 0.0;
}
double FrameAwareRogQuery::originX() const
{
  return raw_ ? raw_->originX() : 0.0;
}
double FrameAwareRogQuery::originY() const
{
  return raw_ ? raw_->originY() : 0.0;
}

uint8_t FrameAwareRogQuery::value(unsigned int mx, unsigned int my) const
{
  return raw_ ? raw_->value(mx, my) : kLethalCost;
}

const unsigned char * FrameAwareRogQuery::values() const
{
  return raw_ ? raw_->values() : nullptr;
}

bool FrameAwareRogQuery::copyValues(std::vector<unsigned char> & out) const
{
  return raw_ && raw_->copyValues(out);
}

bool FrameAwareRogQuery::isValid(unsigned int mx, unsigned int my) const
{
  return raw_ && raw_->isValid(mx, my);
}

bool FrameAwareRogQuery::isFree(unsigned int mx, unsigned int my) const
{
  return raw_ && raw_->isFree(mx, my);
}

bool FrameAwareRogQuery::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  const auto result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

rog_map::QueryResult FrameAwareRogQuery::query(const Eigen::Vector3d & pos) const
{
  rog_map::QueryResult result;
  if (!pos.allFinite()) {
    result.status = rog_map::QueryStatus::NONFINITE_INPUT;
    return result;
  }
  if (!raw_) {
    result.status = rog_map::QueryStatus::SNAPSHOT_INVALID;
    return result;
  }
  Eigen::Vector3d rog_pos = pos;
  if (!transformPlanningToRog(pos, rog_pos)) {
    result.status = rog_map::QueryStatus::TF_FAILED;
    return result;
  }
  result = raw_->query(rog_pos);
  if (!result.ok) {
    return result;
  }
  Eigen::Vector3d planning_grad = Eigen::Vector3d::Zero();
  if (rotateRogToPlanning(result.gradient, planning_grad)) {
    result.gradient = planning_grad;
  }
  if (!std::isfinite(result.distance) || !result.gradient.allFinite()) {
    result.ok = false;
    result.status = rog_map::QueryStatus::NONFINITE_OUTPUT;
  }
  return result;
}

bool FrameAwareRogQuery::transformPlanningToRog(const Eigen::Vector3d & in, Eigen::Vector3d & out) const
{
  if (planning_frame_ == rog_frame_) {
    out = in;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(rog_frame_, planning_frame_, tf2::TimePointZero);
    out = transformPoint(tf, in);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "[MincoPlanner] TF %s -> %s failed for ROGMap dynamic query: %s",
      planning_frame_.c_str(),
      rog_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool FrameAwareRogQuery::transformRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const
{
  if (planning_frame_ == rog_frame_) {
    out = in;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(planning_frame_, rog_frame_, tf2::TimePointZero);
    out = transformPoint(tf, in);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "[MincoPlanner] TF %s -> %s failed for ROGMap dynamic query: %s",
      rog_frame_.c_str(),
      planning_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool FrameAwareRogQuery::rotateRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const
{
  if (planning_frame_ == rog_frame_) {
    out = in;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(planning_frame_, rog_frame_, tf2::TimePointZero);
    out = rotateVector(tf, in);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

}  // namespace minco_planner
