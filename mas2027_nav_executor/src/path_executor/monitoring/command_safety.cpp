#include "mas2027_nav_executor/path_executor/monitoring/command_safety.hpp"

#include <algorithm>
#include <cmath>

#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mas2027_nav_executor {

ExecutorStatus checkCommandSafety(
  const std::shared_ptr<TerrainGrid> & grid,
  const std::shared_ptr<rog_map::MapQueryInterface> & rog_query,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & odom_frame,
  double rog_map_clearance,
  double dynamic_map_timeout_s,
  double dt,
  const minco_controller::State & current,
  const minco_controller::Control & control,
  const std::vector<minco_controller::ReferencePoint> & reference,
  const rclcpp::Time & stamp,
  CommandSafetyDetail * detail)
{
  auto reject = [detail](ExecutorStatus status, const char * reason, double value = 0.0,
                   double threshold = 0.0) {
    if (detail) {
      detail->reason = reason;
      detail->value = value;
      detail->threshold = threshold;
    }
    return status;
  };
  const auto terrain = grid ? grid->snapshot() : nullptr;
  const auto dynamic = grid ? grid->dynamicSnapshot() : nullptr;
  if (!terrain || !dynamic || !tf || !rog_query) {
    return reject(ExecutorStatus::TERRAIN_BLOCKED, "grid_or_frame_missing");
  }
  const double dynamic_age_s = std::abs((stamp - rclcpp::Time(dynamic->grid.header.stamp)).seconds());
  // 阈值必须给足余量，不能等于发布周期：旁路模式下 /dynamic_cost_map 由 map_server 的
  // 500 ms 定时器发一帧全 0 空图，判据若也是 0.5 s，则每个周期的最后一拍必然越界
  // （实测 age 恒在 0.509~0.521），表现为车每秒被清零两次的"一卡一卡"。
  // 默认 1.5 s = 3 倍发布周期，改动只放宽"地图新鲜度"，不触碰任何净空阈值。
  if (std::isfinite(dynamic_map_timeout_s) && dynamic_map_timeout_s > 0.0 &&
    dynamic_age_s > dynamic_map_timeout_s) {
    return reject(ExecutorStatus::TERRAIN_BLOCKED, "dynamic_stale", dynamic_age_s,
      dynamic_map_timeout_s);
  }
  try {
    const auto transform = tf->lookupTransform(
      terrain->cost.header.frame_id, odom_frame, tf2::TimePointZero);
    const double map_yaw = tf2::getYaw(transform.transform.rotation);
    const double mc = std::cos(map_yaw), ms = std::sin(map_yaw);
    const Eigen::Vector2d position(
      mc * current.x - ms * current.y + transform.transform.translation.x,
      ms * current.x + mc * current.y + transform.transform.translation.y);
    const Eigen::Vector2d velocity(
      mc * control.vx - ms * control.vy,
      ms * control.vx + mc * control.vy);
    const double horizon = std::max(dt, 0.35);
    if (!terrain->transition(position, position + horizon * velocity)) {
      return reject(ExecutorStatus::TERRAIN_BLOCKED, "terrain_transition");
    }
    // 净空判据与规划侧的发布/监视门一致：机器人当前所在位置（近场，车体安全半径以内）
    // 只要求不比现在的实测净空更差，离开近场后必须满足 rog_map_clearance。
    // 否则「贴着墙停下」会让每一条速度指令都在 t=0 处被否决，cmd_vel 恒为 0。
    // 【2026-09-17 统一四道门】完整要求同样要减掉 ESDF 抖动余量（kEsdfJitterTolerance），
    // 与发布前校验/20 Hz 监视/局部种子门取同一个有效阈值。此前本门直接用 rog_map_clearance，
    // 比发布门严 2 cm ⇒ 规划器按 0.26 发布、执行器按 0.28 否决，实车日志里
    // `reason=clearance value=0.263~0.280 threshold=0.280` 7 次全部落在 [0.26,0.28] 这条缝里，
    // 表现为"一顿一顿、反复启停"。近场半径仍取车体安全半径 rog_map_clearance，不变。
    const double effective_clearance = effectiveClearanceThreshold(rog_map_clearance);
    const auto current_clearance =
      rog_query->query(Eigen::Vector3d(current.x, current.y, 0.0));
    const ClearanceRequirement clearance_gate = makeClearanceRequirement(
      effective_clearance, rog_map_clearance, current_clearance.distance, current_clearance.ok);
    const double travel_speed = velocity.norm();
    const int command_steps = std::max(1, static_cast<int>(std::ceil(horizon / dt)));
    for (int i = 0; i <= command_steps; ++i) {
      const double t = horizon * i / command_steps;
      if (!dynamic->freeAt(position + t * velocity)) {
        return reject(ExecutorStatus::DYNAMIC_BLOCKED, "dynamic_horizon", t, horizon);
      }
      const Eigen::Vector3d odom_point(current.x + t * control.vx,
        current.y + t * control.vy, 0.0);
      const auto clearance = rog_query->query(odom_point);
      if (!clearance.ok || clearance.distance <= clearance_gate.requiredAt(t * travel_speed)) {
        return reject(ExecutorStatus::DYNAMIC_BLOCKED,
          clearance.ok ? "clearance" : "clearance_out_of_map",
          clearance.distance,
          clearance_gate.requiredAt(t * travel_speed));
      }
    }
    for (const auto & point : reference) {
      const Eigen::Vector2d & p = point.pos;
      const Eigen::Vector2d map_point(
        mc * p.x() - ms * p.y() + transform.transform.translation.x,
        ms * p.x() + mc * p.y() + transform.transform.translation.y);
      if (!dynamic->freeAt(map_point)) {
        return reject(ExecutorStatus::DYNAMIC_BLOCKED, "dynamic_reference");
      }
    }
  } catch (const tf2::TransformException &) {
    return reject(ExecutorStatus::TERRAIN_BLOCKED, "tf_unavailable");
  }
  return reject(ExecutorStatus::PUBLISHED, "none");
}

}  // namespace mas2027_nav_executor
