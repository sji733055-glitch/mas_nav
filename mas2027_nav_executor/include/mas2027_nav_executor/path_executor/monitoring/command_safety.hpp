#pragma once

#include <memory>
#include <string>

#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "mas2027_nav_executor/path_executor/mpc/mpc_types.hpp"
#include "mas2027_nav_executor/path_executor/state/execution_state.hpp"
#include "rog_map/map_query_interface.hpp"
#include "tf2_ros/buffer.h"

namespace mas2027_nav_executor {

/// 命令门（checkCommandSafety）里到底是哪一条判据否决的。
/// 以前只返回 ExecutorStatus，地形层拒绝、TF 查不到、
/// ROGMap 净空不足在日志里都是同一句
/// "Braking: terrain layer or map transform unavailable, or next command violates terrain"，
/// 现场 43 条 Braking 里 28 条属于这一类却无法区分，排障只能猜。加这个结构体把成因带出来。
struct CommandSafetyDetail
{
  /// 命中的判据名（固定字符串，便于 grep/统计）：
  ///   "none"                   通过
  ///   "grid_or_frame_missing"  地形快照、tf 或 rog_query 为空
  ///   "rog_map_stale"          ROGMap 快照时间戳缺失或超时
  ///   "tf_unavailable"         lookupTransform 抛异常（map↔odom 一时拿不到）
  ///   "terrain_transition"     地形层 transition(pos -> pos+horizon*v) 返回 false
  ///   "clearance"              ROGMap 净空在指令方向上不满足要求
  const char * reason{"none"};
  /// 与 reason 对应的量：净空不足时是实测距离与要求值（m），时间为秒，其余为 0。
  double value{0.0};
  double threshold{0.0};
};

ExecutorStatus checkCommandSafety(
  const std::shared_ptr<TerrainGrid> & terrain,
  const std::shared_ptr<rog_map::MapQueryInterface> & rog_query,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & odom_frame,
  double rog_map_clearance,
  double rog_map_timeout_s,
  double dt,
  const minco_controller::State & current,
  const minco_controller::Control & control,
  const rclcpp::Time & stamp,
  CommandSafetyDetail * detail = nullptr);

}  // namespace mas2027_nav_executor
