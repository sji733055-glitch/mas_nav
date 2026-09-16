#ifndef MAS2027_NAV_EXECUTOR__COMMON__ENVIRONMENT__CLEARANCE_GATE_HPP_
#define MAS2027_NAV_EXECUTOR__COMMON__ENVIRONMENT__CLEARANCE_GATE_HPP_

#include <algorithm>
#include <cmath>

namespace mas2027_nav_executor {

/// 近场净空判据（轨迹安全检查、运行时监视、MPC 下一段指令检查共用）。
///
/// 背景：所有净空检查都会从机器人**当前所在的位置**开始采样，而机器人已经站在这里了。
/// 一旦机器人停在离障碍物比要求净空更近的地方（靠墙停放、地图更新把附近的未知格判成占据等），
/// “起点净空不足”就会否决每一条轨迹、每一条指令，车永远不动。
///
/// 因此把「机器人当前已经占住的一小段」单独处理：
///   - 弧长 < near_field 的采样点：只要求不比当前实测净空更差（留 slack 吸收 ESDF 抖动）；
///   - 弧长 >= near_field 的采样点：必须满足完整的 required 净空。
/// 近场半径默认取车体安全半径（collision_dist），即机器人自己身子大小的一块地方。
///
/// 该判据是保守的：ESDF 满足 1-Lipschitz，起点净空 c0 已经说明「没有任何障碍物比 c0 更近」，
/// 因此近场内只允许丢掉 slack（默认 0.02 m，取在 ESDF 逐帧抖动幅度与地图格 0.05 m 之间），
/// 不会放过真正往障碍上撞的轨迹。
struct ClearanceRequirement
{
  /// 近场之外必须满足的净空。
  double required{0.0};
  /// 近场之内必须满足的净空（由当前实测净空减 slack 推出）。
  double near_required{0.0};
  /// 近场弧长半径；<= 0 表示关闭近场放宽，全程都用 required。
  double near_field{0.0};

  /// 按「离起点多远」取当前采样点该满足的净空。
  double requiredAt(double arc_from_start) const
  {
    return (near_field > 1e-6 && arc_from_start < near_field) ? near_required : required;
  }

  bool nearFieldEnabled() const { return near_field > 1e-6; }
};

/// 组装判据。current_clearance_ok 为 false（拿不到当前净空）时不放宽：全程按 required 判。
inline ClearanceRequirement makeClearanceRequirement(
  double required, double near_field, double current_clearance, bool current_clearance_ok,
  double slack = 0.02)
{
  ClearanceRequirement gate;
  gate.required = std::max(0.0, required);
  gate.near_field = std::max(0.0, near_field);
  const double safe_slack = std::max(0.0, slack);
  if (current_clearance_ok && std::isfinite(current_clearance)) {
    gate.near_required = std::min(gate.required, std::max(0.0, current_clearance - safe_slack));
  } else {
    gate.near_required = gate.required;
  }
  return gate;
}

}  // namespace mas2027_nav_executor

#endif  // MAS2027_NAV_EXECUTOR__COMMON__ENVIRONMENT__CLEARANCE_GATE_HPP_
