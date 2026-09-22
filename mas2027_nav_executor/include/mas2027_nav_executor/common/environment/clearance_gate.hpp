#ifndef MAS2027_NAV_EXECUTOR__COMMON__ENVIRONMENT__CLEARANCE_GATE_HPP_
#define MAS2027_NAV_EXECUTOR__COMMON__ENVIRONMENT__CLEARANCE_GATE_HPP_

#include <algorithm>
#include <cmath>

namespace mas2027_nav_executor {

/// 【唯一定义处】ESDF 逐帧抖动余量：四道净空门须取值一致，否则重现“规划放行、执行刹车”的假否决。
/// 取值不得再下调：0.05 曾把有效硬阈值压到车体半宽以下并导致实车撞墙；量级保持在“几毫米~2 cm”。
inline constexpr double kEsdfJitterTolerance = 0.02;

/// 完整净空要求 → 各门实际使用的有效阈值。四道门都必须经此函数取值。
inline double effectiveClearanceThreshold(double full_requirement)
{
  return std::max(0.0, full_requirement - kEsdfJitterTolerance);
}

/// 近场净空判据：检查都从机器人当前位置开始采样，若已停在比要求净空更近处（如靠墙停放），
/// “起点净空不足”会否决每条轨迹与指令，车一动不动；故对已占住的那段放宽：弧长 < near_field 只要求
/// 不比当前实测净空更差，之外仍须满足完整 required。slack 默认 0.02 m（介于 ESDF 抖动与地图格
/// 0.05 m 之间）；ESDF 满足 1-Lipschitz 保证该放宽最多只会丢掉 slack，不会放过真正撞障的轨迹。
struct ClearanceRequirement
{
  /// 近场之外必须满足的净空。
  double required{0.0};
  /// 近场之内必须满足的净空（由当前实测净空减 slack 推出）。
  double near_required{0.0};
  /// 近场弧长半径（约定取车体安全半径 collision_dist）；<= 0 表示关闭近场放宽，全程用 required。
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
