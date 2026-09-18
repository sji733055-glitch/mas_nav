#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "mas2027_nav_executor/path_executor/mpc/mpc_solver.hpp"

int main()
{
  minco_controller::MPCConfig config;
  config.dt = 0.05;
  config.lookahead_time = 2.0;
  config.horizon = 40;
  config.vx_min = -3.0;
  config.vx_max = 3.0;
  config.vy_min = -3.0;
  config.vy_max = 3.0;
  config.omega_min = 0.0;
  config.omega_max = 0.0;
  config.use_acc_constraints = true;
  config.ax_min = config.ay_min = -4.0;
  config.ax_max = config.ay_max = 4.0;
  config.alpha_min = config.alpha_max = 0.0;
  minco_controller::MpcSolver solver(config);
  minco_controller::State state;
  std::vector<minco_controller::ReferencePoint> reference(40);
  for (size_t i = 0; i < reference.size(); ++i) {
    reference[i].pos.x() = (i + 1) * config.dt;
    reference[i].vel.x() = 1.0;
  }
  minco_controller::Control command;
  const auto start = std::chrono::steady_clock::now();
  assert(solver.solve(state, reference, command));
  const auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  std::cout << "40-step MPC solve: " << elapsed << " ms\n";
  assert(std::isfinite(command.vx));

  config.horizon = 8;
  config.omega_min = -2.0;
  config.omega_max = 2.0;
  config.alpha_min = -4.0;
  config.alpha_max = 4.0;
  std::vector<minco_controller::ReferencePoint> omni_reference(8);
  for (auto & point : omni_reference) {
    point.pos.y() = 0.5;
    point.vel.y() = 0.5;
    point.yaw = 0.3;
  }
  minco_controller::MpcSolver omni_solver(config);
  assert(omni_solver.solve(state, omni_reference, command));
  assert(command.vy > 0.0);
  assert(command.omega > 0.0);

  // The shortest turn from +pi to -pi is positive, not a full negative revolution.
  state.yaw = 3.1;
  for (auto & point : omni_reference) {
    point.pos.y() = 0.0;
    point.vel.y() = 0.0;
    point.yaw = -3.1;
  }
  omni_solver.resetLastControl();
  assert(omni_solver.solve(state, omni_reference, command));
  assert(command.omega > 0.0);

  state.yaw = -3.1;
  for (auto & point : omni_reference) point.yaw = 3.1;
  omni_solver.resetLastControl();
  assert(omni_solver.solve(state, omni_reference, command));
  assert(command.omega < 0.0);

  // 【2026-09-17】加速度锚点语义：清零 vs 锚到实测车速，恢复后的第一拍差 1.3 m/s。
  //
  // 现场症状：指令门/20 Hz 监视/缺帧只要打断一拍就 resetLastControl()，恢复后的第一拍被
  // 限成 |u_0| <= a_max·dt = 0.2 m/s，而车还在 1.5 m/s —— 指令被硬砍到 0.2、底盘急刹、
  // 再按每拍 0.2 m/s 爬回来，现场就是"反复启停 / MPC 频繁冷启动"。锚到实测车速后，
  // 第一拍可以从车实际所在的速度接着走（速度/加速度上下限与三道净空门都没变）。
  {
    minco_controller::MPCConfig cruise = config;
    cruise.horizon = 40;
    cruise.vx_min = -3.0;
    cruise.vx_max = 3.0;
    cruise.omega_min = -2.0;
    cruise.omega_max = 2.0;
    cruise.alpha_min = -4.0;
    cruise.alpha_max = 4.0;
    std::vector<minco_controller::ReferencePoint> cruise_ref(40);
    for (size_t i = 0; i < cruise_ref.size(); ++i) {
      // 参考是一条恒速 2.0 m/s 的直线：不受加速度约束时 u_0 会取到 2.0。
      cruise_ref[i].pos.x() = static_cast<double>(i + 1) * cruise.dt * 2.0;
      cruise_ref[i].vel.x() = 2.0;
    }
    minco_controller::State rest;          // 车在原点、静止
    minco_controller::MpcSolver anchor_solver(cruise);

    assert(anchor_solver.solve(rest, cruise_ref, command));
    // 锚点清零（等价于"以为车停着"）：第一拍只能到 a_max·dt = 0.2 m/s。
    assert(command.vx <= 4.0 * cruise.dt + 1e-6);

    // 锚到实测车速 1.5 m/s：第一拍可以到 1.5 + a_max·dt = 1.7 m/s。
    anchor_solver.setLastControl(Eigen::Vector3d(1.5, 0.0, 0.0));
    assert(anchor_solver.solve(rest, cruise_ref, command));
    assert(command.vx >= 1.5);
    assert(command.vx <= 1.5 + 4.0 * cruise.dt + 1e-6);
    std::cout << "anchor semantics: reset cap " << 4.0 * cruise.dt
              << " m/s, measured 1.5 m/s -> first step " << command.vx << " m/s\n";
  }
}
