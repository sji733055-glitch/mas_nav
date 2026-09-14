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
}
