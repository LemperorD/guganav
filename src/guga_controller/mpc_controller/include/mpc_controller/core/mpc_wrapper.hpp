#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "mpc_controller/core/types.hpp"
#include "mpc_controller/core/omni_kinematics.hpp"

namespace mpc_controller
{

/**
 * @brief Modle A MPC 求解器 — 原生 C++ 实现。
 *
 * LTV-MPC QP 求解，使用内置 Projected Gradient Descent。
 * 零外部依赖（除 Eigen3）。
 */
class MpcSolver
{
public:
  MpcSolver() = default;
  ~MpcSolver() = default;

  MpcSolver(const MpcSolver &) = delete;
  MpcSolver & operator=(const MpcSolver &) = delete;

  void configure(const MpcConfig & config);
  void setWarmStart(const Eigen::Vector3d & u);

  [[nodiscard]] Eigen::Vector3d solve(
    const Eigen::Vector3d & x0,
    const ReferenceTrajectory & ref_traj,
    MpcState & mpc_state);

private:
  void assembleQP(
    const Eigen::Vector3d & x0,
    const ReferenceTrajectory & ref_traj,
    const MpcState & mpc_state,
    Eigen::MatrixXd & H,
    Eigen::VectorXd & g);

  MpcConfig config_;
  int n_vars_{};
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
};

}  // namespace mpc_controller
