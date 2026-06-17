#pragma once

#include <Eigen/Dense>
#include <vector>
#include "mpc_controller/core/types.hpp"
#include "mpc_controller/core/omni_kinematics.hpp"

namespace mpc_controller
{

/**
 * @brief LTV-MPC QP 求解器。
 *
 * 将非线性 MPC 问题在每个控制周期线性化，组装为标准 QP：
 *
 * @qp  min  0.5·ΔUᵀ·H·ΔU + gᵀ·ΔU
 *      s.t. lb ≤ ΔU ≤ ub
 *
 * 其中 ΔU = [ũ₀, ..., ũ_{N-1}] 是相对于参考控制的偏差序列。
 *
 * 内置 Projected Gradient Descent 求解器（无外部依赖）。
 * 当 CMake 定义 HAS_OSQP 时，启用 OSQP C API 加速。
 */
class MpcSolver
{
public:
  MpcSolver() = default;
  ~MpcSolver() = default;

  // 禁止拷贝
  MpcSolver(const MpcSolver &) = delete;
  MpcSolver & operator=(const MpcSolver &) = delete;

  /** @brief 初始化 QP 维度（分配内存）。 */
  void configure(const MpcConfig & config);

  /** @brief 设置初始参考控制（用于第一次求解时的线性化点）。 */
  void setWarmStart(const Eigen::Vector3d & u);

  /**
   * @brief 求解 MPC 并返回最优控制。
   * @param x0 当前机器人状态 [x, y, θ] (世界系)
   * @param ref_traj 预测时域内 N 个参考轨迹点
   * @param mpc_state 运行时状态（输入 / 输出 last_vx 等）
   * @return 最优控制 [vx, vy, omega]
   */
  [[nodiscard]] Eigen::Vector3d solve(
    const Eigen::Vector3d & x0,
    const ReferenceTrajectory & ref_traj,
    MpcState & mpc_state);

private:
  /** @brief 组装 Hessian H 和梯度 g（稠密形式）。 */
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
