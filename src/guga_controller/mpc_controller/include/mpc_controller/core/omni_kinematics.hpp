#pragma once

#include <Eigen/Dense>

namespace mpc_controller
{

/**
 * @brief 全向移动机器人运动学模型及 Jacobian 计算。
 *
 * 状态 x = [x, y, θ]^T  (世界系)
 * 控制 u = [vx, vy, ω]^T  (车体系)
 *
 * @dot  ẋ = R_z(θ)·u
 * @dot  ẏ = R_z(θ)·u
 * @dot  θ̇ = ω
 */
class OmniKinematics
{
public:
  static constexpr int NX = 3;  // 状态维度
  static constexpr int NU = 3;  // 控制维度

  /** @brief RK4 一步递推。 */
  [[nodiscard]] static Eigen::Vector3d rk4Step(
    const Eigen::Vector3d & x,
    const Eigen::Vector3d & u,
    double dt);

  /** @brief 离散动力学 f(x, u) = x + dt·R_z(θ)·u (前向欧拉)。 */
  [[nodiscard]] static Eigen::Vector3d forwardEuler(
    const Eigen::Vector3d & x,
    const Eigen::Vector3d & u,
    double dt);

  /** @brief 状态 Jacobian A_k = ∂f/∂x  (3×3)。 */
  [[nodiscard]] static Eigen::Matrix3d stateJacobian(
    const Eigen::Vector3d & x_ref,
    const Eigen::Vector3d & u_ref,
    double dt);

  /** @brief 控制 Jacobian B_k = ∂f/∂u = dt·R_z(θ) (3×3)。 */
  [[nodiscard]] static Eigen::Matrix3d controlJacobian(
    double theta_ref,
    double dt);

  /** @brief 旋转矩阵 R_z(θ) in 2D (3×3 齐次形态)。 */
  [[nodiscard]] static Eigen::Matrix3d rotationMatrixZ(double theta);

private:
  /** @brief 连续动力学右端 f_cont(x, u)。 */
  [[nodiscard]] static Eigen::Vector3d fCont(
    const Eigen::Vector3d & x,
    const Eigen::Vector3d & u);
};

}  // namespace mpc_controller
