#include "mpc_controller/core/omni_kinematics.hpp"

namespace mpc_controller
{

// ==========================================================================
Eigen::Vector3d OmniKinematics::fCont(
  const Eigen::Vector3d & x,
  const Eigen::Vector3d & u)
{
  const double theta = x(2);
  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);

  return {
    u(0) * cos_t - u(1) * sin_t,   // ẋ
    u(0) * sin_t + u(1) * cos_t,   // ẏ
    u(2)                           // θ̇
  };
}

// ==========================================================================
Eigen::Vector3d OmniKinematics::forwardEuler(
  const Eigen::Vector3d & x,
  const Eigen::Vector3d & u,
  double dt)
{
  return x + dt * fCont(x, u);
}

// ==========================================================================
Eigen::Vector3d OmniKinematics::rk4Step(
  const Eigen::Vector3d & x,
  const Eigen::Vector3d & u,
  double dt)
{
  const Eigen::Vector3d k1 = fCont(x, u);
  const Eigen::Vector3d k2 = fCont(x + 0.5 * dt * k1, u);
  const Eigen::Vector3d k3 = fCont(x + 0.5 * dt * k2, u);
  const Eigen::Vector3d k4 = fCont(x + dt * k3, u);

  return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// ==========================================================================
Eigen::Matrix3d OmniKinematics::stateJacobian(
  const Eigen::Vector3d & x_ref,
  const Eigen::Vector3d & u_ref,
  double dt)
{
  const double theta = x_ref(2);
  const double vx = u_ref(0);
  const double vy = u_ref(1);
  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);

  // ∂f/∂x at (x_ref, u_ref):
  // ∂ẋ/∂θ = -vx*sinθ - vy*cosθ
  // ∂ẏ/∂θ =  vx*cosθ - vy*sinθ
  Eigen::Matrix3d A;
  A << 1.0, 0.0, dt * (-vx * sin_t - vy * cos_t),
       0.0, 1.0, dt * ( vx * cos_t - vy * sin_t),
       0.0, 0.0, 1.0;
  return A;
}

// ==========================================================================
Eigen::Matrix3d OmniKinematics::controlJacobian(
  double theta_ref,
  double dt)
{
  const double cos_t = std::cos(theta_ref);
  const double sin_t = std::sin(theta_ref);

  Eigen::Matrix3d B;
  B << cos_t, -sin_t, 0.0,
       sin_t,  cos_t, 0.0,
       0.0,    0.0,  1.0;
  return dt * B;
}

// ==========================================================================
Eigen::Matrix3d OmniKinematics::rotationMatrixZ(double theta)
{
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R(0, 0) = c;  R(0, 1) = -s;
  R(1, 0) = s;  R(1, 1) =  c;
  return R;
}

}  // namespace mpc_controller
