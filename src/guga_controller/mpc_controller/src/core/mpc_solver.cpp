#include "mpc_controller/core/mpc_solver.hpp"
#include <cmath>
#include <algorithm>

namespace mpc_controller
{

namespace
{

double wrapToPi(double angle)
{
  while (angle > M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

// PGD box-QP solver (same as before)
Eigen::VectorXd solveProjectedGradientQp(
  const Eigen::MatrixXd & H,
  const Eigen::VectorXd & g,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub,
  int max_iter = 200,
  double tol = 1e-4)

{
  const int n = static_cast<int>(H.rows());
  Eigen::VectorXd x(n);
  for (int i = 0; i < n; ++i) {
    x(i) = std::max(lb(i), std::min(ub(i), 0.0));
  }
  Eigen::VectorXd diag_h(n);
  for (int i = 0; i < n; ++i) {
    diag_h(i) = std::max(H(i, i), 1e-6);
  }
  Eigen::VectorXd grad(n), x_proj(n);
  for (int iter = 0; iter < max_iter; ++iter) {
    grad = H * x + g;
    double kkt = 0.0;
    for (int i = 0; i < n; ++i) {
      if (x(i) <= lb(i) + 1e-10) {
        kkt += std::max(0.0, -grad(i));
      } else if (x(i) >= ub(i) - 1e-10) {
        kkt += std::max(0.0, grad(i));
      } else {
        kkt += std::abs(grad(i));
      }
    }
    if (kkt < tol * n) { break; }
    double step = 1.0;
    for (int i = 0; i < n; ++i) {
      step = std::max(step, std::abs(grad(i)) / diag_h(i));
    }
    step = 1.0 / step;
    for (int ls = 0; ls < 20; ++ls) {
      for (int i = 0; i < n; ++i) {
        x_proj(i) = std::max(lb(i), std::min(ub(i),
          x(i) - step * grad(i) / diag_h(i)));
      }
      const Eigen::VectorXd dx = x_proj - x;
      const double df = grad.dot(dx);
      const double qd = 0.5 * dx.transpose() * H * dx;
      if (df + qd <= 1e-12) { break; }
      step *= 0.5;
    }
    x = x_proj;
  }
  return x;
}

}  // anonymous namespace

// ==========================================================================
// MpcSolver
// ==========================================================================
void MpcSolver::configure(const MpcConfig & config)
{
  config_ = config;
  n_vars_ = config.horizon_n * OmniKinematics::NU;
  lb_.resize(n_vars_);
  ub_.resize(n_vars_);
  for (int k = 0; k < config.horizon_n; ++k) {
    const int base = k * 3;
    lb_(base + 0) = config.vx_min;
    lb_(base + 1) = config.vy_min;
    lb_(base + 2) = config.omega_min;
    ub_(base + 0) = config.vx_max;
    ub_(base + 1) = config.vy_max;
    ub_(base + 2) = config.omega_max;
  }
}

void MpcSolver::setWarmStart(const Eigen::Vector3d & u)
{
  (void)u;
}

// ==========================================================================
void MpcSolver::assembleQP(
  const Eigen::Vector3d & x0,
  const ReferenceTrajectory & ref_traj,
  const MpcState & mpc_state,
  Eigen::MatrixXd & H,
  Eigen::VectorXd & g)
{
  const int N = config_.horizon_n;
  const double dt = config_.control_dt;

  H.setZero(n_vars_, n_vars_);
  g.setZero(n_vars_);

  std::vector<Eigen::Matrix3d> A_k(N), B_k(N);
  for (int k = 0; k < N; ++k) {
    const auto & ref = ref_traj.at(k);
    const Eigen::Vector3d x_ref(ref.x, ref.y, ref.theta);
    const Eigen::Vector3d u_ref(ref.vx, ref.vy, ref.omega);
    A_k[k] = OmniKinematics::stateJacobian(x_ref, u_ref, dt);
    B_k[k] = OmniKinematics::controlJacobian(ref.theta, dt);
  }

  const Eigen::DiagonalMatrix<double, 3> Q(config_.qx, config_.qy, config_.qtheta);
  const Eigen::Matrix3d R_mat = Eigen::DiagonalMatrix<double, 3>(
    config_.rvx, config_.rvy, config_.romega).toDenseMatrix();
  const Eigen::Matrix3d Rd_mat = Eigen::DiagonalMatrix<double, 3>(
    config_.rdvx, config_.rdvy, config_.rdomega).toDenseMatrix();

  // ---- H = H_track + H_ctrl + H_smooth ----
  std::vector<Eigen::Matrix3d> M_cur(N);
  for (int j = 0; j < N; ++j) { M_cur[j] = Eigen::Matrix3d::Zero(); }
  for (int k = 1; k <= N; ++k) {
    for (int j = 0; j < k; ++j) {
      if (j == k - 1) { M_cur[j] = B_k[k - 1]; }
      else            { M_cur[j] = A_k[k - 1] * M_cur[j]; }
      for (int l = 0; l <= j; ++l) {
        const Eigen::Matrix3d contrib = M_cur[j].transpose() * Q * M_cur[l];
        H.block<3, 3>(3 * j, 3 * l) += contrib;
        if (j != l) { H.block<3, 3>(3 * l, 3 * j) += contrib.transpose(); }
      }
    }
  }
  for (int j = 0; j < N; ++j) { H.block<3, 3>(3 * j, 3 * j) += R_mat; }

  for (int j = 0; j < N; ++j) {
    if (j == 0) { H.block<3, 3>(0, 0) += Rd_mat; }
    else {
      H.block<3, 3>(3 * j, 3 * j) += Rd_mat;
      H.block<3, 3>(3 * (j - 1), 3 * (j - 1)) += Rd_mat;
      H.block<3, 3>(3 * j, 3 * (j - 1)) -= Rd_mat;
      H.block<3, 3>(3 * (j - 1), 3 * j) -= Rd_mat;
    }
  }

  // ---- g vector ----
  std::vector<Eigen::Matrix3d> Mg(N);
  for (int j = 0; j < N; ++j) { Mg[j] = Eigen::Matrix3d::Zero(); }
  Eigen::Vector3d x_free = x0;
  for (int k = 1; k <= N; ++k) {
    x_free = A_k[k - 1] * x_free;
    const Eigen::Vector3d x_ref(ref_traj.at(k - 1).x, ref_traj.at(k - 1).y, ref_traj.at(k - 1).theta);
    Eigen::Vector3d err = x_free - x_ref;
    err(2) = wrapToPi(err(2));
    for (int j = 0; j < k; ++j) {
      if (j == k - 1) { Mg[j] = B_k[k - 1]; }
      else            { Mg[j] = A_k[k - 1] * Mg[j]; }
      g.segment<3>(3 * j) += 2.0 * Mg[j].transpose() * Q * err;
    }
  }

  Eigen::Vector3d u_prev(mpc_state.last_vx - ref_traj[0].vx,
                          mpc_state.last_vy - ref_traj[0].vy,
                          mpc_state.last_omega - ref_traj[0].omega);
  g.segment<3>(0) -= 2.0 * Rd_mat * u_prev;

  for (int i = 0; i < n_vars_; ++i) { H(i, i) += 1e-6; }
}

// ==========================================================================
Eigen::Vector3d MpcSolver::solve(
  const Eigen::Vector3d & x0,
  const ReferenceTrajectory & ref_traj,
  MpcState & mpc_state)
{
  if (static_cast<int>(ref_traj.size()) < config_.horizon_n) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  assembleQP(x0, ref_traj, mpc_state, H, g);

  const Eigen::VectorXd du_vec = solveProjectedGradientQp(H, g, lb_, ub_);
  const Eigen::Vector3d du_opt = du_vec.segment<3>(0);

  mpc_state.last_vx = du_opt(0) + ref_traj[0].vx;
  mpc_state.last_vy = du_opt(1) + ref_traj[0].vy;
  mpc_state.last_omega = du_opt(2) + ref_traj[0].omega;

  return Eigen::Vector3d(du_opt(0) + ref_traj[0].vx,
                          du_opt(1) + ref_traj[0].vy,
                          du_opt(2) + ref_traj[0].omega);
}

}  // namespace mpc_controller
