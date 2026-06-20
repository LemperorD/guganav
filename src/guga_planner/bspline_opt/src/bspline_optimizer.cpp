#include "bspline_opt/bspline_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include <ceres/ceres.h>
#include <unsupported/Eigen/Splines>

namespace bspline_opt
{

namespace
{

// ──────────────────────────────────────────────────────────
// Ceres cost functors (numeric diff — no autodiff needed
// for this small dense problem)
// ──────────────────────────────────────────────────────────

/**
 * @brief Cost functor: penalize ‖C''(u)‖² (curvature energy integrand).
 *
 * Parameter block: flattened control points (2×M doubles).
 * First and last control points are NOT in the block — they are fixed.
 */
struct SmoothnessCostFunctor
{
  SmoothnessCostFunctor(
    double u, double weight, int degree,
    const Eigen::VectorXd & knots, int n_ctrl)
    : u_(u), weight_(weight), degree_(degree), knots_(knots), n_ctrl_(n_ctrl)
  {
  }

  bool operator()(double const * const * params, double * residual) const
  {
    // Reconstruct full control-point matrix (2 × n_ctrl)
    // params[0] has (n_ctrl - 2) × 2 = 2*n_ctrl - 4 doubles
    // representing all interior control points.
    // First ctrl pt = state_.original_points[0] (fixed)
    // Last ctrl pt  = state_.original_points.back() (fixed)
    Eigen::MatrixXd ctrl(2, n_ctrl_);
    // We receive only interior points. We need to know the fixed endpoints.
    // The optimize() caller sets first_pt and last_pt in the functor context.
    ctrl(0, 0) = first_x_;
    ctrl(1, 0) = first_y_;
    ctrl(0, n_ctrl_ - 1) = last_x_;
    ctrl(1, n_ctrl_ - 1) = last_y_;
    for (int i = 0; i < n_ctrl_ - 2; ++i) {
      ctrl(0, i + 1) = params[0][2 * i];
      ctrl(1, i + 1) = params[0][2 * i + 1];
    }

    // Build spline
    using Spline2D = Eigen::Spline<double, 2, Eigen::Dynamic>;
    Spline2D spl(knots_, ctrl);

    // 2nd derivative at u
    auto derivs = spl.derivatives<2>(u_);
    residual[0] = weight_ * derivs(0, 2);
    residual[1] = weight_ * derivs(1, 2);
    return true;
  }

  double u_, weight_;
  int degree_, n_ctrl_;
  Eigen::VectorXd knots_;
  double first_x_{}, first_y_{}, last_x_{}, last_y_{};
};

/**
 * @brief Cost functor: penalize ‖C(u_i) − p_i‖² (distance from original).
 */
struct DistanceCostFunctor
{
  DistanceCostFunctor(
    double u, double px, double py, double weight,
    const Eigen::VectorXd & knots, int n_ctrl)
    : u_(u), weight_(weight), px_(px), py_(py), knots_(knots), n_ctrl_(n_ctrl)
  {
  }

  bool operator()(double const * const * params, double * residual) const
  {
    Eigen::MatrixXd ctrl(2, n_ctrl_);
    ctrl(0, 0) = first_x_;
    ctrl(1, 0) = first_y_;
    ctrl(0, n_ctrl_ - 1) = last_x_;
    ctrl(1, n_ctrl_ - 1) = last_y_;
    for (int i = 0; i < n_ctrl_ - 2; ++i) {
      ctrl(0, i + 1) = params[0][2 * i];
      ctrl(1, i + 1) = params[0][2 * i + 1];
    }

    using Spline2D = Eigen::Spline<double, 2, Eigen::Dynamic>;
    Spline2D spl(knots_, ctrl);
    Eigen::Vector2d p = spl(u_);
    residual[0] = weight_ * (p.x() - px_);
    residual[1] = weight_ * (p.y() - py_);
    return true;
  }

  double u_, weight_, px_, py_;
  int n_ctrl_;
  Eigen::VectorXd knots_;
  double first_x_{}, first_y_{}, last_x_{}, last_y_{};
};

}  // namespace

// ──────────────────────────────────────────────────────────
// Public methods
// ──────────────────────────────────────────────────────────

BSplineOptimizer::BSplineOptimizer(const BSplineConfig & config)
    : config_(config)
{
}

bool BSplineOptimizer::fit(
  const std::vector<std::pair<double, double>> & path)
{
  fitted_ = false;
  state_ = {};
  spline_.reset();

  const int n_pts = static_cast<int>(path.size());
  if (n_pts < 2) { return false; }

  // ── Auto-reduce degree for very short paths ──
  int eff_deg = config_.degree;
  if (n_pts <= eff_deg) { eff_deg = n_pts - 1; }
  if (eff_deg < 1) { eff_deg = 1; }
  state_.effective_degree = eff_deg;

  // ── Store original points ──
  state_.original_points.reserve(n_pts);
  for (const auto & [x, y] : path) {
    state_.original_points.emplace_back(x, y);
  }

  // ── Chord-length parameterization ──
  std::vector<double> arc_lengths(n_pts);
  arc_lengths[0] = 0.0;
  for (int i = 1; i < n_pts; ++i) {
    double dx = path[i].first - path[i - 1].first;
    double dy = path[i].second - path[i - 1].second;
    arc_lengths[i] = arc_lengths[i - 1] + std::hypot(dx, dy);
  }
  state_.total_arc_length = arc_lengths.back();
  if (state_.total_arc_length < 1e-9) { return false; }

  state_.parameters.resize(n_pts);
  for (int i = 0; i < n_pts; ++i) {
    state_.parameters[i] = arc_lengths[i] / state_.total_arc_length;
  }
  state_.parameters[n_pts - 1] = 1.0;

  // ── Build point matrix (2 × N) ──
  Eigen::MatrixXd pts(2, n_pts);
  for (int i = 0; i < n_pts; ++i) {
    pts(0, i) = path[i].first;
    pts(1, i) = path[i].second;
  }

  // ── Knot averaging ──
  Eigen::VectorXd knot_vec;
  Eigen::KnotAveraging(state_.parameters, eff_deg, knot_vec);

  state_.knots = knot_vec;

  // ── B-spline interpolation ──
  using SplineFitter =
    Eigen::SplineFitting<Eigen::Spline<double, 2, Eigen::Dynamic>>;
  auto fitted_spline = SplineFitter::Interpolate(pts, eff_deg, knot_vec);

  // ── Extract control points ──
  const auto & ctrl = fitted_spline.ctrls();
  state_.control_points.resize(2, ctrl.cols());
  state_.control_points = ctrl;

  // ── Build persistent spline for evaluation ──
  rebuildSpline();

  fitted_ = true;
  return true;
}

void BSplineOptimizer::rebuildSpline()
{
  if (state_.control_points.cols() < state_.effective_degree + 1) { return; }
  spline_ = std::make_unique<Spline2D>(state_.knots, state_.control_points);
}

std::vector<std::pair<double, double>> BSplineOptimizer::sample(int N) const
{
  std::vector<std::pair<double, double>> out{};
  if (!fitted_ || !spline_) { return out; }
  out.reserve(N);
  for (int i = 0; i < N; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(N - 1);
    Eigen::Vector2d p = (*spline_)(u);
    out.emplace_back(p.x(), p.y());
  }
  return out;
}

double BSplineOptimizer::curvatureAt(double u) const
{
  if (!fitted_ || !spline_) { return 0.0; }

  // Eigen::Spline::derivatives() returns a matrix:
  // col 0 = value, col 1 = first deriv, col 2 = second deriv
  Eigen::Matrix<double, 2, 3> derivs = spline_->derivatives<2>(u);
  double dx = derivs(0, 1);
  double dy = derivs(1, 1);
  double ddx = derivs(0, 2);
  double ddy = derivs(1, 2);

  double denom = dx * dx + dy * dy;
  if (denom < 1e-12) { return 0.0; }
  return std::abs(dx * ddy - dy * ddx) / (denom * std::sqrt(denom));
}

double BSplineOptimizer::computeCurvatureEnergy() const
{
  if (!fitted_ || !spline_) { return 0.0; }
  double energy{};
  constexpr int K = 200;
  for (int i = 0; i <= K; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(K);
    auto derivs = spline_->derivatives<2>(u);
    double ddx = derivs(0, 2);
    double ddy = derivs(1, 2);
    energy += (ddx * ddx + ddy * ddy);
  }
  return energy / static_cast<double>(K + 1);
}

BSplineResult BSplineOptimizer::optimize(int num_samples)
{
  BSplineResult result{};
  if (!fitted_ || !spline_) { return result; }

  const int M = static_cast<int>(state_.control_points.cols());
  if (M < 3) {
    // Too few control points for meaningful optimization — just sample
    result.smoothed_path = sample(num_samples);
    result.curvature_profile.resize(num_samples);
    for (int i = 0; i < num_samples; ++i) {
      double u = static_cast<double>(i) / static_cast<double>(num_samples - 1);
      result.curvature_profile[i] = curvatureAt(u);
    }
    result.total_curvature_energy = computeCurvatureEnergy();
    result.converged = true;
    return result;
  }

  // Save initial cost
  result.cost_initial = computeCurvatureEnergy();

  // ── Setup Ceres problem ──
  ceres::Problem problem;

  // Parameter block: flattened interior control points (2 × (M-2) doubles)
  // Exclude first and last (fixed).
  const int n_interior = M - 2;
  const int param_size = 2 * n_interior;

  std::vector<double> params(param_size);
  for (int i = 0; i < n_interior; ++i) {
    params[2 * i] = state_.control_points(0, i + 1);
    params[2 * i + 1] = state_.control_points(1, i + 1);
  }

  // Save fixed endpoints
  double first_x = state_.control_points(0, 0);
  double first_y = state_.control_points(1, 0);
  double last_x = state_.control_points(0, M - 1);
  double last_y = state_.control_points(1, M - 1);

  // ── Smoothness cost (sample at K parameter values) ──
  if (config_.smoothness_weight > 0.0) {
    constexpr int K = 60;
    for (int i = 0; i <= K; ++i) {
      double u = static_cast<double>(i) / static_cast<double>(K);
      auto * cost = new SmoothnessCostFunctor(
        u, config_.smoothness_weight, state_.effective_degree,
        state_.knots, M);
      cost->first_x_ = first_x;
      cost->first_y_ = first_y;
      cost->last_x_ = last_x;
      cost->last_y_ = last_y;
      auto * cost_fn = new ceres::DynamicNumericDiffCostFunction(
        cost, ceres::DO_NOT_TAKE_OWNERSHIP);
      cost_fn->AddParameterBlock(param_size);
      cost_fn->SetNumResiduals(2);
      problem.AddResidualBlock(cost_fn, nullptr, params.data());
    }
  }

  // ── Distance cost (stay close to original path) ──
  if (config_.distance_weight > 0.0) {
    const int N_orig = static_cast<int>(state_.original_points.size());
    for (int i = 0; i < N_orig; ++i) {
      double u = state_.parameters(i);
      double px = state_.original_points[i].x();
      double py = state_.original_points[i].y();
      auto * cost = new DistanceCostFunctor(
        u, px, py, config_.distance_weight, state_.knots, M);
      cost->first_x_ = first_x;
      cost->first_y_ = first_y;
      cost->last_x_ = last_x;
      cost->last_y_ = last_y;
      auto * cost_fn = new ceres::DynamicNumericDiffCostFunction(
        cost, ceres::DO_NOT_TAKE_OWNERSHIP);
      cost_fn->AddParameterBlock(param_size);
      cost_fn->SetNumResiduals(2);
      problem.AddResidualBlock(cost_fn, nullptr, params.data());
    }
  }

  // ── Solve ──
  ceres::Solver::Options options;
  options.max_num_iterations = config_.ceres_max_iterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // ── Copy optimized control points back ──
  // First and last stay unchanged
  for (int i = 0; i < n_interior; ++i) {
    state_.control_points(0, i + 1) = params[2 * i];
    state_.control_points(1, i + 1) = params[2 * i + 1];
  }

  // ── Rebuild spline ──
  rebuildSpline();

  // ── Produce output ──
  result.smoothed_path = sample(num_samples);

  result.curvature_profile.resize(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(num_samples - 1);
    result.curvature_profile[i] = curvatureAt(u);
  }

  result.total_curvature_energy = computeCurvatureEnergy();
  result.ceres_iterations = summary.iterations.size();
  result.cost_final = result.total_curvature_energy;
  result.converged = summary.IsSolutionUsable();

  // Extract control points as vector of pairs
  result.control_points_xy.reserve(M);
  for (int i = 0; i < M; ++i) {
    result.control_points_xy.emplace_back(
      state_.control_points(0, i), state_.control_points(1, i));
  }

  return result;
}

}  // namespace bspline_opt
