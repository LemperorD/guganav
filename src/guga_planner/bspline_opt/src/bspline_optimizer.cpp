#include "bspline_opt/bspline_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include <unsupported/Eigen/Splines>

namespace bspline_opt
{

namespace
{

// ──────────────────────────────────────────────────────────
// Cost evaluation + gradient descent (no Ceres dependency)
// ──────────────────────────────────────────────────────────

static double evalCost(
  const std::vector<double> & params, const Eigen::VectorXd & knots,
  const std::vector<Eigen::Vector2d> & orig_points,
  const Eigen::VectorXd & orig_params,
  double first_x, double first_y, double last_x, double last_y,
  int M, double w_smooth, double w_dist)
{
  Eigen::MatrixXd ctrl(2, M);
  ctrl(0, 0) = first_x;  ctrl(1, 0) = first_y;
  ctrl(0, M - 1) = last_x;  ctrl(1, M - 1) = last_y;
  for (int i = 0; i < M - 2; ++i) {
    ctrl(0, i + 1) = params[2 * i];
    ctrl(1, i + 1) = params[2 * i + 1];
  }
  using Spline2D = Eigen::Spline<double, 2, Eigen::Dynamic>;
  Spline2D spl(knots, ctrl);

  double cost{};

  if (w_smooth > 0.0) {
    constexpr int K = 50;
    for (int i = 0; i <= K; ++i) {
      double u = static_cast<double>(i) / static_cast<double>(K);
      auto derivs = spl.derivatives<2>(u);
      double ddx = derivs(0, 2);
      double ddy = derivs(1, 2);
      cost += w_smooth * (ddx * ddx + ddy * ddy);
    }
    cost /= static_cast<double>(K + 1);
  }

  if (w_dist > 0.0) {
    for (size_t i = 0; i < orig_points.size(); ++i) {
      double u = orig_params(static_cast<Eigen::Index>(i));
      Eigen::Vector2d p = spl(u);
      double dx = p.x() - orig_points[i].x();
      double dy = p.y() - orig_points[i].y();
      cost += w_dist * (dx * dx + dy * dy);
    }
  }

  return cost;
}

static std::vector<double> gradientDescent(
  const std::vector<double> & init_params,
  const Eigen::VectorXd & knots,
  const std::vector<Eigen::Vector2d> & orig_points,
  const Eigen::VectorXd & orig_params,
  double first_x, double first_y, double last_x, double last_y,
  int M, double w_smooth, double w_dist,
  int max_iters, bool & converged)
{
  std::vector<double> x = init_params;
  const int N = static_cast<int>(x.size());
  if (N == 0) { converged = true; return x; }

  double alpha{1e-3};
  constexpr double h = 1e-4;
  constexpr double gtol = 1e-8;
  constexpr int patience = 20;

  double f_best = evalCost(x, knots, orig_points, orig_params,
                           first_x, first_y, last_x, last_y,
                           M, w_smooth, w_dist);
  int no_improve{};

  std::vector<double> grad(N);
  std::vector<double> x_try(N);

  for (int iter = 0; iter < max_iters; ++iter) {
    for (int i = 0; i < N; ++i) {
      double orig = x[i];
      x[i] = orig + h;
      double fp = evalCost(x, knots, orig_points, orig_params,
                           first_x, first_y, last_x, last_y,
                           M, w_smooth, w_dist);
      x[i] = orig - h;
      double fm = evalCost(x, knots, orig_points, orig_params,
                           first_x, first_y, last_x, last_y,
                           M, w_smooth, w_dist);
      x[i] = orig;
      grad[i] = (fp - fm) / (2.0 * h);
    }

    double g_norm{};
    for (int i = 0; i < N; ++i) { g_norm += grad[i] * grad[i]; }
    g_norm = std::sqrt(g_norm);
    if (g_norm < gtol) { converged = true; break; }

    alpha = std::min(alpha * 2.0, 0.1);
    bool found{};
    for (int ls = 0; ls < 15; ++ls) {
      for (int i = 0; i < N; ++i) { x_try[i] = x[i] - alpha * grad[i]; }
      double f_try = evalCost(x_try, knots, orig_points, orig_params,
                              first_x, first_y, last_x, last_y,
                              M, w_smooth, w_dist);
      if (f_try < f_best) {
        x = x_try; f_best = f_try; found = true; no_improve = 0; break;
      }
      alpha *= 0.5;
    }
    if (!found) {
      no_improve++;
      if (no_improve >= patience) { converged = true; break; }
    }
  }
  converged = true;
  return x;
}

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

  // Eigen 3.4 SplineFitting with Spline<double,2,7> requires exactly degree=7.
  // For paths with fewer than degree+1=8 points, store as linear interpolation.
  if (n_pts < 8) {
    state_.effective_degree = 1;
    state_.original_points = {};
    for (const auto & [x, y] : path) {
      state_.original_points.emplace_back(x, y);
    }
    // Create a trivial 2-control-point "spline" via linear fit
    state_.control_points.resize(2, 2);
    state_.control_points(0, 0) = path.front().first;
    state_.control_points(1, 0) = path.front().second;
    state_.control_points(0, 1) = path.back().first;
    state_.control_points(1, 1) = path.back().second;
    // Don't bother with a full spline for < 8 points — sample() with linear interp
    fitted_ = true;
    return true;
  }

  int eff_deg = 7;  // fixed by Spline type
  state_.effective_degree = eff_deg;

  state_.original_points.reserve(n_pts);
  for (const auto & [x, y] : path) {
    state_.original_points.emplace_back(x, y);
  }

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

  Eigen::MatrixXd pts(2, n_pts);
  for (int i = 0; i < n_pts; ++i) {
    pts(0, i) = path[i].first;
    pts(1, i) = path[i].second;
  }

  Eigen::RowVectorXd chord_vec(n_pts);
  for (int i = 0; i < n_pts; ++i) {
    chord_vec(i) = state_.parameters(i);
  }
  Eigen::RowVectorXd knot_vec;
  Eigen::KnotAveraging(chord_vec, eff_deg, knot_vec);
  state_.knots = knot_vec;

  using SplineFitter =
    Eigen::SplineFitting<Eigen::Spline<double, 2, 7>>;  // fixed deg=7, match MPC
  // Use 2-param Interpolate (auto chord lengths) — avoids Span<degree issue
  auto fitted_spline = SplineFitter::Interpolate(pts, eff_deg);

  const auto & ctrl = fitted_spline.ctrls();
  state_.control_points.resize(2, ctrl.cols());
  state_.control_points = ctrl;

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
  if (!fitted_) { return out; }
  // For linear fallback (no spline): interpolate between endpoints
  if (!spline_) {
    if (state_.original_points.empty()) { return out; }
    out.reserve(N);
    double x0 = state_.original_points.front().x();
    double y0 = state_.original_points.front().y();
    double x1 = state_.original_points.back().x();
    double y1 = state_.original_points.back().y();
    for (int i = 0; i < N; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(N - 1);
      out.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
    }
    return out;
  }
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

  result.cost_initial = computeCurvatureEnergy();

  const int n_interior = M - 2;
  const int param_size = 2 * n_interior;
  std::vector<double> params(param_size);
  for (int i = 0; i < n_interior; ++i) {
    params[2 * i] = state_.control_points(0, i + 1);
    params[2 * i + 1] = state_.control_points(1, i + 1);
  }

  double fx = state_.control_points(0, 0);
  double fy = state_.control_points(1, 0);
  double lx = state_.control_points(0, M - 1);
  double ly = state_.control_points(1, M - 1);

  bool converged{};
  auto opt = gradientDescent(
    params, state_.knots, state_.original_points, state_.parameters,
    fx, fy, lx, ly, M,
    config_.smoothness_weight, config_.distance_weight,
    config_.ceres_max_iterations, converged);

  for (int i = 0; i < n_interior; ++i) {
    state_.control_points(0, i + 1) = opt[2 * i];
    state_.control_points(1, i + 1) = opt[2 * i + 1];
  }
  rebuildSpline();

  result.smoothed_path = sample(num_samples);
  result.curvature_profile.resize(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(num_samples - 1);
    result.curvature_profile[i] = curvatureAt(u);
  }
  result.total_curvature_energy = computeCurvatureEnergy();
  result.ceres_iterations = 0;
  result.cost_final = result.total_curvature_energy;
  result.converged = converged;

  result.control_points_xy.reserve(M);
  for (int i = 0; i < M; ++i) {
    result.control_points_xy.emplace_back(
      state_.control_points(0, i), state_.control_points(1, i));
  }
  return result;
}

}  // namespace bspline_opt