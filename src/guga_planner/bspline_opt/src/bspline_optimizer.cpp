#include "bspline_opt/bspline_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include <unsupported/Eigen/Splines>

namespace bspline_opt
{

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

}  // namespace bspline_opt
