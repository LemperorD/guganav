#include "bspline_opt/bspline_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include <unsupported/Eigen/Splines>

namespace bspline_opt
{

namespace
{

// ──────────────────────────────────────────────────────────
// B-spline evaluation helpers
// ──────────────────────────────────────────────────────────

// Read costmap cell at integer grid coords (bounds-checked).
inline unsigned char cellCost(
  const unsigned char * cmap, int w, int h, int cx, int cy)
{
  if (!cmap || cx < 0 || cx >= w || cy < 0 || cy >= h) { return 255; }
  return cmap[static_cast<size_t>(cy * w + cx)];
}

// Check whether a world-coord point is inside an obstacle cell.
inline bool inObstacle(
  const unsigned char * cmap, int w, int h, double px, double py)
{
  int cx = static_cast<int>(px);
  int cy = static_cast<int>(py);
  return cellCost(cmap, w, h, cx, cy) >= 253;
}

// Project a single point to the nearest free cell (spiral search, r ≤ 8).
// Returns true if projection succeeded.
inline bool projectPointToFree(
  const unsigned char * cmap, int w, int h, double & px, double & py)
{
  int ix = static_cast<int>(px);
  int iy = static_cast<int>(py);
  if (!inObstacle(cmap, w, h, px, py)) { return true; }

  for (int r = 1; r <= 8; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::abs(dx) < r && std::abs(dy) < r) { continue; }
        int tx = ix + dx;
        int ty = iy + dy;
        if (cellCost(cmap, w, h, tx, ty) < 253) {
          px = static_cast<double>(tx) + 0.5;
          py = static_cast<double>(ty) + 0.5;
          return true;
        }
      }
    }
  }
  return false;
}

// ===================================================================
// Gradient descent (only compiled when enabled via config)
// ===================================================================

static double evalCost(
  const std::vector<double> & params, const Eigen::VectorXd & knots,
  const std::vector<Eigen::Vector2d> & orig_points,
  const Eigen::VectorXd & orig_params,
  double first_x, double first_y, double last_x, double last_y,
  int M, double w_smooth, double w_dist, double w_obs,
  const unsigned char * costmap, int cm_w, int cm_h)
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

  if (w_obs > 0.0 && costmap) {
    constexpr int K = 200;
    for (int i = 0; i <= K; ++i) {
      double u = static_cast<double>(i) / static_cast<double>(K);
      Eigen::Vector2d p = spl(u);
      int cx = static_cast<int>(p.x());
      int cy = static_cast<int>(p.y());
      if (cx >= 0 && cx < cm_w && cy >= 0 && cy < cm_h) {
        unsigned char v = costmap[static_cast<size_t>(cy * cm_w + cx)];
        if (v >= 253) {
          double dx = p.x() - static_cast<double>(cx) - 0.5;
          double dy = p.y() - static_cast<double>(cy) - 0.5;
          cost += w_obs * (1.0 + std::abs(dx) + std::abs(dy));
        }
      }
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
  int M, double w_smooth, double w_dist, double w_obs,
  const unsigned char * costmap, int cm_w, int cm_h,
  int max_iters, double corridor_hw, bool & converged)
{
  std::vector<double> x = init_params;
  const int N = static_cast<int>(x.size());
  if (N == 0) { converged = true; return x; }

  double alpha{1e-3};
  constexpr double h = 0.5;  // half-cell step to detect obstacle boundaries
  constexpr double gtol = 1e-8;
  constexpr int patience = 20;

  double f_best = evalCost(x, knots, orig_points, orig_params,
                           first_x, first_y, last_x, last_y,
                           M, w_smooth, w_dist, w_obs, costmap, cm_w, cm_h);
  int no_improve{};

  std::vector<double> grad(N);
  std::vector<double> x_try(N);

  for (int iter = 0; iter < max_iters; ++iter) {
    for (int i = 0; i < N; ++i) {
      double orig = x[i];
      x[i] = orig + h;
      double fp = evalCost(x, knots, orig_points, orig_params,
                           first_x, first_y, last_x, last_y,
                           M, w_smooth, w_dist, w_obs, costmap, cm_w, cm_h);
      x[i] = orig - h;
      double fm = evalCost(x, knots, orig_points, orig_params,
                           first_x, first_y, last_x, last_y,
                           M, w_smooth, w_dist, w_obs, costmap, cm_w, cm_h);
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
      for (int i = 0; i < N; ++i) {
        x_try[i] = x[i] - alpha * grad[i];
        // Corridor constraint: clamp to initial position ± corridor_hw
        x_try[i] = std::clamp(x_try[i], init_params[i] - corridor_hw,
                              init_params[i] + corridor_hw);
      }
      double f_try = evalCost(x_try, knots, orig_points, orig_params,
                              first_x, first_y, last_x, last_y,
                              M, w_smooth, w_dist, w_obs, costmap, cm_w, cm_h);
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
// Public API
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

  // ── Short-path fallback (linear) ──
  if (n_pts < 8) {
    state_.effective_degree = 1;
    state_.original_points.reserve(n_pts);
    for (const auto & [x, y] : path) {
      state_.original_points.emplace_back(x, y);
    }
    state_.control_points.resize(2, 2);
    state_.control_points(0, 0) = path.front().first;
    state_.control_points(1, 0) = path.front().second;
    state_.control_points(0, 1) = path.back().first;
    state_.control_points(1, 1) = path.back().second;
    fitted_ = true;
    return true;
  }

  const int eff_deg = 7;
  state_.effective_degree = eff_deg;

  state_.original_points.reserve(n_pts);
  for (const auto & [x, y] : path) {
    state_.original_points.emplace_back(x, y);
  }

  // Chord-length parameterization
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

  // Point matrix (2 × N)
  Eigen::MatrixXd pts(2, n_pts);
  for (int i = 0; i < n_pts; ++i) {
    pts(0, i) = path[i].first;
    pts(1, i) = path[i].second;
  }

  // Knot averaging
  Eigen::RowVectorXd chord_vec(n_pts);
  for (int i = 0; i < n_pts; ++i) {
    chord_vec(i) = state_.parameters(i);
  }
  Eigen::RowVectorXd knot_vec;
  Eigen::KnotAveraging(chord_vec, eff_deg, knot_vec);
  state_.knots = knot_vec;

  // ── Step 1: Interpolate through ALL N waypoints via SplineFitting ──
  using SplineFitter =
    Eigen::SplineFitting<Eigen::Spline<double, 2, 7>>;
  auto dense_spline = SplineFitter::Interpolate(pts, eff_deg);

  // ── Step 2: Determine control point count ──
  int M = std::min(config_.max_control_points, n_pts);
  if (M < 8) { M = 8; }
  if (M > n_pts) { M = n_pts; }

  // ── Step 3: Re-sample M control points at chord-length parameters ──
  // Instead of uniform spacing, use the chord-length parameters of a subset
  // of the original waypoints. This preserves the geometric shape.
  // Strategy: pick M waypoints uniformly along the ARC LENGTH (not u).
  state_.control_points.resize(2, M);
  for (int i = 0; i < M; ++i) {
    // Map index i (0..M-1) to arc-length fraction, then find nearest chord param
    double target_arc_frac = static_cast<double>(i) / static_cast<double>(M - 1);
    // Find the waypoint index with closest arc-length fraction
    int best_idx = 0;
    double best_dist = 1e9;
    for (int j = 0; j < n_pts; ++j) {
      double d = std::abs(state_.parameters(j) - target_arc_frac);
      if (d < best_dist) { best_dist = d; best_idx = j; }
    }
    // Evaluate the dense spline at this waypoint's chord-length parameter
    double u = state_.parameters(best_idx);
    Eigen::Vector2d p = dense_spline(u);
    state_.control_points(0, i) = p.x();
    state_.control_points(1, i) = p.y();
  }

  // ── Step 4: Create knot vector for M control points ──
  // Use chord-length from the selected M waypoints
  Eigen::RowVectorXd sub_chord(M);
  for (int i = 0; i < M; ++i) {
    sub_chord(i) = static_cast<double>(i) / static_cast<double>(M - 1);
  }
  Eigen::RowVectorXd new_knots;
  Eigen::KnotAveraging(sub_chord, eff_deg, new_knots);
  state_.knots = new_knots;

  // Save initial control-point positions (flat doubles) for corridor constraint
  state_.initial_params.clear();
  state_.initial_params.reserve(2 * (M - 2));
  for (int i = 1; i < M - 1; ++i) {
    state_.initial_params.push_back(state_.control_points(0, i));
    state_.initial_params.push_back(state_.control_points(1, i));
  }

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
  if (!spline_) {
    // Linear fallback
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

  // ── Step 1: Optional gradient descent ──
  if (config_.enable_gradient_descent) {
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
      config_.obstacle_weight,
      state_.costmap_data, state_.costmap_w, state_.costmap_h,
      config_.max_iterations, config_.corridor_halfwidth, converged);

    for (int i = 0; i < n_interior; ++i) {
      state_.control_points(0, i + 1) = opt[2 * i];
      state_.control_points(1, i + 1) = opt[2 * i + 1];
    }
  }

  // ── Step 2: Obstacle-avoidance projection on control points ──
  if (state_.costmap_data != nullptr) {
    for (int i = 1; i < M - 1; ++i) {
      double & cx = state_.control_points(0, i);
      double & cy = state_.control_points(1, i);
      projectPointToFree(state_.costmap_data, state_.costmap_w,
                         state_.costmap_h, cx, cy);
    }
  }

  rebuildSpline();

  // ── Step 3: Sample and check each sample point for obstacles ──
  auto path = sample(num_samples);
  if (state_.costmap_data != nullptr) {
    for (auto & [px, py] : path) {
      projectPointToFree(state_.costmap_data, state_.costmap_w,
                         state_.costmap_h, px, py);
    }
  }

  result.smoothed_path = std::move(path);
  result.curvature_profile.resize(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(num_samples - 1);
    result.curvature_profile[i] = curvatureAt(u);
  }
  result.total_curvature_energy = computeCurvatureEnergy();
  result.cost_final = result.total_curvature_energy;
  result.converged = true;

  result.control_points_xy.reserve(M);
  for (int i = 0; i < M; ++i) {
    result.control_points_xy.emplace_back(
      state_.control_points(0, i), state_.control_points(1, i));
  }
  return result;
}

}  // namespace bspline_opt
