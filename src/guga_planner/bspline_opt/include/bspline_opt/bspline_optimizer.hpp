#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

namespace bspline_opt
{

/**
 * @brief Immutable configuration for B-spline trajectory optimization.
 *
 * @note  Defaults are tuned so that fit() produces an exact interpolation
 *        and optimize() only runs obstacle-avoidance post-projection.
 *        Gradient descent is OFF by default (true interpolating spline
 *        needs no smoothing).
 */
struct BSplineConfig
{
  int degree{7};                // Fixed degree matching Eigen Spline type

  // ── Optimization weights (used only when gradient descent is enabled) ──
  double smoothness_weight{0.1};  // Low: don't dominate the interpolation
  double distance_weight{10.0};   // Low: spline already interpolates exactly
  double obstacle_weight{50000.0}; // High: strict obstacle avoidance

  // ── Tuning knobs ──
  int max_iterations{200};       // Max gradient descent iterations
  bool enable_gradient_descent{false}; // Default OFF — interpolation is already optimal
  double corridor_halfwidth{2.5}; // Half-width of mobility corridor (cells)
  int max_control_points{200};    // Use ALL interpolated ctrl pts by default
};

/**
 * @brief Internal state built during fit() and mutated during optimize().
 */
struct BSplineState
{
  /** Control points (2 × M): column i holds (x_i, y_i). */
  Eigen::Matrix2Xd control_points{};

  /** Knot vector (size = M + degree + 1). Must be RowVectorXd. */
  Eigen::RowVectorXd knots{};

  /** Chord-length parameters for each original path point in [0, 1]. */
  Eigen::VectorXd parameters{};

  /** Initial control point positions (before optimization), used for
   *  corridor constraint when gradient descent is enabled. */
  std::vector<double> initial_params{};

  /** Total arc length from chord parameterization (m). */
  double total_arc_length{};

  /** Original path points in map coordinates for distance cost. */
  std::vector<Eigen::Vector2d> original_points{};

  /** Effective degree used for fitting. */
  int effective_degree{};

  /** @brief Optional costmap grid for obstacle avoidance (cost >= 253 = obstacle). */
  const unsigned char * costmap_data{nullptr};
  int costmap_w{};
  int costmap_h{};
};

/**
 * @brief Output of optimize() containing smoothed path and metrics.
 */
struct BSplineResult
{
  std::vector<std::pair<double, double>> smoothed_path{};
  std::vector<double> curvature_profile{};
  std::vector<std::pair<double, double>> control_points_xy{};
  double total_curvature_energy{};
  int iterations{};
  double cost_initial{};
  double cost_final{};
  bool converged{};
};

/**
 * @class BSplineOptimizer
 * @brief B-spline fitting with optional gradient-descent optimization.
 *
 * ## Algorithm (default, no gradient descent)
 *
 * ```text
 *   JPS waypoints (N pts)
 *        │
 *        ▼
 *   SplineFitting::Interpolate  ──→  dense B-spline through ALL waypoints
 *        │
 *        ▼ (extract N control points)
 *   BSplineState::control_points
 *        │
 *        ▼ (obstacle-avoidance post-projection on control points)
 *   rebuildSpline() → sample(N_out) → BSplineResult
 * ```
 *
 * ## Cost function (when gradient descent IS enabled)
 *   J(P) = w_s · ∫‖C''‖² du + w_d · Σ‖C(τᵢ)–qᵢ‖² + w_o · Σ penalty(obstacle)
 *
 * @see docs/DESIGN.md for full design rationale.
 */
class BSplineOptimizer
{
public:
  explicit BSplineOptimizer(const BSplineConfig & config = BSplineConfig{});

  /**
   * @brief Fit a degree-7 B-spline to JPS waypoints via SplineFitting::Interpolate.
   *
   * All waypoints are EXACTLY interpolated (control points = N).
   * No down-sampling is performed — call optimize() for obstacle avoidance.
   *
   * @param path  JPS waypoints in map coordinates (≥8 for full B-spline).
   * @return true if fit succeeded.
   */
  [[nodiscard]] bool fit(
    const std::vector<std::pair<double, double>> & path);

  /**
   * @brief Post-process the spline for obstacle avoidance.
   *
   * When gradient descent is OFF (default):
   *   — Projects control points out of obstacle cells
   *   — Checks sampled path against costmap
   *   — Returns BSplineResult with curvature profile
   *
   * When gradient descent is ON:
   *   — Runs numerical gradient descent on interior control points
   *   — Then does the same post-processing as above
   *
   * @param num_samples  Number of evenly-spaced output waypoints.
   * @return Optimization result.
   */
  [[nodiscard]] BSplineResult optimize(int num_samples = 200);

  /** @brief Sample the current spline at N evenly-spaced u in [0,1]. */
  [[nodiscard]] std::vector<std::pair<double, double>> sample(int N) const;

  /** @brief Curvature at parameter u ∈ [0, 1]. */
  [[nodiscard]] double curvatureAt(double u) const;

  /** @brief True if fit() succeeded. */
  [[nodiscard]] bool isFitted() const { return fitted_; }

  /** @brief Access the internal state (const / mutable). */
  [[nodiscard]] const BSplineState & state() const { return state_; }
  [[nodiscard]] BSplineState & state() { return state_; }
  [[nodiscard]] const BSplineConfig & config() const { return config_; }

  /** @brief Compute integrated curvature energy ∫₀¹ ‖C''(u)‖² du. */
  [[nodiscard]] double computeCurvatureEnergy() const;

private:
  BSplineConfig config_{};
  BSplineState state_{};
  bool fitted_{false};

  using Spline2D = Eigen::Spline<double, 2, 7>;

  void rebuildSpline();

  std::unique_ptr<Spline2D> spline_{};
};

}  // namespace bspline_opt
