#pragma once

#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace bspline_opt
{

/**
 * @brief Immutable configuration for B-spline trajectory optimization.
 */
struct BSplineConfig
{
  int degree{5};
  double smoothness_weight{1.0};
  double distance_weight{10.0};
  int ceres_max_iterations{100};
};

/**
 * @brief Internal state built during fit() and mutated during optimize().
 */
struct BSplineState
{
  /** Control points (2 × M): column i holds (x_i, y_i). */
  Eigen::Matrix2Xd control_points{};

  /** Knot vector (size = M + degree + 1). */
  Eigen::VectorXd knots{};

  /** Chord-length parameters for each original path point in [0, 1]. */
  Eigen::VectorXd parameters{};

  /** Total arc length from chord parameterization (m). */
  double total_arc_length{};

  /** Original path points in map coordinates for distance cost. */
  std::vector<Eigen::Vector2d> original_points{};

  /** Effective degree used for fitting (may be reduced for short paths). */
  int effective_degree{};
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
  int ceres_iterations{};
  double cost_initial{};
  double cost_final{};
  bool converged{};
};

/**
 * @class BSplineOptimizer
 * @brief B-spline fitting and Ceres-based trajectory optimization.
 *
 * Follows Pattern A (函数式数据流): stateless except for the fitted spline
 * stored as member state.
 *
 * Usage:
 * @code
 *   BSplineConfig cfg;
 *   BSplineOptimizer opt(cfg);
 *   if (opt.fit(jps_path)) {
 *     auto result = opt.optimize(200);
 *   }
 * @endcode
 */
class BSplineOptimizer
{
public:
  explicit BSplineOptimizer(const BSplineConfig & config = BSplineConfig{});

  /**
   * @brief Fit a B-spline to path points using chord-length parameterization
   *        and knot averaging (Eigen::SplineFitting).
   * @param path  Input waypoints in map coordinates.
   * @return true if fit succeeded.
   */
  [[nodiscard]] bool fit(
    const std::vector<std::pair<double, double>> & path);

  /**
   * @brief Optimize control points via Ceres nonlinear least-squares.
   *        Must call fit() first.
   * @param num_samples  Number of evenly-spaced output waypoints.
   * @return Optimization result with smoothed path and curvature profile.
   */
  [[nodiscard]] BSplineResult optimize(int num_samples = 200);

  /**
   * @brief Sample the current spline at N evenly-spaced parameter values.
   * @param N  Number of samples.
   * @return Waypoints in map coordinates.
   */
  [[nodiscard]] std::vector<std::pair<double, double>> sample(int N) const;

  /**
   * @brief Compute curvature at parameter u ∈ [0, 1].
   * @param u  Spline parameter.
   * @return Curvature κ (ν= 1 / turning radius).
   */
  [[nodiscard]] double curvatureAt(double u) const;

  /** @brief True if fit() has been called successfully. */
  [[nodiscard]] bool isFitted() const { return fitted_; }

  /** @brief Access the internal state (for debugging / tests). */
  [[nodiscard]] const BSplineState & state() const { return state_; }
  [[nodiscard]] const BSplineConfig & config() const { return config_; }

private:
  BSplineConfig config_{};
  BSplineState state_{};
  bool fitted_{false};

  using Spline2D = Eigen::Spline<double, 2, Eigen::Dynamic>;

  /**
   * @brief Rebuild the Eigen Spline object from current state.
   *        Must be called after any modification to control points or knots.
   */
  void rebuildSpline();

  /**
   * @brief Raw pointer to the current spline. Only valid after rebuildSpline().
   */
  std::unique_ptr<Spline2D> spline_{};

  /**
   * @brief Compute total curvature energy ∫₀¹ ‖C''(u)‖² du (numerical quadrature).
   */
  [[nodiscard]] double computeCurvatureEnergy() const;
};

}  // namespace bspline_opt
