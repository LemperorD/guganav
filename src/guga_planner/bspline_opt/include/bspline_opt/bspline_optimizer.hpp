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
 */
struct BSplineConfig
{
  int degree{7};  // Degree 7 to match Eigen 3.4 SplineFitting fixed-degree requirement
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

  /** Knot vector (size = M + degree + 1). Must be RowVectorXd for Eigen Spline ctor. */
  Eigen::RowVectorXd knots{};

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
 */
class BSplineOptimizer
{
public:
  explicit BSplineOptimizer(const BSplineConfig & config = BSplineConfig{});

  [[nodiscard]] bool fit(
    const std::vector<std::pair<double, double>> & path);

  [[nodiscard]] BSplineResult optimize(int num_samples = 200);

  [[nodiscard]] std::vector<std::pair<double, double>> sample(int N) const;

  [[nodiscard]] double curvatureAt(double u) const;

  /** @brief True if fit() has been called successfully. */
  [[nodiscard]] bool isFitted() const { return fitted_; }

  /** @brief Access the internal state (for debugging / tests). */
  [[nodiscard]] const BSplineState & state() const { return state_; }
  [[nodiscard]] const BSplineConfig & config() const { return config_; }

  /**
   * @brief Compute total curvature energy (numerical quadrature on 2nd deriv).
   */
  [[nodiscard]] double computeCurvatureEnergy() const;

private:
  BSplineConfig config_{};
  BSplineState state_{};
  bool fitted_{false};

  using Spline2D = Eigen::Spline<double, 2, 7>;  // fixed degree to avoid Eigen 3.4 Dynamic bug

  void rebuildSpline();

  std::unique_ptr<Spline2D> spline_{};
};

}  // namespace bspline_opt
