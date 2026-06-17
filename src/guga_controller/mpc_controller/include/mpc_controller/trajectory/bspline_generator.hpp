#pragma once

#include "mpc_controller/trajectory/trajectory_generator.hpp"
#include "mpc_controller/core/types.hpp"
#include <Eigen/Dense>
#include <vector>

namespace mpc_controller
{

/**
 * @brief B 样条轨迹生成器。
 *
 * 使用 Eigen::SplineFitting 对 Nav2 Path 航点做全局插值，
 * 生成 7 次 B 样条参数化轨迹。knot 矢量由 chord-length 自动计算。
 *
 * 依赖: <unsupported/Eigen/Splines>
 */
class BSplineGenerator : public TrajectoryGenerator
{
public:
  BSplineGenerator() = default;

  [[nodiscard]] ReferenceTrajectory generate(
    const nav_msgs::msg::Path & path,
    int horizon_n,
    double dt,
    double lookahead) override;

  [[nodiscard]] double curvature(double s) const override;
  [[nodiscard]] double arcLength() const override { return total_arc_length_; }
  [[nodiscard]] TrajectoryMode mode() const override { return TrajectoryMode::B_SPLINE; }

  /** @brief B 样条阶数（7 次，对应 MINCO snap 阶次）。 */
  static constexpr int kDegree = 7;

private:
  /** @brief 弧长参数 u(s) 映射 [0, total_arc] → [0, 1]。 */
  [[nodiscard]] double arcToParam(double s) const { return s / total_arc_length_; }

  /** @brief 求弧长 s 处的一阶导数，用于切线方向。 */
  [[nodiscard]] Eigen::Vector2d tangent(double s) const;

  /** @brief 求弧长 s 处的二阶导数，用于曲率。 */
  [[nodiscard]] Eigen::Vector2d secondDeriv(double s) const;

  double total_arc_length_{};
  std::vector<Eigen::Vector2d> ctrl_points_;  // B 样条控制点
  std::vector<double> knots_;                  // knot 矢量

  // chord-length 参数化
  std::vector<double> chord_params_;
  std::vector<double> chord_arc_lengths_;
};

}  // namespace mpc_controller
