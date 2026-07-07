#pragma once

#include "mpc_controller/trajectory/trajectory_generator.hpp"
#include "mpc_controller/trajectory/discrete_generator.hpp"
#include <memory>

namespace mpc_controller
{

/**
 * @brief MINCO 轨迹生成器（占位接口）。
 *
 * 后续将复用 minco_smoother 的 minco::Minco + L-BFGS 优化器，
 * 实现 snap 能量最小化的轨迹优化。当前用 DiscreteGenerator 作为回退。
 */
class MincoGenerator : public TrajectoryGenerator
{
public:
  MincoGenerator() = default;

  [[nodiscard]] ReferenceTrajectory generate(
    const nav_msgs::msg::Path & path,
    int horizon_n,
    double dt,
    double lookahead) override;

  [[nodiscard]] double curvature(double s) const override;
  [[nodiscard]] double arcLength() const override { return fallback_.arcLength(); }
  [[nodiscard]] TrajectoryMode mode() const override { return TrajectoryMode::MINCO; }

private:
  DiscreteGenerator fallback_;
};

}  // namespace mpc_controller
