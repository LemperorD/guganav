#pragma once

#include "mpc_controller/trajectory/trajectory_generator.hpp"

namespace mpc_controller
{

/**
 * @brief 离散线性插值轨迹生成器。
 *
 * 对 Nav2 Path 航点做线性插值，速度由曲率约束衰减。
 * 这是默认模式，无外部依赖，计算量最小。
 */
class DiscreteGenerator : public TrajectoryGenerator
{
public:
  DiscreteGenerator() = default;

  [[nodiscard]] ReferenceTrajectory generate(
    const nav_msgs::msg::Path & path,
    int horizon_n,
    double dt,
    double lookahead) override;

  [[nodiscard]] double curvature(double s) const override;
  [[nodiscard]] double arcLength() const override { return total_arc_length_; }
  [[nodiscard]] TrajectoryMode mode() const override { return TrajectoryMode::DISCRETE; }

private:
  /** @brief 在累积弧长数组中二分查找最近点。 */
  [[nodiscard]] size_t findIndexAtArcLength(double s) const;

  /** @brief 三点曲率估计。 */
  [[nodiscard]] double estimateCurvature(size_t idx, double forward_dist) const;

  // 内部缓存的航点数据
  struct Waypoint {
    double x{}, y{}, theta{};
    double s{};       // 弧长
  };
  std::vector<Waypoint> waypoints_;
  double total_arc_length_{};
};

}  // namespace mpc_controller
