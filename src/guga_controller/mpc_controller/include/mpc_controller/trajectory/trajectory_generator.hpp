#pragma once

#include "mpc_controller/core/types.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mpc_controller
{

/**
 * @brief 轨迹生成器抽象接口。
 *
 * 将 Nav2 Path（或自定义格式）转换为预测时域内的等距 ReferencePoint 序列。
 */
class TrajectoryGenerator
{
public:
  virtual ~TrajectoryGenerator() = default;

  /**
   * @param path      输入路径（通常是车体坐标系下的 local path）
   * @param horizon_n MPC 预测时域步数 N
   * @param dt       控制步长
   * @param lookahead 前瞻距离 [m]
   * @return 预测时域内 N 个等距 ReferencePoint
   */
  [[nodiscard]] virtual ReferenceTrajectory generate(
    const nav_msgs::msg::Path & path,
    int horizon_n,
    double dt,
    double lookahead) = 0;

  /** @brief 计算弧长 s 处的路径曲率。 */
  [[nodiscard]] virtual double curvature(double s) const = 0;

  /** @brief 总弧长。 */
  [[nodiscard]] virtual double arcLength() const = 0;

  /** @brief 获取轨迹生成模式。 */
  [[nodiscard]] virtual TrajectoryMode mode() const = 0;
};

}  // namespace mpc_controller
