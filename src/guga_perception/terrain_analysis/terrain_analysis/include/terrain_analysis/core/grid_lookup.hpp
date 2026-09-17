#pragma once

#include <cmath>

namespace terrain_analysis {

  /**
   * @brief 网格行列下标；越界时 valid 为 false，row/col 无意义。
   *
   * 索引基准由调用方显式传入（anchor_x/anchor_y），不再从任何全局状态读取——
   * 这是"阶段可以单独调用"的前提。
   */
  struct GridIndex {
    int row = 0;
    int col = 0;
    bool valid = false;
  };

  /**
   * @brief 把平面坐标换算成网格行列下标。
   *
   * 半格偏移使格心对齐整数下标：锚点落在网格正中央。
   * @param x 点在 odom 坐标系下的 x。
   * @param y 点在 odom 坐标系下的 y。
   * @param anchor_x 索引基准的 x（雷达位置）。
   * @param anchor_y 索引基准的 y（雷达位置）。
   * @param voxel_size 网格边长。
   * @param width 网格边长（格数，应为奇数）。
   * @return 行列下标；越界时 valid 为 false。
   */
  [[nodiscard]] inline GridIndex gridIndex(double x, double y, double anchor_x,
                                           double anchor_y, double voxel_size,
                                           int width) {
    const double half_voxel = voxel_size / 2;
    const int half_width = (width - 1) / 2;
    const auto axis_index = [&](double coordinate, double anchor) {
      return static_cast<int>(
                 std::floor((coordinate - anchor + half_voxel) / voxel_size))
             + half_width;
    };

    GridIndex out;
    out.row = axis_index(y, anchor_y);
    out.col = axis_index(x, anchor_x);
    out.valid = out.row >= 0 && out.row < width && out.col >= 0
                && out.col < width;
    return out;
  }

  /** @brief 点到锚点的水平距离。 */
  [[nodiscard]] inline double horizontalDistance(double px, double py,
                                                 double anchor_x,
                                                 double anchor_y) {
    return std::hypot(px - anchor_x, py - anchor_y);
  }

}  // namespace terrain_analysis
