#pragma once

#include <cmath>

namespace terrain_analysis {

  /**
   * @brief 网格工具：坐标与下标的换算，以及网格与相邻表示之间的搬运。
   *
   * 这里的函数既不专属于某一半管线，也不持有任何状态：都是"格 ←→ 坐标"和
   * "格 ←→ 点/邻域"的纯换算，两半与测试都可以直接用。线性下标的换算式由各网格
   * 类型自己提供（linearIndex），这里不重复它；网格类型作为模板参数传入，
   * 因此同一份函数对两张网格（将来加第三张）都成立。
   */

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
   * @brief 把平面坐标换算成网格行列下标（坐标 → 格）。
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

  /** @brief 点到锚点的水平距离（坐标 → 标量）。 */
  [[nodiscard]] inline double horizontalDistance(double px, double py,
                                                 double anchor_x,
                                                 double anchor_y) {
    return std::hypot(px - anchor_x, py - anchor_y);
  }

  /**
   * @brief 把网格中央的方形窗口拼成一份点云（格 → 点云）。
   *
   * 窗口大小是网格布局自身的事实，故由调用方给出半宽；中心格与 gridIndex 的
   * 约定一致（锚点落在正中央）。网格布局（宽度、线性下标）取自 VoxelGrid，
   * 因此这里不重复行主序的换算式。
   * @tparam VoxelGrid 提供 WIDTH / HALF_WIDTH / linearIndex 的网格类型。
   * @param cells 按该网格线性下标排列的格子数组。
   * @param half_window 窗口半宽（格数）；0 表示只取中心格。
   * @param out 输出点云，会被先清空。
   */
  template <typename VoxelGrid, typename CellArray, typename Cell>
  void collectWindow(const CellArray& cells, int half_window, Cell& out) {
    out.clear();
    constexpr int CENTER = VoxelGrid::HALF_WIDTH;
    for (int row = CENTER - half_window; row <= CENTER + half_window; row++) {
      for (int column = CENTER - half_window; column <= CENTER + half_window;
           column++) {
        out += *cells[VoxelGrid::linearIndex(row, column)];
      }
    }
  }

  /**
   * @brief 把一个值摊到指定格及其 3×3 邻域（单点 → 邻域）。
   *
   * 邻域膨胀的目的是让每格的结果由约 0.6 m 范围内的观测共同决定，以抗单点噪声；
   * 代价有二，改动时需一并考虑：
   *   - 实际空间分辨率低于标称的格边长；
   *   - 以"每格最少点数"作门限的判据被稀释——只靠邻居的点也能凑够阈值。
   * 越界的邻居直接跳过。
   * @tparam VoxelGrid 提供 WIDTH / linearIndex 的网格类型。
   * @param row 中心格的行下标（须已在网格范围内）。
   * @param col 中心格的列下标（须已在网格范围内）。
   * @param value 要摊开的值。
   * @param cells 按该网格线性下标排列、元素可 push_back 的格子数组。
   */
  template <typename VoxelGrid, typename CellArray>
  void addToNeighborhood3x3(int row, int col, double value, CellArray& cells) {
    constexpr int WIDTH = VoxelGrid::WIDTH;
    for (int delta_row = -1; delta_row <= 1; delta_row++) {
      const int neighbor_row = row + delta_row;
      if (neighbor_row < 0 || neighbor_row >= WIDTH) {
        continue;
      }
      for (int delta_col = -1; delta_col <= 1; delta_col++) {
        const int neighbor_col = col + delta_col;
        if (neighbor_col < 0 || neighbor_col >= WIDTH) {
          continue;
        }
        cells[VoxelGrid::linearIndex(neighbor_row, neighbor_col)].push_back(
            value);
      }
    }
  }

}  // namespace terrain_analysis
