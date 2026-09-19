#pragma once

#include <cstddef>

/**
 * @brief 前半段（TerrainVoxelMap）使用的固定网格：累积历史点云的滑动窗口。
 *
 * 与 PlanarVoxelGrid 相互独立：本网格随车滚动、存点云，尺寸由 1 m 格决定；
 * 那个网格锚定在车上、存地面高度，尺寸由 0.2 m 格决定。两者过去挤在同一个
 * TerrainGrid 里，各自的使用方看不出边界，故拆开。
 */
struct TerrainVoxelGrid {
  /** @brief 网格边长（格数）。 */
  static constexpr int WIDTH = 21;
  /** @brief 网格半边长（格数）。 */
  static constexpr int HALF_WIDTH = (WIDTH - 1) / 2;
  /** @brief 网格总格数。 */
  static constexpr int NUM = WIDTH * WIDTH;

  /**
   * @brief 把行列坐标转换为线性下标。
   * @param row 行坐标，须已确认在 [0, WIDTH) 内。
   * @param col 列坐标，须已确认在 [0, WIDTH) 内。
   */
  static constexpr size_t linearIndex(int row, int col) {
    return (WIDTH * row) + col;
  }
};
