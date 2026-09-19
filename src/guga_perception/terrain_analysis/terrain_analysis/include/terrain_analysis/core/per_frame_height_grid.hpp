#pragma once

#include <cstddef>

/**
 * @brief 后半段（PerFrameHeightMap）使用的固定网格：逐格估计的地面高度。
 *
 * 本网格不存点云，只存每格的地面高程与地面候选高度；锚点是雷达当前位置，
 * 不随车移动而滚动，超出窗口的点直接不参与（见 estimateTerrainGround）。
 */
struct PerFrameHeightGrid {
  /** @brief 网格边长（格数）。 */
  static constexpr int WIDTH = 51;
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
