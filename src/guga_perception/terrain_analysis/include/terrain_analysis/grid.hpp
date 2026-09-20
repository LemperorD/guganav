#pragma once

/**
 * @brief 两张固定尺寸的网格类型。
 *
 * 前半段的 PersistentVoxelGrid 累积点云并随车滚动，后半段的 PerFrameHeightGrid
 * 逐帧存地面高度。两者的布局约定相同（行主序，线性下标 = 宽度 × 行 + 列），
 * 由各自的 linearIndex 提供；坐标与下标之间的换算、以及格与点云/邻域的搬运，
 * 见同目录的 grid_utils.hpp。
 *
 * 命名空间沿用本包约定：数据结构（本文件的两个类型，与 PersistentVoxelConfig 等
 * 一样）在全局命名空间，算法与工具函数在 terrain_analysis 内。
 */

#include <cstddef>

/**
 * @brief 前半段（PersistentVoxelMap）使用的固定网格：累积历史点云的滑动窗口。
 *
 * 与 PerFrameHeightGrid 相互独立：本网格随车滚动、存点云，尺寸由 1 m 格决定；
 * 那个网格锚定在车上、存地面高度，尺寸由 0.2 m 格决定。两者过去挤在同一个
 * TerrainGrid 里，各自的使用方看不出边界，故拆开。
 */
struct PersistentVoxelGrid {
  static constexpr int WIDTH = 21;
  /** @brief 半边长即锚点所在格的下标，故 WIDTH 取奇数。 */
  static constexpr int HALF_WIDTH = (WIDTH - 1) / 2;
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

/**
 * @brief 后半段（PerFrameHeightMap）使用的固定网格：逐格估计的地面高度。
 *
 * 本网格不存点云，只存每格的地面高程与地面候选高度；锚点是雷达当前位置，
 * 不随车移动而滚动，超出窗口的点直接不参与（见 estimateTerrainGround）。
 */
struct PerFrameHeightGrid {
  static constexpr int WIDTH = 51;
  /** @brief 半边长即锚点所在格的下标，故 WIDTH 取奇数。 */
  static constexpr int HALF_WIDTH = (WIDTH - 1) / 2;
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
