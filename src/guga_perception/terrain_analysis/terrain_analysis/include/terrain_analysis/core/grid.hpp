#pragma once

#include <cstddef>

/**
 * @brief 地形分析使用的固定尺寸网格参数和索引工具。
 *
 * Terrain voxel 用于累积历史点云，Planar voxel 用于地面高度估计。
 * 调用索引函数前必须保证 row 和 col 位于对应网格范围内。
 */
struct TerrainGrid {
  /** @brief Terrain voxel 网格边长。 */
  static constexpr int TERRAIN_VOXEL_WIDTH = 21;
  /** @brief Terrain voxel 网格半边长。 */
  static constexpr int TERRAIN_VOXEL_HALF_WIDTH = (TERRAIN_VOXEL_WIDTH - 1) / 2;
  /** @brief Terrain voxel 网格总单元数。 */
  static constexpr int TERRAIN_VOXEL_NUM = TERRAIN_VOXEL_WIDTH
                                           * TERRAIN_VOXEL_WIDTH;

  /** @brief Planar voxel 网格边长。 */
  static constexpr int PLANAR_VOXEL_WIDTH = 51;
  /** @brief Planar voxel 网格半边长。 */
  static constexpr int PLANAR_VOXEL_HALF_WIDTH = (PLANAR_VOXEL_WIDTH - 1) / 2;
  /** @brief Planar voxel 网格总单元数。 */
  static constexpr int PLANAR_VOXEL_NUM = PLANAR_VOXEL_WIDTH
                                          * PLANAR_VOXEL_WIDTH;

  /**
   * @brief 将 Terrain voxel 的行列坐标转换为线性索引。
   * @param row 行坐标。
   * @param col 列坐标。
   * @return 线性数组索引。
   */
  static constexpr size_t terrainVoxelIndex(int row, int col) {
    return (TERRAIN_VOXEL_WIDTH * row) + col;
  }

  /**
   * @brief 将 Planar voxel 的行列坐标转换为线性索引。
   * @param row 行坐标。
   * @param col 列坐标。
   * @return 线性数组索引。
   */
  static constexpr size_t planarVoxelIndex(int row, int col) {
    return (PLANAR_VOXEL_WIDTH * row) + col;
  }
};
