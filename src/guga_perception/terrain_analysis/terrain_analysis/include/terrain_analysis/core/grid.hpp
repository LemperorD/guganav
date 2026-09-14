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
   * @brief 车顶上方安全间隙，单位为米（固定 10 cm，不对外暴露为参数）。
   *
   * 两处使用，语义一致——"比车顶高出一个安全间隙"：
   *   - estimateTerrainGround：相对车高达到该值的点（天花板/横梁）不参与地面
   *     估计。窄隧道里这类点占比大，混入分位数会抬高 elev，导致真实地面点
   *     高度差变为负值丢失、天花板点高度差落入障碍区间；
   *   - computeHeightMap：相对车高达到该值的点不作为障碍输出（车辆可从下方
   *     通过）。
   *
   * 固定为常量而非 ROS 参数：它同时是地面候选的有效上界，而该上界还会与
   * max_relative_z 竞争——0.1 < max_relative_z(默认 0.5) 由本常量保证，
   * 故上界恒归它，不存在被遮蔽而语义静默反转的可能。若需按实车隧道顶隙调整，
   * 改此处并重新编译，同时确认仍小于 max_relative_z。
   */
  static constexpr double CEILING_CLEARANCE = 0.1;

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
