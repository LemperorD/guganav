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
   * @brief 障碍上方安全间隙，单位为米（固定 10 cm，不对外暴露为参数）。
   *
   * 语义是"距**局部地面**的净空"：距地面达到该值的点不作为障碍输出——净空
   * 足够时车辆可从下方通过（隧道场景）。基准取局部地面（`planar_voxel_elev`）
   * 而非车辆，使判据在坡面上一致。
   *
   * **仅由 computeHeightMap 使用。** 原在 estimateTerrainGround 也用它筛地面
   * 候选，已移除——净空是"障碍能否通过"的判据，与"哪些点属于地面"无关，
   * 放在那里只会按车高砍掉抬升的地面（坡面），并使候选数随车高漂移。
   *
   * 固定为常量而非 ROS 参数：一是它的本意是按实车隧道顶隙定死（如 260 mm
   * 顶隙 → 0.1 有裕量），二是避免暴露后与 max_relative_z 竞争同一"上界"角色
   * 而发生语义静默反转。若需按实车调整，改此处并重新编译。
   *
   * 注意取值偏小会造成功能性漏检：它同时是**可输出障碍的高度上限**，
   * 即高于地面 0.1 m 的点一律不输出为障碍（与 vehicle_height 的意图需对齐）。
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
