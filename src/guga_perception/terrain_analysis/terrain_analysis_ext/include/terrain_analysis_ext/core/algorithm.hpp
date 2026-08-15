#pragma once

struct TerrainExtState;

namespace terrain_analysis_ext::algorithm {

  /**
   * @brief 执行扩展地形管线，输出指定半径内的局部地形点云。
   * @param local_terrain_map_radius 局部地图半径，单位为米。
   * @param state 扩展节点运行时状态。
   */
  void runExt(double local_terrain_map_radius, TerrainExtState& state);

  /**
   * @brief 将半径范围内的局部地形点复制到输出点云。
   * @param local_terrain_map_radius 局部地图半径，单位为米。
   * @param state 扩展节点运行时状态。
   */
  void mergeLocalTerrain(double local_terrain_map_radius,
                         TerrainExtState& state);

  /**
   * @brief 计算点相对车辆位置的水平距离。
   * @param px 点的 x 坐标。
   * @param py 点的 y 坐标。
   * @param state 包含车辆位置的运行时状态。
   * @return 水平距离，单位为米。
   */
  [[nodiscard]] double horizontalDistanceTo(double px, double py,
                                            const TerrainExtState& state);

}  // namespace terrain_analysis_ext::algorithm
