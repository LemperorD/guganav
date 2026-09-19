#pragma once

/**
 * @brief 管线前半段（PersistentVoxelMap）读取的参数。
 *
 * 前半段负责"接收并累积观测"：融合叶尺寸、时间衰减、输入高度带与接收半径。
 * 与 PerFrameHeightConfig 分开，是为了让每一半读了哪些参数在类型上就可见——原先
 * 一个 TerrainConfig 里 16 个参数混在一起，看不出哪些属于哪一半。
 *
 * 距离单位为米，时间单位为秒。
 */
struct PersistentVoxelConfig {
  /** @brief 融合叶尺寸（水平，x/y）。 */
  double scan_voxel_size = 0.1;
  /** @brief 融合叶尺寸（垂直，z）。
   *
   *  垂直方向必须比水平方向细：每个叶只保留最新观测的那一个点，若地面点与
   *  矮物体点落进同一个叶，地面点每帧都被观测到，会把物体点顶掉（叶宽 0.2 m
   *  时实测 6 cm 矮台阶的输出点数归零）。实测：水平 0.1 / 垂直 0.05 相比两者
   *  都取 0.05，单帧耗时降到约 56%，而矮台阶输出点数不再下降。 */
  double scan_voxel_size_z = 0.05;
  /** @brief 历史体素点的衰减时间。 */
  double decay_time = 2.0;
  /** @brief 在该距离内不执行时间衰减。 */
  double no_decay_distance = 4.0;

  /** @brief 接收点云相对雷达的高度下限（相对 lidar_z 的偏移量）。
   *
   *  两处共用：ingest 的裁剪带、rebuild 的体素点保留判定（keepPoint）。
   *  地面候选的地板不用它——那是绝对高度 ground_floor_z，属于
   * PerFrameHeightConfig。 */
  double min_relative_z = -1.5;
  /** @brief 接收点云相对雷达的高度上限（相对 lidar_z 的偏移量）。
   *  **仅由 ingest 的裁剪带与 keepPoint 使用**。 */
  double max_relative_z = 0.2;
  /** @brief 随水平距离放宽高度范围的比例。 */
  double distance_ratio_z = 0.2;

  /** @brief 地形体素边长（同时也是接收半径的基准）。 */
  double terrain_voxel_size = 1.0;
};
