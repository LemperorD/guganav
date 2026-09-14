#pragma once

#include <cmath>

/**
 * @brief terrain_analysis 算法运行参数。
 *
 * 所有距离和长度参数单位为米，时间参数单位为秒，角度参数在算法内部
 * 使用弧度表示。ROS 节点负责从参数服务器读取并完成角度单位转换。
 */
struct TerrainConfig {
  // 输入点云体素化和历史数据衰减
  /** @brief 输入点云降采样分辨率。 */
  double scan_voxel_size = 0.05;
  /** @brief 历史体素点的衰减时间。 */
  double decay_time = 2.0;
  /** @brief 在该距离内不执行时间衰减。 */
  double no_decay_distance = 4.0;

  // 地面高度估计
  /** @brief 是否使用分位数估计地面高度，否则使用最小值。 */
  bool use_sorting = true;
  /** @brief 地面高度分位数。 */
  double quantile_z = 0.25;
  /** @brief 是否将负高度差转换为绝对值。 */
  bool consider_drop = false;
  /** @brief 是否限制地面估计相对最低点的抬升量。 */
  bool limit_ground_lift = false;
  /** @brief 地面估计允许的最大抬升量。 */
  double max_ground_lift = 0.15;

  // 动态障碍过滤
  /** @brief 动态障碍检测的最小水平距离。 */
  double min_dy_obs_distance = 0.3;
  /** @brief 动态障碍检测的最小仰角，单位为弧度。 */
  double min_dy_obs_angle = 0.0;
  /** @brief 动态障碍相对高度参考值。 */
  double min_dy_obs_relative_z = -0.5;
  /** @brief 允许通过绝对高度条件的阈值。 */
  double abs_dy_obs_relative_z_threshold = 0.2;
  /** @brief 传感器垂直视场下限，单位为弧度。 */
  double min_dy_obs_vfov = -16.0 * M_PI / 180.0;
  /** @brief 传感器垂直视场上限，单位为弧度。 */
  double max_dy_obs_vfov = 16.0 * M_PI / 180.0;
  /** @brief 判定动态障碍所需的最小点数。 */
  int min_dy_obs_point_num = 1;

  // 无数据区域和障碍高度过滤
  /** @brief planar voxel 的最小有效点数。 */
  int min_block_point_num = 10;
  /** @brief 高度小于该值的障碍点才会输出。 */
  double vehicle_height = 1.5;
  /** @brief 车顶上方安全间隙：相对车高达到该值的点（天花板/横梁）不作为
   *  障碍输出，也不参与地面高度估计。需小于实测隧道顶隙（如 260mm →
   *  0.2），否则低矮隧道仍会被判为不可通过。
   *
   *  注意它同时是**地面候选的有效上界**：当小于 max_relative_z 时（当前
   *  0.2 < 0.5），estimateTerrainGround 的上界由它决定，max_relative_z 在
   *  该阶段不生效。调大它到超过 max_relative_z 会使上界归属静默反转。 */
  double ceiling_clearance = 0.3;

  // 体素更新和点云范围
  /** @brief 触发体素重建的累计更新点数阈值。 */
  int voxel_point_update_thre = 100;
  /** @brief 触发体素重建的时间阈值。 */
  double voxel_time_update_thre = 2.0;
  /** @brief 有效点云相对车辆的高度下限（相对 vehicle_z 的偏移量）。
   *  两处共用：ingestLaserCloud 的裁剪带、keepTerrainVoxelPoint 的体素点
   *  保留判定。estimateTerrainGround 不用它——那里的地板是绝对高度
   *  ground_floor_z。 */
  double min_relative_z = -1.5;
  /** @brief 有效点云相对车辆的高度上限（相对 vehicle_z 的偏移量）。
   *  两处共用同上；但在这两处都会被更小的 ceiling_clearance 遮蔽而不生效
   *  （当前 0.2 < 0.5）。 */
  double max_relative_z = 0.2;
  /** @brief 随水平距离放宽高度范围的比例。 */
  double distance_ratio_z = 0.2;

  // 地面估计
  /** @brief 地面候选的绝对高度地板（odom z）：低于该值的点不参与地面估计。
   *
   *  用绝对量而非"相对车辆"：地面在 odom 中大体水平，且该阈值不应随车体
   *  俯仰/上下抖动而移动。仅由 estimateTerrainGround 使用。 */
  double ground_floor_z = -2.0;

  // 网格分辨率
  /** @brief 地形体素边长。 */
  double terrain_voxel_size = 1.0;
  /** @brief planar voxel 边长。 */
  double planar_voxel_size = 0.2;
};
