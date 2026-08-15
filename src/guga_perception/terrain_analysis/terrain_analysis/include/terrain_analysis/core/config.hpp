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
  /** @brief 清除请求影响的最大距离。 */
  double clearing_distance = 8.0;

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
  /** @brief 是否启用动态障碍过滤。 */
  bool clear_dy_obs = false;
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
  /** @brief 是否将无数据区域发布为虚拟障碍。 */
  bool no_data_obstacle = false;
  /** @brief 无数据边缘标签向外扩展的迭代次数。 */
  int no_data_block_skip_num = 0;
  /** @brief planar voxel 的最小有效点数。 */
  int min_block_point_num = 10;
  /** @brief 高度小于该值的障碍点才会输出。 */
  double vehicle_height = 1.5;
  /** @brief 车顶上方安全间隙：相对车高达到该值的点（天花板/横梁）不作为
   *  障碍输出，也不参与地面高度估计。需小于实测隧道顶隙（如 260mm →
   *  0.2），否则低矮隧道仍会被判为不可通过。 */
  double ceiling_clearance = 0.3;

  // 体素更新和点云范围
  /** @brief 触发体素重建的累计更新点数阈值。 */
  int voxel_point_update_thre = 100;
  /** @brief 触发体素重建的时间阈值。 */
  double voxel_time_update_thre = 2.0;
  /** @brief 点云相对车辆高度下限。 */
  double min_relative_z = -1.5;
  /** @brief 点云相对车辆高度上限。 */
  double max_relative_z = 0.2;
  /** @brief 随水平距离放宽高度范围的比例。 */
  double distance_ratio_z = 0.2;

  // 网格分辨率
  /** @brief 地形体素边长。 */
  double terrain_voxel_size = 1.0;
  /** @brief planar voxel 边长。 */
  double planar_voxel_size = 0.2;
};
