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
  /** @brief 障碍输出上界：距**局部地面**达到该值的点不作为障碍（车辆可从其
   *  下方通过，或高于车体不构成碰撞威胁）。
   *
   *  按实车车体高度 + 100 mm 安全间隙设定（车高 520 mm → 0.62）。因车高随车
   *  而异（后续有第二台车），故为 ROS 参数而非编译期常量。
   *
   *  这是障碍输出**唯一**的高度上界。曾另有一条 `height < vehicle_height` 的
   *  截断，它与本值重叠且更严（0.52 < 0.62），会把车高与净空之间那一带的点
   *  一并丢掉，而那个高度上的悬空结构车是过不去的——该截断已移除，参数一并
   *  删除。换车只需重设本值。 */
  double ceiling_clearance = 0.62;
  // 体素更新和点云范围
  /** @brief 触发体素重建的累计更新点数阈值。 */
  int voxel_point_update_thre = 100;
  /** @brief 触发体素重建的时间阈值。 */
  double voxel_time_update_thre = 2.0;
  /** @brief 有效点云相对车辆的高度下限（相对 vehicle_z 的偏移量）。
   *  两处共用：ingestLaserCloud 的裁剪带、keepTerrainVoxelPoint 的体素点
   *  保留判定。地面候选的地板不用它——那是绝对高度 ground_floor_z。 */
  double min_relative_z = -1.5;
  /** @brief 有效点云相对车辆的高度上限（相对 vehicle_z 的偏移量）。
   *  **仅由 ingestLaserCloud 的裁剪带与 keepTerrainVoxelPoint 使用**。
   *  地面候选（estimateTerrainGround）与障碍输出（computeHeightMap）的上界
   *  都由更紧的 TerrainGrid::CEILING_CLEARANCE(0.1) 决定，本参数在这两处
   *  不参与判定。 */
  double max_relative_z = 0.2;
  /** @brief 随水平距离放宽高度范围的比例。 */
  double distance_ratio_z = 0.2;

  // 地面估计
  /** @brief 地面候选的绝对高度地板（odom z）：低于该值的点不参与地面估计。
   *
   *  用绝对量而非"相对车辆"：地面在 odom 中大体水平，且该阈值不应随车体
   *  俯仰/上下抖动而移动。仅由 estimateTerrainGround 使用，且是该阶段**唯一**
   *  的候选筛选（原有的净空上界已移除，见该函数注释）。
   *
   *  取值须贴近实际地面 z：odom 原点与 base_footprint 重合（实测
   *  `odom → base_footprint` 为单位变换），平地地面 z ≈ 0，故取地面下方约
   *  0.2 m（≈ −0.2）。注意 `vehicle_z` 是**雷达**在 odom 下的高度（平地为
   *  +0.230，即 base_footprint→front_mid360 的安装高度），不是本值的参考基准。
   *  下坡或地面下沉时，真实地面会低于该地板而被排除，坡面场景需重设。
   *  若换车或改安装，需按新实测值重设。 */
  double ground_floor_z = -0.2;

  // 网格分辨率
  /** @brief 地形体素边长。 */
  double terrain_voxel_size = 1.0;
  /** @brief planar voxel 边长。 */
  double planar_voxel_size = 0.2;
};
