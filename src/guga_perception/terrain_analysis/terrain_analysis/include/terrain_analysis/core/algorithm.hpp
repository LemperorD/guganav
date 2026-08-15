#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct TerrainState;
struct TerrainConfig;

namespace terrain_analysis::algorithm {

  /**
   * @brief 接收里程计并更新车辆位姿及无数据区域状态。
   * @param config 地形分析配置。
   * @param state 地形分析运行时状态。
   * @param x 车辆在 odom 坐标系下的 x 位置。
   * @param y 车辆在 odom 坐标系下的 y 位置。
   * @param z 车辆在 odom 坐标系下的 z 位置。
   * @param roll 车辆 roll 角，单位为弧度。
   * @param pitch 车辆 pitch 角，单位为弧度。
   * @param yaw 车辆 yaw 角，单位为弧度。
   */
  void ingestOdometry(const TerrainConfig& config, TerrainState& state,
                      double x, double y, double z, double roll, double pitch,
                      double yaw);

  /**
   * @brief 接收当前帧激光点云并执行范围、相对高度预过滤。
   * @param config 地形分析配置。
   * @param state 地形分析运行时状态。
   * @param cloud 输入点云，坐标应位于 odom 坐标系。
   * @param timestamp_sec 点云时间戳，单位为秒。
   */
  void ingestLaserCloud(const TerrainConfig& config, TerrainState& state,
                        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
                        double timestamp_sec);

  /**
   * @brief 请求清除指定距离内的历史地形体素。
   * @param state 地形分析运行时状态。
   * @param distance_clearing 清除半径，单位为米。
   */
  void ingestClearing(TerrainState& state, double distance_clearing);

  /** @brief 执行完整的地形分析管线。 */
  void run(const TerrainConfig& config, TerrainState& state);
  /** @brief 根据车辆位移滚动以车辆为中心的地形体素网格。 */
  void rolloverVoxels(const TerrainConfig& config, TerrainState& state);
  /** @brief 将预过滤点云分配到地形体素网格。 */
  void voxelize(const TerrainConfig& config, TerrainState& state);
  /** @brief 对地形体素执行降采样、衰减、清除和高度范围过滤。 */
  void updateVoxels(const TerrainConfig& config, TerrainState& state);
  /** @brief 提取车辆周边窗口内的累积地形点云。 */
  void extractTerrainCloud(TerrainState& state);
  /** @brief 将地形点膨胀到 planar voxel，收集地面高度候选值。 */
  void estimateGround(const TerrainConfig& config, TerrainState& state);
  /** @brief 根据仰角和传感器视场统计潜在动态障碍点。 */
  void detectDynamicObstacles(const TerrainConfig& config, TerrainState& state);
  /** @brief 使用当前帧高角度点过滤固定结构造成的动态障碍误报。 */
  void filterDynamicObstaclePoints(const TerrainConfig& config,
                                   TerrainState& state);
  /** @brief 按分位数或最小值计算每个 planar voxel 的地面高度。 */
  void computeElevation(const TerrainConfig& config, TerrainState& state);
  /** @brief 计算点相对地面的高度并生成输出地形点云。 */
  void computeHeightMap(const TerrainConfig& config, TerrainState& state);
  /** @brief 为无数据或数据稀疏区域生成虚拟障碍点。 */
  void addNoDataObstacles(const TerrainConfig& config, TerrainState& state);

}  // namespace terrain_analysis::algorithm
