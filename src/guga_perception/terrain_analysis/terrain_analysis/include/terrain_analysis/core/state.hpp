#pragma once

#include "terrain_analysis/core/grid.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <memory>
#include <vector>

/** @brief 创建并初始化 Terrain voxel 点云指针数组。 */
inline std::array<pcl::PointCloud<pcl::PointXYZI>::Ptr,
                  TerrainGrid::TERRAIN_VOXEL_NUM>
makeTerrainVoxelClouds() {
  std::array<pcl::PointCloud<pcl::PointXYZI>::Ptr,
             TerrainGrid::TERRAIN_VOXEL_NUM>
      clouds;
  for (auto& ptr : clouds) {
    ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }
  return clouds;
}

/**
 * @brief terrain_analysis 的可变运行时状态。
 *
 * 该结构由 ROS 回调写入，并由算法管线消费和更新。点云坐标统一使用 odom
 * 坐标系；输出点云的 intensity 表示点相对估计地面的高度。
 */
struct TerrainState {
  // ---- 车辆位姿 ----
  /** @brief 车辆在 odom 坐标系下的位置。 */
  double vehicle_x = 0.0, vehicle_y = 0.0, vehicle_z = 0.0;
  /** @brief 车辆 roll 的正弦和余弦。 */
  double sin_vehicle_roll = 0.0, cos_vehicle_roll = 0.0;
  /** @brief 车辆 pitch 的正弦和余弦。 */
  double sin_vehicle_pitch = 0.0, cos_vehicle_pitch = 0.0;
  /** @brief 车辆 yaw 的正弦和余弦。 */
  double sin_vehicle_yaw = 0.0, cos_vehicle_yaw = 0.0;

  // ---- 点云和网格 ----
  /** @brief 按高度和距离预过滤后的当前帧点云。 */
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_crop =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  /** @brief 从历史体素提取的局部地形点云。 */
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  /** @brief 最终发布的带离地高度 intensity 点云。 */
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_elev =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  /** @brief 按 Terrain voxel 累积的历史点云。 */
  std::array<pcl::PointCloud<pcl::PointXYZI>::Ptr,
             TerrainGrid::TERRAIN_VOXEL_NUM>
      terrain_voxel_cloud = makeTerrainVoxelClouds();
  /** @brief 每个 Terrain voxel 自上次重建后的更新点数。 */
  std::array<int, TerrainGrid::TERRAIN_VOXEL_NUM> terrain_voxel_update_num{};
  /** @brief 每个 Terrain voxel 最近一次重建的相对时间。 */
  std::array<double, TerrainGrid::TERRAIN_VOXEL_NUM>
      terrain_voxel_update_time{};
  /** @brief 每个 Planar voxel 的估计地面高度。 */
  std::array<double, TerrainGrid::PLANAR_VOXEL_NUM> planar_voxel_elev{};
  /** @brief 每个 Planar voxel 的动态障碍计数。 */
  std::array<int, TerrainGrid::PLANAR_VOXEL_NUM> planar_voxel_dy_obs{};
  /** @brief 每个 Planar voxel 收集到的地面高度候选值。 */
  std::array<std::vector<double>, TerrainGrid::PLANAR_VOXEL_NUM>
      planar_point_elev;

  // ---- Terrain voxel 滚动偏移 ----
  /** @brief Terrain voxel 网格相对初始中心的 x 方向偏移。 */
  int terrain_voxel_shift_x = 0;
  /** @brief Terrain voxel 网格相对初始中心的 y 方向偏移。 */
  int terrain_voxel_shift_y = 0;

  // ---- 输入点云状态 ----
  /** @brief 最近一帧点云时间戳，单位为秒。 */
  double laser_cloud_time = 0.0;
  /** @brief 是否有尚未处理的新点云。 */
  bool new_laser_cloud = false;

  // ---- 系统状态 ----
  /** @brief 首帧点云时间戳，单位为秒。 */
  double system_init_time = 0.0;
  /** @brief 是否已经接收到首帧点云。 */
  bool system_inited = false;
};
