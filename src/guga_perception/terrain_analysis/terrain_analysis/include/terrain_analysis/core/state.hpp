#pragma once

#include "terrain_analysis/core/planar_voxel_grid.hpp"
#include "guga_common/geometry.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <memory>
#include <vector>

/**
 * @brief terrain_analysis 的可变运行时状态。
 *
 * 该结构由 ROS 回调写入，并由算法管线消费和更新。点云坐标统一使用 odom
 * 坐标系；输出点云的 intensity 表示点相对估计地面的高度。
 */
struct TerrainState {
  // ---- 雷达位置（terrain 订阅的是雷达里程计，不是车体里程计）----
  /** @brief 雷达在 odom 坐标系下的位置（姿态在下面以三角函数缓存）。 */
  guga_common::Point3d lidar;
  /** @brief 雷达 roll 的正弦和余弦。 */
  double sin_lidar_roll = 0.0, cos_lidar_roll = 0.0;
  /** @brief 雷达 pitch 的正弦和余弦。 */
  double sin_lidar_pitch = 0.0, cos_lidar_pitch = 0.0;
  /** @brief 雷达 yaw 的正弦和余弦。 */
  double sin_lidar_yaw = 0.0, cos_lidar_yaw = 0.0;

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
  /** @brief 每个 Planar voxel 的估计地面高度。 */
  std::array<double, PlanarVoxelGrid::NUM> planar_voxel_elev{};
  /** @brief 每个 Planar voxel 收集到的地面高度候选值。 */
  std::array<std::vector<double>, PlanarVoxelGrid::NUM> planar_point_elev;

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
