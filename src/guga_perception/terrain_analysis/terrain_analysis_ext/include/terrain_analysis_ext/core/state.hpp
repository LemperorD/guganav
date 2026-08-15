#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

/**
 * @brief terrain_analysis_ext 节点的可变运行时状态。
 *
 * 保存车辆平面位置、输入地形点云、输出点云及输入消息时间戳。
 */
struct TerrainExtState {
  /** @brief 车辆在 odom 坐标系下的 x 位置。 */
  double vehicle_x = 0.0;
  /** @brief 车辆在 odom 坐标系下的 y 位置。 */
  double vehicle_y = 0.0;

  /** @brief 最近接收到的地形点云时间戳，单位为秒。 */
  double laser_cloud_time = 0.0;
  /** @brief 是否有尚未处理的新地形点云。 */
  bool has_new_laser_cloud = false;

  /** @brief 扩展节点最终发布的局部地形点云。 */
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_elev =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  /** @brief 从 terrain_analysis 接收到的局部地形点云。 */
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_local =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
};
