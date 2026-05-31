#pragma once

#include "terrain_analysis/core/context.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

inline TerrainConfig DefaultTerrainConfig() {
  return {};
}

inline auto MakeCloud(float x, float y, float z) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud->push_back({x, y, z, 0});
  return cloud;
}

inline auto MakeCloudWithTime(float x, float y, float z, float timestamp_sec) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud->push_back({x, y, z, timestamp_sec});
  return cloud;
}

inline auto MakeGroundCloud(int grid_size, double spacing, double z) {
  // grid_size points per side, centered at origin
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = -grid_size / 2; ix <= grid_size / 2; ix++) {
    for (int iy = -grid_size / 2; iy <= grid_size / 2; iy++) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(ix) * static_cast<float>(spacing);
      p.y = static_cast<float>(iy) * static_cast<float>(spacing);
      p.z = static_cast<float>(z);
      p.intensity = 0;
      cloud->push_back(p);
    }
  }
  return cloud;
}

inline auto MakeGroundAndObstacleCloud(int grid_size, double spacing,
                                       double ground_z, double obstacle_z) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = -grid_size / 2; ix <= grid_size / 2; ix++) {
    for (int iy = -grid_size / 2; iy <= grid_size / 2; iy++) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(ix) * static_cast<float>(spacing);
      p.y = static_cast<float>(iy) * static_cast<float>(spacing);
      p.z = static_cast<float>(ground_z);
      p.intensity = 0;
      cloud->push_back(p);
      p.z = static_cast<float>(obstacle_z);
      cloud->push_back(p);
    }
  }
  return cloud;
}
