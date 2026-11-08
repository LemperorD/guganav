#pragma once

#include <vector>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointType = pcl::PointXYZINormal;

inline float point_time_offset_ms(const PointType& point) {
  return point.curvature;
}

inline void set_point_time_offset_ms(PointType& point, float offset_ms) {
  point.curvature = offset_ms;
}

using PointCloudXYZI = pcl::PointCloud<PointType>;
using PointVector =
    std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
