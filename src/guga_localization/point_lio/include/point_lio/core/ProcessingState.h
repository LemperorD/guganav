#pragma once

#include "point_lio/core/common_lib.h"

// State owned by the main frame-processing pipeline and its ROS outputs.
struct MainLoopState {
  int sleep_time = 0;
  pcl::VoxelGrid<PointType> downsize_filter_surf;
  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odom_aft_mapped;
  geometry_msgs::msg::PoseStamped msg_body_pose;
  PointCloudXYZI::Ptr feats_undistort = std::make_shared<PointCloudXYZI>();
  PointCloudXYZI::Ptr init_feats_world = std::make_shared<PointCloudXYZI>();
  PointCloudXYZI::Ptr pcl_wait_save = std::make_shared<PointCloudXYZI>();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
