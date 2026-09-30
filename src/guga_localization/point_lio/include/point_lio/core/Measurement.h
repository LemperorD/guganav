#pragma once

#include "point_lio/core/common_lib.h"

// Data contract produced by Synchronizer and consumed by FrameProcessor/Imu.
struct MeasureGroup {
  double lidar_start_time{0.0};
  double lidar_last_time{0.0};
  PointCloudXYZI::Ptr lidar;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
};
