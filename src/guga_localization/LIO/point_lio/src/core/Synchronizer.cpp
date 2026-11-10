#include "point_lio/core/Synchronizer.h"
#include "point_lio/core/Imu.h"
#include "point_lio/core/Lidar.h"
#include <algorithm>

bool Synchronizer::syncPackages(
    Lidar& lidar, Imu& imu,
    MeasureGroup& measurement) {  // TODO 队列长度考量
  if (lidar.empty() || imu.empty()) {
    return false;
  }

  const LidarFrame& frame = lidar.frame_buffer_.front();
  measurement.lidar_measure = frame.cloud;
  measurement.lidar_start_time = frame.start_time;
  measurement.lidar_last_time = frame.end_time;

  if (imu.lastTimestamp() < frame.end_time) {
    return false;
  }

  imu.collectUntil(frame.end_time, measurement.imu_measure);
  lidar.frame_buffer_.pop_front();
  return true;
}  // TODO 检查collectUnitil的空值处理