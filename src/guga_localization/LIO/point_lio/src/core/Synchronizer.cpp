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

  // 不按时间跨度丢帧, 保证 IMU 样本连续, 传播的 dt 始终是样本间隔.
  const LidarFrame& frame = lidar.frame_buffer_.front();

  if (imu.lastTimestamp() < frame.end_time) {
    return false;
  }

  measurement.lidar_measure_origin = frame.cloud;
  measurement.lidar_start_time = frame.start_time;
  measurement.lidar_last_time = frame.end_time;

  measurement.imu_measure.clear();
  imu.collectUntil(frame.end_time, measurement.imu_measure);
  lidar.frame_buffer_.pop_front();
  return true;
}  // TODO 检查collectUnitil的空值处理