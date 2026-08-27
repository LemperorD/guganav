#include "point_lio/Synchronizer.h"
#include "point_lio/Imu.h"
#include "point_lio/Lidar.h"
#include <algorithm>

namespace {
void updateFrameEndTime(MeasureGroup& measurement) {
  const auto max_point = std::max_element(
      measurement.lidar->points.begin(), measurement.lidar->points.end(),
      [](const auto& lhs, const auto& rhs) {
        return point_time_offset_ms(lhs) < point_time_offset_ms(rhs);
      });
  measurement.lidar_last_time = measurement.lidar_start_time
      + point_time_offset_ms(*max_point) / 1000.0;
}
}

bool Synchronizer::syncPackages(Lidar& lidar, Imu& imu,
                                MeasureGroup& measurement) {
  if (lidar.lidar_buffer_.empty() || imu.empty()) {
    return false;
  }
  measurement.lidar = lidar.lidar_buffer_.front();
  measurement.lidar_start_time = lidar.time_buffer_.front();
  const bool lose_lid = measurement.lidar->points.empty();
  if (!lose_lid) {
    updateFrameEndTime(measurement);
  }
  const double required_end_time = lose_lid
      ? measurement.lidar_start_time + lidar_time_interval_
      : measurement.lidar_last_time;
  if (imu.lastTimestamp() < required_end_time) {
    return false;
  }
  imu.collectUntil(required_end_time, measurement);
  lidar.lidar_buffer_.pop_front();
  lidar.time_buffer_.pop_front();
  return true;
}
