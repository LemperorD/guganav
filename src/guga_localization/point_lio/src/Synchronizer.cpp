#include "point_lio/Synchronizer.h"
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

void Synchronizer::getMeasurements(MeasureGroup& measurement) const {
  measurement.lidar = lidar_buffer_.front();
  measurement.lidar_start_time = time_buffer_.front();
}

void Synchronizer::popLidarFrame() {
  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
}

void Synchronizer::appendCutFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                                   std::deque<double>& timestamps) {
  while (!frames.empty() && !timestamps.empty()) {
    lidar_buffer_.emplace_back(frames.front()); frames.pop_front();
    time_buffer_.emplace_back(timestamps.front() / 1000.0); timestamps.pop_front();
  }
}

void Synchronizer::appendFrame(PointCloudXYZI::Ptr points, double timestamp) {
  lidar_buffer_.emplace_back(std::move(points));
  time_buffer_.emplace_back(timestamp);
}

void Synchronizer::appendMergedFrame(const PointCloudXYZI::Ptr& points,
                                     double timestamp) {
  if (frame_ct_ == 0) time_con_ = timestamp;
  if (frame_ct_ < params_.con_frame_num) {
    for (auto point : points->points) {
      set_point_time_offset_ms(point, point_time_offset_ms(point)
          + static_cast<float>((timestamp - time_con_) * 1000.0));
      ptr_con_->push_back(point);
    }
    ++frame_ct_;
    return;
  }
  lidar_buffer_.emplace_back(std::make_shared<PointCloudXYZI>(*ptr_con_));
  time_buffer_.emplace_back(time_con_);
  ptr_con_->clear(); frame_ct_ = 0;
}

bool Synchronizer::syncPackages(Imu& imu, MeasureGroup& measurement) {
  if (lidar_buffer_.empty() || imu.empty()) return false;
  if (!lidar_pushed_) {
    lose_lid_ = lidar_buffer_.front()->points.empty();
    getMeasurements(measurement);
    if (!lose_lid_) updateFrameEndTime(measurement);
    lidar_pushed_ = true;
  }
  const double required_end_time = lose_lid_
      ? measurement.lidar_start_time + params_.lidar_time_interval
      : measurement.lidar_last_time;
  if (imu.lastTimestamp() < required_end_time) return false;
  if (!imu_pushed_) {
    imu.collectUntil(required_end_time, measurement);
    imu_pushed_ = true;
  }
  popLidarFrame(); lidar_pushed_ = false; imu_pushed_ = false; lose_lid_ = false;
  return true;
}
