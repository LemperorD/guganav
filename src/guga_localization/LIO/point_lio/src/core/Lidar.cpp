/**
 * @file Lidar.cpp
 * @brief LiDAR 点面量测模型实现
 *
 * 包含点云接入、切帧/合帧、LiDAR-IMU 同步、点面残差和坐标变换。
 */

#include "point_lio/core/Lidar.h"

#include <algorithm>

namespace {
  /// @brief 计算一帧点云的绝对结束时间：起始时间加上帧内最大时间偏移。
  double frameEndTime(const PointCloudXYZI::Ptr& cloud, double start_time,
                      double fallback_interval) {
    if (cloud == nullptr || cloud->points.empty()) {
      return start_time + fallback_interval;
    }
    const auto max_point = std::max_element(
        cloud->points.begin(), cloud->points.end(),
        [](const auto& lhs, const auto& rhs) {
          return point_time_offset_ms(lhs) < point_time_offset_ms(rhs);
        });
    return start_time + point_time_offset_ms(*max_point) / 1000.0;
  }
}  // namespace

void LidarMeasurementModel::configure(const LidarParams& params) {
  params_ = params;
}

bool Lidar::empty() const {
  return frame_buffer_.empty();
}

void LidarMeasurementModel::pointBodyToWorld(PointType const* pi, PointType* po,
                                             const State& state) const {
  const V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global;
  if (params_.extrinsic_estimation) {
    p_global = state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I)
               + state.pos;
  } else {
    p_global = state.rot * ((*lidar_rotation_) * p_body + (*lidar_translation_))
               + state.pos;
  }
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void Lidar::configure(const Params& params) {
  params_ = params;
  if (params.cut_frame_interval > 0.0) {
    params_.cut_frame_num = std::max(
        1, static_cast<int>(std::lround(params_.lidar_time_interval
                                        / params_.cut_frame_interval)));
  }
  params_.con_frame_num = std::max(1, params_.con_frame_num);
  params_.cut_frame_num = std::max(1, params_.cut_frame_num);
  preprocess_.configure(params_.preprocess);
  measurement_model_.configure(params_);
}

void Lidar::reset() {
  ptr_con_->clear();
  frame_buffer_.clear();
  scan_count_ = 0;
  frame_ct_ = 0;
  last_timestamp_lidar_ = -1.0;
  time_con_ = 0.0;
}

void Lidar::appendCutFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                            std::deque<double>& timestamps) {
  while (!frames.empty() && !timestamps.empty()) {
    const double start_time = timestamps.front() / 1000.0;
    const double end_time = frameEndTime(frames.front(), start_time,
                                         params_.lidar_time_interval);
    frame_buffer_.emplace_back(LidarFrame{.cloud = std::move(frames.front()),
                                          .start_time = start_time,
                                          .end_time = end_time});
    frames.pop_front();
    timestamps.pop_front();
  }
}

void Lidar::appendFrame(PointCloudXYZI::Ptr pointcloud, double timestamp) {
  const double end_time = frameEndTime(pointcloud, timestamp,
                                       params_.lidar_time_interval);
  frame_buffer_.emplace_back(LidarFrame{.cloud = std::move(pointcloud),
                                        .start_time = timestamp,
                                        .end_time = end_time});
}

void Lidar::appendMergedFrame(const PointCloudXYZI::Ptr& points,
                              double timestamp) {
  if (frame_ct_ == 0) {
    time_con_ = timestamp;
  }
  if (frame_ct_ < mergeFrameCount()) {
    for (auto point : points->points) {
      set_point_time_offset_ms(
          point, point_time_offset_ms(point)
                     + static_cast<float>((timestamp - time_con_) * 1000.0));
      ptr_con_->push_back(point);
    }
    ++frame_ct_;
    return;
  }
  frame_buffer_.emplace_back(
      LidarFrame{.cloud = std::make_shared<PointCloudXYZI>(*ptr_con_),
                 .start_time = time_con_,
                 .end_time = frameEndTime(ptr_con_, time_con_,
                                          params_.lidar_time_interval)});
  ptr_con_->clear();
  frame_ct_ = 0;
}

void Lidar::onStandardPcl(
    const sensor_msgs::msg::PointCloud2::SharedPtr& message) {
  ++scan_count_;
  const double timestamp = rclcpp::Time(message->header.stamp).seconds();

  if (timestamp < last_timestamp_lidar_) {
    RCLCPP_ERROR(rclcpp::get_logger("Lidar"), "lidar loop back");
    return;
  }

  last_timestamp_lidar_ = timestamp;

  const int type = params_.preprocess.lidar_type;
  if ((type == VELO16 || type == OUST64 || type == HESA_IXT32)
      && params_.cut_frame) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    preprocess_.processCutFramePCL2(message, frames, timestamps,
                                    params_.cut_frame_num, scan_count_);
    appendCutFrames(frames, timestamps);
    return;
  }

  auto points = std::make_shared<PointCloudXYZI>(20000, 1);
  preprocess_.process(message, points);
  if (params_.con_frame) {
    appendMergedFrame(points, timestamp);
  } else if (!points->empty()) {
    appendFrame(std::move(points), timestamp);
  }
}

void Lidar::onLivoxPcl(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& message) {
  ++scan_count_;
  const double timestamp = rclcpp::Time(message->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar_) {
    RCLCPP_ERROR(rclcpp::get_logger("Lidar"), "lidar loop back");
    return;
  }
  last_timestamp_lidar_ = timestamp;

  if (params_.cut_frame) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    preprocess_.processCutFrameLivox(message, frames, timestamps,
                                     params_.cut_frame_num, scan_count_);
    appendCutFrames(frames, timestamps);
    return;
  }
  auto points = std::make_shared<PointCloudXYZI>(10000, 1);
  preprocess_.process(message, points);
  if (params_.con_frame) {
    appendMergedFrame(points, timestamp);
  } else if (!points->empty()) {
    appendFrame(std::move(points), timestamp);
  }
}
