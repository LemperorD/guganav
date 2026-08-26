#include "point_lio/li_initialization.h"

#include <algorithm>

namespace {
  void update_frame_end_time(MeasureGroup& meas) {
    const auto max_point_ptr = std::max_element(
        meas.lidar->points.begin(), meas.lidar->points.end(),
        [](const auto& lhs, const auto& rhs) {
          return point_time_offset_ms(lhs) < point_time_offset_ms(rhs);
        });  // 寻找lidar帧的最大点,确定结束时间lidar_last_time.

    const double max_point_time_offset_ms = point_time_offset_ms(
        *max_point_ptr);
    meas.lidar_last_time = meas.lidar_start_time
                           + (max_point_time_offset_ms / 1000.0);
  }

}

void Lidar::getMeasurements(MeasureGroup& meas) const {
  meas.lidar = lidar_buffer_.front();
  meas.lidar_start_time = time_buffer_.front();
}

void Lidar::popLidarFrame() {
  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
}

void Lidar::appendCutFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                            std::deque<double>& timestamps) {
  while (!frames.empty() && !timestamps.empty()) {
      lidar_buffer_.emplace_back(frames.front());
      frames.pop_front();
      time_buffer_.emplace_back(timestamps.front() / 1000.0);
      timestamps.pop_front();
  }
}

void Lidar::appendMergedFrame(const PointCloudXYZI::Ptr& points,
                              double timestamp) {
    if (frame_ct_ == 0) {
      time_con_ = timestamp;
    }
    if (frame_ct_ < mergeFrameCount()) {
      for (auto point : points->points) {
        set_point_time_offset_ms(
            point,
            point_time_offset_ms(point)
                + static_cast<float>((timestamp - time_con_) * 1000.0));
        ptr_con_->push_back(point);
      }
      ++frame_ct_;
      return;
    }

    lidar_buffer_.emplace_back(std::make_shared<PointCloudXYZI>(*ptr_con_));
    time_buffer_.emplace_back(time_con_);
    ptr_con_->clear();
    frame_ct_ = 0;
}

bool Lidar::syncPackages(  // TODO函数逻辑具有大量flag,准备提取状态机
    Imu& imu,
    MeasureGroup&
        meas) {  // TODO:组装不应是lidar的职责,其必然暴露imu内部实现.将构建syncalizor同步器处理.
  if (lidar_buffer_.empty() || imu.empty()) {  // lidar/imu空
    return false;
  }

  if (!lidar_pushed_) {
    lose_lid_ = lidar_buffer_.front()->points.empty();
    getMeasurements(meas);

    if (!lose_lid_) {
      update_frame_end_time(meas);
    }
    lidar_pushed_ = true;
  }

  // 丢失为开始+偏置,未丢失为结束
  const double required_end_time = lose_lid_ ? meas.lidar_start_time
                                                  + params_.lidar_time_interval
                                            : meas.lidar_last_time;

  if (imu.lastTimestamp() < required_end_time) {
    return false;
  }

  if (!imu_pushed_) {
    imu.collectUntil(required_end_time, meas);
    imu_pushed_ = true;
  }

  popLidarFrame();
  lidar_pushed_ = false;
  imu_pushed_ = false;
  lose_lid_ = false;
  return true;
}

void Lidar::configure(const Params& params) {
  params_ = params;
  params_.con_frame_num = std::max(1, params_.con_frame_num);
  params_.cut_frame_num = std::max(1, params_.cut_frame_num);
  preprocess_.configure(params_.preprocess);
}

void Lidar::onStandardPcl(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  ++scan_count_;
  const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar_) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                 "lidar loop back, clear buffer");
    return;
  }
  last_timestamp_lidar_ = timestamp;

  if ((params_.preprocess.lidar_type == VELO16
       || params_.preprocess.lidar_type == OUST64
       || params_.preprocess.lidar_type == HESA_IXT32)
      && params_.cut_frame) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    preprocess_.processCutFramePCL2(msg, frames, timestamps,
                                    params_.cut_frame_num, scan_count_);
    appendCutFrames(frames, timestamps);
  } else {
    auto points = std::make_shared<PointCloudXYZI>(20000, 1);
    preprocess_.process(msg, points);
    if (params_.con_frame) {
      appendMergedFrame(points, timestamp);
    } else if (!points->empty()) {
      lidar_buffer_.emplace_back(std::move(points));
      time_buffer_.emplace_back(timestamp);
    }
  }
}

void Lidar::onLivoxPcl(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
  ++scan_count_;
  const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar_) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                 "lidar loop back, clear buffer");
    return;
  }
  last_timestamp_lidar_ = timestamp;

  if (params_.cut_frame) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    preprocess_.processCutFrameLivox(msg, frames, timestamps,
                                     params_.cut_frame_num, scan_count_);
    appendCutFrames(frames, timestamps);
  } else {
    auto points = std::make_shared<PointCloudXYZI>(10000, 1);
    preprocess_.process(msg, points);
    if (params_.con_frame) {
      appendMergedFrame(points, timestamp);
    } else if (!points->empty()) {
      lidar_buffer_.emplace_back(std::move(points));
      time_buffer_.emplace_back(timestamp);
    }
  }
}
