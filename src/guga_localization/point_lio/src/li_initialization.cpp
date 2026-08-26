#include "li_initialization.h"

#include <algorithm>

namespace {

  void get_lidar_measurements(const Lidar& lidar, MeasureGroup& meas) {
    meas.lidar = lidar.lidar_buffer.front();
    meas.lidar_start_time = lidar.time_buffer.front();
  }

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

  void pop_lidar_frame(Lidar& lidar) {
    lidar.lidar_buffer.pop_front();
    lidar.time_buffer.pop_front();
  }

  void append_cut_frames(Lidar& lidar, std::deque<PointCloudXYZI::Ptr>& frames,
                         std::deque<double>& timestamps) {
    while (!frames.empty() && !timestamps.empty()) {
      lidar.lidar_buffer.emplace_back(frames.front());
      frames.pop_front();
      lidar.time_buffer.emplace_back(timestamps.front() / 1000.0);
      timestamps.pop_front();
    }
  }

  void append_merged_frame(Lidar& lidar, const PointCloudXYZI::Ptr& points,
                           double timestamp) {
    if (lidar.frame_ct == 0) {
      lidar.time_con = timestamp;
    }
    if (lidar.frame_ct < lidar.mergeFrameCount()) {
      for (auto point : points->points) {
        set_point_time_offset_ms(
            point,
            point_time_offset_ms(point)
                + static_cast<float>((timestamp - lidar.time_con) * 1000.0));
        lidar.ptr_con->push_back(point);
      }
      ++lidar.frame_ct;
      return;
    }

    lidar.lidar_buffer.emplace_back(
        std::make_shared<PointCloudXYZI>(*lidar.ptr_con));
    lidar.time_buffer.emplace_back(lidar.time_con);
    lidar.ptr_con->clear();
    lidar.frame_ct = 0;
  }

}  // namespace

bool Lidar::syncPackages(  // TODO函数逻辑具有大量flag,准备提取状态机
    Imu& imu,
    MeasureGroup&
        meas) {  // TODO:组装不应是lidar的职责,其必然暴露imu内部实现.将构建syncalizor同步器处理.
  if (lidar_buffer.empty() || imu.empty()) {  // lidar/imu空
    return false;
  }

  if (!lidar_pushed) {
    lose_lid = lidar_buffer.front()->points.empty();
    get_lidar_measurements(*this, meas);

    if (!lose_lid) {
      update_frame_end_time(meas);
    }
    lidar_pushed = true;
  }

  // 丢失为开始+偏置,未丢失为结束
  const double required_end_time = lose_lid ? meas.lidar_start_time
                                                  + params_.lidar_time_interval
                                            : meas.lidar_last_time;

  if (imu.lastTimestamp() < required_end_time) {
    return false;
  }

  if (!imu_pushed) {
    imu.collectUntil(required_end_time, meas);
    imu_pushed = true;
  }

  pop_lidar_frame(*this);
  lidar_pushed = false;
  imu_pushed = false;
  lose_lid = false;
  return true;
}

void Lidar::configure(const Params& params) {
  params_ = params;
  params_.con_frame_num = std::max(1, params_.con_frame_num);
  preprocess_.configure(params_.preprocess);
}

void Lidar::onStandardPcl(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  ++scan_count;
  const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                 "lidar loop back, clear buffer");
    return;
  }
  last_timestamp_lidar = timestamp;

  if ((params_.preprocess.lidar_type == VELO16
       || params_.preprocess.lidar_type == OUST64
       || params_.preprocess.lidar_type == HESA_IXT32)
      && params_.cut_frame) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    preprocess_.processCutFramePCL2(msg, frames, timestamps,
                                    params_.cut_frame_num, scan_count);
    append_cut_frames(*this, frames, timestamps);
  } else {
    auto points = std::make_shared<PointCloudXYZI>(20000, 1);
    preprocess_.process(msg, points);
    if (params_.con_frame) {
      append_merged_frame(*this, points, timestamp);
    } else if (!points->empty()) {
      lidar_buffer.emplace_back(std::move(points));
      time_buffer.emplace_back(timestamp);
    }
  }
}

void Lidar::onLivoxPcl(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
  ++scan_count;
  const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                 "lidar loop back, clear buffer");
    return;
  }
  last_timestamp_lidar = timestamp;

  if (params_.cut_frame) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    preprocess_.processCutFrameLivox(msg, frames, timestamps,
                                     params_.cut_frame_num, scan_count);
    append_cut_frames(*this, frames, timestamps);
  } else {
    auto points = std::make_shared<PointCloudXYZI>(10000, 1);
    preprocess_.process(msg, points);
    if (params_.con_frame) {
      append_merged_frame(*this, points, timestamp);
    } else if (!points->empty()) {
      lidar_buffer.emplace_back(std::move(points));
      time_buffer.emplace_back(timestamp);
    }
  }
}
