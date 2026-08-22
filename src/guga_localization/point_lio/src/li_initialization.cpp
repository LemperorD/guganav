#include "li_initialization.h"

#include <algorithm>

namespace {

  bool take_lidar_frame(const Lidar& lidar, MeasureGroup& meas) {
    meas.lidar = lidar.lidar_buffer.front();
    meas.lidar_beg_time = lidar.time_buffer.front();
    if (meas.lidar->points.empty()) {
      std::cout << "lose lidar\n";
      return false;
    }
    return true;
  }

  void update_frame_end_time(MeasureGroup& meas) {
    const auto max_point = std::max_element(
        meas.lidar->points.begin(), meas.lidar->points.end(),
        [](const auto& lhs, const auto& rhs) {
          return lhs.curvature < rhs.curvature;
        });
    lidar_end_time = meas.lidar_beg_time + max_point->curvature / 1000.0;
    meas.lidar_last_time = lidar_end_time;
  }

  void pop_lidar_frame(Lidar& lidar) {
    lidar.lidar_buffer.pop_front();
    lidar.time_buffer.pop_front();
  }

  void collect_imu_until(Lidar& lidar, double end_time, MeasureGroup& meas) {
    if (!p_imu->imu_need_init_ || lidar.imu_deque.empty()) {
      return;
    }

    lidar.imu_next = *lidar.imu_deque.front();
    meas.imu.shrink_to_fit();
    while (!lidar.imu_deque.empty()
           && get_time_sec(lidar.imu_deque.front()->header.stamp) < end_time) {
      meas.imu.emplace_back(lidar.imu_deque.front());
      lidar.imu_last_ = lidar.imu_next;
      lidar.imu_deque.pop_front();
      if (!lidar.imu_deque.empty()) {
        lidar.imu_next = *lidar.imu_deque.front();
      }
    }
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
    if (lidar.frame_ct < 10) {
      for (auto point : points->points) {
        point.curvature += (timestamp - lidar.time_con) * 1000.0;
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

void Lidar::onStandardPcl(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  ++scan_count;
  const double start = omp_get_wtime();
  const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                 "lidar loop back, clear buffer");
    return;
  }
  last_timestamp_lidar = timestamp;

  if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32)
      && cut_frame_init) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    p_pre->process_cut_frame_pcl2(msg, frames, timestamps, cut_frame_num,
                                  scan_count);
    append_cut_frames(*this, frames, timestamps);
  } else {
    auto points = std::make_shared<PointCloudXYZI>(20000, 1);
    p_pre->process(msg, points);
    if (con_frame) {
      append_merged_frame(*this, points, timestamp);
    } else if (!points->empty()) {
      lidar_buffer.emplace_back(std::move(points));
      time_buffer.emplace_back(timestamp);
    }
  }
  if (scan_count < MAXN) {
    s_plot11[scan_count] = omp_get_wtime() - start;
  }
}

void Lidar::onLivoxPcl(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
  ++scan_count;
  const double start = omp_get_wtime();
  const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
  if (timestamp < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                 "lidar loop back, clear buffer");
    return;
  }
  last_timestamp_lidar = timestamp;

  if (cut_frame_init) {
    std::deque<PointCloudXYZI::Ptr> frames;
    std::deque<double> timestamps;
    p_pre->process_cut_frame_livox(msg, frames, timestamps, cut_frame_num,
                                   scan_count);
    append_cut_frames(*this, frames, timestamps);
  } else {
    auto points = std::make_shared<PointCloudXYZI>(10000, 1);
    p_pre->process(msg, points);
    if (con_frame) {
      append_merged_frame(*this, points, timestamp);
    } else if (!points->empty()) {
      lidar_buffer.emplace_back(std::move(points));
      time_buffer.emplace_back(timestamp);
    }
  }
  if (scan_count < MAXN) {
    s_plot11[scan_count] = omp_get_wtime() - start;
  }
}

void Lidar::onIMU(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_in) {
  auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
  msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp));
  const double timestamp = get_time_sec(msg->header.stamp);
  if (timestamp < last_timestamp_imu) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"),
                 "imu loop back, clear deque");
    return;
  }
  imu_deque.emplace_back(std::move(msg));
  last_timestamp_imu = timestamp;
}

bool Lidar::syncPackages(MeasureGroup& meas) {
  if (!imu_enabled) {
    if (lidar_buffer.empty()) {
      return false;
    }
    if (!take_lidar_frame(*this, meas)) {
      pop_lidar_frame(*this);
      return false;
    }
    update_frame_end_time(meas);
    pop_lidar_frame(*this);
    return true;
  }

  if (lidar_buffer.empty() || imu_deque.empty()) {
    return false;
  }

  if (!lidar_pushed) {
    lose_lid = !take_lidar_frame(*this, meas);
    if (!lose_lid) {
      update_frame_end_time(meas);
    }
    lidar_pushed = true;
  }

  const double required_end = lose_lid ? meas.lidar_beg_time + lidar_time_inte
                                       : lidar_end_time;
  if (last_timestamp_imu < required_end) {
    return false;
  }

  if (!imu_pushed) {
    collect_imu_until(*this, required_end, meas);
    imu_pushed = true;
  }

  pop_lidar_frame(*this);
  lidar_pushed = false;
  imu_pushed = false;
  lose_lid = false;
  return true;
}

void Lidar::reset() {
  imu_last_ = sensor_msgs::msg::Imu();
  imu_next = sensor_msgs::msg::Imu();
  ptr_con->clear();
  lidar_buffer.clear();
  time_buffer.clear();
  imu_deque.clear();
  scan_count = 0;
  frame_ct = 0;
  lidar_pushed = false;
  imu_pushed = false;
  last_timestamp_lidar = -1.0;
  last_timestamp_imu = -1.0;
  time_con = 0.0;
  lose_lid = false;
}
