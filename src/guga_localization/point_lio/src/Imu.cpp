#include "Imu.h"

Imu::Imu() {
}

void Imu::configure(int lidar_type, bool enabled,
                    const std::vector<double>& gravity) {
  p_imu->lidar_type = lidar_type;
  p_imu->imu_en = enabled;
  p_imu->gravity_ = to_vec3d(gravity);
}

void Imu::onMessage(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_in) {
  auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
  msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp));

  const double timestamp = get_time_sec(msg->header.stamp);
  if (timestamp < last_timestamp_) {
    RCLCPP_ERROR(rclcpp::get_logger("Imu"),
                 "imu loop back, clear deque");
    return;
  }

  buffer_.emplace_back(std::move(msg));
  last_timestamp_ = timestamp;
}

bool Imu::empty() const {
  return buffer_.empty();
}

const sensor_msgs::msg::Imu& Imu::last() const {
  return last_;
}

const sensor_msgs::msg::Imu& Imu::next() const {
  return next_;
}

const sensor_msgs::msg::Imu::ConstSharedPtr& Imu::front() const {
  return buffer_.front();
}

sensor_msgs::msg::Imu& Imu::lastMutable() {
  return last_;
}

sensor_msgs::msg::Imu& Imu::nextMutable() {
  return next_;
}

std::deque<sensor_msgs::msg::Imu::ConstSharedPtr>& Imu::buffer() {
  return buffer_;
}

void Imu::loadNextFromFront() {
  if (!buffer_.empty()) {
    next_ = *buffer_.front();
  }
}

void Imu::discardBefore(double timestamp) {
  if (buffer_.empty()) {
    return;
  }
  loadNextFromFront();
  while (!buffer_.empty() && timestamp > get_time_sec(next_.header.stamp)) {
    buffer_.pop_front();
    if (buffer_.empty()) {
      return;
    }
    last_ = next_;
    loadNextFromFront();
  }
}

void Imu::advanceCursor() {
  if (!buffer_.empty()) {
    last_ = next_;
    next_ = *buffer_.front();
  }
}

void Imu::popFront() {
  if (!buffer_.empty()) {
    buffer_.pop_front();
  }
}

void Imu::popAndAdvance() {
  if (buffer_.empty()) {
    return;
  }
  last_ = next_;
  buffer_.pop_front();
  loadNextFromFront();
}

void Imu::setNeedInit(bool value) {
  p_imu->imu_need_init_ = value;
}

bool Imu::collectUntil(double end_time, MeasureGroup& meas) {
  if (!needInit() || buffer_.empty()) {
    return false;
  }

  loadNextFromFront();
  meas.imu.shrink_to_fit();
  while (!buffer_.empty() &&
         get_time_sec(buffer_.front()->header.stamp) < end_time) {
    meas.imu.emplace_back(buffer_.front());
    last_ = next_;
    buffer_.pop_front();
    loadNextFromFront();
  }
  return true;
}

void Imu::process(const MeasureGroup& meas, PointCloudXYZI::Ptr& undistort) {
  p_imu->Process(meas, undistort);
}

void Imu::reset() {
  p_imu->Reset();
  buffer_.clear();
  last_ = sensor_msgs::msg::Imu();
  next_ = sensor_msgs::msg::Imu();
  last_timestamp_ = -1.0;
}

bool Imu::needInit() const {
  return p_imu->imu_need_init_;
}

bool Imu::stateInitialized() const {
  return p_imu->after_imu_init_;
}

double Imu::lastTimestamp() const {
  return last_timestamp_;
}
