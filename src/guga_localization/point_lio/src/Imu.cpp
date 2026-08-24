#include "Imu.h"

void Imu::configure(int lidar_type, bool enabled,
                    const std::vector<double>& gravity) {
  processor_->lidar_type = lidar_type;
  processor_->imu_en = enabled;
  processor_->gravity_ = to_vec3d(gravity);
}

void Imu::onMessage(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_in) {
  auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
  msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp));

  const double timestamp = get_time_sec(msg->header.stamp);
  if (timestamp < last_timestamp_) {
    RCLCPP_ERROR(rclcpp::get_logger("Imu"), "imu loop back, clear deque");
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

void Imu::popAndAdvance() {
  if (buffer_.empty()) {
    return;
  }
  last_ = next_;
  buffer_.pop_front();
  loadNextFromFront();
}

void Imu::setNeedInit(bool value) {
  processor_->imu_need_init_ = value;
}

namespace {
  input_ikfom makeInput(const sensor_msgs::msg::Imu& msg, double acc_scale) {
    input_ikfom input;
    input.gyro << msg.angular_velocity.x, msg.angular_velocity.y,
        msg.angular_velocity.z;
    input.acc << msg.linear_acceleration.x, msg.linear_acceleration.y,
        msg.linear_acceleration.z;
    input.acc *= acc_scale;
    return input;
  }

  ImuMeasurement makeMeasurement(const sensor_msgs::msg::Imu& msg) {
    ImuMeasurement measurement;
    measurement.angular_velocity << msg.angular_velocity.x,
        msg.angular_velocity.y, msg.angular_velocity.z;
    measurement.linear_acceleration << msg.linear_acceleration.x,
        msg.linear_acceleration.y, msg.linear_acceleration.z;
    return measurement;
  }
}  // namespace

input_ikfom Imu::lastInput(double acc_scale) const {
  return makeInput(last_, acc_scale);
}

input_ikfom Imu::nextInput(double acc_scale) const {
  return makeInput(next_, acc_scale);
}

ImuMeasurement Imu::lastMeasurement() const {
  return makeMeasurement(last_);
}

ImuMeasurement Imu::nextMeasurement() const {
  return makeMeasurement(next_);
}

bool Imu::collectUntil(double end_time, MeasureGroup& meas) {
  if (!needInit() || buffer_.empty()) {
    return false;
  }

  loadNextFromFront();
  meas.imu.shrink_to_fit();
  while (!buffer_.empty()
         && get_time_sec(buffer_.front()->header.stamp) < end_time) {
    meas.imu.emplace_back(buffer_.front());
    last_ = next_;
    buffer_.pop_front();
    loadNextFromFront();
  }
  return true;
}

void Imu::process(const MeasureGroup& meas, PointCloudXYZI::Ptr& undistort) {
  processor_->process(meas, undistort);
}

void Imu::reset() {
  processor_->reset();
  buffer_.clear();
  last_ = sensor_msgs::msg::Imu();
  next_ = sensor_msgs::msg::Imu();
  last_timestamp_ = -1.0;
}

bool Imu::needInit() const {
  return processor_->imu_need_init_;
}

double Imu::lastTimestamp() const {
  return last_timestamp_;
}

bool Imu::isSameStamp() const {
  return get_time_sec(next_.header.stamp)
         == get_time_sec(buffer_.front()->header.stamp);
}

void Imu::popBuffer() {
  buffer_.pop_front();
}
