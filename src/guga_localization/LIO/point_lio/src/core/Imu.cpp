#include "point_lio/core/Imu.h"

#include "point_lio/core/Lidar.h"

Imu::Imu() : measurement_model_(measurement_workspace_) {
}

void ImuMeasurementModel::configure(const ImuParams& params) {
  params_ = params;
}

void Imu::configure(const Params& params) {
  params_ = params;
  processor_->configure(params.processor);
  measurement_model_.configure(params_);
}

void Imu::setCurrentMeasurement(const ImuMeasurement& measurement) {
  measurement_workspace_.angular_velocity = measurement.angular_velocity;
  measurement_workspace_.linear_acceleration = measurement.linear_acceleration;
}

void Imu::reset() {
  buffer_.clear();
  last_ = sensor_msgs::msg::Imu{};
  next_ = sensor_msgs::msg::Imu{};
  last_timestamp_ = -1.0;
  processor_->reset();
}

void Imu::onMessage(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_in) {
  auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
  msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp)
                                   + params_.timestamp_offset);

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

void Imu::loadNext() {
  if (!buffer_.empty()) {
    next_ = *buffer_.front();
  }
}

void Imu::discardBefore(double timestamp) {
  while (!buffer_.empty()) {
    loadNext();
    if (timestamp - get_time_sec(next_.header.stamp) < TOLERANCE) {
      return;
    }
    last_ = next_;
    buffer_.pop_front();
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
  loadNext();
}

ImuMeasurement Imu::lastMeasurement() const {
  return makeMeasurement(last_);
}

ImuMeasurement Imu::nextMeasurement() const {
  return makeMeasurement(next_);
}

void Imu::collectUntil(
    double end_time, deque<sensor_msgs::msg::Imu::ConstSharedPtr>& collected) {
  if (empty()) {
    return;
  }

  loadNext();
  collected.shrink_to_fit();
  while (!buffer_.empty()
         && get_time_sec(buffer_.front()->header.stamp) < end_time) {
    collected.emplace_back(buffer_.front());
    last_ = next_;
    buffer_.pop_front();
    loadNext();
  }
}

void Imu::process(const MeasureGroup& meas, PointCloudXYZI::Ptr& undistort,
                  State& output_state) {
  processor_->process(meas, undistort, output_state);
}

bool Imu::needInit() const {
  return processor_->needInit();
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
