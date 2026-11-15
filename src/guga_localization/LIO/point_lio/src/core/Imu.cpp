#include <cmath>

#include "point_lio/core/Lidar.h"
#include "point_lio/core/Imu.h"

bool time_list(PointType& x, PointType& y) {
  return point_time_offset_ms(x) < point_time_offset_ms(y);
}

Imu::Imu()
    : measurement_model_(measurement_workspace_),
      logger_(rclcpp::get_logger("Imu")) {
}

void ImuMeasurementModel::configure(const ImuParams& params) {
  params_ = params;
}

void Imu::configure(const Params& params) {
  params_ = &params;
  measurement_model_.configure(params);
}

void Imu::setCurrentMeasurement(const ImuMeasurement& measurement) {
  measurement_workspace_.angular_velocity = measurement.angular_velocity;
  measurement_workspace_.linear_acceleration = measurement.linear_acceleration;
}

void Imu::append(const ImuMeasurement& measurement, double timestamp) {
  const double adjusted = timestamp + params_->timestamp_offset;
  // <= 而不是 <: 桥接偶尔会发出时间戳完全相同的两条消息, 保留第一条,
  // 后面的丢掉, 避免滤波器按 dt=0 积分。
  if (adjusted <= last_timestamp_) {
    return;
  }
  buffer_.push_back(ImuSample{measurement, adjusted});
  last_timestamp_ = adjusted;
}

bool Imu::empty() const {
  return buffer_.empty();
}

const ImuSample& Imu::next() const {
  return next_;
}

void Imu::loadNext() {
  if (!buffer_.empty()) {
    next_ = buffer_.front();
  }
}

void Imu::discardBefore(double timestamp) {
  while (!buffer_.empty()) {
    loadNext();
    if (timestamp - next_.timestamp < TOLERANCE) {
      return;
    }
    last_ = next_;
    buffer_.pop_front();
  }
}

void Imu::collectUntil(double end_time, deque<ImuSample>& collected) {
  if (empty()) {
    return;
  }

  loadNext();
  collected.shrink_to_fit();
  while (!buffer_.empty() && buffer_.front().timestamp < end_time) {
    collected.push_back(buffer_.front());
    last_ = next_;
    buffer_.pop_front();
    loadNext();
  }
}

std::optional<ImuInitEstimate> Imu::process(const MeasureGroup& meas) {
  if (meas.imu_measure.empty() || stage_ != Stage::Initializing) {
    return std::nullopt;
  }

  IMU_init(meas, init_iter_num_);
  if (init_iter_num_ <= kMaxInitCount) {
    return std::nullopt;
  }

  RCLCPP_INFO(logger_, "IMU Initializing: %.1f %%", 100.0);
  stage_ = Stage::Ready;
  return ImuInitEstimate{static_count_, mean_acc_, mean_gyr_};
}

void Imu::IMU_init(const MeasureGroup& meas, int& N) {
  RCLCPP_INFO(logger_, "IMU Initializing: %.1f %%",
              double(N) / kMaxInitCount * 100);
  V3D cur_acc, cur_gyr;

  for (const auto& imu : meas.imu_measure) {
    const auto& imu_acc = imu.measurement.linear_acceleration;
    const auto& imu_gyr = imu.measurement.angular_velocity;
    cur_acc = imu_acc;
    cur_gyr = imu_gyr;

    // 只累积静止样本. 旋转中的角速度当成零偏, 会把真实旋转吸收掉.
    if (cur_gyr.norm() < kStaticGyroThreshold) {
      mean_acc_ += (cur_acc - mean_acc_) / (static_count_ + 1);
      mean_gyr_ += (cur_gyr - mean_gyr_) / (static_count_ + 1);
      ++static_count_;
    }
    ++N;
  }
}

bool Imu::needInit() const {
  return stage_ == Stage::Initializing;
}

double Imu::lastTimestamp() const {
  return last_timestamp_;
}
