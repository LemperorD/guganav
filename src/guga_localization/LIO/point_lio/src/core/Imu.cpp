#include "point_lio/core/Imu.h"

#include <cmath>

#include "point_lio/core/Lidar.h"

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
  params_ = params;
  gravity_ = params.processor.gravity;
  gravity_init_ = params.processor.gravity_init;
  gravity_magnitude_ = params.processor.gravity_magnitude;
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
  stage_ = Stage::Initializing;
  init_iter_num_ = 1;
  mean_acc_ = V3D::Zero();
  mean_gyr_ = V3D::Zero();
  static_count_ = 0;
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

void Imu::process(const MeasureGroup& meas, State& output_state) {
  if (meas.imu_measure.empty()) {
    return;
  }

  if (stage_ == Stage::Initializing) {
    IMU_init(meas, init_iter_num_);
    if (init_iter_num_ <= kMaxInitCount) {
      return;
    }

    RCLCPP_INFO(logger_, "IMU Initializing: %.1f %%", 100.0);
    initState(output_state);
    stage_ = Stage::Ready;
  }
}

void Imu::IMU_init(const MeasureGroup& meas, int& N) {
  RCLCPP_INFO(logger_, "IMU Initializing: %.1f %%",
              double(N) / kMaxInitCount * 100);
  V3D cur_acc, cur_gyr;

  for (const auto& imu : meas.imu_measure) {
    const auto& imu_acc = imu->linear_acceleration;
    const auto& imu_gyr = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << imu_gyr.x, imu_gyr.y, imu_gyr.z;

    // 只累积静止样本. 旋转中的角速度当成零偏, 会把真实旋转吸收掉.
    if (cur_gyr.norm() < kStaticGyroThreshold) {
      mean_acc_ += (cur_acc - mean_acc_) / (static_count_ + 1);
      mean_gyr_ += (cur_gyr - mean_gyr_) / (static_count_ + 1);
      ++static_count_;
    }
    ++N;
  }
}

void Imu::initState(State& state) {
  if (static_count_ == 0) {
    // 没有静止段, 不做重力对齐, 零偏保持零, 交给滤波器在线收敛.
    state.rot = IMatrix3;
    state.acc = -state.gravity;
    state.bg = V3D::Zero();
    state.ba = V3D::Zero();
    return;
  }

  const V3D tmp_gravity = -mean_acc_ / mean_acc_.norm() * gravity_magnitude_;
  M3D rot_init;
  Set_init_z(tmp_gravity, rot_init);
  state.rot = rot_init;
  state.acc = -rot_init.transpose() * state.gravity;
  state.bg = mean_gyr_;
  state.ba = V3D::Zero();
}

void Imu::Set_init_z(const Eigen::Vector3d& tmp_gravity,
                     Eigen::Matrix3d& rot) {
  const M3D hat_grav = -Hat(gravity_);

  const V3D cross = hat_grav * tmp_gravity;
  const double cross_norm = cross.norm();
  const double gravity_norm = gravity_.norm();
  const double tmp_norm = tmp_gravity.norm();

  const double align_sin = cross_norm / gravity_norm / tmp_norm;
  const double align_cos = gravity_.dot(tmp_gravity) / gravity_norm / tmp_norm;

  if (align_sin < TOLERANCE) {
    if (align_cos > TOLERANCE) {
      rot = IMatrix3;
    } else {
      // 两向量严格反向时绕任意垂直轴转 180 度, 避免使用反射矩阵 -I.
      const V3D unit_gravity = gravity_ / gravity_norm;
      V3D axis;

      if (std::abs(unit_gravity.x()) <= std::abs(unit_gravity.y())
          && std::abs(unit_gravity.x()) <= std::abs(unit_gravity.z())) {
        axis = unit_gravity.cross(V3D::UnitX());
      } else if (std::abs(unit_gravity.y()) <= std::abs(unit_gravity.z())) {
        axis = unit_gravity.cross(V3D::UnitY());
      } else {
        axis = unit_gravity.cross(V3D::UnitZ());
      }

      axis.normalize();
      rot = 2.0 * axis * axis.transpose() - IMatrix3;
    }
  } else {
    const V3D align_angle = cross / cross_norm * std::acos(align_cos);
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

bool Imu::needInit() const {
  return stage_ == Stage::Initializing;
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
