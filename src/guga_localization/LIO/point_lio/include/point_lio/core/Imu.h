/**
 * @file Imu.h
 * @brief IMU 样本缓冲与静态初始化.
 *
 * 接收纯数据样本, 维护时间顺序, 在静止段估计零偏与初始姿态.
 */

#pragma once

#include <deque>

#include "point_lio/core/common_lib.h"
#include "point_lio/core/parameters.h"
#include "point_lio/preprocess.h"

inline ImuMeasurement makeMeasurement(const sensor_msgs::msg::Imu& msg) {
  ImuMeasurement measurement;
  measurement.angular_velocity << msg.angular_velocity.x,
      msg.angular_velocity.y, msg.angular_velocity.z;
  measurement.linear_acceleration << msg.linear_acceleration.x,
      msg.linear_acceleration.y, msg.linear_acceleration.z;
  return measurement;
}

struct ImuMeasurementWorkspace {
  V3D angular_velocity{V3D::Zero()};
  V3D linear_acceleration{V3D::Zero()};
};

class ImuMeasurementModel {
public:
  explicit ImuMeasurementModel(ImuMeasurementWorkspace& workspace)
      : workspace_(workspace) {
  }
  void configure(const ImuParams& params);

private:
  ImuParams params_;
  ImuMeasurementWorkspace& workspace_;
};

class Imu {
public:
  Imu();
  using Params = ImuParams;

  enum class Stage { Initializing, Ready };

  void configure(const Params& params);
  [[nodiscard]] ImuMeasurementModel& measurementModel() {
    return measurement_model_;
  }
  void setCurrentMeasurement(const ImuMeasurement& measurement);

  void append(const ImuMeasurement& measurement, double timestamp);

  [[nodiscard]] bool empty() const;
  [[nodiscard]] const ImuSample& next() const;
  void loadNext();
  void discardBefore(double timestamp);

  void collectUntil(double end_time, deque<ImuSample>& collected);
  void process(const MeasureGroup& meas, State& output_state);

  [[nodiscard]] bool needInit() const;
  [[nodiscard]] double lastTimestamp() const;

private:
  constexpr static int kMaxInitCount = 100;
  constexpr static double TOLERANCE = 1e-6;
  /// 角速度低于该阈值时判定为静止, 只累积静止样本估计零偏.
  constexpr static double kStaticGyroThreshold = 0.1;

  void IMU_init(const MeasureGroup& meas, int& N);
  void initState(State& state);
  void Set_init_z(const Eigen::Vector3d& tmp_gravity, Eigen::Matrix3d& rot);

  const Params* params_{nullptr};
  ImuMeasurementWorkspace measurement_workspace_;
  ImuMeasurementModel measurement_model_;
  std::deque<ImuSample> buffer_;
  ImuSample last_;
  ImuSample next_;
  double last_timestamp_ = -1.0;

  Stage stage_{Stage::Initializing};
  int init_iter_num_{1};
  V3D mean_acc_{V3D::Zero()};
  V3D mean_gyr_{V3D::Zero()};
  int static_count_{0};
  V3D gravity_{V3D::Zero()};
  V3D gravity_init_{0.0, 0.0, -9.81};
  double gravity_magnitude_{9.81};
  rclcpp::Logger logger_;
};
