#pragma once

#include <deque>

#include "point_lio/core/IMU_Processing.h"
#include "point_lio/core/parameters.h"
#include "point_lio/core/preprocess.h"
#include "point_lio/core/common_lib.h"

struct ImuMeasurement {
  V3D angular_velocity;
  V3D linear_acceleration;
};

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

  void configure(const Params& params);
  void reset();
  [[nodiscard]] ImuMeasurementModel& measurementModel() {
    return measurement_model_;
  }
  void setCurrentMeasurement(const ImuMeasurement& measurement);

  void onMessage(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

  [[nodiscard]] bool empty() const;
  [[nodiscard]] bool isSameStamp() const;
  [[nodiscard]] const sensor_msgs::msg::Imu& last() const;
  [[nodiscard]] const sensor_msgs::msg::Imu& next() const;
  void popBuffer();
  void loadNext();
  void discardBefore(double timestamp);
  void advanceCursor();
  void popAndAdvance();
  [[nodiscard]] ImuMeasurement lastMeasurement() const;
  [[nodiscard]] ImuMeasurement nextMeasurement() const;

  void collectUntil(double end_time,
                    deque<sensor_msgs::msg::Imu::ConstSharedPtr>& collected);
  void process(const MeasureGroup& meas, PointCloudXYZI::Ptr& undistort,
               State& output_state);

  [[nodiscard]] bool needInit() const;
  [[nodiscard]] double lastTimestamp() const;

private:
  constexpr static double TOLERANCE = 1e-6;
  Params params_;
  ImuMeasurementWorkspace measurement_workspace_;
  ImuMeasurementModel measurement_model_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> buffer_;
  sensor_msgs::msg::Imu last_;
  sensor_msgs::msg::Imu next_;
  double last_timestamp_ = -1.0;
  shared_ptr<ImuProcessor> processor_ =
      std::make_shared<ImuProcessor>();  ///< IMU 处理模块
};
