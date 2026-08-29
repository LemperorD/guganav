#pragma once

#include <deque>

#include "point_lio/IMU_Processing.h"
#include "point_lio/parameters.h"
#include "point_lio/preprocess.h"

struct ImuMeasurement {
  V3D angular_velocity;
  V3D linear_acceleration;
};

struct ImuMeasurementWorkspace {
  V3D angular_velocity{V3D::Zero()};
  V3D linear_acceleration{V3D::Zero()};
};

class ImuMeasurementModel {
public:
  explicit ImuMeasurementModel(ImuMeasurementWorkspace& workspace)
      : workspace_(workspace) {}
  void configure(const ImuParams& params);
  void hModelOutput(state_output& state,
                    esekfom::dyn_share_modified<double>& data) const;

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
  void loadNextFromFront();
  void discardBefore(double timestamp);
  void advanceCursor();
  void popAndAdvance();
  [[nodiscard]] input_ikfom lastInput(double acc_scale) const;
  [[nodiscard]] input_ikfom nextInput(double acc_scale) const;
  [[nodiscard]] ImuMeasurement lastMeasurement() const;
  [[nodiscard]] ImuMeasurement nextMeasurement() const;

  bool collectUntil(double end_time, MeasureGroup& meas);
  void process(const MeasureGroup& meas, PointCloudXYZI::Ptr& undistort,
               state_input& input_state, state_output& output_state);

  [[nodiscard]] bool needInit() const;
  [[nodiscard]] double lastTimestamp() const;

private:
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
