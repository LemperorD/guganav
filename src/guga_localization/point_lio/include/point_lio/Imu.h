#pragma once

#include <deque>
#include <vector>

#include "point_lio/IMU_Processing.h"
#include "point_lio/preprocess.h"

struct ImuMeasurement {
  V3D angular_velocity;
  V3D linear_acceleration;
};

class Imu {
public:
  struct Params {
    bool enabled{true};
    std::vector<double> gravity;
    std::vector<double> gravity_init;
    double gravity_magnitude{9.81};
    double integration_interval{0.005};
    double timestamp_offset{0.0};  ///< corrected_time = raw_time + offset
  };

  void configure(const Params& params);

  void onMessage(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

  [[nodiscard]] bool empty() const;
  [[nodiscard]] bool isSameStamp() const;
  [[nodiscard]] const sensor_msgs::msg::Imu& last() const;
  [[nodiscard]] const sensor_msgs::msg::Imu& next() const;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr>& buffer();

  void popBuffer();
  void loadNextFromFront();
  void discardBefore(double timestamp);
  void advanceCursor();
  void popAndAdvance();
  void setNeedInit(bool value);
  [[nodiscard]] input_ikfom lastInput(double acc_scale) const;
  [[nodiscard]] input_ikfom nextInput(double acc_scale) const;
  [[nodiscard]] ImuMeasurement lastMeasurement() const;
  [[nodiscard]] ImuMeasurement nextMeasurement() const;

  bool collectUntil(double end_time, MeasureGroup& meas);
  void process(const MeasureGroup& meas, PointCloudXYZI::Ptr& undistort);

  [[nodiscard]] bool needInit() const;
  [[nodiscard]] double lastTimestamp() const;

private:
  Params params_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> buffer_;
  sensor_msgs::msg::Imu last_;
  sensor_msgs::msg::Imu next_;
  double last_timestamp_ = -1.0;
  shared_ptr<ImuProcessor> processor_ =
      std::make_shared<ImuProcessor>();  ///< IMU 处理模块
};
