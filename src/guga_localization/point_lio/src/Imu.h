#pragma once

#include <deque>
#include <vector>

#include "IMU_Processing.h"

class Imu {
public:
  Imu();
  ~Imu() = default;

  void configure(int lidar_type, bool enabled,
                 const std::vector<double>& gravity);

  void onMessage(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

  bool empty() const;
  const sensor_msgs::msg::Imu& last() const;
  const sensor_msgs::msg::Imu& next() const;
  const sensor_msgs::msg::Imu::ConstSharedPtr& front() const;
  sensor_msgs::msg::Imu& lastMutable();
  sensor_msgs::msg::Imu& nextMutable();
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr>& buffer();

  void loadNextFromFront();
  void discardBefore(double timestamp);
  void advanceCursor();
  void popFront();
  void popAndAdvance();
  void setNeedInit(bool value);

  bool collectUntil(double end_time, MeasureGroup& meas);
  void process(const MeasureGroup& meas, PointCloudXYZI::Ptr& undistort);
  void reset();

  bool needInit() const;
  bool stateInitialized() const;
  double lastTimestamp() const;

private:
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> buffer_;
  sensor_msgs::msg::Imu last_;
  sensor_msgs::msg::Imu next_;
  double last_timestamp_ = -1.0;
};
