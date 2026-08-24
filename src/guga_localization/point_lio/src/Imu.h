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

  [[nodiscard]] bool empty() const;
  [[nodiscard]] bool isSameStamp() const;
  void popBuffer();
  sensor_msgs::msg::Imu& lastMutable();
  sensor_msgs::msg::Imu& nextMutable();
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr>& buffer();

  void loadNextFromFront();
  void discardBefore(double timestamp);
  void advanceCursor();
  void popAndAdvance();
  void setNeedInit(bool value);

  bool collectUntil(double end_time, MeasureGroup& meas);
  void process(const MeasureGroup& meas, PointCloudXYZI::Ptr& undistort);
  void reset();

  [[nodiscard]] bool needInit() const;
  [[nodiscard]] double lastTimestamp() const;

private:
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> buffer_;
  sensor_msgs::msg::Imu last_;
  sensor_msgs::msg::Imu next_;
  double last_timestamp_ = -1.0;
  shared_ptr<ImuProcessor> processor_ =
      std::make_shared<ImuProcessor>();  ///< IMU 处理模块
};
