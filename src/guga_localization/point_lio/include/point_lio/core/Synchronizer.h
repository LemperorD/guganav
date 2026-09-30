#pragma once

#include "point_lio/core/Measurement.h"

class Lidar;
class Imu;

class Synchronizer {
public:
  void configure(double lidar_time_interval) {
    lidar_time_interval_ = lidar_time_interval;
  }
  bool syncPackages(Lidar& lidar, Imu& imu, MeasureGroup& measurement);

private:
  double lidar_time_interval_{0.1};
};
