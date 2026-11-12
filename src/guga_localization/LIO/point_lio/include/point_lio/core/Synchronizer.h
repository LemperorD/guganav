#pragma once

#include "point_lio/core/common_lib.h"

class Lidar;
class Imu;

class Synchronizer {
public:
  void configure(double lidar_time_interval) {
    lidar_time_interval_ = lidar_time_interval;
  }
  static bool syncPackages(Lidar& lidar, Imu& imu, MeasureGroup& measurement);

private:
  double lidar_time_interval_{0.1};
  constexpr static double TIME_TOLERANCE{0.5};
};
