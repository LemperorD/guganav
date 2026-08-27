#pragma once

class Lidar;
class Imu;
struct MeasureGroup;

class Synchronizer {
public:
  void configure(double lidar_time_interval) {
    lidar_time_interval_ = lidar_time_interval;
  }
  bool syncPackages(Lidar& lidar, Imu& imu, MeasureGroup& measurement);

private:
  double lidar_time_interval_{0.1};
};
