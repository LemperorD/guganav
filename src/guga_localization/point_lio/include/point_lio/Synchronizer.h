#pragma once

#include <deque>
#include <utility>
#include "point_lio/common_lib.h"
#include "point_lio/Imu.h"
#include "point_lio/parameters.h"

class Synchronizer {
public:
  using Params = LidarParams;
  void configure(const Params& params) { params_ = params; }
  void appendCutFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                       std::deque<double>& timestamps);
  void appendFrame(PointCloudXYZI::Ptr points, double timestamp);
  void appendMergedFrame(const PointCloudXYZI::Ptr& points, double timestamp);
  bool syncPackages(Imu& imu, MeasureGroup& measurement);

private:
  void getMeasurements(MeasureGroup& measurement) const;
  void popLidarFrame();
  Params params_;
  PointCloudXYZI::Ptr ptr_con_{std::make_shared<PointCloudXYZI>()};
  int frame_ct_{0};
  bool lidar_pushed_{false};
  bool imu_pushed_{false};
  bool lose_lid_{false};
  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> time_buffer_;
  double time_con_{0.0};
};
