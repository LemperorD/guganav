/**
 * @file li_initialization.h
 * @brief LiDAR-IMU 初始化和传感器数据同步
 *
 * 本模块负责:
 * - **传感器数据接收**: 三种回调函数 (标准点云 / Livox 点云 / IMU)
 * - **帧预处理**: 切帧 (cut_frame) / 合帧 (con_frame) 逻辑
 * - **时间同步**: IMU 时间戳校准 (timediff_imu_wrt_lidar)
 * - **数据打包**: sync_packages() 将雷达帧与对应时间的 IMU 队列组合成
 * MeasureGroup
 *
 * 数据流:
 *   传感器 → 回调 (cbk) → 预处理 → lidar_buffer / imu_deque → sync_packages →
 * MeasureGroup
 */

#pragma once

#include <memory>
#include "point_lio/preprocess.h"
#include "point_lio/common_lib.h"
#include "point_lio/Imu.h"

#include "point_lio/Estimator.h"

/// @brief 最大缓冲区大小
#define MAXN (720000)

class Lidar {
public:
  Lidar() = default;
  ~Lidar() = default;

  using Params = LidarParams;

  void configure(const Params& params);
  void onStandardPcl(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
  void onLivoxPcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);
  bool syncPackages(Imu& imu, MeasureGroup& meas);

private:
  [[nodiscard]] int mergeFrameCount() const {
    return params_.con_frame_num;
  }
  void getMeasurements(MeasureGroup& meas) const;
  void popLidarFrame();
  void appendCutFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                       std::deque<double>& timestamps);
  void appendMergedFrame(const PointCloudXYZI::Ptr& points, double timestamp);

  Params params_;
  Preprocess preprocess_;
  PointCloudXYZI::Ptr ptr_con_{std::make_shared<PointCloudXYZI>()};
  int scan_count_{0};
  int frame_ct_{0};
  bool lidar_pushed_{false};
  bool imu_pushed_{false};
  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> time_buffer_;
  double last_timestamp_lidar_{-1.0};
  double time_con_{0.0};
  bool lose_lid_{false};
};
