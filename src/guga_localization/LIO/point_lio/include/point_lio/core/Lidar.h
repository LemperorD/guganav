/**
 * @file Lidar.h
 * @brief LiDAR 数据接入、同步和点面量测模型
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

#include <bitset>
#include <memory>
#include "point_lio/preprocess.h"
#include "point_lio/core/common_lib.h"
#include "point_lio/core/Imu.h"
class Synchronizer;

/// @brief 一帧点云及其绝对起止时间。点云与时间成对存放，避免平行队列错位。
struct LidarFrame {
  PointCloudXYZI::Ptr cloud;
  double start_time{0.0};
  double end_time{0.0};
};

class LidarMeasurementModel {
public:
  explicit LidarMeasurementModel(V3D& lidar_translation, M3D& lidar_rotation)
      : lidar_translation_(&lidar_translation),
        lidar_rotation_(&lidar_rotation) {
  }
  void configure(const LidarParams& params);
  void pointBodyToWorld(PointType const* input, PointType* output,
                        const State& state) const;

private:
  LidarParams params_;
  V3D* lidar_translation_;
  M3D* lidar_rotation_;
};

class Lidar {
public:
  Lidar() : measurement_model_(lidar_t_wrt_imu_, lidar_r_wrt_imu_) {
  }
  ~Lidar() = default;

  using Params = LidarParams;

  void configure(const Params& params);
  [[nodiscard]] LidarMeasurementModel& measurementModel() {
    return measurement_model_;
  }
  [[nodiscard]] IVoxType::Ptr& ivox() {
    return ivox_;
  }
  [[nodiscard]] const IVoxType::Ptr& ivox() const {
    return ivox_;
  }
  void setExtrinsics(const V3D& translation, const M3D& rotation) {
    lidar_t_wrt_imu_ = translation;
    lidar_r_wrt_imu_ = rotation;
  }
  [[nodiscard]] bool empty() const;
  [[nodiscard]] const V3D& TranslationLidarToImu() const {
    return lidar_t_wrt_imu_;
  }
  [[nodiscard]] const M3D& RotationLidarToImu() const {
    return lidar_r_wrt_imu_;
  }

private:
  friend class Synchronizer;
  friend class FrameProcessor;
  [[nodiscard]] int mergeFrameCount() const {
    return params_->con_frame_num;
  }
  void append(PointCloudXYZI::Ptr points, double timestamp);
  void appendFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                    std::deque<double>& timestamps);
  void appendFrame(PointCloudXYZI::Ptr pointcloud, double timestamp);
  void appendMergedFrame(const PointCloudXYZI::Ptr& points, double timestamp);

  const Params* params_{nullptr};
  IVoxType::Ptr ivox_{nullptr};
  V3D lidar_t_wrt_imu_{Zero3d};
  M3D lidar_r_wrt_imu_{IMatrix3};
  LidarMeasurementModel measurement_model_;
  PointCloudXYZI::Ptr ptr_con_{std::make_shared<PointCloudXYZI>()};
  int scan_count_{0};
  int frame_ct_{0};
  std::deque<LidarFrame> frame_buffer_;
  double last_timestamp_lidar_{-1.0};
  double time_con_{0.0};
};
