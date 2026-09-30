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
#include "point_lio/core/preprocess.h"
#include "point_lio/core/common_lib.h"
#include "point_lio/core/Imu.h"
class Synchronizer;

struct LidarWorkspace {
  PointCloudXYZI::Ptr normvec{new PointCloudXYZI(100000, 1)};
  std::vector<int> time_seq;
  PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI(10000, 1)};
  PointCloudXYZI::Ptr feats_down_world{new PointCloudXYZI(10000, 1)};
  std::vector<V3D> pbody_list;
  std::vector<PointVector> Nearest_Points;
  IVoxType::Ptr ivox_{nullptr};
  std::bitset<100000> point_selected_surf;
  std::vector<M3D> crossmat_list;
  int k{0};
  int idx{-1};
  size_t feats_down_size{0};
};

class LidarMeasurementModel {
public:
  explicit LidarMeasurementModel(LidarWorkspace& workspace,
                                 V3D& lidar_translation,
                                 M3D& lidar_rotation)
      : workspace_(&workspace),
        lidar_translation_(&lidar_translation),
        lidar_rotation_(&lidar_rotation) {}
  void configure(const LidarParams& params);
  void hModelInput(state_input& state,
                   esekfom::dyn_share_modified<double>& data) const;
  void hModelOutput(state_output& state,
                    esekfom::dyn_share_modified<double>& data) const;
  void pointBodyToWorld(PointType const* input, PointType* output,
                        const state_input& state) const;
  void pointBodyToWorld(PointType const* input, PointType* output,
                        const state_output& state) const;

private:
  LidarParams params_;
  LidarWorkspace* workspace_;
  V3D* lidar_translation_;
  M3D* lidar_rotation_;
};

class Lidar {
public:
  Lidar()
      : measurement_model_(workspace_, lidar_t_wrt_imu_, lidar_r_wrt_imu_) {}
  ~Lidar() = default;

  using Params = LidarParams;

  void configure(const Params& params);
  void reset();
  [[nodiscard]] LidarMeasurementModel& measurementModel() {
    return measurement_model_;
  }
  [[nodiscard]] LidarWorkspace& workspace() { return workspace_; }
  [[nodiscard]] const LidarWorkspace& workspace() const { return workspace_; }
  void setExtrinsics(const V3D& translation, const M3D& rotation) {
    lidar_t_wrt_imu_ = translation;
    lidar_r_wrt_imu_ = rotation;
  }
  [[nodiscard]] const V3D& lidarTranslation() const { return lidar_t_wrt_imu_; }
  [[nodiscard]] const M3D& lidarRotation() const { return lidar_r_wrt_imu_; }
  void onStandardPcl(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
  void onLivoxPcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);

private:
  friend class Synchronizer;
  [[nodiscard]] int mergeFrameCount() const { return params_.con_frame_num; }
  void appendCutFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                       std::deque<double>& timestamps);
  void appendFrame(PointCloudXYZI::Ptr points, double timestamp);
  void appendMergedFrame(const PointCloudXYZI::Ptr& points, double timestamp);

  Params params_;
  LidarWorkspace workspace_;
  V3D lidar_t_wrt_imu_{Zero3d};
  M3D lidar_r_wrt_imu_{Eye3d};
  LidarMeasurementModel measurement_model_;
  Preprocess preprocess_;
  PointCloudXYZI::Ptr ptr_con_{std::make_shared<PointCloudXYZI>()};
  int scan_count_{0};
  int frame_ct_{0};
  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> time_buffer_;
  double last_timestamp_lidar_{-1.0};
  double time_con_{0.0};
};
