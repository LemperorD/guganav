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
#include "point_lio/common_lib.h"
#include "point_lio/Imu.h"

/// @brief 最大缓冲区大小
#define MAXN (720000)

struct LioWorkspace {
  PointCloudXYZI::Ptr normvec{new PointCloudXYZI(100000, 1)};
  std::vector<int> time_seq;
  PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI(10000, 1)};
  PointCloudXYZI::Ptr feats_down_world{new PointCloudXYZI(10000, 1)};
  std::vector<V3D> pbody_list;
  std::vector<PointVector> Nearest_Points;
  IVoxType::Ptr ivox_{nullptr};
  std::bitset<100000> point_selected_surf;
  std::vector<M3D> crossmat_list;
  int effct_feat_num{0};
  int k{0};
  int idx{-1};
  input_ikfom input_in;
  V3D angvel_avr;
  V3D acc_avr;
  size_t feats_down_size{0};
  V3D Lidar_T_wrt_IMU{Zero3d};
  M3D Lidar_R_wrt_IMU{Eye3d};
};

extern LioWorkspace lio_workspace;

class LidarMeasurementModel {
public:
  void configure(const LidarParams& params);
  void hModelInput(state_input& state, Eigen::Matrix3d cov_p,
                   Eigen::Matrix3d cov_R,
                   esekfom::dyn_share_modified<double>& data) const;
  void hModelOutput(state_output& state, Eigen::Matrix3d cov_p,
                    Eigen::Matrix3d cov_R,
                    esekfom::dyn_share_modified<double>& data) const;
  void pointBodyToWorld(PointType const* input, PointType* output,
                        const state_input& state) const;
  void pointBodyToWorld(PointType const* input, PointType* output,
                        const state_output& state) const;

private:
  LidarParams params_;
};

class Lidar {
public:
  Lidar() = default;
  ~Lidar() = default;

  using Params = LidarParams;

  void configure(const Params& params);
  [[nodiscard]] LidarMeasurementModel& measurementModel() {
    return measurement_model_;
  }
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
  LidarMeasurementModel measurement_model_;
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
