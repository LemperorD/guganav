/**
 * @file ManifoldEskf.h
 * @brief 24 维误差状态卡尔曼滤波器声明.
 *
 * 包含状态传播、协方差传播、IMU 与雷达量测更新, 以及初始化配置.
 */

#pragma once

#include <array>

#include <Eigen/Dense>

#include "point_lio/core/Lidar.h"
#include "point_lio/core/parameters.h"
#include "point_lio/utilities/preprocess.h"

// 24 维误差的流形 ESKF: 状态传播、协方差传播、IMU 与 LiDAR 量测更新.
class ManifoldEskf {
public:
  ManifoldEskf();
  ~ManifoldEskf() = default;

  // 一次性完成滤波器的全部初始化: 噪声、初始协方差、外参与地图.
  void initialize(const PointLioParams& config, const IVoxType::Ptr& map,
                  const Eigen::Matrix3d& lidar_rotation,
                  const Eigen::Vector3d& lidar_translation);

  // 噪声配置.
  void configureProcessNoise(double gyr_cov, double acc_cov, double b_gyr_cov,
                             double b_acc_cov);
  void configureNoise(double lidar_cov, double imu_acc_cov, double imu_omg_cov);
  void configureFixedExtrinsic(const Eigen::Matrix3d& R,
                               const Eigen::Vector3d& t);
  void setInitialCovariance(const Eigen::Matrix<double, 24, 24>& P);
  void setMap(const IVoxType::Ptr& map) {
    map_ = map;
  }

  // 时间与残差.
  void setStartTime(double t);
  void setImuResidual(const Eigen::Matrix<double, 6, 1>& r);

  // 传播与更新.
  bool stateTransform(double lidar_time, const V3D& omega, const V3D& acc);
  void applyLidarUpdate();
  bool propagateImu(double imu_time);
  void applyImuUpdate(const std::array<bool, 6>& saturated = {});
  void commitPrediction();
  int updateLidar(MeasureGroup& meas, FilterOutput& out,
                  const LidarParams& params, int group_idx, int point_offset);
  void updateWorldPoints(MeasureGroup& meas, FilterOutput& out,
                         const LidarParams& params, int group_idx,
                         int point_offset);

  // 处理一个 IMU 样本: 传播到该时刻并做 IMU 量测更新.
  // 返回 false 表示传播被拒绝, 该样本不参与更新.
  bool imuUpdate(const ImuMeasurement& measurement, double imu_time,
                 const ImuParams& params, double acc_scale);

  // 处理一组 LiDAR 量测: 传播到该段时刻并做量测更新.
  // allow_update 为 false 时只提交预测 (启动阶段).
  // 返回 -1 表示传播失败, 0 表示只提交预测, 正数为更新的点数.
  int lidarUpdate(MeasureGroup& meas, FilterOutput& out,
                  const LidarParams& params, int group_idx, int point_offset,
                  bool allow_update);

  // 状态访问.
  State& mutable_x();
  const State& get_x() const;
  const State& get_x_predict() const;
  state_output toStateOutput() const;
  const std::vector<PointVector>& nearestPoints() const {
    return nearest_points_;
  }

private:
  Eigen::Matrix<double, 24, 1> f_(const State& x_, const V3D& omega,
                                  const V3D& acc) const;
  Eigen::Matrix<double, 24, 24> F_x(const State& x_, double dt) const;
  void pointBodyToWorld(PointType const* pi, PointType* po,
                        const LidarParams& params, const State& state) const;

  double last_time_{0.0};

  State x_;
  State x_predict;

  Eigen::Matrix<double, 24, 24> P_ = Eigen::Matrix<double, 24, 24>::Identity();
  Eigen::Matrix<double, 24, 24> P_bias =
      Eigen::Matrix<double, 24, 24>::Identity();
  Eigen::Matrix<double, 24, 24> P_predict =
      Eigen::Matrix<double, 24, 24>::Identity();

  // 噪声顺序: [w_omega, w_a, n_bg, n_ba], 共 12 维.
  Eigen::Matrix<double, 24, 12> F_w_ = Eigen::Matrix<double, 24, 12>::Zero();
  Eigen::Matrix<double, 12, 12> Q_ = Eigen::Matrix<double, 12, 12>::Zero();

  const Eigen::Matrix<double, 24, 24> I =
      Eigen::Matrix<double, 24, 24>::Identity();
  Eigen::Matrix<double, 24, 24> J = Eigen::Matrix<double, 24, 24>::Identity();

  // LiDAR 量测: 有效点数动态, 由 updateLidar 决定.
  Eigen::MatrixXd H_L;
  Eigen::MatrixXd D_L;
  Eigen::MatrixXd R_L;
  Eigen::MatrixXd S_L;
  Eigen::MatrixXd K_L;
  double lidar_cov_{1e-3};
  Eigen::Matrix3d R_LI_fixed_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_LI_fixed_ = Eigen::Vector3d::Zero();

  // IMU 量测: 6 维.
  Eigen::Matrix<double, 6, 24> H_I = Eigen::Matrix<double, 6, 24>::Zero();
  Eigen::Matrix<double, 6, 6> D_I = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> R_I = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> S_I = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 24, 6> K_I = Eigen::Matrix<double, 24, 6>::Zero();

  Eigen::Matrix<double, 24, 1> d_x = Eigen::Matrix<double, 24, 1>::Zero();
  Eigen::Matrix<double, 24, 1> dx_o = Eigen::Matrix<double, 24, 1>::Zero();

  // 残差由外部设入, 默认零.
  Eigen::VectorXd r_L;
  Eigen::Matrix<double, 6, 1> r_I = Eigen::Matrix<double, 6, 1>::Zero();

  // LiDAR 量测的中间缓存, 只在这里用.
  PointCloudXYZI::Ptr feats_down_world_ = std::make_shared<PointCloudXYZI>();
  PointCloudXYZI::Ptr normvec_ = std::make_shared<PointCloudXYZI>();
  std::vector<PointVector> nearest_points_;
  std::bitset<100000> point_selected_surf_;
  std::vector<V3D> pbody_list_;
  IVoxType::Ptr map_{nullptr};
};
