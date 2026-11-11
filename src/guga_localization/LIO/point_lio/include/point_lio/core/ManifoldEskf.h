#pragma once

#include <array>

#include <Eigen/Dense>

#include "point_lio/core/Lidar.h"
#include "point_lio/core/preprocess.h"

// 30 维误差的流形 ESKF: 状态传播、协方差传播、IMU 与 LiDAR 量测更新.
class ManifoldEskf {
public:
  ManifoldEskf();
  ~ManifoldEskf() = default;

  // 噪声配置.
  void configureProcessNoise(double gyr_cov, double acc_cov, double b_gyr_cov,
                             double b_acc_cov);
  void configureNoise(double lidar_cov, double imu_acc_cov, double imu_omg_cov);
  void configureFixedExtrinsic(const Eigen::Matrix3d& R,
                               const Eigen::Vector3d& t);
  void setInitialCovariance(const Eigen::Matrix<double, 30, 30>& P);

  // 时间与残差.
  void setStartTime(double t);
  void setLidarResidual(const Eigen::VectorXd& r);
  void setImuResidual(const Eigen::Matrix<double, 6, 1>& r);

  // 传播与更新.
  bool propagateLidar(double lidar_time);
  void applyLidarUpdate();
  bool propagateImu(double imu_time);
  void applyImuUpdate(const std::array<bool, 6>& saturated = {});
  void commitPrediction();
  int updateLidar(LidarWorkspace& workspace, const LidarParams& params,
                  int group_idx, int point_offset);
  void updateWorldPoints(LidarWorkspace& workspace, const LidarParams& params,
                         int group_idx, int point_offset);
  void Kalman_filter(double lidar_time, double imu_time);

  // 状态访问.
  State& mutable_x();
  const State& get_x() const;
  const State& get_x_predict() const;
  const Eigen::Matrix<double, 30, 30>& get_P() const;
  state_output toStateOutput() const;
  void setFromStateOutput(const state_output& s);

private:
  Eigen::Matrix<double, 30, 1> f_(const State& x_, const State&) const;
  Eigen::Matrix<double, 30, 30> F_x(const State& x_, double dt) const;
  void pointBodyToWorld(PointType const* pi, PointType* po,
                        const LidarParams& params, const State& state) const;

  double last_time_{0.0};

  State x_;
  State x_predict;

  Eigen::Matrix<double, 30, 30> P_ =
      Eigen::Matrix<double, 30, 30>::Identity();
  Eigen::Matrix<double, 30, 30> P_bias =
      Eigen::Matrix<double, 30, 30>::Identity();
  Eigen::Matrix<double, 30, 30> P_predict =
      Eigen::Matrix<double, 30, 30>::Identity();

  // 噪声顺序: [w_omega, w_a, n_bg, n_ba], 共 12 维.
  Eigen::Matrix<double, 30, 12> F_w_ =
      Eigen::Matrix<double, 30, 12>::Zero();
  Eigen::Matrix<double, 12, 12> Q_ = Eigen::Matrix<double, 12, 12>::Zero();

  const Eigen::Matrix<double, 30, 30> I =
      Eigen::Matrix<double, 30, 30>::Identity();
  Eigen::Matrix<double, 30, 30> J = Eigen::Matrix<double, 30, 30>::Identity();

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
  Eigen::Matrix<double, 6, 30> H_I = Eigen::Matrix<double, 6, 30>::Zero();
  Eigen::Matrix<double, 6, 6> D_I = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> R_I = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 1> n_I = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> S_I = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 30, 6> K_I = Eigen::Matrix<double, 30, 6>::Zero();

  Eigen::Matrix<double, 30, 1> d_x = Eigen::Matrix<double, 30, 1>::Zero();
  Eigen::Matrix<double, 30, 1> dx_o = Eigen::Matrix<double, 30, 1>::Zero();

  // 残差由外部设入, 默认零.
  Eigen::VectorXd r_L;
  Eigen::Matrix<double, 6, 1> r_I = Eigen::Matrix<double, 6, 1>::Zero();
};
