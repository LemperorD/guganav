#include <cmath>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "point_lio/core/ManifoldEskf.h"
#include "point_lio/utilities/common_lib.h"

namespace {

using M3 = Eigen::Matrix3d;
using M12 = Eigen::Matrix<double, 12, 12>;
using M24 = Eigen::Matrix<double, 24, 24>;
using M24x12 = Eigen::Matrix<double, 24, 12>;

/// 稠密参考实现: F P F^T + F_w Q F_w^T dt, 用作块写法的对拍.
M24 denseReference(const State& s, const M24& P, double dt, double q_bg,
                   double q_ba, double q_g, double q_a) {
  M24 F = M24::Identity();
  F.block<3, 3>(0, 0) = Exp(-dt * s.omega);
  F.block<3, 3>(0, 18) = dt * M3::Identity();
  F.block<3, 3>(3, 6) = dt * M3::Identity();
  F.block<3, 3>(6, 0) = -dt * s.rot * Hat(s.acc);
  F.block<3, 3>(6, 21) = dt * s.rot;

  M24x12 Fw = M24x12::Zero();
  Fw.block<3, 3>(9, 0) = M3::Identity();
  Fw.block<3, 3>(12, 3) = M3::Identity();
  Fw.block<3, 3>(18, 6) = M3::Identity();
  Fw.block<3, 3>(21, 9) = M3::Identity();

  M12 Q = M12::Zero();
  Q.block<3, 3>(0, 0) = q_bg * M3::Identity();
  Q.block<3, 3>(3, 3) = q_ba * M3::Identity();
  Q.block<3, 3>(6, 6) = q_g * M3::Identity();
  Q.block<3, 3>(9, 9) = q_a * M3::Identity();

  return F * P * F.transpose() + Fw * Q * Fw.transpose() * dt;
}

/// 稠密且非对称性为零的初始协方差, 保证每个块都参与计算.
M24 makeDenseSpd() {
  M24 A;
  for (int i = 0; i < 24; ++i) {
    for (int j = 0; j < 24; ++j) {
      A(i, j) = 0.01 * std::sin(0.7 * i + 1.3 * j);
    }
  }
  return A * A.transpose() + 0.1 * M24::Identity();
}

}  // namespace

TEST(EskfPropagation, BlockwiseMatchesDenseReference) {
  const double q_g = 1000.0;
  const double q_a = 500.0;
  const double q_bg = 1e-4;
  const double q_ba = 1e-4;

  ManifoldEskf eskf;
  eskf.configureProcessNoise(q_g, q_a, q_bg, q_ba);
  eskf.setInitialCovariance(makeDenseSpd());

  // 静止初始化得到重力对齐的加速度; 再用一个 IMU 样本把 omega 推离零,
  // 让 F00 不是单位阵, 五个非零块都能被覆盖.
  ImuInitEstimate estimate;
  estimate.static_count = 100;
  estimate.mean_acc = V3D(0.0, 0.0, 9.81);
  ImuParams imu_params;
  eskf.initState(estimate, imu_params);
  eskf.setStartTime(0.0);

  ImuSample sample;
  sample.timestamp = 0.005;
  sample.measurement.angular_velocity = V3D(0.1, -0.05, 0.2);
  sample.measurement.linear_acceleration = V3D(0.03, -0.02, 1.0);
  ASSERT_TRUE(eskf.imuUpdate(sample, imu_params));

  const State state = eskf.get_x();
  const M24 prior = eskf.getP();
  const double dt = 0.005;
  ASSERT_TRUE(eskf.stateTransform(0.010, state.omega, state.acc));

  const M24 ref = denseReference(state, prior, dt, q_bg, q_ba, q_g, q_a);
  const M24 got = eskf.getP_predict();
  const double scale = ref.cwiseAbs().maxCoeff();
  ASSERT_GT(scale, 1e-6);
  EXPECT_LT((got - ref).cwiseAbs().maxCoeff() / scale, 1e-12);
}

TEST(EskfLidarUpdate, SequentialMatchesBatch) {
  const double sigma2 = 1e-2;
  const M24 P0 = makeDenseSpd();

  for (int m : {1, 3, 10, 50}) {
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(m, 6);
    Eigen::VectorXd r(m);
    for (int i = 0; i < m; ++i) {
      for (int j = 0; j < 6; ++j) {
        B(i, j) = std::sin(0.3 * i + 1.7 * j + 0.1 * j * j);
      }
      r(i) = 0.1 * std::cos(0.5 * i);
    }

    M24 P_seq = P0;
    Eigen::Matrix<double, 24, 1> dx_seq = Eigen::Matrix<double, 24, 1>::Zero();
    ManifoldEskf::applySequentialLidarUpdate(B, r, sigma2, P_seq, dx_seq);

    // 批量参考: S = H P H^T + R, K = P H^T S^{-1}, Joseph 形式.
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, 24);
    H.leftCols(6) = B;
    const Eigen::MatrixXd I24 = Eigen::MatrixXd::Identity(24, 24);
    const Eigen::MatrixXd R = sigma2 * Eigen::MatrixXd::Identity(m, m);
    const Eigen::MatrixXd S = H * P0 * H.transpose() + R;
    const Eigen::MatrixXd K = P0 * H.transpose() * S.inverse();
    const Eigen::Matrix<double, 24, 1> dx_ref = K * r;
    const Eigen::MatrixXd A = I24 - K * H;
    const M24 P_ref = A * P0 * A.transpose() + K * R * K.transpose();

    const double dP = (P_seq - P_ref).cwiseAbs().maxCoeff()
                      / P_ref.cwiseAbs().maxCoeff();
    const double ddx = (dx_seq - dx_ref).cwiseAbs().maxCoeff()
                       / dx_ref.cwiseAbs().maxCoeff();
    EXPECT_LT(dP, 1e-12) << "m=" << m;
    EXPECT_LT(ddx, 1e-10) << "m=" << m;
  }
}
