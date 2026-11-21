/**
 * @file test_imu_calibration.cpp
 * @brief 六面标定参数在 IMU 入口处生效的单元测试.
 *
 * 覆盖两件事:
 * 1. 启用修正后, 静态初始化拿到的均值已经扣掉零偏并按矩阵修正;
 * 2. 关闭修正时, 原始测量保持不变.
 */

#include <cmath>
#include <deque>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "point_lio/core/Imu.h"
#include "point_lio/utilities/common_lib.h"

namespace {

constexpr double kGravity = 9.80665;

ImuParams makeParams() {
  ImuParams params;
  params.acc_norm = 1.0;
  params.processor.gravity_magnitude = kGravity;
  params.acc_bias = Eigen::Vector3d(0.15, -0.21, 0.09);
  params.gyro_bias = Eigen::Vector3d(0.01, -0.02, 0.005);
  params.acc_correction << 1.001, 0.002, -0.001,
                           0.003, 0.999, 0.002,
                          -0.002, 0.001, 1.000;
  return params;
}

/// 真实比力, 与 z 轴成 11 度, 贴近实车安装姿态; 模长恰好等于重力.
Eigen::Vector3d trueAcceleration() {
  constexpr double kTilt = 11.0 * M_PI / 180.0;
  return Eigen::Vector3d(0.0, std::sin(kTilt), -std::cos(kTilt)) * kGravity;
}

ImuMeasurement makeMeasurement(const ImuParams& params) {
  const Eigen::Vector3d true_acc = trueAcceleration();
  const double acc_scale =
      params.processor.gravity_magnitude / params.acc_norm;
  // 测量模型: a_meas = M a_true + b, 其中 M = correction^-1.
  const Eigen::Matrix3d response = params.acc_correction.inverse();
  const Eigen::Vector3d measured = response * true_acc + params.acc_bias;
  ImuMeasurement measurement;
  measurement.linear_acceleration = measured / acc_scale;  // 原始单位 g
  measurement.angular_velocity = params.gyro_bias;         // 静止
  return measurement;
}

}  // namespace

TEST(ImuCalibration, RemovesBiasAndScale) {
  ImuParams params = makeParams();
  params.calib_enable = true;

  Imu imu;
  imu.configure(params);
  const ImuMeasurement measurement = makeMeasurement(params);
  for (int i = 0; i < 150; ++i) {
    imu.append(measurement, i * 0.005);
  }

  MeasureGroup meas;
  imu.collectUntil(1e9, meas.imu_measure);
  ASSERT_EQ(meas.imu_measure.size(), 150U);

  const auto estimate = imu.process(meas);
  ASSERT_TRUE(estimate.has_value());

  const Eigen::Vector3d true_acc = trueAcceleration();
  EXPECT_NEAR(estimate->mean_gyr.norm(), 0.0, 1e-9);
  EXPECT_NEAR(
      (estimate->mean_acc.normalized() - true_acc.normalized()).norm(), 0.0,
      1e-9);
  EXPECT_NEAR(estimate->mean_acc.norm(), 1.0, 1e-9);
}

TEST(ImuCalibration, DisabledKeepsRawMeasurement) {
  ImuParams params = makeParams();
  params.calib_enable = false;

  Imu imu;
  imu.configure(params);
  const ImuMeasurement measurement = makeMeasurement(params);
  for (int i = 0; i < 150; ++i) {
    imu.append(measurement, i * 0.005);
  }

  MeasureGroup meas;
  imu.collectUntil(1e9, meas.imu_measure);
  const auto estimate = imu.process(meas);
  ASSERT_TRUE(estimate.has_value());

  EXPECT_NEAR((estimate->mean_gyr - params.gyro_bias).norm(), 0.0, 1e-9);
  EXPECT_NEAR(
      (estimate->mean_acc.normalized()
       - measurement.linear_acceleration.normalized())
          .norm(),
      0.0, 1e-9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
