#include "pb_omni_pid_pursuit_controller/core/pid.hpp"
#include "gtest/gtest.h"

using pb_omni_pid_pursuit_controller::PID;

// 纯比例响应：P_out = kp * error
TEST(PidTest, ProportionalResponse) {
  PID pid(1.0, 10.0, -10.0, 2.0, 0.0, 0.0);

  double output = pid.calculate(1.0, 0.5);  // error = 0.5 → P = 2.0 * 0.5 = 1.0

  EXPECT_DOUBLE_EQ(output, 1.0);
}

// 积分项随时间累积
TEST(PidTest, IntegralAccumulation) {
  PID pid(0.1, 10.0, -10.0, 0.0, 0.0, 1.0);  // only ki=1

  double prev = 0;
  // Each call with same error accumulates integral
  for (int i = 0; i < 5; i++) {
    double output = pid.calculate(1.0, 0.0);  // error = 1
    // integral increases by 0.1 each frame, ki=1 → i_out = integral
    EXPECT_GT(output, prev) << "Frame " << i << ": output should increase";
    prev = output;
  }
}

// setSumError 重置积分项
TEST(PidTest, SetSumError_ResetsIntegral) {
  PID pid(0.1, 10.0, -10.0, 0.0, 0.0, 1.0);

  // accumulate some integral
  pid.calculate(1.0, 0.0);
  pid.calculate(1.0, 0.0);

  // reset integral
  pid.setSumError(0.0);

  // next output should only have the fresh error-based integral
  double output = pid.calculate(1.0, 0.0);
  // integral = 0 + 1*0.1 = 0.1, ki=1 → i_out = 0.1
  EXPECT_NEAR(output, 0.1, 1e-9);
}

// 微分项：误差突变时产生尖峰
TEST(PidTest, DerivativeResponse) {
  PID pid(0.05, 10.0, -10.0, 0.0, 0.5, 0.0);  // only kd=0.5, dt=0.05

  // steady state (no error change)
  pid.calculate(0.0, 0.0);  // error=0, pre_error=0

  // step change in error: pre_error=0, new error=1, de/dt = 1/0.05 = 20
  double output = pid.calculate(1.0, 0.0);  // D_out = 0.5 * 20 = 10

  EXPECT_GT(output, 0.0) << "Step error should produce positive derivative";
}

// 输出被 clamp 到 [min, max]
TEST(PidTest, OutputClampedToRange) {
  PID pid(1.0, 2.0, -2.0, 100.0, 0.0, 0.0);  // kp=100, limit ±2

  // error = 1 → P = 100, but clamped to max=2
  double output = pid.calculate(1.0, 0.0);
  EXPECT_DOUBLE_EQ(output, 2.0);

  // set_point below pv → large negative error
  output = pid.calculate(-1.0, 1.0);  // error = -2 → P = -200, clamped to -2
  EXPECT_DOUBLE_EQ(output, -2.0);
}

// 零增益时输出为零
TEST(PidTest, ZeroGains_ZeroOutput) {
  PID pid(1.0, 10.0, -10.0, 0.0, 0.0, 0.0);

  double output = pid.calculate(5.0, 0.0);  // all gains zero

  EXPECT_DOUBLE_EQ(output, 0.0);
}
