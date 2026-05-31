#include "pb_omni_pid_pursuit_controller/core/types.hpp"
#include "gtest/gtest.h"

namespace pb_omni_pid_pursuit_controller {
namespace {

TEST(ChassisModeTest, EnumValues) {
  EXPECT_EQ(static_cast<uint8_t>(ChassisMode::CHASSIS_FOLLOWED), 1);
  EXPECT_EQ(static_cast<uint8_t>(ChassisMode::LITTLE_TES), 2);
  EXPECT_EQ(static_cast<uint8_t>(ChassisMode::GO_HOME), 3);
}

TEST(ControllerConfigTest, DefaultValues) {
  ControllerConfig cfg;
  EXPECT_DOUBLE_EQ(cfg.translation_kp, 3.0);
  EXPECT_DOUBLE_EQ(cfg.translation_ki, 0.1);
  EXPECT_DOUBLE_EQ(cfg.translation_kd, 0.3);
  EXPECT_TRUE(cfg.enable_rotation);
  EXPECT_DOUBLE_EQ(cfg.rotation_kp, 3.0);
  EXPECT_DOUBLE_EQ(cfg.rotation_ki, 0.1);
  EXPECT_DOUBLE_EQ(cfg.rotation_kd, 0.3);
  EXPECT_DOUBLE_EQ(cfg.min_max_sum_error, 1.0);
  EXPECT_DOUBLE_EQ(cfg.control_duration, 0.05);
  EXPECT_TRUE(cfg.use_interpolation);
  EXPECT_DOUBLE_EQ(cfg.lookahead_dist, 0.3);
  EXPECT_TRUE(cfg.use_velocity_scaled_lookahead_dist);
  EXPECT_DOUBLE_EQ(cfg.min_lookahead_dist, 0.2);
  EXPECT_DOUBLE_EQ(cfg.max_lookahead_dist, 1.0);
  EXPECT_DOUBLE_EQ(cfg.lookahead_time, 1.0);
  EXPECT_TRUE(cfg.use_rotate_to_heading);
  EXPECT_DOUBLE_EQ(cfg.use_rotate_to_heading_threshold, 0.1);
  EXPECT_DOUBLE_EQ(cfg.v_linear_min, -3.0);
  EXPECT_DOUBLE_EQ(cfg.v_linear_max, 3.0);
  EXPECT_DOUBLE_EQ(cfg.v_angular_min, -3.0);
  EXPECT_DOUBLE_EQ(cfg.v_angular_max, 3.0);
  EXPECT_DOUBLE_EQ(cfg.min_approach_linear_velocity, 0.05);
  EXPECT_DOUBLE_EQ(cfg.approach_velocity_scaling_dist, 0.6);
  EXPECT_DOUBLE_EQ(cfg.curvature_min, 0.4);
  EXPECT_DOUBLE_EQ(cfg.curvature_max, 0.7);
  EXPECT_DOUBLE_EQ(cfg.reduction_ratio_at_high_curvature, 0.5);
  EXPECT_DOUBLE_EQ(cfg.curvature_forward_dist, 0.7);
  EXPECT_DOUBLE_EQ(cfg.curvature_backward_dist, 0.3);
  EXPECT_DOUBLE_EQ(cfg.max_velocity_scaling_factor_rate, 0.9);
  EXPECT_DOUBLE_EQ(cfg.transform_tolerance, 0.5);
}

TEST(ControllerStateTest, DefaultState) {
  ControllerState state;
  EXPECT_DOUBLE_EQ(state.last_velocity_scaling_factor, 0.0);
}

}  // namespace
}  // namespace pb_omni_pid_pursuit_controller
