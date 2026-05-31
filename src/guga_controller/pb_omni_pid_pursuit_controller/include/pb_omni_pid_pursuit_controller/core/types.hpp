#pragma once

#include <cstdint>

namespace pb_omni_pid_pursuit_controller {

enum class ChassisMode : uint8_t {
  CHASSIS_FOLLOWED = 1,
  LITTLE_TES = 2,
  GO_HOME = 3,
};

struct ControllerConfig {
  double translation_kp{3.0};
  double translation_ki{0.1};
  double translation_kd{0.3};
  bool enable_rotation{true};
  double rotation_kp{3.0};
  double rotation_ki{0.1};
  double rotation_kd{0.3};
  double min_max_sum_error{1.0};
  double control_duration{0.05};
  double max_robot_pose_search_dist{};
  bool use_interpolation{true};
  double lookahead_dist{0.3};
  bool use_velocity_scaled_lookahead_dist{true};
  double min_lookahead_dist{0.2};
  double max_lookahead_dist{1.0};
  double lookahead_time{1.0};
  bool use_rotate_to_heading{true};
  double use_rotate_to_heading_threshold{0.1};
  double v_linear_min{-3.0};
  double v_linear_max{3.0};
  double v_angular_min{-3.0};
  double v_angular_max{3.0};
  double min_approach_linear_velocity{0.05};
  double approach_velocity_scaling_dist{0.6};
  double curvature_min{0.4};
  double curvature_max{0.7};
  double reduction_ratio_at_high_curvature{0.5};
  double curvature_forward_dist{0.7};
  double curvature_backward_dist{0.3};
  double max_velocity_scaling_factor_rate{0.9};
  double transform_tolerance{0.5};
  int64_t collision_sample_points{10};
};

struct ControllerState {
  double last_velocity_scaling_factor{};
};

}  // namespace pb_omni_pid_pursuit_controller
