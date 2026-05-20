// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/node/omni_pid_pursuit_controller_node.hpp"
#include "pb_omni_pid_pursuit_controller/core/visualise.hpp"
#include "pb_omni_pid_pursuit_controller/core/geometry_utils.hpp"
#include "pb_omni_pid_pursuit_controller/core/pathhandler.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include <algorithm>
#include <cmath>

using nav2_util::declare_parameter_if_not_declared;

namespace pb_omni_pid_pursuit_controller {

  void OmniPidPursuitControllerNode::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    node_ = parent;
    if (!node) {
      throw nav2_core::PlannerException("Unable to lock node!");
    }

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    double control_frequency{20.0};

    declare_parameter_if_not_declared(
        node, plugin_name_ + ".translation_kp",
        rclcpp::ParameterValue(config_.translation_kp));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".translation_ki",
        rclcpp::ParameterValue(config_.translation_ki));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".translation_kd",
        rclcpp::ParameterValue(config_.translation_kd));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".enable_rotation",
        rclcpp::ParameterValue(config_.enable_rotation));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".rotation_kp",
        rclcpp::ParameterValue(config_.rotation_kp));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".rotation_ki",
        rclcpp::ParameterValue(config_.rotation_ki));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".rotation_kd",
        rclcpp::ParameterValue(config_.rotation_kd));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance",
        rclcpp::ParameterValue(config_.transform_tolerance));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".min_max_sum_error",
        rclcpp::ParameterValue(config_.min_max_sum_error));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_dist",
        rclcpp::ParameterValue(config_.lookahead_dist));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
        rclcpp::ParameterValue(config_.use_velocity_scaled_lookahead_dist));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".min_lookahead_dist",
        rclcpp::ParameterValue(config_.min_lookahead_dist));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_lookahead_dist",
        rclcpp::ParameterValue(config_.max_lookahead_dist));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_time",
        rclcpp::ParameterValue(config_.lookahead_time));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".use_interpolation",
        rclcpp::ParameterValue(config_.use_interpolation));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".use_rotate_to_heading",
        rclcpp::ParameterValue(config_.use_rotate_to_heading));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".use_rotate_to_heading_threshold",
        rclcpp::ParameterValue(config_.use_rotate_to_heading_threshold));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".min_approach_linear_velocity",
        rclcpp::ParameterValue(config_.min_approach_linear_velocity));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".approach_velocity_scaling_dist",
        rclcpp::ParameterValue(config_.approach_velocity_scaling_dist));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".v_linear_min",
        rclcpp::ParameterValue(config_.v_linear_min));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".v_linear_max",
        rclcpp::ParameterValue(config_.v_linear_max));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".v_angular_min",
        rclcpp::ParameterValue(config_.v_angular_min));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".v_angular_max",
        rclcpp::ParameterValue(config_.v_angular_max));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_robot_pose_search_dist",
        rclcpp::ParameterValue(config_.max_robot_pose_search_dist));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".curvature_min",
        rclcpp::ParameterValue(config_.curvature_min));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".curvature_max",
        rclcpp::ParameterValue(config_.curvature_max));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".reduction_ratio_at_high_curvature",
        rclcpp::ParameterValue(config_.reduction_ratio_at_high_curvature));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".curvature_forward_dist",
        rclcpp::ParameterValue(config_.curvature_forward_dist));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".curvature_backward_dist",
        rclcpp::ParameterValue(config_.curvature_backward_dist));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_velocity_scaling_factor_rate",
        rclcpp::ParameterValue(config_.max_velocity_scaling_factor_rate));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".collision_sample_points",
        rclcpp::ParameterValue(config_.collision_sample_points));

    node->get_parameter(plugin_name_ + ".translation_kp",
                        config_.translation_kp);
    node->get_parameter(plugin_name_ + ".translation_ki",
                        config_.translation_ki);
    node->get_parameter(plugin_name_ + ".translation_kd",
                        config_.translation_kd);
    node->get_parameter(plugin_name_ + ".enable_rotation",
                        config_.enable_rotation);
    node->get_parameter(plugin_name_ + ".rotation_kp", config_.rotation_kp);
    node->get_parameter(plugin_name_ + ".rotation_ki", config_.rotation_ki);
    node->get_parameter(plugin_name_ + ".rotation_kd", config_.rotation_kd);
    node->get_parameter(plugin_name_ + ".transform_tolerance",
                        config_.transform_tolerance);
    node->get_parameter(plugin_name_ + ".min_max_sum_error",
                        config_.min_max_sum_error);
    node->get_parameter(plugin_name_ + ".lookahead_dist",
                        config_.lookahead_dist);
    node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist",
                        config_.use_velocity_scaled_lookahead_dist);
    node->get_parameter(plugin_name_ + ".min_lookahead_dist",
                        config_.min_lookahead_dist);
    node->get_parameter(plugin_name_ + ".max_lookahead_dist",
                        config_.max_lookahead_dist);
    node->get_parameter(plugin_name_ + ".lookahead_time",
                        config_.lookahead_time);
    node->get_parameter(plugin_name_ + ".use_interpolation",
                        config_.use_interpolation);
    node->get_parameter(plugin_name_ + ".use_rotate_to_heading",
                        config_.use_rotate_to_heading);
    node->get_parameter(plugin_name_ + ".use_rotate_to_heading_threshold",
                        config_.use_rotate_to_heading_threshold);
    node->get_parameter(plugin_name_ + ".min_approach_linear_velocity",
                        config_.min_approach_linear_velocity);
    node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist",
                        config_.approach_velocity_scaling_dist);
    if (config_.approach_velocity_scaling_dist
        > costmap_->getSizeInMetersX() / 2.0) {
      RCLCPP_WARN(logger_,
                  "approach_velocity_scaling_dist is larger than forward "
                  "costmap extent, leading to permanent slowdown");
    }
    node->get_parameter(plugin_name_ + ".v_linear_max", config_.v_linear_max);
    node->get_parameter(plugin_name_ + ".v_linear_min", config_.v_linear_min);
    node->get_parameter(plugin_name_ + ".v_angular_max", config_.v_angular_max);
    node->get_parameter(plugin_name_ + ".v_angular_min", config_.v_angular_min);
    node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist",
                        config_.max_robot_pose_search_dist);
    node->get_parameter(plugin_name_ + ".curvature_min", config_.curvature_min);
    node->get_parameter(plugin_name_ + ".curvature_max", config_.curvature_max);
    node->get_parameter(plugin_name_ + ".reduction_ratio_at_high_curvature",
                        config_.reduction_ratio_at_high_curvature);
    node->get_parameter(plugin_name_ + ".curvature_forward_dist",
                        config_.curvature_forward_dist);
    node->get_parameter(plugin_name_ + ".curvature_backward_dist",
                        config_.curvature_backward_dist);
    node->get_parameter(plugin_name_ + ".max_velocity_scaling_factor_rate",
                        config_.max_velocity_scaling_factor_rate);
    node->get_parameter(plugin_name_ + ".collision_sample_points",
                        config_.collision_sample_points);

    node->get_parameter("controller_frequency", control_frequency);

    config_.control_duration = 1.0 / control_frequency;

    local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan",
                                                                  1);
    carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
        "lookahead_point", 1);
    curvature_points_pub_ =
        node_.lock()->create_publisher<visualization_msgs::msg::MarkerArray>(
            "curvature_points_marker_array", rclcpp::QoS(10));

    chassis_mode_sub_ = node->create_subscription<std_msgs::msg::UInt8>(
        "chassis_mode", 1, [this](std_msgs::msg::UInt8::SharedPtr msg) {
          chassisModeCallback(msg);
        });

    path_handler_ = std::make_unique<PathHandler>(tf_, costmap_ros_,
                                                  local_path_pub_,
                                                  config_.transform_tolerance);
    config_.max_robot_pose_search_dist = path_handler_->getCostmapMaxExtent();

    move_pid_ = std::make_shared<PID>(
        config_.control_duration, config_.v_linear_max, config_.v_linear_min,
        config_.translation_kp, config_.translation_kd, config_.translation_ki,
        config_.min_max_sum_error);
    heading_pid_ = std::make_shared<PID>(
        config_.control_duration, config_.v_angular_max, config_.v_angular_min,
        config_.rotation_kp, config_.rotation_kd, config_.rotation_ki,
        config_.min_max_sum_error);
  }

  void OmniPidPursuitControllerNode::cleanup() {
    RCLCPP_INFO(logger_,
                "Cleaning up controller: %s of type"
                " pb_omni_pid_pursuit_controller::OmniPidPursuitControllerNode",
                plugin_name_.c_str());
    local_path_pub_.reset();
    carrot_pub_.reset();
    curvature_points_pub_.reset();
  }

  void OmniPidPursuitControllerNode::activate() {
    RCLCPP_INFO(logger_,
                "Activating controller: %s of type "
                "pb_omni_pid_pursuit_controller::OmniPidPursuitControllerNode",
                plugin_name_.c_str());
    local_path_pub_->on_activate();
    carrot_pub_->on_activate();
    curvature_points_pub_->on_activate();
  }

  void OmniPidPursuitControllerNode::deactivate() {
    RCLCPP_INFO(logger_,
                "Deactivating controller: %s of type "
                "pb_omni_pid_pursuit_controller::OmniPidPursuitControllerNode",
                plugin_name_.c_str());
    local_path_pub_->on_deactivate();
    carrot_pub_->on_deactivate();
    curvature_points_pub_->on_deactivate();
  }

  geometry_msgs::msg::TwistStamped
  OmniPidPursuitControllerNode::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose,
      const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* /*goal_checker*/) {
    std::lock_guard<std::mutex> lock_reinit(mutex_);

    auto* costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap->getMutex()));

    auto transformed_plan = transformPath(pose);
    double linear_distance{};
    double theta_distance{};
    double path_yaw{};
    double angle_to_goal{};

    auto carrot_pose = computeLookahead(velocity, transformed_plan,
                                        linear_distance, theta_distance,
                                        path_yaw, angle_to_goal);
    carrot_pub_->publish(visualization_helper::createCarrotMsg(carrot_pose));

    double lin_vel{};
    double angular_vel{};
    computeVelocity(linear_distance, angle_to_goal, lin_vel, angular_vel);
    applyVelocityLimits(transformed_plan, carrot_pose, lin_vel);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    if (!checkCollision(transformed_plan)) {
      cmd_vel = assembleCmdVel(pose, lin_vel, angular_vel, theta_distance,
                               path_yaw);
    } else {
      throw nav2_core::PlannerException(
          "Collision detected in the trajectory. Stopping the robot!");
    }

    return cmd_vel;
  }

  geometry_msgs::msg::TwistStamped OmniPidPursuitControllerNode::assembleCmdVel(
      const geometry_msgs::msg::PoseStamped& pose, double lin_vel,
      double angular_vel, double theta_dist, double path_yaw) const {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    if (config_.enable_rotation) {
      cmd_vel.twist.linear.x = lin_vel * std::cos(path_yaw);
      cmd_vel.twist.linear.y = lin_vel * std::sin(path_yaw);
    } else {
      cmd_vel.twist.linear.x = lin_vel * std::cos(theta_dist);
      cmd_vel.twist.linear.y = lin_vel * std::sin(theta_dist);
    }
    cmd_vel.twist.angular.z = angular_vel;
    return cmd_vel;
  }

  void OmniPidPursuitControllerNode::setPlan(const nav_msgs::msg::Path& path) {
    global_plan_ = path;
  }

  void OmniPidPursuitControllerNode::setSpeedLimit(
      const double& /*speed_limit*/, const bool& /*percentage*/) {
    RCLCPP_WARN(logger_, "Speed limit is not implemented in this controller.");
  }

  // ── pipeline steps ──

  nav_msgs::msg::Path OmniPidPursuitControllerNode::transformPath(
      const geometry_msgs::msg::PoseStamped& pose) {
    return path_handler_->transformGlobalPlan(pose, global_plan_);
  }

  geometry_msgs::msg::PoseStamped
  OmniPidPursuitControllerNode::computeLookahead(
      const geometry_msgs::msg::Twist& velocity,
      const nav_msgs::msg::Path& transformed_plan, double& lin_dist,
      double& theta_dist, double& path_yaw, double& angle_to_goal) {
    double lookahead_dist = getLookAheadDistance(velocity);
    auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);

    lin_dist = std::hypot(carrot_pose.pose.position.x,
                          carrot_pose.pose.position.y);
    theta_dist = std::atan2(carrot_pose.pose.position.y,
                            carrot_pose.pose.position.x);

    if (transformed_plan.poses.size() > 1) {
      const auto& p0 = transformed_plan.poses[0].pose.position;
      const auto& p1 = transformed_plan.poses[1].pose.position;
      path_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    } else {
      path_yaw = theta_dist;
    }

    if (config_.use_rotate_to_heading) {
      angle_to_goal = tf2::getYaw(
          transformed_plan.poses.back().pose.orientation);
      if (std::fabs(angle_to_goal) > config_.use_rotate_to_heading_threshold) {
        lin_dist = 0;
      }
    } else {
      angle_to_goal = path_yaw;
    }

    return carrot_pose;
  }

  void OmniPidPursuitControllerNode::computeVelocity(double linear_distance,
                                                     double angle_to_goal,
                                                     double& linear_vel,
                                                     double& angular_vel) {
    linear_vel = move_pid_->calculate(linear_distance, 0);
    angular_vel = config_.enable_rotation
                    ? heading_pid_->calculate(angle_to_goal, 0)
                    : 0.0;
  }

  void OmniPidPursuitControllerNode::applyVelocityLimits(
      const nav_msgs::msg::Path& transformed_plan,
      const geometry_msgs::msg::PoseStamped& carrot_pose, double& lin_vel) {
    applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);
    applyApproachVelocityScaling(transformed_plan, lin_vel);
  }

  // ── helpers ──

  double OmniPidPursuitControllerNode::getLookAheadDistance(
      const geometry_msgs::msg::Twist& speed) const {
    double lookahead_dist = config_.lookahead_dist;
    if (config_.use_velocity_scaled_lookahead_dist) {
      lookahead_dist = std::hypot(speed.linear.x, speed.linear.y)
                     * config_.lookahead_time;
      lookahead_dist = std::clamp(lookahead_dist, config_.min_lookahead_dist,
                                  config_.max_lookahead_dist);
    }
    return lookahead_dist;
  }

  geometry_msgs::msg::PoseStamped
  OmniPidPursuitControllerNode::getLookAheadPoint(
      const double& lookahead_dist,
      const nav_msgs::msg::Path& transformed_plan) const {
    auto goal_pose_it = std::find_if(
        transformed_plan.poses.begin(), transformed_plan.poses.end(),
        [&](const auto& ps) {
          return std::hypot(ps.pose.position.x, ps.pose.position.y)
              >= lookahead_dist;
        });

    if (goal_pose_it == transformed_plan.poses.end()) {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    } else if (config_.use_interpolation
               && goal_pose_it != transformed_plan.poses.begin()) {
      auto prev_pose_it = std::prev(goal_pose_it);
      auto point = geometry_utils::circleSegmentIntersection(
          prev_pose_it->pose.position, goal_pose_it->pose.position,
          lookahead_dist);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = prev_pose_it->header.frame_id;
      pose.header.stamp = goal_pose_it->header.stamp;
      pose.pose.position = point;
      return pose;
    }

    return *goal_pose_it;
  }

  double OmniPidPursuitControllerNode::approachVelocityScalingFactor(
      const nav_msgs::msg::Path& transformed_path) const {
    double remaining_distance =
        nav2_util::geometry_utils::calculate_path_length(transformed_path);
    if (remaining_distance < config_.approach_velocity_scaling_dist) {
      const auto& last = transformed_path.poses.back();
      double distance_to_last_pose = std::hypot(last.pose.position.x,
                                                last.pose.position.y);
      return distance_to_last_pose / config_.approach_velocity_scaling_dist;
    }
    return 1.0;
  }

  void OmniPidPursuitControllerNode::applyApproachVelocityScaling(
      const nav_msgs::msg::Path& path, double& linear_vel) const {
    double velocity_scaling = approachVelocityScalingFactor(path);
    double scaled_vel = linear_vel * velocity_scaling;
    linear_vel = std::max(scaled_vel, config_.min_approach_linear_velocity);
  }

  void OmniPidPursuitControllerNode::applyCurvatureLimitation(
      const nav_msgs::msg::Path& path,
      const geometry_msgs::msg::PoseStamped& lookahead_pose,
      double& linear_vel) {
    double curvature = calculateCurvature(path, lookahead_pose,
                                          config_.curvature_forward_dist,
                                          config_.curvature_backward_dist);

    double scaled_linear_vel = linear_vel;
    if (curvature > config_.curvature_min) {
      double reduction_ratio{1.0};
      if (curvature > config_.curvature_max) {
        reduction_ratio = config_.reduction_ratio_at_high_curvature;
      } else {
        reduction_ratio = 1.0
                        - ((curvature - config_.curvature_min)
                           / (config_.curvature_max - config_.curvature_min)
                           * (1.0 - config_.reduction_ratio_at_high_curvature));
      }

      double target_scaled_vel = linear_vel * reduction_ratio;
      double rate_limit = config_.max_velocity_scaling_factor_rate
                        * config_.control_duration;
      scaled_linear_vel = state_.last_velocity_scaling_factor
                        + std::clamp(target_scaled_vel
                                         - state_.last_velocity_scaling_factor,
                                     -rate_limit, rate_limit);
    }
    scaled_linear_vel = std::max(scaled_linear_vel,
                                 2.0 * config_.min_approach_linear_velocity);

    linear_vel = std::min(linear_vel, scaled_linear_vel);
    state_.last_velocity_scaling_factor = scaled_linear_vel;
  }

  double OmniPidPursuitControllerNode::calculateCurvature(
      const nav_msgs::msg::Path& path,
      const geometry_msgs::msg::PoseStamped& lookahead_pose,
      double forward_dist, double backward_dist) const {
    std::vector<double> cumulative_distances =
        geometry_utils::calculateCumulativeDistances(path);

    geometry_msgs::msg::PoseStamped robot_base_frame_pose;
    robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
    double lookahead_pose_cumulative_distance =
        nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose,
                                                      lookahead_pose);

    auto backward_pose = geometry_utils::findPoseAtDistance(
        path, cumulative_distances,
        lookahead_pose_cumulative_distance - backward_dist);

    auto forward_pose = geometry_utils::findPoseAtDistance(
        path, cumulative_distances,
        lookahead_pose_cumulative_distance + forward_dist);

    double curvature_radius = geometry_utils::calculateCurvatureRadius(
        backward_pose.pose.position, lookahead_pose.pose.position,
        forward_pose.pose.position);
    double curvature = 1.0 / curvature_radius;
    auto markers = visualization_helper::visualizeCurvaturePoints(
        backward_pose, forward_pose);
    curvature_points_pub_->publish(markers);
    return curvature;
  }

  bool OmniPidPursuitControllerNode::checkCollision(
      const nav_msgs::msg::Path& transformed_plan) {
    nav_msgs::msg::Path costmap_frame_local_plan;
    int sample_points = static_cast<int>(config_.collision_sample_points);
    auto plan_size = static_cast<int>(transformed_plan.poses.size());
    for (int i = 0; i < sample_points; ++i) {
      int index = std::min((i * plan_size) / sample_points, plan_size - 1);
      auto map_pose = path_handler_->transformPose(
          costmap_ros_->getGlobalFrameID(),
          transformed_plan.poses[static_cast<size_t>(index)]);
      if (map_pose) {
        costmap_frame_local_plan.poses.push_back(*map_pose);
      }
    }
    return isCollisionDetected(costmap_frame_local_plan);
  }

  bool OmniPidPursuitControllerNode::isCollisionDetected(
      const nav_msgs::msg::Path& path) {
    auto* costmap = costmap_ros_->getCostmap();
    for (const auto& pose_stamped : path.poses) {
      const auto& pose = pose_stamped.pose;
      unsigned int mx{};
      unsigned int my{};
      if (costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
        if (costmap->getCost(mx, my)
            >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          return true;
        }
      }
    }
    return false;
  }

  void OmniPidPursuitControllerNode::chassisModeCallback(
      const std_msgs::msg::UInt8::SharedPtr msg) {
    if (msg->data == static_cast<uint8_t>(ChassisMode::CHASSIS_FOLLOWED)) {
      config_.enable_rotation = true;
    } else if (msg->data == static_cast<uint8_t>(ChassisMode::LITTLE_TES)) {
      config_.enable_rotation = false;
    }
  }

}  // namespace pb_omni_pid_pursuit_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    pb_omni_pid_pursuit_controller::OmniPidPursuitControllerNode,
    nav2_core::Controller)
