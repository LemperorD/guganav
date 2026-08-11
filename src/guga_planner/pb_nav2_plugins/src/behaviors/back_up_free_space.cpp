// Copyright 2024 Polaris Xia
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

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace pb_nav2_behaviors
{

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "max_escape_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "escape_clearance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("max_escape_distance", max_escape_distance_);
  node->get_parameter("escape_clearance", escape_clearance_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("visualize", visualize_);

  if (max_radius_ <= 0.0 || max_escape_distance_ <= 0.0 || escape_clearance_ < 0.0) {
    throw std::invalid_argument(
      "max_radius and max_escape_distance must be positive; escape_clearance must be "
      "non-negative");
  }

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
    return nav2_behaviors::Status::FAILED;
  }

  // get costmap
  auto costmap = result.get()->map;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  // get current pose
  geometry_msgs::msg::Pose2D pose;
  pose.x = initial_pose_.pose.position.x;
  pose.y = initial_pose_.pose.position.y;
  pose.theta = tf2::getYaw(initial_pose_.pose.orientation);

  const float requested_distance =
    std::min(static_cast<float>(std::fabs(command->target.x)), static_cast<float>(max_radius_));
  const auto escape =
    findBestDirection(costmap, pose, -M_PI, M_PI, requested_distance, M_PI / 32.0);

  if (!escape) {
    stopRobot();
    RCLCPP_ERROR(
      logger_, "No recovery direction reaches footprint-safe space within %.2f meters",
      max_escape_distance_);
    return nav2_behaviors::Status::FAILED;
  }

  // Calculate move command
  // findBestDirection returns an angle in global_frame_. DriveOnHeading expects
  // velocity components in robot_base_frame_, so rotate the vector into the
  // current robot frame before publishing it.
  const double speed = std::fabs(command->speed);
  const double global_vx = std::cos(escape->angle) * speed;
  const double global_vy = std::sin(escape->angle) * speed;
  const double robot_yaw = tf2::getYaw(initial_pose_.pose.orientation);
  twist_x_ = global_vx * std::cos(robot_yaw) + global_vy * std::sin(robot_yaw);
  twist_y_ = -global_vx * std::sin(robot_yaw) + global_vy * std::cos(robot_yaw);
  command_x_ = escape->command_distance;
  escaping_initial_high_cost_ = escape->started_in_high_cost;
  collision_check_start_distance_ = escape->command_distance;
  command_time_allowance_ = command->time_allowance;

  end_time_ = clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }
  RCLCPP_WARN(
    logger_, "moving %.3f meters towards free space at angle %.3f (high-cost exit %.3f)",
    command_x_, escape->angle, escape->escape_distance);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the "
      "DriveOnHeading goal - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  float diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  float diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  float distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = twist_y_;
  cmd_vel->linear.x = twist_x_;

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isOmniCollisionFree(distance, cmd_vel.get(), pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

bool BackUpFreeSpace::isOmniCollisionFree(
  const double & distance, geometry_msgs::msg::Twist * cmd_vel, geometry_msgs::msg::Pose2D & pose2d)
{
  const double remaining_distance = std::max(0.0, std::fabs(command_x_) - distance);
  const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * this->simulate_ahead_time_);
  const geometry_msgs::msg::Pose2D initial_pose = pose2d;
  const double heading = initial_pose.theta;
  const double world_vx =
    cmd_vel->linear.x * std::cos(heading) - cmd_vel->linear.y * std::sin(heading);
  const double world_vy =
    cmd_vel->linear.x * std::sin(heading) + cmd_vel->linear.y * std::cos(heading);
  const double speed = std::hypot(world_vx, world_vy);
  bool fetch_data = true;

  if (speed <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  for (int cycle = 1; cycle <= max_cycle_count; ++cycle) {
    const double elapsed = static_cast<double>(cycle) / this->cycle_frequency_;
    const double traveled = std::min(speed * elapsed, remaining_distance);
    const double sample_time = traveled / speed;
    pose2d.x = initial_pose.x + world_vx * sample_time;
    pose2d.y = initial_pose.y + world_vy * sample_time;
    pose2d.theta = initial_pose.theta + cmd_vel->angular.z * sample_time;

    const double projected_distance = distance + traveled;
    const bool inside_initial_high_cost =
      escaping_initial_high_cost_ && projected_distance + 1e-6 < collision_check_start_distance_;
    if (!inside_initial_high_cost) {
      if (!this->collision_checker_->isCollisionFree(pose2d, fetch_data)) {
        return false;
      }
      fetch_data = false;
    }
    if (traveled >= remaining_distance) {
      break;
    }
  }
  return true;
}

std::optional<BackUpFreeSpace::EscapeDirection> BackUpFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float requested_distance, float angle_increment)
{
  constexpr auto kCollisionCost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  constexpr int kInflationCostTolerance = 5;

  const float resolution = costmap.metadata.resolution;
  const float origin_x = costmap.metadata.origin.position.x;
  const float origin_y = costmap.metadata.origin.position.y;
  const int size_x = static_cast<int>(costmap.metadata.size_x);
  const int size_y = static_cast<int>(costmap.metadata.size_y);
  const size_t expected_size = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);

  if (
    resolution <= 0.0f || requested_distance <= 0.0f || angle_increment <= 0.0f ||
    start_angle >= end_angle || costmap.data.size() < expected_size) {
    return std::nullopt;
  }

  const auto cost_at = [&](float x, float y) -> std::optional<uint8_t> {
    const int i = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int j = static_cast<int>(std::floor((y - origin_y) / resolution));
    if (i < 0 || i >= size_x || j < 0 || j >= size_y) {
      return std::nullopt;
    }
    return costmap.data[static_cast<size_t>(i) + static_cast<size_t>(j) * size_x];
  };

  const auto start_cost = cost_at(pose.x, pose.y);
  if (!start_cost) {
    return std::nullopt;
  }
  const bool starts_in_high_cost = *start_cost >= kCollisionCost;

  struct Candidate
  {
    EscapeDirection direction;
    double accumulated_cost;
    bool valid;
  };
  std::vector<Candidate> candidates;

  for (float angle = start_angle; angle < end_angle; angle += angle_increment) {
    bool escaped = !starts_in_high_cost;
    bool valid = true;
    float escape_distance = 0.0f;
    float command_distance = requested_distance;
    uint8_t previous_cost = *start_cost;
    double accumulated_cost = 0.0;

    for (float r = resolution; r <= requested_distance + resolution * 0.5f; r += resolution) {
      const auto cost = cost_at(pose.x + r * std::cos(angle), pose.y + r * std::sin(angle));
      if (!cost) {
        valid = false;
        break;
      }

      if (!escaped) {
        if (*cost >= kCollisionCost) {
          if (r > max_escape_distance_ + resolution * 0.5f || *cost > previous_cost) {
            valid = false;
            break;
          }
          previous_cost = *cost;
          continue;
        }

        escaped = true;
        escape_distance = r;
        command_distance = escape_distance + static_cast<float>(escape_clearance_);
        if (command_distance > requested_distance + resolution * 0.5f) {
          valid = false;
          break;
        }
      }

      if (*cost >= kCollisionCost) {
        valid = false;
        break;
      }

      // Reject directions that turn back toward inflated obstacles immediately
      // after leaving the initial high-cost component. Small rasterization
      // changes are tolerated on diagonal rays.
      if (
        starts_in_high_cost && r <= command_distance + resolution * 0.5f &&
        static_cast<int>(*cost) > static_cast<int>(previous_cost) + kInflationCostTolerance) {
        valid = false;
        break;
      }

      accumulated_cost += *cost;
      previous_cost = *cost;

      if (starts_in_high_cost && r >= command_distance - resolution * 0.5f) {
        break;
      }
    }

    if (starts_in_high_cost && !escaped) {
      valid = false;
    }

    if (valid && collision_checker_) {
      bool footprint_safe = false;
      for (float target_distance = command_distance;
           target_distance <= requested_distance + resolution * 0.5f;
           target_distance += resolution) {
        const auto target_cost = cost_at(
          pose.x + target_distance * std::cos(angle), pose.y + target_distance * std::sin(angle));
        if (!target_cost || *target_cost >= kCollisionCost) {
          break;
        }
        if (
          starts_in_high_cost && static_cast<int>(*target_cost) >
                                   static_cast<int>(previous_cost) + kInflationCostTolerance) {
          break;
        }

        geometry_msgs::msg::Pose2D target_pose;
        target_pose.x = pose.x + target_distance * std::cos(angle);
        target_pose.y = pose.y + target_distance * std::sin(angle);
        target_pose.theta = pose.theta;
        if (collision_checker_->isCollisionFree(target_pose, true)) {
          command_distance = target_distance;
          footprint_safe = true;
          break;
        }

        accumulated_cost += *target_cost;
        previous_cost = *target_cost;
        if (!starts_in_high_cost) {
          break;
        }
      }
      if (!footprint_safe) {
        valid = false;
      }
    }

    candidates.push_back(
      {{angle, escape_distance, command_distance, starts_in_high_cost}, accumulated_cost, valid});
  }

  float minimum_escape_distance = std::numeric_limits<float>::infinity();
  for (const auto & candidate : candidates) {
    if (candidate.valid) {
      minimum_escape_distance =
        std::min(minimum_escape_distance, candidate.direction.escape_distance);
    }
  }

  if (!std::isfinite(minimum_escape_distance)) {
    if (visualize_) {
      visualize(pose, requested_distance, 0.0f, 0.0f, 0.0f);
    }
    return std::nullopt;
  }

  size_t best_index = 0;
  size_t best_sector_size = 0;
  double best_cost = std::numeric_limits<double>::infinity();
  size_t best_left_steps = 0;
  size_t best_right_steps = 0;

  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates[i];
    if (
      !candidate.valid ||
      candidate.direction.escape_distance > minimum_escape_distance + resolution) {
      continue;
    }

    size_t left_steps = 0;
    size_t right_steps = 0;
    while (left_steps + right_steps + 1 < candidates.size() &&
           candidates[(i + candidates.size() - left_steps - 1) % candidates.size()].valid) {
      ++left_steps;
    }
    while (left_steps + right_steps + 1 < candidates.size() &&
           candidates[(i + right_steps + 1) % candidates.size()].valid) {
      ++right_steps;
    }
    const size_t sector_size = left_steps + right_steps + 1;

    if (
      sector_size > best_sector_size ||
      (sector_size == best_sector_size && candidate.accumulated_cost < best_cost)) {
      best_index = i;
      best_sector_size = sector_size;
      best_cost = candidate.accumulated_cost;
      best_left_steps = left_steps;
      best_right_steps = right_steps;
    }
  }

  const auto & best = candidates[best_index].direction;
  const float first_safe_angle = best.angle - best_left_steps * angle_increment;
  const float last_safe_angle = best.angle + best_right_steps * angle_increment;

  if (visualize_) {
    visualize(pose, requested_distance, first_safe_angle, last_safe_angle, best.angle);
  }

  return best;
}

std::vector<geometry_msgs::msg::Point> BackUpFreeSpace::gatherFreePoints(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius)
{
  std::vector<geometry_msgs::msg::Point> results;
  for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
    for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
      auto idx = i + j * costmap.metadata.size_x;
      auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
      auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
      if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        results.push_back(p);
      }
    }
  }
  return results;
}

void BackUpFreeSpace::visualize(
  geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_safe_angle,
  float best_angle)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = global_frame_;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const float angle_step = 0.05f;
  for (float angle = first_safe_angle; angle <= last_safe_angle; angle += angle_step) {
    const float next_angle = std::min(angle + angle_step, last_safe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = 0.0;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);
    p2.z = 0.0;

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_safe_angle, 2, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(best_angle, 3, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
