// Copyright 2026 HaoZhen Liu
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

#include "pb_nav2_plugins/behaviors/escape_obstacle_layer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace pb_nav2_behaviors
{

namespace
{

// 253 = INSCRIBED_INFLATED_OBSTACLE（内切/膨胀硬区，规划器和 MPPI 都当作不可进入）
constexpr uint8_t kInscribedCost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
// 254 = LETHAL_OBSTACLE（真障碍）
constexpr uint8_t kLethalCost = nav2_costmap_2d::LETHAL_OBSTACLE;

bool lookupCost(const nav2_msgs::msg::Costmap & costmap, double wx, double wy, uint8_t & cost)
{
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  if (resolution <= 0.0) {
    return false;
  }
  const int i =
    static_cast<int>(std::floor((wx - costmap.metadata.origin.position.x) / resolution));
  const int j =
    static_cast<int>(std::floor((wy - costmap.metadata.origin.position.y) / resolution));
  const int size_x = static_cast<int>(costmap.metadata.size_x);
  const int size_y = static_cast<int>(costmap.metadata.size_y);
  if (i < 0 || i >= size_x || j < 0 || j >= size_y) {
    return false;
  }
  const size_t index =
    static_cast<size_t>(i) + static_cast<size_t>(j) * static_cast<size_t>(size_x);
  if (index >= costmap.data.size()) {
    return false;
  }
  cost = costmap.data[index];
  return true;
}

/// 足迹圆内是否压到真障碍（254）。253 只是安全余量，允许压在足迹里。
bool footprintTouchesLethal(
  const nav2_msgs::msg::Costmap & costmap, double x, double y, double radius)
{
  constexpr int kSamples = 12;
  uint8_t cost = 0;
  if (!lookupCost(costmap, x, y, cost) || cost >= kLethalCost) {
    return true;
  }
  for (int i = 0; i < kSamples; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kSamples);
    const double sample_x = x + radius * std::cos(angle);
    const double sample_y = y + radius * std::sin(angle);
    if (!lookupCost(costmap, sample_x, sample_y, cost) || cost >= kLethalCost) {
      return true;
    }
  }
  return false;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace

void EscapeObstacleLayer::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue(std::string("local_costmap/get_costmap")));
  nav2_util::declare_parameter_if_not_declared(
    node, "costmap_frame", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, "default_max_distance", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(node, "default_speed", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, "default_time_allowance", rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(node, "exit_clearance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "lethal_traverse_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "footprint_radius", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(
    node, "angle_step", rclcpp::ParameterValue(0.09817477042468103));
  nav2_util::declare_parameter_if_not_declared(node, "rescan_period", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, "angle_hysteresis", rclcpp::ParameterValue(0.3490658503988659));

  node->get_parameter("service_name", service_name_);
  node->get_parameter("costmap_frame", costmap_frame_);
  node->get_parameter("default_max_distance", default_max_distance_);
  node->get_parameter("default_speed", default_speed_);
  node->get_parameter("default_time_allowance", default_time_allowance_);
  node->get_parameter("exit_clearance", exit_clearance_);
  node->get_parameter("lethal_traverse_distance", lethal_traverse_distance_);
  node->get_parameter("footprint_radius", footprint_radius_);
  node->get_parameter("angle_step", angle_step_);
  node->get_parameter("rescan_period", rescan_period_);
  node->get_parameter("angle_hysteresis", angle_hysteresis_);

  if (costmap_frame_.empty()) {
    costmap_frame_ = global_frame_;
  }

  // 方向判定读的是局部代价地图（odom 系），位姿也必须取同一张图的坐标系
  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);
}

void EscapeObstacleLayer::onCleanup() { costmap_client_.reset(); }

EscapeObstacleLayer::EscapeConfig EscapeObstacleLayer::makeConfig() const
{
  EscapeConfig config;
  config.max_distance = max_distance_;
  config.exit_clearance = exit_clearance_;
  config.lethal_traverse_distance = lethal_traverse_distance_;
  config.footprint_radius = footprint_radius_;
  config.angle_step = angle_step_;
  return config;
}

std::optional<nav2_msgs::msg::Costmap> EscapeObstacleLayer::fetchCostmap()
{
  if (!costmap_client_) {
    return std::nullopt;
  }
  while (!costmap_client_->wait_for_service(std::chrono::milliseconds(200))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the costmap service.");
      return std::nullopt;
    }
    RCLCPP_WARN(
      logger_, "Costmap service %s not available, waiting again...", service_name_.c_str());
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Timed out waiting for the costmap service %s", service_name_.c_str());
    return std::nullopt;
  }
  auto response = result.get();
  if (!response) {
    return std::nullopt;
  }
  return response->map;
}

std::optional<EscapeObstacleLayer::EscapeDirection> EscapeObstacleLayer::findEscapeDirection(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
  const EscapeConfig & config)
{
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  if (resolution <= 0.0 || config.max_distance <= 0.0 || config.angle_step <= 0.0) {
    return std::nullopt;
  }

  uint8_t start_cost = 0;
  if (!lookupCost(costmap, pose.x, pose.y, start_cost)) {
    return std::nullopt;
  }
  if (start_cost < kInscribedCost) {
    // 已经不在膨胀硬区，没有"脱困"可做
    return std::nullopt;
  }
  const bool start_in_lethal = start_cost >= kLethalCost;

  std::optional<EscapeDirection> best;
  for (double angle = -M_PI; angle < M_PI - 1e-9; angle += config.angle_step) {
    const double dir_x = std::cos(angle);
    const double dir_y = std::sin(angle);
    double exit_distance = -1.0;
    bool valid = true;

    for (double r = resolution; r <= config.max_distance + 0.5 * resolution; r += resolution) {
      uint8_t cost = 0;
      if (!lookupCost(costmap, pose.x + r * dir_x, pose.y + r * dir_y, cost)) {
        valid = false;
        break;
      }
      if (cost >= kLethalCost) {
        // 真障碍：只有起点本身已经压在障碍里，才允许穿过有限厚度把车拉出来
        if (!start_in_lethal || r > config.lethal_traverse_distance) {
          valid = false;
          break;
        }
        continue;
      }
      if (cost >= kInscribedCost) {
        // 膨胀硬区：可以穿过，但必须在上限内走出去
        continue;
      }
      exit_distance = r;
      break;
    }

    if (!valid || exit_distance < 0.0) {
      continue;
    }

    // 从"出口 + clearance"往外找第一个足迹安全的落点：
    //   - 只出 253 时，exit + clearance 通常就已经安全，行走距离由几何决定且最小；
    //   - 起点本身压在 254 里时，必须一直走到障碍外再加一个车体半径，否则足迹仍压障碍。
    // 同时要求出口到落点之间的中心线不穿真障碍（射线检查只在出口之前做过）。
    double command_distance = -1.0;
    for (double candidate = exit_distance + config.exit_clearance;
         candidate <= config.max_distance + 0.5 * resolution; candidate += resolution) {
      if (candidate > config.max_distance) {
        break;
      }

      bool ray_clear = true;
      for (double r = exit_distance; r <= candidate + 0.5 * resolution; r += resolution) {
        uint8_t cost = 0;
        if (
          !lookupCost(costmap, pose.x + r * dir_x, pose.y + r * dir_y, cost) ||
          cost >= kLethalCost) {
          ray_clear = false;
          break;
        }
      }
      if (!ray_clear) {
        break;
      }

      const double end_x = pose.x + candidate * dir_x;
      const double end_y = pose.y + candidate * dir_y;
      // 终点不能压真障碍；253 只是安全余量，允许压在足迹里
      if (footprintTouchesLethal(costmap, end_x, end_y, config.footprint_radius)) {
        continue;
      }

      command_distance = candidate;
      break;
    }

    if (command_distance < 0.0) {
      continue;
    }

    if (!best || exit_distance < best->exit_distance) {
      best = EscapeDirection{angle, exit_distance, command_distance};
    }
  }

  return best;
}

nav2_behaviors::Status EscapeObstacleLayer::onRun(
  const std::shared_ptr<const EscapeObstacleLayerAction::Goal> command)
{
  auto costmap = fetchCostmap();
  if (!costmap) {
    RCLCPP_ERROR(logger_, "No costmap available from service %s", service_name_.c_str());
    return nav2_behaviors::Status::FAILED;
  }

  if (!nav2_util::getCurrentPose(
        start_pose_, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  max_distance_ = command->max_distance > 0.0F ? static_cast<double>(command->max_distance)
                                               : default_max_distance_;
  speed_ = command->speed > 0.0F ? static_cast<double>(command->speed) : default_speed_;
  time_allowance_ = command->time_allowance > 0.0F ? static_cast<double>(command->time_allowance)
                                                   : default_time_allowance_;

  uint8_t cost = 0;
  const double start_x = start_pose_.pose.position.x;
  const double start_y = start_pose_.pose.position.y;
  if (!lookupCost(*costmap, start_x, start_y, cost)) {
    RCLCPP_ERROR(logger_, "Robot pose is outside of the costmap window.");
    return nav2_behaviors::Status::FAILED;
  }

  noop_ = false;
  escaped_ = false;
  traveled_ = 0.0;
  escape_start_distance_ = 0.0;

  if (cost < kInscribedCost) {
    // 已经不在膨胀硬区：这个节点没有职责，直接成功，让行为树走下一个恢复项
    noop_ = true;
    RCLCPP_INFO(
      logger_, "Robot is outside the inflated obstacle layer (cost %u), nothing to escape",
      static_cast<unsigned int>(cost));
    return nav2_behaviors::Status::SUCCEEDED;
  }

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = start_x;
  pose2d.y = start_y;
  pose2d.theta = tf2::getYaw(start_pose_.pose.orientation);

  const auto direction = findEscapeDirection(*costmap, pose2d, makeConfig());
  if (!direction) {
    RCLCPP_ERROR(
      logger_, "No escape direction from (%.2f, %.2f) within %.2f m", start_x, start_y,
      max_distance_);
    return nav2_behaviors::Status::FAILED;
  }

  direction_ = direction->angle;
  start_time_ = clock_->now();
  last_scan_time_ = start_time_;

  RCLCPP_WARN(
    logger_, "Escaping cost %u: heading %.3f rad, exit in %.2f m, command %.2f m, speed %.2f m/s",
    static_cast<unsigned int>(cost), direction_, direction->exit_distance,
    direction->command_distance, speed_);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status EscapeObstacleLayer::onCycleUpdate()
{
  if (noop_) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_)) {
    stopRobot();
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  const double pose_x = current_pose.pose.position.x;
  const double pose_y = current_pose.pose.position.y;
  traveled_ =
    std::hypot(pose_x - start_pose_.pose.position.x, pose_y - start_pose_.pose.position.y);

  auto costmap = fetchCostmap();
  uint8_t cost = 0;
  const bool inside = costmap && lookupCost(*costmap, pose_x, pose_y, cost);

  auto feedback = std::make_shared<EscapeObstacleLayerAction::Feedback>();
  feedback->distance_traveled = static_cast<float>(traveled_);
  feedback->current_cost = inside ? static_cast<float>(cost) : -1.0F;
  action_server_->publish_feedback(feedback);

  const double elapsed = (clock_->now() - start_time_).seconds();
  if (traveled_ >= max_distance_ || elapsed >= time_allowance_) {
    stopRobot();
    RCLCPP_WARN(
      logger_, "Escape budget exhausted: traveled %.2f m of %.2f m, elapsed %.1f s of %.1f s",
      traveled_, max_distance_, elapsed, time_allowance_);
    return nav2_behaviors::Status::FAILED;
  }

  if (!inside) {
    stopRobot();
    RCLCPP_ERROR(logger_, "Costmap or robot pose unavailable while escaping.");
    return nav2_behaviors::Status::FAILED;
  }

  if (cost < kInscribedCost) {
    if (!escaped_) {
      escaped_ = true;
      escape_start_distance_ = traveled_;
      RCLCPP_INFO(
        logger_, "Left the inflated obstacle layer after %.2f m, moving %.2f m more", traveled_,
        exit_clearance_);
    } else if (traveled_ - escape_start_distance_ >= exit_clearance_) {
      stopRobot();
      RCLCPP_INFO(logger_, "Escaped the inflated obstacle layer after %.2f m", traveled_);
      return nav2_behaviors::Status::SUCCEEDED;
    }
  } else {
    escaped_ = false;
  }

  const rclcpp::Time now = clock_->now();
  if (!escaped_ && (now - last_scan_time_).seconds() >= rescan_period_) {
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = pose_x;
    pose2d.y = pose_y;
    pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

    const auto direction = findEscapeDirection(*costmap, pose2d, makeConfig());
    if (!direction) {
      stopRobot();
      RCLCPP_ERROR(
        logger_, "No escape direction from (%.2f, %.2f) within %.2f m", pose_x, pose_y,
        max_distance_);
      return nav2_behaviors::Status::FAILED;
    }
    last_scan_time_ = now;
    if (std::fabs(normalizeAngle(direction->angle - direction_)) > angle_hysteresis_) {
      direction_ = direction->angle;
      RCLCPP_INFO(logger_, "Escape direction updated to %.3f rad", direction_);
    }
  }

  const double robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = speed_ * std::cos(direction_ - robot_yaw);
  cmd_vel->linear.y = speed_ * std::sin(direction_ - robot_yaw);
  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::EscapeObstacleLayer, nav2_core::Behavior)
