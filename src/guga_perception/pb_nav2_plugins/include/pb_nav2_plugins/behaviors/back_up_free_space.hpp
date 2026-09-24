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

#ifndef PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
#define PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace pb_nav2_behaviors
{

/**
 * @class pb_nav2_behaviors::BackUpFreeSpace
 * @brief An enhanced back_up action that move toward free space
 */
class BackUpFreeSpace : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  BackUpFreeSpace() = default;

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Cleanup server on lifecycle transition
   */
  void onCleanup() override;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  struct EscapeDirection
  {
    float angle;
    float escape_distance;
    float command_distance;
    bool started_in_high_cost;
  };

  /**
   * @brief Gather free points within a specified radius from the center in the costmap.
   *
   * This function iterates through the costmap and collects points that are free (costmap value is 0)
   * and within the specified radius from the given center coordinates (center_x, center_y).
   *
   * @param costmap The costmap to search for free points.
   * @param center_x The x-coordinate of the center point.
   * @param center_y The y-coordinate of the center point.
   * @param radius The radius within which to gather free points.
   * @return A vector of points that are free and within the specified radius.
   */
  std::vector<geometry_msgs::msg::Point> gatherFreePoints(
    const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius);

  /**
   * @brief Find a collision-free omnidirectional recovery command.
   *
   * @param costmap Costmap used to evaluate sampled rays and endpoint footprints.
   * @param pose Robot pose in the costmap global frame.
   * @param start_angle First sampled direction in radians.
   * @param end_angle Last sampled direction in radians.
   * @param requested_distance Maximum command distance for a normal-cost start.
   * @param angle_increment Angular spacing between sampled directions in radians.
   * @return The selected direction, distance to leave the initial high-cost region, and safe
   * command distance; std::nullopt when no valid direction exists.
   */
  std::optional<EscapeDirection> findBestDirection(
    const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
    float end_angle, float requested_distance, float angle_increment);

  bool isOmniCollisionFree(
    const double & distance, geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  void visualize(
    geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_safe_angle,
    float best_angle);

  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    marker_pub_;
  double twist_x_{0.0};
  double twist_y_{0.0};
  double collision_check_start_distance_{0.0};
  bool escaping_initial_high_cost_{false};

  // parameters
  std::string service_name_;
  double max_radius_{1.0};
  double max_escape_distance_{0.5};
  double escape_clearance_{0.1};
  bool visualize_{false};
};

}  // namespace pb_nav2_behaviors

#endif  // PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
