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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__NODE__OMNI_PID_PURSUIT_CONTROLLER_NODE_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__NODE__OMNI_PID_PURSUIT_CONTROLLER_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "pb_omni_pid_pursuit_controller/core/pid.hpp"
#include "pb_omni_pid_pursuit_controller/core/types.hpp"

#include "std_msgs/msg/u_int8.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

class PathHandler;

namespace pb_omni_pid_pursuit_controller {

  class OmniPidPursuitControllerNode : public nav2_core::Controller {
  public:
    OmniPidPursuitControllerNode() = default;
    ~OmniPidPursuitControllerNode() override = default;

    OmniPidPursuitControllerNode(const OmniPidPursuitControllerNode&) = delete;
    OmniPidPursuitControllerNode& operator=(
        const OmniPidPursuitControllerNode&) = delete;
    OmniPidPursuitControllerNode(OmniPidPursuitControllerNode&&) = delete;
    OmniPidPursuitControllerNode& operator=(OmniPidPursuitControllerNode&&) =
        delete;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker* goal_checker) override;

    void setPlan(const nav_msgs::msg::Path& path) override;
    void setSpeedLimit(const double& speed_limit,
                       const bool& percentage) override;

  private:
    // ── pipeline steps ──

    nav_msgs::msg::Path transformPath(
        const geometry_msgs::msg::PoseStamped& pose);

    geometry_msgs::msg::PoseStamped computeLookahead(
        const geometry_msgs::msg::Twist& velocity,
        const nav_msgs::msg::Path& transformed_plan, double& lin_dist,
        double& theta_dist, double& path_yaw, double& angle_to_goal);

    void computeVelocity(double linear_distance, double angle_to_goal,
                         double& linear_vel, double& angular_vel);
    [[nodiscard]] geometry_msgs::msg::TwistStamped assembleCmdVel(
        const geometry_msgs::msg::PoseStamped& pose, double lin_vel,
        double angular_vel, double theta_dist, double path_yaw) const;

    void applyVelocityLimits(const nav_msgs::msg::Path& transformed_plan,
                             const geometry_msgs::msg::PoseStamped& carrot_pose,
                             double& lin_vel);

    bool checkCollision(const nav_msgs::msg::Path& transformed_plan,
                        const geometry_msgs::msg::PoseStamped& pose);

    // ── helpers ──

    double getLookAheadDistance(const geometry_msgs::msg::Twist& speed) const;
    geometry_msgs::msg::PoseStamped getLookAheadPoint(
        const double& lookahead_dist,
        const nav_msgs::msg::Path& transformed_plan) const;
    [[nodiscard]] double getCostmapMaxExtent() const;

    [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> transformPose(
        const std::string& frame,
        const geometry_msgs::msg::PoseStamped& in_pose) const;

    [[nodiscard]] double approachVelocityScalingFactor(
        const nav_msgs::msg::Path& transformed_path) const;
    void applyApproachVelocityScaling(const nav_msgs::msg::Path& path,
                                      double& linear_vel) const;
    void applyCurvatureLimitation(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::PoseStamped& lookahead_pose,
        double& linear_vel);
    [[nodiscard]] double calculateCurvature(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::PoseStamped& lookahead_pose,
        double forward_dist, double backward_dist) const;

    bool isCollisionDetected(const nav_msgs::msg::Path& path);

    void chassisModeCallback(std_msgs::msg::UInt8::SharedPtr msg);

    // ── infrastructure ──

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_{};
    rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitControllerNode")};
    rclcpp::Clock::SharedPtr clock_;

    ControllerConfig config_;
    ControllerState state_;
    nav_msgs::msg::Path global_plan_;

    std::shared_ptr<PID> move_pid_;
    std::shared_ptr<PID> heading_pid_;

    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
        local_path_pub_;
    rclcpp_lifecycle::LifecyclePublisher<
        geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
    rclcpp_lifecycle::LifecyclePublisher<
        visualization_msgs::msg::MarkerArray>::SharedPtr curvature_points_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;

    std::mutex mutex_;
    std::unique_ptr<PathHandler> path_handler_;
  };

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__NODE__OMNI_PID_PURSUIT_CONTROLLER_NODE_HPP_
