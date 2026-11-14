#include "guga_decision_common/goal_controller.hpp"

#include <cmath>
#include <exception>
#include <memory>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"

namespace guga_decision_common {

  GoalController::GoalController(
      rclcpp::Node& node, const std::string& action_name,
      const std::string& frame_id, const std::string& marker_topic)
      : node_(&node), action_name_(action_name), frame_id_(frame_id) {
    // 单独发布 RViz Marker，避免为了可视化而重复发布 goal_pose。Transient Local
    // 让 RViz 在目标已经发出后才启动时，也能收到最近一次目标。
    marker_publisher_ = node.create_publisher<visualization_msgs::msg::Marker>(
        marker_topic, rclcpp::QoS(1).reliable().transient_local());
    action_client_ =
        rclcpp_action::create_client<NavigateToPose>(&node, action_name_);
  }

  bool GoalController::sendGoal(const Waypoint2D& goal, bool force) {
    if (!std::isfinite(goal.x) || !std::isfinite(goal.y)
        || !std::isfinite(goal.yaw)) {
      return false;
    }

    if (!force && has_last_goal_ && goal.x == last_goal_.x
        && goal.y == last_goal_.y && goal.yaw == last_goal_.yaw) {
      return true;
    }

    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "NavigateToPose action server 尚未就绪: %s", action_name_.c_str());
      return false;
    }

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, goal.yaw);

    NavigateToPose::Goal message;
    message.pose.header.frame_id = frame_id_;
    message.pose.header.stamp = node_->now();
    message.pose.pose.position.x = goal.x;
    message.pose.pose.position.y = goal.y;
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();

    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      generation = ++generation_;
      result_ = NavigationResult::PENDING;
      current_goal_.reset();
    }

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
        [this, generation](GoalHandle::SharedPtr goal_handle) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (generation != generation_) {
            return;
          }
          if (!goal_handle) {
            result_ = NavigationResult::FAILED;
            return;
          }
          current_goal_ = std::move(goal_handle);
        };
    options.result_callback =
        [this, generation](const GoalHandle::WrappedResult& result) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (generation != generation_) {
            return;
          }
          current_goal_.reset();
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              result_ = NavigationResult::SUCCEEDED;
              RCLCPP_INFO(
                  node_->get_logger(), "NavigateToPose 导航成功");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              result_ = NavigationResult::CANCELED;
              RCLCPP_WARN(
                  node_->get_logger(), "NavigateToPose 导航被取消");
              break;
            case rclcpp_action::ResultCode::ABORTED:
            case rclcpp_action::ResultCode::UNKNOWN:
            default:
              result_ = NavigationResult::FAILED;
              RCLCPP_ERROR(
                  node_->get_logger(), "NavigateToPose 导航失败，code=%d",
                  static_cast<int>(result.code));
              break;
          }
        };

    action_client_->async_send_goal(message, options);
    publishMarker(goal);
    RCLCPP_INFO(
        node_->get_logger(),
        "发送 NavigateToPose 目标: (%.3f, %.3f, yaw=%.3f)"
        " 于 %s",
        goal.x, goal.y, goal.yaw, frame_id_.c_str());

    last_goal_ = goal;
    has_last_goal_ = true;
    return true;
  }

  NavigationResult GoalController::pollResult() {
    std::lock_guard<std::mutex> lock(mutex_);
    const NavigationResult result = result_;
    if (result == NavigationResult::SUCCEEDED
        || result == NavigationResult::FAILED
        || result == NavigationResult::CANCELED) {
      result_ = NavigationResult::NONE;
    }
    return result;
  }

  void GoalController::cancelGoal() {
    GoalHandle::SharedPtr goal_handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++generation_;
      result_ = NavigationResult::NONE;
      goal_handle = std::move(current_goal_);
      current_goal_.reset();
    }
    if (goal_handle) {
      try {
        action_client_->async_cancel_goal(goal_handle);
      } catch (const std::exception& error) {
        RCLCPP_WARN(
            node_->get_logger(), "取消 NavigateToPose 目标失败: %s",
            error.what());
      }
    }
  }

  void GoalController::publishMarker(const Waypoint2D& goal) {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, goal.yaw);

    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = frame_id_;
    message.header.stamp = node_->now();
    message.pose.position.x = goal.x;
    message.pose.position.y = goal.y;
    message.pose.position.z = 0.0;
    message.pose.orientation.x = orientation.x();
    message.pose.orientation.y = orientation.y();
    message.pose.orientation.z = orientation.z();
    message.pose.orientation.w = orientation.w();

    visualization_msgs::msg::Marker marker;
    marker.header = message.header;
    marker.ns = "decision_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = message.pose;
    marker.pose.position.z = 0.25;
    marker.scale.x = 0.80;
    marker.scale.y = 0.16;
    marker.scale.z = 0.22;
    marker.color.r = 0.10F;
    marker.color.g = 0.85F;
    marker.color.b = 1.00F;
    marker.color.a = 1.00F;
    marker_publisher_->publish(marker);
  }

}  // namespace guga_decision_common
