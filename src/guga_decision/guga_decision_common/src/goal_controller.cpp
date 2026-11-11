#include "guga_decision_common/goal_controller.hpp"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"

namespace guga_decision_common
{

GoalController::GoalController(
  rclcpp::Node& node,
  const std::string& topic,
  const std::string& frame_id)
: node_(&node), frame_id_(frame_id)
{
  publisher_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
    topic, rclcpp::SensorDataQoS());
}

bool GoalController::sendGoal(const Waypoint2D& goal, bool force)
{
  if (!std::isfinite(goal.x) ||
      !std::isfinite(goal.y) ||
      !std::isfinite(goal.yaw))
  {
    return false;
  }

  if (!force && has_last_goal_ &&
      goal.x == last_goal_.x &&
      goal.y == last_goal_.y &&
      goal.yaw == last_goal_.yaw)
  {
    return true;
  }

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
  publisher_->publish(message);

  last_goal_ = goal;
  has_last_goal_ = true;
  return true;
}

}  // namespace guga_decision_common
