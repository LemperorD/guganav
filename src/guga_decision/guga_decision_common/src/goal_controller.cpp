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
  // 单独发布 RViz Marker，避免为了可视化而重复发布 goal_pose。Transient Local
  // 让 RViz 在目标已经发出后才启动时，也能收到最近一次目标。
  marker_publisher_ = node.create_publisher<visualization_msgs::msg::Marker>(
    topic + "_marker", rclcpp::QoS(1).reliable().transient_local());
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

  last_goal_ = goal;
  has_last_goal_ = true;
  return true;
}

}  // namespace guga_decision_common
