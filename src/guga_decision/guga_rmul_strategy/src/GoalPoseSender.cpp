#include "GoalPoseSender.hpp"

#include <stdexcept>
#include <string>
#include <utility>

GoalPoseSender::GoalPoseSender(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  if (!node_) {
    throw std::runtime_error("GoalPoseSender: 注入的 ROS 2 节点为空");
  }

  // 话题用相对名，实际话题由节点所在命名空间决定；frame_id 与项目其余部分一致为 map。
  const std::string topic =
    node_->declare_parameter<std::string>("goal_pose_topic", "goal_pose");
  frame_id_ = node_->declare_parameter<std::string>("frame_id", "map");

  // QoS 必须与 bt_navigator 的订阅一致，它用的是 BEST_EFFORT，
  // rclcpp::SensorDataQoS() 正好是 best effort。
  publisher_ = node_->create_publisher<PoseStamped>(topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(node_->get_logger(), "目标点发布: 话题 %s，坐标系 %s",
              topic.c_str(), frame_id_.c_str());
}

bool GoalPoseSender::send(double x, double y)
{
  // Nav2 收到一次目标点就会开始导航，重复发同一个点会让它重启规划。
  if (has_last_ && x == last_x_ && y == last_y_) {
    return false;
  }

  PoseStamped goal;
  goal.header.frame_id = frame_id_;
  goal.header.stamp = node_->now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;
  // 只给位置不约束朝向，用单位四元数。
  goal.pose.orientation.w = 1.0;

  publisher_->publish(goal);

  last_x_ = x;
  last_y_ = y;
  has_last_ = true;

  RCLCPP_INFO(node_->get_logger(), "发布目标点 (%.2f, %.2f)，坐标系 %s",
              x, y, frame_id_.c_str());
  return true;
}
