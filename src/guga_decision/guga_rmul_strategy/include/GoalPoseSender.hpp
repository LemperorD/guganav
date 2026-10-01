#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// ROS 2 侧：向 goal_pose 话题发布目标点，由 Nav2 的 bt_navigator 接收。
// 与 ROS2Monitor 相对：那个负责收裁判数据，这个负责发导航目标。
//
// publisher、参数和"上次发过什么"都放在这里，所以整棵树只声明一次参数、
// 只有一个 publisher，多个 SetGoalPose 实例共享同一份状态。
class GoalPoseSender
{
public:
  explicit GoalPoseSender(rclcpp::Node::SharedPtr node);

  // 目标点与上次相同时不重复发布，返回 false 表示这次没有发。
  bool send(double x, double y);

private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<PoseStamped>::SharedPtr publisher_;

  std::string frame_id_;

  double last_x_{0.0};
  double last_y_{0.0};
  bool has_last_{false};
};
