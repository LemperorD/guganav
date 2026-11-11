#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "guga_decision_common/decision_input_hub.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// 本策略的 ROS 输出侧。输入订阅、缓存和 TF 查询由
// guga_decision_common::DecisionInputHub 统一提供，行为树节点只依赖本类中
// 的目标点与停车发布接口。
class ROS2Monitor final : public guga_decision_common::DecisionInputHub
{
public:
  explicit ROS2Monitor(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // 发布“要求停”给下游。电平语义：true 表示现在要求停。
  void publishStop(bool stop);

  // 发布导航目标点。目标与上次相同时不重复发布；force=true 时强制重发。
  bool sendGoalPose(double x, double y, bool force = false);

private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;

  bool last_stop_{false};
  bool has_last_stop_{false};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  bool has_last_goal_{false};
};
