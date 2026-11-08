#pragma once

#include "behaviortree_cpp/condition_node.h"
#include <rmul_decision/decision_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
class IsRevival : public BT::ConditionNode {
public:
  IsRevival(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hp_sub_;

  // 保存最新 HP
  std::atomic<int> hp_{-1};

  // 是否收到过 HP 消息
  std::atomic<bool> hp_received_{false};
};