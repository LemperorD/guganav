#pragma once

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include <rmul_decision/decision_type.hpp>
#include "rclcpp/rclcpp.hpp"
class CheckTime : public BT::ConditionNode {
public:
  CheckTime(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};