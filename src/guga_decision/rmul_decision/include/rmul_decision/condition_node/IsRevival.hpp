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
};