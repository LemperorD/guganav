#pragma once
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
class IsOccupationsafe : public BT::ConditionNode {
public:
  IsOccupationsafe(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};