#pragma once
#include "behaviortree_cpp/condition_node.h"
class IsGameStart : public BT::ConditionNode {
public:
  IsGameStart(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};