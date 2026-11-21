#pragma once

#include "behaviortree_cpp/action_node.h"
#include <rmul_decision/decision_type.hpp>
class ChangeHigherState : public BT::SyncActionNode {
public:
  ChangeHigherState(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};