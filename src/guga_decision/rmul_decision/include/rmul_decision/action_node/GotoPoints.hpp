#pragma once

#include <rmul_decision/decision_type.hpp>
#include <rmul_decision/robot_context.hpp>
#include "behaviortree_cpp/action_node.h"

class PointSelector : public BT::SyncActionNode {
public:
  PointSelector(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void navigateToPoints(int destination);
};
