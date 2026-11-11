#include <behaviortree_cpp/basic_types.h>
#include "rmul_decision/condition_node/IsRevival.hpp"
IsRevival::IsRevival(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList IsRevival::providedPorts() {
  return {BT::InputPort<double>("HP"), BT::BidirectionalPort<int>("HP_sign")};
}

BT::NodeStatus IsRevival::tick() {
  int sign = 0;
  double hp = -1;
  if (!getInput("HP", hp)) {
    throw BT::RuntimeError("Missing input [HP]");
  }
  if (!getInput("HP_sign", sign)) {
    throw BT::RuntimeError("Missing input [HP_sign]");
  }
  // 暂时模拟，后续从 RobotContext 获取
  if (hp == 0 && sign == 0) {
    sign = 1;
    setOutput("HP_sign", sign);
  }
  if (hp == 400 && sign == 1) {
    sign = 0;
    setOutput("HP_sign", sign);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
