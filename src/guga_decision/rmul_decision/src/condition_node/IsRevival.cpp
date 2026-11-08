#include <behaviortree_cpp/basic_types.h>
#include "rmul_decision/condition_node/IsRevivedAndDead.hpp"
IsRevival::IsRevival(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList IsRevival::providedPorts() {
  return {BT::InputPort<int>("HP"), BT::InputPort<int>("HP_sign")};
}

BT::NodeStatus IsRevival::tick() {
  int hp = -1;
  int sign = 0;
  if (!getInput("HP", hp)) {
    throw BT::RuntimeError("Missing input [HP]");
  }
  if (!getInput("HP_sign", sign)) {
    throw BT::RuntimeError("Missing input [HP]");
  }
  // 暂时模拟，后续从 RobotContext 获取
  if (hp == 0 && sign == 0) {
    sign++;
    setOutput("HP_sign", sign);
  }
  if (hp > 0 && sign == 1) {
    sign = 0;
    setOutput("HP_sign", sign);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
