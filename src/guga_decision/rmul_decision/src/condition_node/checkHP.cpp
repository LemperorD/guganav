#include "rmul_decision/condition_node/checkHP.hpp"
using namespace rmul_decision;
CheckHP::CheckHP(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList CheckHP::providedPorts() {
  return {BT::InputPort<int>("HP"), BT::InputPort<int>("StrategyMode"),
          BT::OutputPort<int>("Destination")};
}

BT::NodeStatus CheckHP::tick() {
  int hp = -1;
  int destination = -1;
  StrategyMode mode = StrategyMode::UNKNOWN;
  if (!getInput("HP", hp)) {
    throw BT::RuntimeError("Missing input [HP]");
  }
  if (!getInput("StategyMode", mode)) {
    throw BT::RuntimeError("Missing input [StategyMode]");
  }

  // 暂时模拟，后续从 RobotContext 获取

  if (hp < 0) {
    return BT::NodeStatus::FAILURE;
  }
  switch (mode) {
    case StrategyMode::SUPPRESS:
      if (hp > 300) {
        destination = 5;
      } else if (hp > 90) {
        destination = 4;
      }

    case StrategyMode::OCCUPATION:
      if (hp > 150) {
        destination = 4;
      } else if (hp > 90) {
        destination = 3;
      }
    case StrategyMode::PUSHFORWARD:
      if ((destination == 2 && hp < 90) || (destination == 3 && hp < 150)
          || (destination == 4 && hp < 250)) {
        destination = 1;
      }

    case StrategyMode::UNKNOWN:
      return BT::NodeStatus::FAILURE;
  }
  if (destination != -1) {
    setOutput("destination", destination);
  }
  return BT::NodeStatus::SUCCESS;
}
