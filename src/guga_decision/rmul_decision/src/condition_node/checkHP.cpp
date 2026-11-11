#include "rmul_decision/condition_node/checkHP.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <string>
using namespace rmul_decision;

namespace {
  StrategyMode parseStrategyMode(const std::string& value) {
    if (value == "Suppress") {
      return StrategyMode::SUPPRESS;
    }
    if (value == "Occupation") {
      return StrategyMode::OCCUPATION;
    }
    if (value == "PushForward") {
      return StrategyMode::PUSHFORWARD;
    }
    return StrategyMode::UNKNOWN;
  }
}  // namespace

CheckHP::CheckHP(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList CheckHP::providedPorts() {
  return {BT::InputPort<double>("HP"),
          BT::InputPort<std::string>("strategy_mode"),
          BT::BidirectionalPort<int>("destination")};
}

BT::NodeStatus CheckHP::tick() {
  int destination = -1;
  std::string strategy;
  double hp = -1;
  if (!getInput("strategy_mode", strategy)) {
    throw BT::RuntimeError("Missing input [strategy_mode]");
  }
  if (!getInput("HP", hp)) {
    throw BT::RuntimeError("Missing input [HP]");
  }
  if (!getInput("destination", destination)) {
    throw BT::RuntimeError("Missing input [destination]");
  }

  const StrategyMode mode = parseStrategyMode(strategy);
  if (hp < 0) {
    return BT::NodeStatus::FAILURE;
  }
  if (destination == 1 && hp < 400) {
    destination = 1;
    setOutput("destination", destination);
    return BT::NodeStatus::SUCCESS;
  }
  switch (mode) {
    case StrategyMode::SUPPRESS:
      if (hp > 300) {
        destination = 5;
      } else if (hp > 90) {
        destination = 4;
      }
      break;
    case StrategyMode::OCCUPATION:
      if (hp > 150) {
        destination = 4;
      } else if (hp > 90) {
        destination = 3;
      }
      break;
    case StrategyMode::PUSHFORWARD:
      if ((destination == 2 && hp < 90) || (destination == 3 && hp < 150)
          || (destination == 4 && hp < 250)) {
        destination = 1;
      }
      break;
    case StrategyMode::UNKNOWN:
      return BT::NodeStatus::FAILURE;
      break;
  }

  if (destination != -1) {
    setOutput("destination", destination);
  }
  return BT::NodeStatus::SUCCESS;
}
