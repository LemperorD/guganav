#include <behaviortree_cpp/basic_types.h>
#include "rmul_decision/condition_node/IsGameStart.hpp"
IsGameStart::IsGameStart(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList IsGameStart::providedPorts() {
  return {BT::InputPort<int>("game_progress")};
}

BT::NodeStatus IsGameStart::tick() {
  int game_progress = -1;
  if (!getInput("game_progress", game_progress)) {
    throw BT::RuntimeError("Missing input [game_progress]");
  }
  if (game_progress == 4) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}