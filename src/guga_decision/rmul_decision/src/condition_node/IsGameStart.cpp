#include <behaviortree_cpp/basic_types.h>
#include "rmul_decision/condition_node/IsGameStart.hpp"
IsGameStart::IsGameStart(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList IsGameStart::providedPorts() {
  return {BT::InputPort<int>("game_status"), BT::InputPort<int>("game_status")};
}

BT::NodeStatus IsGameStart::tick() {
  int game_status = -1;
  if (!getInput("game_status", game_status)) {
    throw BT::RuntimeError("Missing input [game_status]");
    if (game_status == -1) {
      return BT::NodeStatus::FAILURE;
    }
    if (game_status == 4) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
}