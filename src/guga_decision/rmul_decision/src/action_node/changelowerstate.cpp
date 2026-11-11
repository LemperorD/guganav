#include "rmul_decision/action_node/changelowerstate.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <string>
ChangeLowerState::ChangeLowerState(const std::string& name,
                                   const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {
}

BT::PortsList ChangeLowerState::providedPorts() {
  return {BT::BidirectionalPort<std::string>("global_state_")};
}

BT::NodeStatus ChangeLowerState::tick() {
  std::string global_state;
  if (!getInput("global_state_", global_state)) {
    return BT::NodeStatus::FAILURE;
  }
  if (global_state == "Suppress") {
    global_state = "Occupation";
  } else if (global_state == "Occupation") {
    global_state = "PushForward";
  }
  setOutput("global_state_", global_state);
  return BT::NodeStatus::SUCCESS;
}
