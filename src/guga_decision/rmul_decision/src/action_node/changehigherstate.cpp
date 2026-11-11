#include "rmul_decision/action_node/changehigherstate.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <string>
ChangeHigherState::ChangeHigherState(const std::string& name,
                                     const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {
}

BT::PortsList ChangeHigherState::providedPorts() {
  return {BT::BidirectionalPort<std::string>("global_state_")};
}

BT::NodeStatus ChangeHigherState::tick() {
  std::string global_state;
  if (!getInput("global_state_", global_state)) {
    return BT::NodeStatus::FAILURE;
  }
  if (global_state == "Occupation") {
    global_state = "Suppress";
  } else if (global_state == "PushForward") {
    global_state = "Occupation";
  }
  setOutput("global_state_", global_state);
  return BT::NodeStatus::SUCCESS;
}
