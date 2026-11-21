#include "rmul_decision/action_node/changelowerstate.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <string>
ChangeLowerState::ChangeLowerState(const std::string& name,
                                   const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {
}

BT::PortsList ChangeLowerState::providedPorts() {
  return {BT::BidirectionalPort<std::string>("global_state_"),
          BT::BidirectionalPort<int>("route_epoch")};
}

BT::NodeStatus ChangeLowerState::tick() {
  std::string global_state;
  if (!getInput("global_state_", global_state)) {
    return BT::NodeStatus::FAILURE;
  }
  int route_epoch = 0;
  if (!getInput("route_epoch", route_epoch)) {
    route_epoch = 0;
  }
  const std::string previous_state = global_state;
  if (global_state == "Suppress") {
    global_state = "Occupation";
  } else if (global_state == "Occupation") {
    global_state = "PushForward";
  }
  if (global_state != previous_state) {
    setOutput("route_epoch", route_epoch + 1);
  }
  setOutput("global_state_", global_state);
  return BT::NodeStatus::SUCCESS;
}
