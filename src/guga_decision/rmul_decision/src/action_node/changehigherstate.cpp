#include "rmul_decision/action_node/changehigherstate.hpp"
#include <behaviortree_cpp/basic_types.h>
using namespace rmul_decision;
ChangeHigherState::ChangeHigherState(const std::string& name,
                                     const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {
}

BT::PortsList ChangeHigherState::providedPorts() {
  return {BT::InputPort<StrategyMode>("global_state_in"),
          BT::OutputPort<StrategyMode>("global_state_out")};
}

BT::NodeStatus ChangeHigherState::tick() {
  StrategyMode global_state_in = StrategyMode::UNKNOWN;
  StrategyMode global_state_out = StrategyMode::UNKNOWN;
  if (!getInput("global_state_in", global_state_in)) {
    throw BT::RuntimeError("Missing input [StategyMode]");
  }
  if (global_state_in == StrategyMode::OCCUPATION) {
    global_state_out = StrategyMode::SUPPRESS;
  } else if (global_state_in == StrategyMode::PUSHFORWARD) {
    global_state_out = StrategyMode::OCCUPATION;
  } else {
    global_state_out = global_state_in;
  }
  setOutput("global_state_out", global_state_out);
  return BT::NodeStatus::SUCCESS;
}
