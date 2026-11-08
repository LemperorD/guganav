#include "rmul_decision/action_node/GotoPoints.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
using namespace rmul_decision;
PointSelector::PointSelector(const std::string& name,
                             const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {
}

BT::PortsList PointSelector::providedPorts() {
  return {BT::OutputPort<int>("Destination")};
}

BT::NodeStatus PointSelector::tick() {
  int destination = 1;
  if (!getInput("Destination", destination)) {
    return BT::NodeStatus::FAILURE;
  }
  navigateToPoints(destination);
}

// void NavigateToPoints(int destination){
//     //
// }