#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/exceptions.h>
#include "rmul_decision/condition_node/IsOccupationsafe.hpp"
IsOccupationsafe::IsOccupationsafe(const std::string& name,
                                   const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList IsOccupationsafe::providedPorts() {
  return {BT::InputPort<int>("destination"),
          BT::BidirectionalPort<double>("occupation_time")};
}

BT::NodeStatus IsOccupationsafe::tick() {
  int destination = 1;
  double occupation_time = 0;

  if (!getInput("destination", destination)) {
    throw BT::RuntimeError("Missing input [destination]");
  }
  if (!getInput("occupation_time", occupation_time)) {
    throw BT::RuntimeError("Missing input [occupation_time]");
  }
  if (occupation_time == 0 && destination == 4) {
    auto timer = node_->now();
    occupation_time = timer.seconds();
    setOutput("occupation_time", occupation_time);
  }

  else if (occupation_time > 0 && destination == 4) {
    if (node_->get_clock()->now().seconds() - occupation_time >= 20) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  else {
    occupation_time = 0;
    setOutput("occupation_time", occupation_time);
  }

  return BT::NodeStatus::FAILURE;
}
