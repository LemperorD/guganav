#include "rmul_decision/condition_node/checkTime.hpp"
#include <behaviortree_cpp/basic_types.h>
using namespace rmul_decision;
CheckTime::CheckTime(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList CheckTime::providedPorts() {
  return {BT::BidirectionalPort<int>("destination"),
          BT::BidirectionalPort<double>("stay_time")};
}

BT::NodeStatus CheckTime::tick() {
  int destination = -1;
  double stay_time = 0;
  if (!getInput("destination", destination)) {
    throw BT::RuntimeError("Missing input [destination]");
  }
  if (!getInput("stay_time", stay_time)) {
    throw BT::RuntimeError("Missing input [stay_time]");
  }
  if (destination == 2) {
    if (stay_time == 0) {
      auto timer = node_->now();
      stay_time = timer.seconds();
      setOutput("stay_time", stay_time);
    } else if (stay_time > 0
               && node_->get_clock()->now().seconds() - stay_time >= 15) {
      destination = 3;
      stay_time = 0;
      setOutput("stay_time", stay_time);
      setOutput("destination", destination);
      return BT::NodeStatus::SUCCESS;
    }
  } else if (destination == 3) {
    if (stay_time == 0) {
      auto timer = node_->now();
      stay_time = timer.seconds();
      setOutput("stay_time", stay_time);

    } else if (stay_time > 0
               && node_->get_clock()->now().seconds() - stay_time >= 15) {
      destination = 4;
      stay_time = 0;
      setOutput("stay_time", stay_time);
      setOutput("destination", destination);
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}
