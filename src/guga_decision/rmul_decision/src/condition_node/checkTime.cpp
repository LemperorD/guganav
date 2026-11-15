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
  if (!getInput("destination", destination)) {
    throw BT::RuntimeError("Missing input [destination]");
  }

  const double now_seconds = node_->get_clock()->now().seconds();

  if (destination == 2) {
    if (timer_destination_ != 2) {
      timer_destination_ = 2;
      start_seconds_ = now_seconds;
      setOutput("stay_time", 0.0);
      return BT::NodeStatus::FAILURE;
    }

    const double stay_time = now_seconds - start_seconds_;
    setOutput("stay_time", stay_time);
    if (stay_time >= 15.0) {
      timer_destination_.reset();
      setOutput("stay_time", 0.0);
      setOutput("destination", 3);
      RCLCPP_INFO(
          node_->get_logger(),
          "CheckTime: destination 2 -> 3, 停留 %.2f 秒", stay_time);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  if (destination == 3) {
    if (timer_destination_ != 3) {
      timer_destination_ = 3;
      start_seconds_ = now_seconds;
      setOutput("stay_time", 0.0);
      return BT::NodeStatus::FAILURE;
    }

    const double stay_time = now_seconds - start_seconds_;
    setOutput("stay_time", stay_time);
    if (stay_time >= 15.0) {
      timer_destination_.reset();
      setOutput("stay_time", 0.0);
      setOutput("destination", 4);
      RCLCPP_INFO(
          node_->get_logger(),
          "CheckTime: destination 3 -> 4, 停留 %.2f 秒", stay_time);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  timer_destination_.reset();
  setOutput("stay_time", 0.0);
  return BT::NodeStatus::FAILURE;
}
