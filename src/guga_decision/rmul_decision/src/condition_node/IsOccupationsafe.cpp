#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/exceptions.h>
#include "rmul_decision/condition_node/IsOccupationsafe.hpp"
IsOccupationsafe::IsOccupationsafe(const std::string& name,
                                   const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList IsOccupationsafe::providedPorts() {
  return {BT::InputPort<int>("destination"), BT::InputPort<double>("HP"),
          BT::BidirectionalPort<double>("occupation_time")};
}

BT::NodeStatus IsOccupationsafe::tick() {
  int destination = 1;
  double hp = 0;
  if (!getInput("destination", destination) || !getInput("HP", hp)) {
    return BT::NodeStatus::FAILURE;
  }

  const double now_seconds = node_->get_clock()->now().seconds();
  if (destination == 4) {
    if (timer_destination_ != 4) {
      timer_destination_ = 4;
      start_seconds_ = now_seconds;
      setOutput("occupation_time", 0.0);
      return BT::NodeStatus::FAILURE;
    }

    const double occupation_time = now_seconds - start_seconds_;
    setOutput("occupation_time", occupation_time);
    if (occupation_time >= 20.0 && hp > 350) {
      timer_destination_.reset();
      setOutput("occupation_time", 0.0);
      RCLCPP_INFO(node_->get_logger(),
                  "IsOccupationsafe: 占点停留 %.2f 秒，达到 20 秒",
                  occupation_time);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  timer_destination_.reset();
  setOutput("occupation_time", 0.0);
  return BT::NodeStatus::FAILURE;
}
