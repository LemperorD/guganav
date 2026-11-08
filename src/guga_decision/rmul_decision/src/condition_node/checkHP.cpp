#include "rmul_decision/condition_node/checkHP.hpp"
using namespace rmul_decision;
CheckHP::CheckHP(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 订阅 HP 话题
  hp_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "/HP", rclcpp::QoS(10),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        hp_.store(msg->data);
        hp_received_.store(true);
      });
}

BT::PortsList CheckHP::providedPorts() {
  return {BT::OutputPort<int>("Destination")};
}

BT::NodeStatus CheckHP::tick() {
  int destination = -1;
  StrategyMode mode = StrategyMode::UNKNOWN;
  if (!hp_received_.load()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                         "Waiting for HP message...");
    return BT::NodeStatus::FAILURE;
  }

  int hp = hp_.load();
  if (!getInput("StategyMode", mode)) {
    throw BT::RuntimeError("Missing input [StategyMode]");
  }

  // 暂时模拟，后续从 RobotContext 获取

  if (hp < 0) {
    return BT::NodeStatus::FAILURE;
  }
  switch (mode) {
    case StrategyMode::SUPPRESS:
      if (hp > 300) {
        destination = 5;
      } else if (hp > 90) {
        destination = 4;
      }
      break;
    case StrategyMode::OCCUPATION:
      if (hp > 150) {
        destination = 4;
      } else if (hp > 90) {
        destination = 3;
      }
      break;
    case StrategyMode::PUSHFORWARD:
      if ((destination == 2 && hp < 90) || (destination == 3 && hp < 150)
          || (destination == 4 && hp < 250)) {
        destination = 1;
      }
      break;
    case StrategyMode::UNKNOWN:
      return BT::NodeStatus::FAILURE;
      break;
  }

  if (destination != -1) {
    setOutput("Destination", destination);
  }
  return BT::NodeStatus::SUCCESS;
}
