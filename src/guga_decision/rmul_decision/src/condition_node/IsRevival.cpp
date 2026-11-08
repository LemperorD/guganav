#include <behaviortree_cpp/basic_types.h>
#include "rmul_decision/condition_node/IsRevival.hpp"
IsRevival::IsRevival(const std::string& name, const BT::NodeConfig& config)
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

BT::PortsList IsRevival::providedPorts() {
  return {BT::InputPort<int>("HP"), BT::InputPort<int>("HP_sign")};
}

BT::NodeStatus IsRevival::tick() {
  int sign = 0;
  if (!hp_received_.load()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                         "Waiting for HP message...");
    return BT::NodeStatus::FAILURE;
  }

  int hp = hp_.load();
  if (!getInput("HP_sign", sign)) {
    throw BT::RuntimeError("Missing input [HP]");
  }
  // 暂时模拟，后续从 RobotContext 获取
  if (hp == 0 && sign == 0) {
    sign++;
    setOutput("HP_sign", sign);
  }
  if (hp > 0 && sign == 1) {
    sign = 0;
    setOutput("HP_sign", sign);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
