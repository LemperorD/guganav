#pragma once

#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

// 比赛是否已经开始：比赛阶段持续为"比赛中"达到 hold 秒才算数，其余一律失败。
//
// 为什么要"持续"而不是看到一个 4 就通过：裁判数据在链路抖动、重连或阶段切换的
// 瞬间可能闪一下 4 再退回，瞬时判断会让机器人在不该动的时候动起来。要求连续
// 保持一段时间，等于给这个判断加了个去抖。
//
// 计时在节点里（成员变量）：判定的是"从哪一刻起一直是比赛中"，这份状态跟着这个
// 节点实例走，不需要放黑板——它不是一个要在别处读到的值。阶段一旦不是比赛中就
// 复位，下次重新计时。
//
// 取值见 GameStatus.msg：0 未开始、1 准备、2 自检、3 倒计时、4 比赛中、5 结算。
// 与 simple_decision 的判法一致（它也把 4 当"比赛中"）。
class IsGameStart : public BT::ConditionNode
{
public:
  IsGameStart(const std::string& name, const BT::NodeConfig& config,
              rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(std::move(node)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("game_progress", "比赛阶段，4 = 比赛中"),
      BT::InputPort<double>("hold_sec", 1.0, "要保持多久才算开始"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto progress = getInput<int>("game_progress");
    if (!progress) {
      // 键不存在说明 Ros2Input 还没跑过或端口没接，属于接线问题。
      // 按"所有其他情况为失败"处理，但打一条 ERROR，免得看起来像"比赛还没开始"。
      if (node_) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                              "IsGameStart 读不到 game_progress（应接 "
                              "{game_progress}）");
      }
      holding_ = false;
      return BT::NodeStatus::FAILURE;
    }

    double hold_sec = 1.0;
    getInput("hold_sec", hold_sec);

    if (progress.value() != kRunning) {
      // 不是比赛中：不管是没开始、结算中还是闪了一下，都从零重新计。
      if (holding_ && node_) {
        RCLCPP_INFO(node_->get_logger(), "比赛阶段变为 %d，重新计时",
                    progress.value());
      }
      holding_ = false;
      return BT::NodeStatus::FAILURE;
    }

    if (!node_) {
      return BT::NodeStatus::FAILURE;
    }

    if (!holding_) {
      holding_ = true;
      since_ = node_->now();
    }

    if ((node_->now() - since_).seconds() < hold_sec) {
      return BT::NodeStatus::FAILURE;
    }

    // 只在跨过门槛的那一轮打一条，之后每轮都是成功，不必重复说。
    if (!reported_) {
      reported_ = true;
      RCLCPP_INFO(node_->get_logger(), "比赛已开始并保持 %.1f 秒", hold_sec);
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  // 比赛阶段"比赛中"的取值（协议 0x0001）。
  static constexpr int kRunning = 4;

  // 仅用于打日志与取时间，判定本身不依赖别的 ROS 资源。
  rclcpp::Node::SharedPtr node_;

  // "从哪一刻起一直是比赛中"。阶段离开比赛中的那一轮清零。
  bool holding_{false};
  rclcpp::Time since_;
  bool reported_{false};
};
