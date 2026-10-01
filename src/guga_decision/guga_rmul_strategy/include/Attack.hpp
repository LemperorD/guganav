#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "ROS2Monitor.hpp"

// 遇到敌人时的动作。
//
// 当前只读取敌情、不做动作：不发新目标点，Nav2 继续执行手上的目标，于是
// 机器人保持原目标点前进。读取是为了在日志里留下"攻击分支什么时候被走到、
// 当时视野里有几个敌人"，便于回头核对分支切换的时机。
//
// 等要做真正的攻击动作（停下、转向、开火）时在这里加，那时才会需要发速度
// 或者调 action。
class Attack : public BT::SyncActionNode
{
public:
  Attack(const std::string& name, const BT::NodeConfig& config,
         std::shared_ptr<ROS2Monitor> monitor)
  : BT::SyncActionNode(name, config), monitor_(std::move(monitor)) {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (!monitor_) {
      return BT::NodeStatus::FAILURE;
    }

    // 节流到 2 秒一条：树是按参数频率反复 tick 的，不节流会把日志刷满。
    RCLCPP_INFO_THROTTLE(
      monitor_->get_logger(), *monitor_->get_clock(), 2000,
      "攻击分支：视野内 %d 个敌人，保持原目标前进",
      static_cast<int>(monitor_->enemyCount()));

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<ROS2Monitor> monitor_;
};
