#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

// 交战动作。
//
// 当前只读取数据、不做动作：不发新目标点，Nav2 继续执行手上的目标，于是
// 机器人保持原目标点前进。读取是为了在日志里留下"交战分支什么时候被走到、
// 当时敌情和弹药如何"，便于回头核对分支切换的时机。
//
// 是否需要交战的判断在 ShouldEngage 里——那是条件节点，走到这里的都是已经
// 判定该交战的情形。所以这个节点不做条件判断，写错了会把两处逻辑弄重复。
//
// 等要做真正的动作（停下、转向、开火）时在这里加，那时才会需要发速度或者调 action。
class Attack : public BT::SyncActionNode
{
public:
  Attack(const std::string& name, const BT::NodeConfig& config,
         rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(std::move(node)) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("ammo"), BT::InputPort<int>("enemy_count") };
  }

  BT::NodeStatus tick() override
  {
    auto ammo = getInput<int>("ammo");
    auto count = getInput<int>("enemy_count");

    // 节流到 2 秒一条：树是按参数频率反复 tick 的，不节流会把日志刷满。
    if (node_ && ammo && count) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "交战分支：视野内 %d 个敌人，弹药 %d，保持原目标前进",
        count.value(), ammo.value());
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  // 仅用于打日志，动作本身不依赖它。
  rclcpp::Node::SharedPtr node_;
};
