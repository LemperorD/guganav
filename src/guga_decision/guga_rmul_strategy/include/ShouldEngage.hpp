#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

// 综合判断：当前是否该进入交战分支。
//
// 现在包含两件事：视野里有敌人，且还有弹药。以后要加血量下限、交战距离这类
// 条件都放在这里，树上只看到"该不该交战"这一个结论。
//
// 继承 ConditionNode 而不是动作节点：它只读输入、没有副作用、不跨 tick 持续，
// 属于瞬时判断。
//
// 数据经黑板传入（ROS2Wrapper 写的 enemy_count 与 ammo），节点本身不订阅话题。
//
// 取不到输入时返回 FAILURE，等同于"不该交战"，于是走回中心的 else 分支。
// 这是个简化："没数据"和"条件不满足"被当成一回事，代价是数据断流时决策会
// 静默退回占点而不会报警。要区分的话需要额外判断数据的新鲜度。
class ShouldEngage : public BT::ConditionNode
{
public:
  ShouldEngage(const std::string& name, const BT::NodeConfig& config,
               rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(std::move(node)) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("enemy_count"), BT::InputPort<int>("ammo") };
  }

  BT::NodeStatus tick() override
  {
    auto count = getInput<int>("enemy_count");
    auto ammo = getInput<int>("ammo");
    if (!count || !ammo) {
      // 端口取不到值说明上游没写黑板，或键名对不上。
      if (node_) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "读不到黑板的 enemy_count / ammo，按不交战处理");
      }
      return BT::NodeStatus::FAILURE;
    }

    if (count.value() <= 0) {
      return BT::NodeStatus::FAILURE;
    }
    if (ammo.value() <= 0) {
      if (node_) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "视野内 %d 个敌人，但没有弹药，不交战", count.value());
      }
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  // 仅用于打日志，判断本身不依赖它。
  rclcpp::Node::SharedPtr node_;
};
