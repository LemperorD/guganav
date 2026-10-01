#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "ROS2Monitor.hpp"

// 条件判断：视野内是否有敌人。
// enemy_count > 0 返回 SUCCESS，否则 FAILURE，供 CaptureCenterArea 的
// IfThenElse 在"攻击"与"去场地中心"之间选择。
//
// 继承 ConditionNode 而不是动作节点：它只读一个计数、没有副作用、也不会
// 跨 tick 持续，属于瞬时判断。
//
// 视觉还没发布数据时同样返回 FAILURE，等同于"没看见敌人"，于是继续占点。
// 这是个简化：把"没数据"和"没敌人"当成一回事，代价是视觉断流时决策会静默
// 退回占点而不会报警。要区分的话需要额外判断消息时间戳是否过期。
class EngageEnemy : public BT::ConditionNode
{
public:
  EngageEnemy(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<ROS2Monitor> monitor)
  : BT::ConditionNode(name, config), monitor_(std::move(monitor)) {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (!monitor_) {
      return BT::NodeStatus::FAILURE;
    }
    return monitor_->hasEnemy() ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<ROS2Monitor> monitor_;
};
