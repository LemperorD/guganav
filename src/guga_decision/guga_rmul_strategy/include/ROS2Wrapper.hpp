#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "ROS2Monitor.hpp"

// 行为树侧：持有 ROS 2 节点，每次 tick 把它的最新数据写进黑板。
// 它自己不做判断，判断交给下游节点，这样数据来源和决策逻辑各自独立。
//
// 端口用的是 OutputPort，所以黑板键名由 XML 决定：
//   <ROS2Wrapper HP="{HP}" max_HP="{max_HP}"/>
class ROS2Wrapper : public BT::SyncActionNode
{
public:
  ROS2Wrapper(const std::string& name, const BT::NodeConfiguration& config,
              std::shared_ptr<ROS2Monitor> monitor)
  : BT::SyncActionNode(name, config), monitor_(std::move(monitor)) {}

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<double>("HP")};
  }

  BT::NodeStatus tick() override
  {
    if (!monitor_) {
      // 依赖没注入进来属于配置错误，显式失败而不是静默返回 0。
      return BT::NodeStatus::FAILURE;
    }

    // 还没收到过裁判系统数据时不上报，让上游决定怎么处理，而不是把 0 当成真实血量。
    if (!monitor_->hasData()) {
      return BT::NodeStatus::FAILURE;
    }

    if (!setOutput("HP", static_cast<double>(monitor_->currentHp()))) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<ROS2Monitor> monitor_;
};
