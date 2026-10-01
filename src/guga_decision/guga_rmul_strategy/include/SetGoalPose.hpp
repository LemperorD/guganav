#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "ROS2Monitor.hpp"

// 行为树侧：把目标点交给 ROS2Monitor 发给导航。
class SetGoalPose : public BT::SyncActionNode
{
public:
  SetGoalPose(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<ROS2Monitor> monitor)
  : BT::SyncActionNode(name, config), monitor_(std::move(monitor)) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("pose_x"), BT::InputPort<double>("pose_y") };
  }

  BT::NodeStatus tick() override
  {
    if (!monitor_) {
      // 依赖没注入进来属于配置错误，显式失败而不是静默什么都不做。
      return BT::NodeStatus::FAILURE;
    }

    double x = 0.0;
    double y = 0.0;
    if (!getInput("pose_x", x) || !getInput("pose_y", y)) {
      return BT::NodeStatus::FAILURE;
    }

    // 目标点与上次相同时不会重复发布，这里同样算成功：
    // "目标点已经在那儿了"不是失败。
    monitor_->sendGoalPose(x, y);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<ROS2Monitor> monitor_;
};
