#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "ROS2Monitor.hpp"

// 把目标点交给 ROS2Monitor。
class SetGoalPose : public BT::SyncActionNode {
public:
  SetGoalPose(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<ROS2Monitor> monitor)
      : BT::SyncActionNode(name, config), monitor_(std::move(monitor)) {
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("pose_x"), BT::InputPort<double>("pose_y")};
  }

  BT::NodeStatus tick() override {
    if (!monitor_) {
      // 依赖没注入进来属于配置错误，显式失败而不是静默什么都不做。
      return BT::NodeStatus::FAILURE;
    }

    double x = 0.0;
    double y = 0.0;
    if (!getInput("pose_x", x) || !getInput("pose_y", y)) {
      return BT::NodeStatus::FAILURE;
    }

    // 目标点同时写进根黑板，供 AchieveGoalPose 判断是否到达。
    // 用 rootBlackboard() 而不是 config().blackboard：本节点在 CaptureCenterArea
    // 子树的黑板下，Blackboard::set 只写自己那一层，写在子树上别处读不到。
    // 与上次相同的目标不会再发消息，但黑板里的"当前目标"仍然要写。
    auto* blackboard = config().blackboard->rootBlackboard();
    blackboard->set(kGoalXKey, x);
    blackboard->set(kGoalYKey, y);

    // 目标点与上次相同时不会重复发布，这里同样算成功：
    // "目标点已经在那儿了"不是失败。
    monitor_->sendGoalPose(x, y);
    return BT::NodeStatus::SUCCESS;
  }

private:
  // 写入根黑板的键名，与 AchieveGoalPose 读取的一致，改名要同时改两处。
  static constexpr const char* kGoalXKey = "goal_x";
  static constexpr const char* kGoalYKey = "goal_y";

  std::shared_ptr<ROS2Monitor> monitor_;
};
