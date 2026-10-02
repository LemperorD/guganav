#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "ROS2Monitor.hpp"
#include "SetStop.hpp"

// 把黑板上那个"速度置零"开关的状态发出去，每个 tick 一条。
//
// 发的是电平不是事件：订阅端（controller / 底盘侧）据此决定是否把速度压成零，
// 并且可以拿"超时没收到"当失效信号——上游挂了就停，比保持最后一帧速度安全。
//
// 只读不发判断：开关的值由 SetStop 在树上置位，这里不做任何清除，所以它是个
// 锁存的开关，跟树的当前状态一致。
//
// 放在 MainLoop 的 ReactiveSequence 里、Decide 之前，保证每个 tick 都会执行到
// （放 Decide 之后的话，Decide 返回 RUNNING 时它会被跳过，电平就断了）。
class PublishStop : public BT::SyncActionNode
{
public:
  PublishStop(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<ROS2Monitor> monitor)
  : BT::SyncActionNode(name, config), monitor_(std::move(monitor)) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (!monitor_) {
      return BT::NodeStatus::FAILURE;
    }

    bool stop = false;
    config().blackboard->rootBlackboard()->get(SetStop::kKey, stop);
    monitor_->publishStop(stop);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<ROS2Monitor> monitor_;
};
