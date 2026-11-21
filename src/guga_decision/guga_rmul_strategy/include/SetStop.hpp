#pragma once

#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

// 开关："速度置零"的当前状态，写在根黑板上，由 PublishStop 每 tick 发出去。
//
// 是开关不是请求：置 true 后一直保持，直到某处显式置 false。所以树上的规矩是
// "要动的路径显式放开、要停的路径显式夹住"，两处都必须写清楚：
//   已到位/占上 → SetStop stop="true"    钉在点上
//   还没到位、准备去下一个点 → SetStop stop="false"
//   需要回退（IsSafe 切换）→ SetStop stop="false"
// 为什么不做成"每 tick 重新请求、读完就清"：要停的那些分支往往正被 RUNNING
// 冻住（巡逻、守点），冻住的分支不再被 tick，请求下一 tick 就消失了，等于没停。
//
// 键名与 PublishStop 共用常量，Groot 的黑板里能看到它在开/关。
class SetStop : public BT::SyncActionNode
{
public:
  SetStop(const std::string& name, const BT::NodeConfig& config,
          rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(std::move(node)) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("stop", true, "true = 要求停，false = 放开") };
  }

  // 记录"速度是否置零"的黑板键，与 PublishStop 共用。
  static constexpr const char* kKey = "stop_cmd";

  BT::NodeStatus tick() override
  {
    bool stop = true;
    if (!getInput("stop", stop)) {
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->rootBlackboard()->set(kKey, stop);
    return BT::NodeStatus::SUCCESS;
  }

private:
  // 仅用于打日志，动作本身不依赖它。
  rclcpp::Node::SharedPtr node_;
};
