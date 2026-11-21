#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

// 条件判断：血量是否高于阈值,高返回 SUCCESS,低返回 FAILURE,读不到值返回 RUNNING。
//
// SyncActionNode 返回 RUNNING 会被 BT.CPP 抛异常。
class CheckGreaterThan200 : public BT::StatefulActionNode
{
public:
  CheckGreaterThan200(const std::string& name, const BT::NodeConfig& config,
                      rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(std::move(node)) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("HP") };
  }

  BT::NodeStatus onStart() override { return judge(); }
  BT::NodeStatus onRunning() override { return judge(); }
  void onHalted() override {}

private:
  BT::NodeStatus judge()
  {
    auto hitpoint = getInput<double>("HP");
    if (!hitpoint) {
      // 取不到值说明上游还没写黑板，或键名对不上。这和"血量不够"是两回事，
      // 不该让树立刻走回退分支，返回 RUNNING 等下一个 tick 再看。
      if (node_) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "读不到黑板的 HP，暂不做安全性判定");
      }
      return BT::NodeStatus::RUNNING;
    }

    return (hitpoint.value() > kThreshold) ? BT::NodeStatus::SUCCESS
                                           : BT::NodeStatus::FAILURE;
  }

  // 安全血量的下限。
  static constexpr double kThreshold = 200.0;

  // 仅用于打日志，判断本身不依赖它。
  rclcpp::Node::SharedPtr node_;
};
