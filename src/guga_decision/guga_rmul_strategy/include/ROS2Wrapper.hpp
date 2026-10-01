#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "ROS2Monitor.hpp"

// 行为树侧：持有 ROS 2 节点，每次 tick 把外部数据搬进黑板。
// 它自己不做判断，判断交给下游节点，这样数据来源和决策逻辑各自独立。
//
// 继承 StatefulActionNode 而不是 SyncActionNode：数据还没到时要返回 RUNNING
// 让树停在原处等，而 SyncActionNode 只允许 SUCCESS/FAILURE，返回 RUNNING 会被
// BT.CPP 抛 "MUST never return RUNNING"。
//
// 不声明输出端口：黑板键名固定在本文件里，树上只写 <ROS2Wrapper/>。写了哪些键
// 见下面的常量，树上用 {键名} 引用。改键名就要同步改树，写错不会有加载期报错。
class ROS2Wrapper : public BT::StatefulActionNode
{
public:
  ROS2Wrapper(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<ROS2Monitor> monitor)
  : BT::StatefulActionNode(name, config), monitor_(std::move(monitor)) {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  // 这个节点是无状态的轮询，onStart 和 onRunning 做同一件事。
  BT::NodeStatus onStart() override { return publishInputs(); }
  BT::NodeStatus onRunning() override { return publishInputs(); }
  void onHalted() override {}

private:
  BT::NodeStatus publishInputs()
  {
    if (!monitor_) {
      // 依赖没注入进来属于配置错误，显式失败而不是静默返回 0。
      return BT::NodeStatus::FAILURE;
    }

    // 数据还没到时返回 RUNNING 而不是 FAILURE：
    // "还不知道"和"不安全"是两回事，FAILURE 会让上游立刻走回退分支。
    // 血量、发弹量来自同一条裁判消息，所以 hasData() 为真时三者都有效。
    if (!monitor_->hasData()) {
      RCLCPP_WARN_THROTTLE(
        monitor_->get_logger(), *monitor_->get_clock(), 2000,
        "尚未收到裁判数据，暂不做安全性判定");
      return BT::NodeStatus::RUNNING;
    }

    // Blackboard::set 返回 void，类型不匹配时抛异常，没有可检查的返回值。
    auto blackboard = config().blackboard;
    blackboard->set(kHpKey, static_cast<double>(monitor_->currentHp()));
    blackboard->set(kAmmoKey, static_cast<int>(monitor_->projectileAllowance()));
    // 敌情来自视觉，与裁判消息不是同一路，但同样按最新值写入。
    blackboard->set(kEnemyCountKey, static_cast<int>(monitor_->enemyCount()));
    return BT::NodeStatus::SUCCESS;
  }

  // 写入黑板的键名，写死在这里而不是由 XML 决定。
  static constexpr const char* kHpKey = "HP";
  static constexpr const char* kAmmoKey = "ammo";
  static constexpr const char* kEnemyCountKey = "enemy_count";

  std::shared_ptr<ROS2Monitor> monitor_;
};
