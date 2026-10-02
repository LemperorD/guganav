#pragma once

#include <limits>
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
//
// 当前写入：HP(double)、ammo(int)、enemy_count(int)，
// robot_x、robot_y(double，map 系下的机器人位置，定位不可用时为 NaN)，
// rfid_base、rfid_center(bool，是否正压着对应的 RFID 增益点，离开即回到 false)。
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

    // 机器人位置：map 系下的坐标，由 TF 变换而来（里程计给的是 odom 系的）。
    // 查不到时写 NaN 而不是 0：目标点常常就是 (0, 0)，写 0 会被到达判断误判成
    // "已经到家"；NaN 参与的任何比较都是假，即"还没到"。要区分"没到家"和
    // "不知道在哪"，判断前先看 std::isfinite()。
    double robot_x = std::numeric_limits<double>::quiet_NaN();
    double robot_y = robot_x;
    // 查不到时 lookupRobotPose 不修改入参，两个值保持 NaN。
    monitor_->lookupRobotPose(robot_x, robot_y);
    blackboard->set(kRobotXKey, robot_x);
    blackboard->set(kRobotYKey, robot_y);

    // RFID 增益点：压到卡上为 true，离开回到 false。只给当前状态，不做"触发过
    // 就算"的累积——累积起来就看不出机器人被撞开，不会再回去占。
    blackboard->set(kRfidBaseKey, monitor_->baseGainPoint());
    blackboard->set(kRfidCenterKey, monitor_->centerGainPoint());
    if (!monitor_->hasRfidData()) {
      // 没收到过说明话题没通（名字不对或驱动没发），此时两位恒为 false——
      // 和"没压到卡上"在数据上分不出来，只能靠这条日志区分。
      RCLCPP_WARN_THROTTLE(
        monitor_->get_logger(), *monitor_->get_clock(), 5000,
        "尚未收到 RFID 状态，增益点判定会一直是 false");
    }

    return BT::NodeStatus::SUCCESS;
  }

  // 写入黑板的键名，写死在这里而不是由 XML 决定。
  static constexpr const char* kHpKey = "HP";
  static constexpr const char* kAmmoKey = "ammo";
  static constexpr const char* kEnemyCountKey = "enemy_count";
  static constexpr const char* kRobotXKey = "robot_x";
  static constexpr const char* kRobotYKey = "robot_y";
  static constexpr const char* kRfidBaseKey = "rfid_base";
  static constexpr const char* kRfidCenterKey = "rfid_center";

  std::shared_ptr<ROS2Monitor> monitor_;
};
