#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

// 到达判定：机器人当前位置与当前目标点的距离小于等于容差，算到达。
//
// 四个值都在节点内部从黑板读，不声明输入端口：
//   robot_x / robot_y  ROS2Wrapper 每个 tick 写入，map 系坐标
//   goal_x  / goal_y   SetGoalPose 下发目标点时写入
//
// 读的是根黑板（rootBlackboard）而不是 config().blackboard：SetGoalPose 在
// CaptureCenterArea 子树里，本节点在 NavigateAndTrigger 子树里，两棵子树各有
// 自己的黑板对象，而 Blackboard::set 只写自己那一层，不会往上冒泡。统一用根
// 黑板交换数据，才不依赖树上谁套在谁里面。
//
// 位置来自 TF，定位未就绪时 ROS2Wrapper 写的是 NaN。判断写成 !(距离 <= 容差)
// 而不是 距离 > 容差：NaN 参与的任何比较都为假，用后者会把"不知道在哪"判成
// 到达，用前者则判成未到达。
//
// 是条件节点，所以它在树上占 IfThenElse 的条件槽：到达返回 SUCCESS，
// 未到达返回 FAILURE。
class AchieveGoalPose : public BT::ConditionNode {
public:
  AchieveGoalPose(const std::string& name, const BT::NodeConfig& config,
                  rclcpp::Node::SharedPtr node)
      : BT::ConditionNode(name, config), node_(std::move(node)) {
  }

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override {
    auto* blackboard = config().blackboard->rootBlackboard();

    double robot_x = 0.0;
    double robot_y = 0.0;
    double goal_x = 0.0;
    double goal_y = 0.0;
    if (!blackboard->get("robot_x", robot_x) ||
        !blackboard->get("robot_y", robot_y) ||
        !blackboard->get("goal_x", goal_x) ||
        !blackboard->get("goal_y", goal_y)) {
      // 键不存在说明本节点跑在写入方前面了，是树的结构问题，不是"没到达"。
      // 两者要分开：这里报错，不静默当成未到达。
      if (node_) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "黑板上缺少 robot_x/robot_y 或 goal_x/goal_y，无法判断是否到达");
      }
      return BT::NodeStatus::FAILURE;
    }

    const double distance = std::hypot(robot_x - goal_x, robot_y - goal_y);

    if (!(distance <= kArriveToleranceM)) {
      if (node_) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "未到达：距 (%.2f, %.2f) 还有 %.2f m，当前位置 "
                             "(%.2f, %.2f)",
                             goal_x, goal_y, distance, robot_x, robot_y);
      }
      return BT::NodeStatus::FAILURE;
    }

    if (node_) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "已到达 (%.2f, %.2f)，距离 %.3f m", goal_x, goal_y,
                           distance);
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  // 到达容差，单位米。取 0.15 与 controller 的 xy_goal_tolerance 一致：导航就是在
  // 这个距离内认为到点并停下，不再往前凑，所以判定比它更严就会永远等不到（表现为
  // 一直在重发目标点）。真正"确实压到增益点上"由 RFID 判定，这里只是坐标门槛。
  // 写成常量而不是端口，避免在树上被改错；要调参再改成 InputPort。
  static constexpr double kArriveToleranceM = 0.15;

  // 仅用于打日志，判断本身不依赖它。
  rclcpp::Node::SharedPtr node_;
};
