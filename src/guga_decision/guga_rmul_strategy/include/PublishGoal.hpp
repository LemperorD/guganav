#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "ROS2Monitor.hpp"

// 把当前目标点发布给导航。
//
// 目标点从根黑板读（goal_x / goal_y，由 SetGoalPose 写入），不声明输入端口：
// 本节点在 NavigateAndTrigger 子树里，写入方 SetGoalPose 在 CaptureCenterArea
// 子树里，两棵子树各有自己的黑板对象，Blackboard::set 只写自己那一层，
// 只有根黑板是两边共用的。
//
// 为什么不是"发一次就够"：goal_pose 是即发即忘的话题，订阅端 bt_navigator 用
// BEST_EFFORT 订阅，丢了不会重传；Nav2 比本节点晚启动、或者导航中途放弃了目标，
// 都会让"已经发过"变成事实上没发。所以这里按 kRepublishPeriodS 周期重发。
//
// 为什么不能每个 tick 都发：bt_navigator 收到 goal_pose 就当成一个新目标
// （navigate_to_pose.cpp 的 onGoalPoseReceived 直接调 self_client_->async_send_goal），
// 正在执行的目标会被中止、重新规划。按 tick 频率发等于让导航一直重启，
// 所以重发周期必须远大于 tick 周期。
class PublishGoal : public BT::SyncActionNode {
public:
  PublishGoal(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<ROS2Monitor> monitor)
      : BT::SyncActionNode(name, config), monitor_(std::move(monitor)) {
  }

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override {
    if (!monitor_) {
      // 依赖没注入进来属于配置错误，显式失败而不是静默什么都不做。
      return BT::NodeStatus::FAILURE;
    }

    auto* blackboard = config().blackboard->rootBlackboard();
    double goal_x = 0.0;
    double goal_y = 0.0;
    if (!blackboard->get("goal_x", goal_x) ||
        !blackboard->get("goal_y", goal_y)) {
      // 说明 SetGoalPose 还没执行过，是树的结构问题，不是"目标点已知但发不出去"。
      RCLCPP_ERROR_THROTTLE(monitor_->get_logger(), *monitor_->get_clock(), 5000,
                            "黑板上没有 goal_x/goal_y，无法发布目标点"
                            "（本节点要排在 SetGoalPose 之后）");
      return BT::NodeStatus::FAILURE;
    }

    const auto now = monitor_->now();
    const bool goal_changed =
        !has_published_ || goal_x != last_goal_x_ || goal_y != last_goal_y_;

    // 目标点没变，又还没到重发周期：这个 tick 什么都不发。
    // 这是常态——树每个 tick 都会走到这里，真发消息的是少数几个 tick。
    if (!goal_changed &&
        (now - last_publish_time_).seconds() < kRepublishPeriodS) {
      return BT::NodeStatus::SUCCESS;
    }

    // 目标点变了走正常发布，由 sendGoalPose 自己判断要不要发（同一个点不会重复）；
    // 没变就是到点重发，这时必须 force，否则会被去重挡掉。
    const bool published =
        monitor_->sendGoalPose(goal_x, goal_y, /*force=*/!goal_changed);

    // 无论这次有没有真的发出消息，都要记时间：目标点可能刚被 SetGoalPose 发过，
    // 那也算"刚下发不久"，重发周期从那时算起。
    last_goal_x_ = goal_x;
    last_goal_y_ = goal_y;
    last_publish_time_ = now;
    has_published_ = true;

    if (!published && goal_changed) {
      RCLCPP_INFO_THROTTLE(monitor_->get_logger(), *monitor_->get_clock(), 5000,
                           "目标点 (%.2f, %.2f) 刚下发过，不重复发布", goal_x,
                           goal_y);
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  // 重发周期，单位秒。取值要兼顾两头：太短会把 Nav2 正在执行的目标反复中止，
  // 太长则丢一条消息后要等很久才恢复。2 秒是起点，实车按导航节奏再调。
  static constexpr double kRepublishPeriodS = 2.0;

  std::shared_ptr<ROS2Monitor> monitor_;

  // 本节点上次处理的目标点与时刻，用来判断"该不该重发"。
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  rclcpp::Time last_publish_time_;
  bool has_published_{false};
};
