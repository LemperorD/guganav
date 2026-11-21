#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "ROS2Monitor.hpp"

// 在目标点附近按给定的几个偏移轮流巡逻，直到 RFID 触发为止。
//
// 巡逻点 = 当前目标点（根黑板的 goal_x / goal_y）+ 第 i 个偏移。基准点只读不写：
// 写回的话下一轮的偏移会叠在上一个巡逻点上，越走越远。
//
// 返回值刻意只有 RUNNING 和 FAILURE，没有 SUCCESS：
//   RUNNING —— 一直在巡逻。外面是 Sequence{AchieveGoalPose, SearchRfid}，RUNNING
//              会让这层 Sequence 停在第二个孩子上，于是 AchieveGoalPose 不再每
//              tick 重算、SetGoalPose 也不会再抢目标；被"没到位"分支拽回目标点
//              的问题就是靠这个避免的。离开巡逻只有一个出口：RFID 触发，由同级
//              的 RfidArrived 返回 SUCCESS 结束整支。所以这里一旦返回 SUCCESS，
//              RUNNING 就会放开、机器人立刻被拽回目标点，巡逻只能走完一个点。
//   FAILURE —— 走完 max_rounds 圈还没触发（只在配了上限时出现），让上层改走别的
//              分支（当前是重新发目标点，回到目标点再确认一次）。
//
// 每个点都等"进入容差"或"超时"才换下一个：不能每 tick 重发同一条 goal_pose，
// 订阅端每收到一条都当成新目标、会中止正在执行的导航；而巡逻点也可能落在障碍里
// 根本到不了，所以超时是必需的，不是优化。
class Patrol : public BT::StatefulActionNode {
public:
  Patrol(const std::string& name, const BT::NodeConfig& config,
         std::shared_ptr<ROS2Monitor> monitor)
      : BT::StatefulActionNode(name, config), monitor_(std::move(monitor)) {
  }

  static BT::PortsList providedPorts() {
    // 向量端口用分号分隔："0.2;0.2;-0.2;-0.2"。写逗号不会报错，会被当成一个数，
    // 所以下面显式校验元素个数。
    return {
      BT::InputPort<std::vector<double>>("xs", "各巡逻点相对目标点的 x 偏移"),
      BT::InputPort<std::vector<double>>("ys", "各巡逻点相对目标点的 y 偏移"),
      BT::InputPort<double>("tolerance", 0.20, "认为到达该巡逻点的距离（米）"),
      BT::InputPort<double>("timeout", 4.0, "单个巡逻点最多等多久（秒），超时就换下一个"),
      BT::InputPort<int>("max_rounds", 0, "走完几圈仍没触发就放弃；0 表示不限"),
    };
  }

  BT::NodeStatus onStart() override { return nextStep(); }

  BT::NodeStatus onRunning() override {
    if (!commanded_) {
      // 没下发成功过（上一轮读黑板失败）就再试一次，不要空转。
      return nextStep();
    }
    if (reached() || timedOut()) {
      return nextStep();
    }
    return BT::NodeStatus::RUNNING;
  }

  // 被上层中止（例如敌人出现切进交战分支）时不改状态：索引与圈数留着，下次进来
  // 从下一个点接着走，不重复覆盖已经走过的方向。
  void onHalted() override {}

private:
  // 读端口与黑板、下发下一个巡逻点。第一次调用下发第 0 个点。
  BT::NodeStatus nextStep()
  {
    if (!monitor_) {
      return BT::NodeStatus::FAILURE;
    }
    if (!readPorts() || !readBase()) {
      return BT::NodeStatus::FAILURE;
    }

    if (commanded_) {
      ++index_;
      if (index_ >= static_cast<int>(xs_.size())) {
        index_ = 0;
        ++rounds_;
        RCLCPP_INFO(monitor_->get_logger(),
                    "巡逻走完第 %d 圈，仍未收到 RFID，重新开始", rounds_);
        if (max_rounds_ > 0 && rounds_ >= max_rounds_) {
          RCLCPP_WARN(monitor_->get_logger(), "巡逻 %d 圈未触发 RFID，放弃本轮搜索",
                      rounds_);
          return BT::NodeStatus::FAILURE;
        }
      }
    }

    target_x_ = base_x_ + xs_[index_];
    target_y_ = base_y_ + ys_[index_];

    // force：同一个点下一圈还会再发，去重会把它挡掉。
    monitor_->sendGoalPose(target_x_, target_y_, /*force=*/true);
    command_time_ = monitor_->now();
    commanded_ = true;

    RCLCPP_INFO(monitor_->get_logger(),
                "巡逻点 %d/%zu：(%.2f, %.2f)，即目标点 (%.2f, %.2f) 加偏移 "
                "(%.2f, %.2f)",
                index_ + 1, xs_.size(), target_x_, target_y_, base_x_, base_y_,
                xs_[index_], ys_[index_]);
    return BT::NodeStatus::RUNNING;
  }

  bool readPorts()
  {
    std::vector<double> xs;
    std::vector<double> ys;
    if (!getInput("xs", xs) || !getInput("ys", ys)) {
      RCLCPP_ERROR_THROTTLE(monitor_->get_logger(), *monitor_->get_clock(), 5000,
                            "Patrol 缺少 xs/ys 端口");
      return false;
    }
    if (xs.empty() || xs.size() != ys.size()) {
      // 分号写成分号以外的分隔符时，整串会被解析成"一个数"，这里挡住这种静默退化。
      RCLCPP_ERROR_THROTTLE(
        monitor_->get_logger(), *monitor_->get_clock(), 5000,
        "Patrol 的 xs/ys 不合法：xs %zu 个、ys %zu 个；多个数之间要用分号分隔",
        xs.size(), ys.size());
      return false;
    }
    xs_ = std::move(xs);
    ys_ = std::move(ys);

    getInput("tolerance", tolerance_);
    getInput("timeout", timeout_);
    getInput("max_rounds", max_rounds_);
    return true;
  }

  bool readBase()
  {
    auto* blackboard = config().blackboard->rootBlackboard();
    if (!blackboard->get("goal_x", base_x_) || !blackboard->get("goal_y", base_y_)) {
      RCLCPP_ERROR_THROTTLE(monitor_->get_logger(), *monitor_->get_clock(), 5000,
                            "黑板上没有 goal_x/goal_y，无法生成巡逻点"
                            "（本节点要排在 SetGoalPose 之后）");
      return false;
    }
    return true;
  }

  bool reached() const
  {
    auto* blackboard = config().blackboard->rootBlackboard();
    double x = 0.0;
    double y = 0.0;
    if (!blackboard->get("robot_x", x) || !blackboard->get("robot_y", y)) {
      return false;
    }
    return std::hypot(x - target_x_, y - target_y_) <= tolerance_;
  }

  bool timedOut() const
  {
    return (monitor_->now() - command_time_).seconds() >= timeout_;
  }

  std::shared_ptr<ROS2Monitor> monitor_;

  // 巡逻点序列（端口给的是相对目标点的偏移）
  std::vector<double> xs_;
  std::vector<double> ys_;
  double tolerance_{0.20};
  double timeout_{4.0};
  int max_rounds_{0};

  // 基准点 = 当前目标点
  double base_x_{0.0};
  double base_y_{0.0};

  int index_{0};
  int rounds_{0};
  bool commanded_{false};
  double target_x_{0.0};
  double target_y_{0.0};
  rclcpp::Time command_time_;
};
