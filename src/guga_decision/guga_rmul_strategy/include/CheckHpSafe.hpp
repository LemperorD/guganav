#pragma once

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

// 血量是否需要回血：带滞回的双阈值判断，读不到血量时返回 RUNNING 等数据。
//
// 单阈值会在阈值附近来回切：掉到 199 判"要回血"，在补给点补到 201 又判"安全"，
// 于是在回血点附近反复进出分支。这里用两个阈值，并把"当前用哪一档"记在黑板上
// （键名 hp_threshold，写在文件里而不是由 XML 给）：
//   安全时阈值是 enter_hp（默认 200）：低于它才算需要回血；
//   判成需要回血之后阈值换成 exit_hp（默认 400）：要回到这个血量才恢复安全。
// 也就是说"开始回血"和"结束回血"发生在两个不同的血量上，中间不会来回跳。
//
// 这份状态必须跨 tick 记住，而这个节点的每次 tick 都是从零算起的，黑板就是它的
// 存储：条件判断本身无状态，滞回需要一个地方存阈值。
//
// 用 StatefulActionNode 而不是 ConditionNode：读不到血量时要返回 RUNNING
// （"还不知道"和"血量不够"是两回事），而 ConditionNode 不适合返回 RUNNING。
class CheckHpSafe : public BT::StatefulActionNode
{
public:
  CheckHpSafe(const std::string& name, const BT::NodeConfig& config,
              rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(std::move(node)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("HP"),
      BT::InputPort<double>("enter_hp", kEnterDefault,
                            "安全时用：血量低于它算需要回血"),
      BT::InputPort<double>("exit_hp", kExitDefault,
                            "回血中用：血量到它才恢复安全"),
    };
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

    double enter = kEnterDefault;
    double exit = kExitDefault;
    getInput("enter_hp", enter);
    getInput("exit_hp", exit);

    // 黑板上记的是上一轮用的阈值：等于 exit 就说明上一轮判过"需要回血"。
    auto* blackboard = config().blackboard->rootBlackboard();
    double threshold = enter;
    blackboard->get(kThresholdKey, threshold);

    if (hitpoint.value() < threshold) {
      blackboard->set(kThresholdKey, exit);
      // 只在阈值真正换档的那一轮打日志，否则每 tick 一条会刷屏。
      if (threshold != exit && node_) {
        RCLCPP_INFO(node_->get_logger(),
                    "血量 %.0f 低于 %.0f，判为需要回血；回到 %.0f 才恢复安全",
                    hitpoint.value(), threshold, exit);
      }
      return BT::NodeStatus::FAILURE;
    }

    blackboard->set(kThresholdKey, enter);
    if (threshold != enter && node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "血量 %.0f 达到 %.0f，恢复安全；低于 %.0f 才再判需要回血",
                  hitpoint.value(), threshold, enter);
    }
    return BT::NodeStatus::SUCCESS;
  }

  // 黑板上记录"当前阈值"的键名。写在这里而不是由 XML 决定，避免树上改了名字
  // 而节点还在读旧名字；黑板里能看到 hp_threshold 在 200 与 400 之间切换。
  static constexpr const char* kThresholdKey = "hp_threshold";
  static constexpr double kEnterDefault = 200.0;
  static constexpr double kExitDefault = 400.0;

  // 仅用于打日志，判断本身不依赖它。
  rclcpp::Node::SharedPtr node_;
};
