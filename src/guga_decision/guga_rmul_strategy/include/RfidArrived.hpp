#pragma once

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

// RFID 到位判定：此刻是否压着指定的增益点。
//
// 读的是输入端口，由树上接到黑板键上（`{rfid_center}` 或 `{rfid_base}`）。不自己
// 读键名，因为占点看中心增益点、回家看基地增益点，同一个判定要用在两处。
//
// 判的是"当前状态"，不做"触发过就算"的累积：累积起来就看不出机器人被撞开，
// 也就不会再去占。移动中压过卡的那一瞬间确实可能落在两次 tick 之间被漏掉，但
// 巡逻本来就会反复经过同样的点，漏一次下一圈还会再压到；而在目标点上是停着不动的，
// 卡持续触发，不存在采样窗口问题。
//
// 无条件节点，占 IfThenElse 的条件槽；返回 SUCCESS 表示"确认到位"。
class RfidArrived : public BT::ConditionNode {
public:
  RfidArrived(const std::string& name, const BT::NodeConfig& config,
              rclcpp::Node::SharedPtr node)
      : BT::ConditionNode(name, config), node_(std::move(node)) {
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("rfid", "该增益点是否触发过")};
  }

  BT::NodeStatus tick() override {
    const auto rfid = getInput<bool>("rfid");
    if (!rfid) {
      // 端口没接或键不存在：属于树的接线问题，报错而不是静默返回 FAILURE，
      // 否则会表现为"一直在找 RFID"。
      if (node_) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                              "RfidArrived 没读到 rfid 端口（应接 {rfid_center} "
                              "或 {rfid_base}）");
      }
      return BT::NodeStatus::FAILURE;
    }

    if (!rfid.value()) {
      return BT::NodeStatus::FAILURE;
    }

    if (node_) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "RFID 已触发，判定到位");
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  // 仅用于打日志，判定本身不依赖它。
  rclcpp::Node::SharedPtr node_;
};
