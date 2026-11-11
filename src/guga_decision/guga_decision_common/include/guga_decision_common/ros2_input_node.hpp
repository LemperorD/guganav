#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "guga_decision_common/decision_input_hub.hpp"

namespace guga_decision_common
{

// BehaviorTree.CPP 的公共 ROS 输入节点。
//
// 使用同一个 DecisionInputHub 的数据，在每次 tick 时把输入写入输出端口。
// 输出端口由树配置，节点不绑定任何固定黑板键。树上的示例：
//
//   <Ros2Input HP="{HP}" ammo="{ammo}" game_progress="{game_progress}"
//              rfid_supply="{rfid_supply}" rfid_center="{rfid_center}"/>
//
// 只有机器人状态是必需输入；比赛、RFID 和视觉数据未到时对应端口仍会写入
// 默认值，同时 *_received 端口会给出明确的可用性标志。
class Ros2InputNode : public BT::StatefulActionNode
{
public:
  Ros2InputNode(
    const std::string& name, const BT::NodeConfig& config,
    std::shared_ptr<DecisionInputHub> input_hub)
  : BT::StatefulActionNode(name, config), input_hub_(std::move(input_hub)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<double>("HP", "当前血量"),
      BT::OutputPort<double>("maximum_hp", "最大血量"),
      BT::OutputPort<int>("ammo", "允许发弹量"),
      BT::OutputPort<int>("heat", "当前枪口热量"),
      BT::OutputPort<int>("heat_limit", "枪口热量上限"),
      BT::OutputPort<int>("cooling_value", "枪口每秒冷却值"),
      BT::OutputPort<bool>("hp_deduced", "最近一次状态是否表示受击"),
      BT::OutputPort<int>("enemy_count", "视野内敌方数量"),
      BT::OutputPort<int>("game_progress", "比赛阶段，4 = 比赛中"),
      BT::OutputPort<int>("stage_remain_time", "当前比赛阶段剩余秒数"),
      BT::OutputPort<bool>("rfid_supply", "是否压住己方补给区"),
      BT::OutputPort<bool>("rfid_center", "是否压住中心控制区"),
      BT::OutputPort<double>("robot_x", "机器人 map 系 x 坐标"),
      BT::OutputPort<double>("robot_y", "机器人 map 系 y 坐标"),
      BT::OutputPort<double>("robot_yaw", "机器人 map 系 yaw"),
      BT::OutputPort<bool>("pose_valid", "TF 位姿是否有效"),
      BT::OutputPort<bool>("robot_status_received", "是否收到过机器人状态"),
      BT::OutputPort<bool>("game_status_received", "是否收到过比赛状态"),
      BT::OutputPort<bool>("rfid_status_received", "是否收到过 RFID 状态"),
      BT::OutputPort<bool>("vision_received", "是否收到过视觉汇总"),
    };
  }

  BT::NodeStatus onStart() override { return writeInputs(); }
  BT::NodeStatus onRunning() override { return writeInputs(); }
  void onHalted() override {}

private:
  template<typename T>
  void writeIfMapped(const std::string& key, const T& value)
  {
    if (config().output_ports.find(key) == config().output_ports.end()) {
      return;
    }

    const auto result = setOutput(key, value);
    if (!result && input_hub_) {
      RCLCPP_ERROR_THROTTLE(
        input_hub_->get_logger(), *input_hub_->get_clock(), 5000,
        "Ros2Input 写入端口 %s 失败: %s", key.c_str(),
        result.error().c_str());
    }
  }

  BT::NodeStatus writeInputs()
  {
    if (!input_hub_) {
      return BT::NodeStatus::FAILURE;
    }

    const auto input = input_hub_->snapshot();
    if (!input.robot.received) {
      RCLCPP_WARN_THROTTLE(
        input_hub_->get_logger(), *input_hub_->get_clock(), 2000,
        "尚未收到机器人状态，Ros2Input 返回 RUNNING");
      return BT::NodeStatus::RUNNING;
    }

    writeIfMapped("HP", static_cast<double>(input.robot.current_hp));
    writeIfMapped("maximum_hp", static_cast<double>(input.robot.maximum_hp));
    writeIfMapped("ammo", static_cast<int>(input.robot.projectile_allowance));
    writeIfMapped("heat", static_cast<int>(input.robot.heat));
    writeIfMapped("heat_limit", static_cast<int>(input.robot.heat_limit));
    writeIfMapped(
      "cooling_value", static_cast<int>(input.robot.cooling_value));
    writeIfMapped("hp_deduced", input.robot.hp_deduced);
    writeIfMapped("enemy_count", static_cast<int>(input.vision.enemy_count));
    writeIfMapped(
      "game_progress", static_cast<int>(input.game.game_progress));
    writeIfMapped(
      "stage_remain_time", static_cast<int>(input.game.stage_remain_time));
    writeIfMapped("rfid_supply", input.rfid.supply_zone);
    writeIfMapped("rfid_center", input.rfid.center_gain_point);

    double robot_x = std::numeric_limits<double>::quiet_NaN();
    double robot_y = std::numeric_limits<double>::quiet_NaN();
    double robot_yaw = std::numeric_limits<double>::quiet_NaN();
    const bool pose_valid =
      input_hub_->lookupRobotPose(robot_x, robot_y, robot_yaw);
    writeIfMapped("robot_x", robot_x);
    writeIfMapped("robot_y", robot_y);
    writeIfMapped("robot_yaw", robot_yaw);
    writeIfMapped("pose_valid", pose_valid);

    writeIfMapped("robot_status_received", input.robot.received);
    writeIfMapped("game_status_received", input.game.received);
    writeIfMapped("rfid_status_received", input.rfid.received);
    writeIfMapped("vision_received", input.vision.received);
    return BT::NodeStatus::SUCCESS;
  }

  std::shared_ptr<DecisionInputHub> input_hub_;
};

}  // namespace guga_decision_common
