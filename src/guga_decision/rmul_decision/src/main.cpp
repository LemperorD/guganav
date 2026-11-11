#include <behaviortree_cpp/blackboard.h>
#include "rmul_decision/condition_node/checkHP.hpp"
#include "rmul_decision/condition_node/checkTime.hpp"
#include "rmul_decision/condition_node/IsOccupationsafe.hpp"
#include "rmul_decision/condition_node/IsRevival.hpp"
#include "rmul_decision/condition_node/IsGameStart.hpp"
#include "rmul_decision/action_node/changelowerstate.hpp"
#include "rmul_decision/action_node/changehigherstate.hpp"
#include "rmul_decision/action_node/GotoPoints.hpp"
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include <guga_decision_common/decision_input_hub.hpp>
#include <guga_decision_common/goal_controller.hpp>
#include <guga_decision_common/ros2_input_node.hpp>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto blackboard = BT::Blackboard::create();
  auto input_hub = std::make_shared<guga_decision_common::DecisionInputHub>(
      "rmul_decision_input");
  const std::string goal_topic = input_hub->declare_parameter<std::string>(
      "goal_pose_topic", "goal_pose");
  auto goal_controller = std::make_shared<guga_decision_common::GoalController>(
      *input_hub, goal_topic, input_hub->frameId());
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<guga_decision_common::Ros2InputNode>(
      "Ros2Input",
      [input_hub](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<guga_decision_common::Ros2InputNode>(
            name, config, input_hub);
      });
  factory.registerNodeType<ChangeHigherState>("ChangeHigherState");
  factory.registerNodeType<ChangeLowerState>("ChangeLowerState");
  factory.registerNodeType<CheckHP>("CheckHP");
  factory.registerNodeType<CheckTime>("CheckTime");
  factory.registerNodeType<IsGameStart>("IsGameStart");
  factory.registerNodeType<IsOccupationsafe>("IsOccupationsafe");
  factory.registerNodeType<IsRevival>("IsRevivedAndDead");
  factory.registerBuilder<PointSelector>(
      "GotoPoints", [input_hub, goal_controller](const std::string& name,
                                                 const BT::NodeConfig& config) {
        return std::make_unique<PointSelector>(name, config, input_hub,
                                               goal_controller);
      });
  auto tree = factory.createTreeFromFile("../bt_trees/BT.xml", blackboard);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(input_hub);
  std::thread spin_thread([&executor]() { executor.spin(); });
  tree.tickWhileRunning();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
