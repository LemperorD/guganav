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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<rclcpp::Node>("decision_node");
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ChangeHigherState>("ChangeHigherState");
  factory.registerNodeType<ChangeLowerState>("ChangeLowerState");
  factory.registerNodeType<CheckHP>("CheckHP");
  factory.registerNodeType<CheckTime>("CheckTime");
  factory.registerNodeType<IsGameStart>("IsGameStart");
  factory.registerNodeType<IsOccupationsafe>("IsOccupationsafe");
  factory.registerNodeType<IsRevival>("IsRevivedAndDead");
  factory.registerNodeType<PointSelector>("GotoPoints");
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
  auto tree = factory.createTreeFromFile("../bt_trees/BT.xml", blackboard);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  std::thread spin_thread([&executor]() { executor.spin(); });
  tree.tickWhileRunning();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}