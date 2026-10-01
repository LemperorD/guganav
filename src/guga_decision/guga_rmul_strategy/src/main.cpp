#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

#include "CheckGreaterThan200.hpp"
#include "GoalPoseSender.hpp"
#include "ROS2Monitor.hpp"
#include "ROS2Wrapper.hpp"
#include "SetGoalPose.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // ===== 1. ROS 2 侧：建立节点并让它开始收数据 =====
  auto monitor = std::make_shared<ROS2Monitor>();
  // 目标点发送器与 monitor 共用同一个节点句柄。
  auto goal_sender = std::make_shared<GoalPoseSender>(monitor);

  // 行为树由主线程周期 tick，订阅回调只能靠另一条线程跑。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(monitor);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // ===== 2. 行为树侧：登记节点 =====
  BT::BehaviorTreeFactory factory;
  // 构造函数需要注入依赖的节点，用 registerBuilder 把依赖递进去。
  factory.registerBuilder<CheckGreaterThan200>(
    "CheckGreaterThan200",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<CheckGreaterThan200>(name, config, monitor);
    });

  factory.registerBuilder<SetGoalPose>(
    "SetGoalPose",
    [goal_sender](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<SetGoalPose>(name, config, goal_sender);
    });

  factory.registerBuilder<ROS2Wrapper>(
    "ROS2Wrapper",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<ROS2Wrapper>(name, config, monitor);
    });

  // ===== 3. 加载行为树 =====
  const std::string share = ament_index_cpp::get_package_share_directory("guga_rmul_strategy");
  auto tree = factory.createTreeFromFile(share + "/config/rmul.xml");

  // v4 用 Groot2Publisher 供 Groot2 实时观察：它监听 1667 端口，
  // Groot2 以实时模式连上后能看到每个 tick 经过哪些节点、各节点返回什么状态。
  // 声明在 tree 之后，保证先于 tree 析构。
  //
  // 端口被占用时（例如上一个实例还没退干净）它抛异常。监控只是观察手段，
  // 不该让决策功能跟着挂掉，所以这里兜住，失败就只警告。
  std::unique_ptr<BT::Groot2Publisher> publisher;
  try {
    publisher = std::make_unique<BT::Groot2Publisher>(tree);
  } catch (const std::exception& e) {
    RCLCPP_WARN(monitor->get_logger(), "Groot2 实时监控未启用: %s", e.what());
  }

  // 把 ROS 2 节点也放进黑板，节点里可以按 "node" 键取句柄。
  tree.rootBlackboard()->set("node", std::static_pointer_cast<rclcpp::Node>(monitor));

  // ===== 4. 执行 =====
  // v4 里单次 tick 用 tickOnce()（v3 是 tickRoot()）。
  rclcpp::WallRate rate(100);  // 100 Hz，约合 10 ms
  while (rclcpp::ok()) {
    tree.tickOnce();
    rate.sleep();
  }

  // ===== 5. 收尾 =====
  std::cout << "收到裁判数据: " << (monitor->hasData() ? "是" : "否")
            << "，current_hp=" << monitor->currentHp()
            << "，maximum_hp=" << monitor->maximumHp() << std::endl;

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
