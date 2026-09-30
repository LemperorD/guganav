#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "CheckGreaterThan200.hpp"
#include "ROS2Monitor.hpp"
#include "ROS2Wrapper.hpp"
#include "SetGoalPose.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // ===== 1. ROS 2 侧：建立节点并让它开始收数据 =====
  auto monitor = std::make_shared<ROS2Monitor>();

  // tickRootWhileRunning() 是阻塞调用，订阅回调只能靠另一条线程跑。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(monitor);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // ===== 2. 行为树侧：登记节点 =====
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CheckGreaterThan200>("CheckGreaterThan200");
  factory.registerNodeType<SetGoalPose>("SetGoalPose");

  // ROS2Wrapper 的构造函数多一个参数，所以用 registerBuilder 把依赖递进去。
  factory.registerBuilder<ROS2Wrapper>(
    "ROS2Wrapper",
    [monitor](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<ROS2Wrapper>(name, config, monitor);
    });

  // ===== 3. 加载行为树 =====
  const std::string share = ament_index_cpp::get_package_share_directory("guga_rmul_strategy");
  auto tree = factory.createTreeFromFile(share + "/config/rmul.xml");

  // 把 ROS 2 节点也放进黑板。自己建 factory 时需要手动放，
  // 接入 Nav2 之后这一步由 bt_navigator 完成，节点里用同样的键名取即可。
  tree.rootBlackboard()->set("node", std::static_pointer_cast<rclcpp::Node>(monitor));

  // ===== 4. 执行 =====
  // bt_navigator 是以固定周期反复 tick 行为树的（bt_loop_duration 默认 10 ms）。
  // tickRootWhileRunning() 只在树返回非 RUNNING 时才停，一次就退出了，不适合持续运行的场景。
  rclcpp::WallRate rate(100);  // 100 Hz，约合 10 ms，对齐 Nav2 的 bt_loop_duration
  while (rclcpp::ok()) {
    tree.tickRoot();
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
