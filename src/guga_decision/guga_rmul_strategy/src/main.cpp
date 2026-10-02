#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

#include "CheckGreaterThan200.hpp"
#include "CheckHpSafe.hpp"
#include "ShouldEngage.hpp"
#include "ROS2Monitor.hpp"
#include "ROS2Wrapper.hpp"
#include "SetGoalPose.hpp"
#include "AchieveGoalPose.hpp"
#include "Attack.hpp"
#include "Patrol.hpp"
#include "PublishStop.hpp"
#include "SetStop.hpp"
#include "PublishGoal.hpp"
#include "RfidArrived.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // ===== 1. ROS 2 侧：建立节点 =====
  // 收裁判数据和发导航目标都在这个节点上，全包只有这一个 ROS 2 节点。
  auto monitor = std::make_shared<ROS2Monitor>();

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
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<SetGoalPose>(name, config, monitor);
    });

  factory.registerBuilder<ROS2Wrapper>(
    "ROS2Wrapper",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<ROS2Wrapper>(name, config, monitor);
    });

  factory.registerBuilder<SetStop>(
    "SetStop",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<SetStop>(name, config, monitor);
    });

  factory.registerBuilder<PublishStop>(
    "PublishStop",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<PublishStop>(name, config, monitor);
    });

  factory.registerBuilder<CheckHpSafe>(
    "CheckHpSafe",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<CheckHpSafe>(name, config, monitor);
    });

  factory.registerBuilder<ShouldEngage>(
    "ShouldEngage",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<ShouldEngage>(name, config, monitor);
    });

    factory.registerBuilder<Attack>(
    "Attack",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<Attack>(name, config, monitor);
    });

  factory.registerBuilder<AchieveGoalPose>(
    "AchieveGoalPose",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<AchieveGoalPose>(name, config, monitor);
    });

  factory.registerBuilder<PublishGoal>(
    "PublishGoal",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<PublishGoal>(name, config, monitor);
    });

  factory.registerBuilder<RfidArrived>(
    "RfidArrived",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<RfidArrived>(name, config, monitor);
    });

  factory.registerBuilder<Patrol>(
    "Patrol",
    [monitor](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<Patrol>(name, config, monitor);
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

  // 行为树执行记录：写成 Groot2 的 .btlog 格式，供离线分析（scripts/btlog_view.py）
  // 与 Web UI 实时读取。FileLogger2 内部有独立写线程，路径后缀必须是 .btlog。
  // 传空字符串可关闭记录。
  const std::string btlog_path =
    monitor->declare_parameter<std::string>("btlog_path", "/tmp/bt_trace.btlog");
  std::unique_ptr<BT::FileLogger2> file_logger;
  if (!btlog_path.empty()) {
    try {
      file_logger = std::make_unique<BT::FileLogger2>(tree, btlog_path);
      RCLCPP_INFO(monitor->get_logger(), "行为树执行记录: %s", btlog_path.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(monitor->get_logger(), "行为树记录未启用: %s", e.what());
    }
  }

  // ===== 4. 执行 =====
  // 频率做成参数。默认 1 Hz 是为了便于观察：100 Hz 下一次 tick 只占 10 ms 中的
  // 几十微秒，Groot2 的实时视图和日志都看不出先后顺序。要对接 Nav2 的实际节奏
  // 时把 tick_rate_hz 设成 100，对齐它的 bt_loop_duration（默认 10 ms）。
  const double kDefaultTickHz = 1.0;
  const double tick_hz =
    monitor->declare_parameter<double>("tick_rate_hz", kDefaultTickHz);
  if (tick_hz <= 0.0) {
    RCLCPP_WARN(monitor->get_logger(), "tick_rate_hz 不是正数，回退到 %.1f Hz",
                kDefaultTickHz);
  }
  const double effective_hz = (tick_hz > 0.0) ? tick_hz : kDefaultTickHz;
  RCLCPP_INFO(monitor->get_logger(), "行为树 tick 频率: %.2f Hz", effective_hz);

  rclcpp::WallRate rate(effective_hz);
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
