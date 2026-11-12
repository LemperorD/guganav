#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <guga_decision_common/decision_input_hub.hpp>
#include <guga_decision_common/goal_controller.hpp>
#include <guga_decision_common/ros2_input_node.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rmul_decision/decision_type.hpp"
#include "rmul_decision/action_node/GotoPoints.hpp"
#include "rmul_decision/action_node/changehigherstate.hpp"
#include "rmul_decision/action_node/changelowerstate.hpp"
#include "rmul_decision/condition_node/IsGameStart.hpp"
#include "rmul_decision/condition_node/IsOccupationsafe.hpp"
#include "rmul_decision/condition_node/IsRevival.hpp"
#include "rmul_decision/condition_node/checkHP.hpp"
#include "rmul_decision/condition_node/checkTime.hpp"

namespace {

template<typename T>
void appendBlackboardValue(
    const BT::Blackboard& blackboard, const std::string& key,
    std::vector<diagnostic_msgs::msg::KeyValue>& values) {
  T value{};
  if (!blackboard.get<T>(key, value)) {
    return;
  }

  std::ostringstream shown;
  shown << value;
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = shown.str();
  values.push_back(std::move(item));
}

void publishBlackboard(
    const BT::Blackboard::Ptr& blackboard,
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr&
        publisher) {
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = node->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "rmul_decision blackboard";
  status.message = "BehaviorTree.CPP blackboard snapshot";

  appendBlackboardValue<int>(*blackboard, "game_progress", status.values);
  appendBlackboardValue<double>(*blackboard, "HP", status.values);
  appendBlackboardValue<int>(*blackboard, "destination", status.values);
  appendBlackboardValue<std::string>(
      *blackboard, "global_state_", status.values);
  appendBlackboardValue<double>(
      *blackboard, "occupation_time", status.values);
  appendBlackboardValue<double>(*blackboard, "stay_time", status.values);
  appendBlackboardValue<int>(*blackboard, "HP_sign", status.values);
  appendBlackboardValue<int>(*blackboard, "ammo", status.values);
  appendBlackboardValue<double>(*blackboard, "robot_x", status.values);
  appendBlackboardValue<double>(*blackboard, "robot_y", status.values);
  appendBlackboardValue<bool>(*blackboard, "pose_valid", status.values);

  message.status.push_back(std::move(status));
  publisher->publish(message);
}

const char* statusName(BT::NodeStatus status) {
  switch (status) {
    case BT::NodeStatus::IDLE:
      return "IDLE";
    case BT::NodeStatus::RUNNING:
      return "RUNNING";
    case BT::NodeStatus::SUCCESS:
      return "SUCCESS";
    case BT::NodeStatus::FAILURE:
      return "FAILURE";
    default:
      return "UNKNOWN";
  }
}

struct TickStatusCapture
{
  std::unordered_map<std::uint16_t, BT::NodeStatus> executed;
};

class TickStatusCaptureLogger final : public BT::StatusChangeLogger
{
public:
  TickStatusCaptureLogger(
      BT::TreeNode* root, TickStatusCapture& capture)
  : BT::StatusChangeLogger(root), capture_(capture) {
    // 状态变化只覆盖真正发生迁移的 tick；TickMonitor 会记录每轮实际执行的
    // 节点及其返回值，即使状态和上一轮相同，也能让 UI 展示本轮完整路径。
    applyRecursiveVisitor(root, [this](BT::TreeNode* tree_node) {
      const std::uint16_t uid = tree_node->UID();
      tree_node->setTickMonitorCallback(
          [this, uid](
              BT::TreeNode&, BT::NodeStatus status,
              std::chrono::microseconds) {
            capture_.executed[uid] = status;
          });
      tree_node->setPostTickFunction(
          [this, uid](BT::TreeNode&, BT::NodeStatus status) {
            capture_.executed[uid] = status;
            return status;
          });
    });
  }

  void callback(
      BT::Duration, const BT::TreeNode& tree_node, BT::NodeStatus,
      BT::NodeStatus status) override {
    // 终态之后 BT.CPP 可能立刻把节点重置为 IDLE。这里保留本 tick
    // 最后一个非 IDLE 状态，才能看到 SwitchState 这类瞬时节点。
    if (status != BT::NodeStatus::IDLE) {
      capture_.executed[tree_node.UID()] = status;
    }
  }

  void flush() override {}

private:
  TickStatusCapture& capture_;
};

void publishNodeStatus(
    const BT::Tree& tree, const rclcpp::Node::SharedPtr& node,
    const TickStatusCapture& capture,
    const rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr&
        publisher) {
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = node->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "rmul_decision nodes";
  status.message = "BehaviorTree.CPP node status heartbeat";

  std::unordered_set<std::uint16_t> seen;
  for (const auto& subtree : tree.subtrees) {
    for (const auto& tree_node : subtree->nodes) {
      const std::uint16_t uid = tree_node->UID();
      if (!seen.insert(uid).second) {
        continue;
      }
      diagnostic_msgs::msg::KeyValue item;
      item.key = std::to_string(uid);
      const auto executed = capture.executed.find(uid);
      item.value = statusName(
          executed != capture.executed.end()
          ? executed->second
          : tree_node->status());
      status.values.push_back(std::move(item));
    }
  }

  message.status.push_back(std::move(status));
  publisher->publish(message);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto blackboard = BT::Blackboard::create();
  blackboard->set<int>("game_progress", -1);
  blackboard->set<double>("HP", 400);
  blackboard->set<int>("destination", 1);
  blackboard->set<std::string>("global_state_", "Suppress");
  blackboard->set<double>("occupation_time", 0.0);
  blackboard->set<double>("stay_time", 0.0);
  blackboard->set<int>("HP_sign", 0);

  auto input_hub = std::make_shared<guga_decision_common::DecisionInputHub>(
      "rmul_decision_input");
  blackboard->set<rclcpp::Node::SharedPtr>(
      "node", std::static_pointer_cast<rclcpp::Node>(input_hub));
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

  const std::string default_bt_path =
      ament_index_cpp::get_package_share_directory("rmul_decision") +
      "/bt_trees/BT.xml";
  const std::string bt_path = input_hub->declare_parameter<std::string>(
      "bt_path", default_bt_path);
  auto tree = factory.createTreeFromFile(bt_path, blackboard);

  const std::string btlog_path = input_hub->declare_parameter<std::string>(
      "btlog_path", "/tmp/bt_trace.btlog");
  std::unique_ptr<BT::FileLogger2> file_logger;
  if (!btlog_path.empty()) {
    try {
      file_logger = std::make_unique<BT::FileLogger2>(tree, btlog_path);
      RCLCPP_INFO(input_hub->get_logger(), "行为树执行记录: %s",
                  btlog_path.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(input_hub->get_logger(), "行为树记录未启用: %s", e.what());
    }
  }

  const double default_tick_hz = 1.0;
  const double tick_hz = input_hub->declare_parameter<double>(
      "tick_rate_hz", default_tick_hz);
  const double effective_tick_hz = tick_hz > 0.0 ? tick_hz : default_tick_hz;
  if (tick_hz <= 0.0) {
    RCLCPP_WARN(input_hub->get_logger(),
                "tick_rate_hz 不是正数，回退到 %.1f Hz", default_tick_hz);
  }
  RCLCPP_INFO(input_hub->get_logger(), "行为树: %s，tick %.2f Hz",
              bt_path.c_str(), effective_tick_hz);

  auto blackboard_pub =
      input_hub->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "/btview/blackboard", 10);
  auto node_status_pub =
      input_hub->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "/btview/node_status", 10);
  TickStatusCapture tick_capture;
  TickStatusCaptureLogger tick_capture_logger(tree.rootNode(), tick_capture);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(input_hub);
  std::thread spin_thread([&executor]() { executor.spin(); });
  rclcpp::WallRate rate(effective_tick_hz);
  while (rclcpp::ok()) {
    tick_capture.executed.clear();
    tree.tickOnce();
    publishBlackboard(tree.rootBlackboard(), input_hub, blackboard_pub);
    publishNodeStatus(tree, input_hub, tick_capture, node_status_pub);
    rate.sleep();
  }
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
