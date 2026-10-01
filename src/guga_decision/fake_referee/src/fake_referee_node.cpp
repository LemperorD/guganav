#include "fake_referee/fake_referee_node.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace
{
// RMUL 哨兵满状态（U26 表 3-4）
constexpr std::int64_t kSentryId = 7;
constexpr std::int64_t kSentryMaxHp = 400;
constexpr std::int64_t kSentryHeatLimit = 260;
constexpr std::int64_t kSentryCooling = 30;
constexpr double kDefaultRate = 20.0;
}  // namespace

FakeRefereeNode::FakeRefereeNode()
: rclcpp::Node("fake_referee")
{
  // 话题名与频率。频率在启动时建定时器，运行中改 publish_rate 需重启节点。
  const std::string topic = declare_parameter<std::string>("topic", "referee/robot_status");
  const double publish_rate = declare_parameter<double>("publish_rate", kDefaultRate);

  // 本机性能体系，全部可在运行时调整
  declare_parameter<std::int64_t>("robot_id", kSentryId);
  declare_parameter<std::int64_t>("robot_level", 1);
  declare_parameter<std::int64_t>("current_hp", kSentryMaxHp);
  declare_parameter<std::int64_t>("maximum_hp", kSentryMaxHp);
  declare_parameter<std::int64_t>("shooter_barrel_cooling_value", kSentryCooling);
  declare_parameter<std::int64_t>("shooter_barrel_heat_limit", kSentryHeatLimit);
  declare_parameter<std::int64_t>("shooter_17mm_1_barrel_heat", 0);
  declare_parameter<std::int64_t>("projectile_allowance_17mm", 100);
  declare_parameter<std::int64_t>("remaining_gold_coin", 0);

  param_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      return on_parameters_changed(params);
    });

  publisher_ = create_publisher<RobotStatusMsg>(topic, 10);

  last_hp_ = static_cast<std::uint16_t>(param_int("current_hp"));

  // 频率非法时回退到默认值，避免除零或负周期
  const double rate = (publish_rate > 0.0) ? publish_rate : kDefaultRate;
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    [this]() { publish_status(); });

  RCLCPP_INFO(
    get_logger(), "假裁判已启动：话题 %s，频率 %.1f Hz，ID %ld，HP %u/%ld",
    topic.c_str(), rate, param_int("robot_id"), last_hp_, param_int("maximum_hp"));
}

rcl_interfaces::msg::SetParametersResult FakeRefereeNode::on_parameters_changed(
  const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 参数值本身由 ROS 2 保存，这里只回报改了哪些，便于确认调整是否生效。
  for (const auto& param : params) {
    RCLCPP_INFO(
      get_logger(), "参数 %s 改为 %s",
      param.get_name().c_str(), param.value_to_string().c_str());
  }
  return result;
}

void FakeRefereeNode::publish_status()
{
  RobotStatusMsg msg;

  msg.robot_id = static_cast<std::uint8_t>(param_int("robot_id"));
  msg.robot_level = static_cast<std::uint8_t>(param_int("robot_level"));
  msg.current_hp = static_cast<std::uint16_t>(param_int("current_hp"));
  msg.maximum_hp = static_cast<std::uint16_t>(param_int("maximum_hp"));
  msg.shooter_barrel_cooling_value =
    static_cast<std::uint16_t>(param_int("shooter_barrel_cooling_value"));
  msg.shooter_barrel_heat_limit =
    static_cast<std::uint16_t>(param_int("shooter_barrel_heat_limit"));
  msg.shooter_17mm_1_barrel_heat =
    static_cast<std::uint16_t>(param_int("shooter_17mm_1_barrel_heat"));
  msg.projectile_allowance_17mm =
    static_cast<std::uint16_t>(param_int("projectile_allowance_17mm"));
  msg.remaining_gold_coin = static_cast<std::uint16_t>(param_int("remaining_gold_coin"));

  // robot_pos 留原点：RMUL 下哨兵位置由定位给出，裁判系统不下发。
  // armor_id 与 hp_deduction_reason 留 0：这里不模拟具体扣血事件。

  // is_hp_deduced 是上位机二次处理的标志，按相邻两帧比较得出。
  msg.is_hp_deduced = (msg.current_hp < last_hp_);
  last_hp_ = msg.current_hp;

  publisher_->publish(msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeRefereeNode>());
  rclcpp::shutdown();
  return 0;
}
