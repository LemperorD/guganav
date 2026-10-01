#include "fake_msg_source/fake_msg_source.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
// RMUL 哨兵满状态（U26 表 3-4）
constexpr std::int64_t kSentryId = 7;
constexpr std::int64_t kSentryMaxHp = 400;
constexpr std::int64_t kSentryHeatLimit = 260;
constexpr std::int64_t kSentryCooling = 30;

constexpr double kDefaultStatusRate = 20.0;
constexpr double kDefaultVisionRate = 10.0;
}  // namespace

FakeMsgSource::FakeMsgSource()
: rclcpp::Node("fake_msg_source")
{
  // ===== 话题与频率 =====
  const std::string status_topic =
    declare_parameter<std::string>("robot_status_topic", "referee/robot_status");
  const std::string vision_topic =
    declare_parameter<std::string>("vision_topic", "vision/info");
  const double status_rate = declare_parameter<double>("publish_rate", kDefaultStatusRate);
  const double vision_rate = declare_parameter<double>("vision_rate", kDefaultVisionRate);

  // ===== 本机裁判状态：全部可在运行时调整 =====
  declare_parameter<std::int64_t>("robot_id", kSentryId);
  declare_parameter<std::int64_t>("robot_level", 1);
  declare_parameter<std::int64_t>("current_hp", kSentryMaxHp);
  declare_parameter<std::int64_t>("maximum_hp", kSentryMaxHp);
  declare_parameter<std::int64_t>("shooter_barrel_cooling_value", kSentryCooling);
  declare_parameter<std::int64_t>("shooter_barrel_heat_limit", kSentryHeatLimit);
  declare_parameter<std::int64_t>("shooter_17mm_1_barrel_heat", 0);
  declare_parameter<std::int64_t>("projectile_allowance_17mm", 100);
  declare_parameter<std::int64_t>("remaining_gold_coin", 0);

  // ===== 视觉汇总 =====
  // 只有一个计数字段：> 0 表示视野里有敌人。改成 0 即可模拟敌人消失。
  declare_parameter<std::int64_t>("enemy_count", 0);
  declare_parameter<std::string>("vision_frame_id", "map");

  param_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      return on_parameters_changed(params);
    });

  status_pub_ = create_publisher<RobotStatusMsg>(status_topic, 10);
  vision_pub_ = create_publisher<VisionInfoMsg>(vision_topic, 10);

  last_hp_ = static_cast<std::uint16_t>(param_int("current_hp"));

  // 频率非法时回退到默认值，避免除零或负周期
  const double eff_status_rate = (status_rate > 0.0) ? status_rate : kDefaultStatusRate;
  const double eff_vision_rate = (vision_rate > 0.0) ? vision_rate : kDefaultVisionRate;

  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / eff_status_rate),
    [this]() { publish_robot_status(); });
  vision_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / eff_vision_rate),
    [this]() { publish_vision_info(); });

  RCLCPP_INFO(
    get_logger(), "假数据源已启动：%s @ %.1f Hz，%s @ %.1f Hz，ID %ld，HP %u/%ld，enemy_count %ld",
    status_topic.c_str(), eff_status_rate, vision_topic.c_str(), eff_vision_rate,
    param_int("robot_id"), last_hp_, param_int("maximum_hp"), param_int("enemy_count"));
}

rcl_interfaces::msg::SetParametersResult FakeMsgSource::on_parameters_changed(
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

void FakeMsgSource::publish_robot_status()
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

  status_pub_->publish(msg);
}

void FakeMsgSource::publish_vision_info()
{
  VisionInfoMsg msg;
  msg.header.stamp = now();
  msg.header.frame_id = get_parameter("vision_frame_id").as_string();
  msg.enemy_count = static_cast<std::int32_t>(param_int("enemy_count"));

  vision_pub_->publish(msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeMsgSource>());
  rclcpp::shutdown();
  return 0;
}
