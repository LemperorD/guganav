#include "guga_decision_common/decision_input_hub.hpp"

#include <string>
#include <utility>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace guga_decision_common
{

DecisionInputHub::DecisionInputHub(const rclcpp::NodeOptions& options)
: DecisionInputHub("decision_input_hub", options)
{
}

DecisionInputHub::DecisionInputHub(
  const std::string& node_name, const rclcpp::NodeOptions& options)
: rclcpp::Node(node_name, options)
{
  setupInputs();
}

void DecisionInputHub::setupInputs()
{
  const std::string robot_topic = declare_parameter<std::string>(
    "robot_status_topic", "/referee/robot_status");
  const std::string game_topic = declare_parameter<std::string>(
    "game_status_topic", "/referee/game_status");
  const std::string rfid_topic = declare_parameter<std::string>(
    "rfid_status_topic", "/referee/rfid_status");
  const std::string vision_topic = declare_parameter<std::string>(
    "vision_topic", "vision/info");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  base_frame_id_ =
    declare_parameter<std::string>("base_frame_id", "base_footprint");

  robot_status_sub_ = create_subscription<RobotStatusMsg>(
    robot_topic, rclcpp::QoS(10),
    [this](RobotStatusMsg::SharedPtr msg) {
      if (!msg) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      auto& robot = snapshot_.robot;
      const bool first = !robot.received;
      robot.received = true;
      robot.received_at_ns = now().nanoseconds();
      robot.current_hp = msg->current_hp;
      robot.maximum_hp = msg->maximum_hp;
      robot.projectile_allowance = msg->projectile_allowance_17mm;
      robot.heat = msg->shooter_17mm_1_barrel_heat;
      robot.heat_limit = msg->shooter_barrel_heat_limit;
      robot.cooling_value = msg->shooter_barrel_cooling_value;
      robot.hp_deduced = msg->is_hp_deduced;
      if (first) {
        RCLCPP_INFO(get_logger(), "首次收到机器人状态: HP=%u/%u, 弹药=%u",
                    robot.current_hp, robot.maximum_hp,
                    robot.projectile_allowance);
      }
    });

  game_status_sub_ = create_subscription<GameStatusMsg>(
    game_topic, rclcpp::QoS(10),
    [this](GameStatusMsg::SharedPtr msg) {
      if (!msg) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      auto& game = snapshot_.game;
      const bool first = !game.received;
      const std::uint8_t previous = game.game_progress;
      game.received = true;
      game.received_at_ns = now().nanoseconds();
      game.game_progress = msg->game_progress;
      game.stage_remain_time = msg->stage_remain_time;
      if (first || previous != game.game_progress) {
        RCLCPP_INFO(get_logger(), "比赛阶段: %u -> %u（4 = 比赛中）",
                    first ? 0 : previous, game.game_progress);
      }
    });

  rfid_status_sub_ = create_subscription<RfidStatusMsg>(
    rfid_topic, rclcpp::QoS(10),
    [this](RfidStatusMsg::SharedPtr msg) {
      if (!msg) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      auto& rfid = snapshot_.rfid;
      const bool first = !rfid.received;
      const bool previous_supply = rfid.supply_zone;
      const bool previous_center = rfid.center_gain_point;
      rfid.received = true;
      rfid.received_at_ns = now().nanoseconds();
      rfid.supply_zone = msg->friendly_supply_zone_non_exchange;
      rfid.center_gain_point = msg->center_gain_point;
      if (first || previous_supply != rfid.supply_zone ||
        previous_center != rfid.center_gain_point)
      {
        RCLCPP_INFO(get_logger(), "RFID 状态: 补给区=%d, 中心区=%d",
                    rfid.supply_zone, rfid.center_gain_point);
      }
    });

  vision_sub_ = create_subscription<VisionInfoMsg>(
    vision_topic, rclcpp::QoS(10),
    [this](VisionInfoMsg::SharedPtr msg) {
      if (!msg) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      auto& vision = snapshot_.vision;
      vision.received = true;
      vision.received_at_ns = now().nanoseconds();
      vision.enemy_count = msg->enemy_count;
    });

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, this, true);

  RCLCPP_INFO(
    get_logger(),
    "决策输入话题: robot=%s, game=%s, rfid=%s, vision=%s; "
    "位姿查询=%s -> %s",
    robot_topic.c_str(), game_topic.c_str(), rfid_topic.c_str(),
    vision_topic.c_str(), base_frame_id_.c_str(), frame_id_.c_str());
}

DecisionInputSnapshot DecisionInputHub::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

bool DecisionInputHub::hasRobotStatus() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.robot.received;
}

bool DecisionInputHub::hasGameStatus() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.game.received;
}

bool DecisionInputHub::hasRfidStatus() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.rfid.received;
}

bool DecisionInputHub::hasVision() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.vision.received;
}

std::uint16_t DecisionInputHub::currentHp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.robot.current_hp;
}

std::uint16_t DecisionInputHub::maximumHp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.robot.maximum_hp;
}

std::uint16_t DecisionInputHub::projectileAllowance() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.robot.projectile_allowance;
}

std::uint16_t DecisionInputHub::heat() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.robot.heat;
}

std::uint16_t DecisionInputHub::heatLimit() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.robot.heat_limit;
}

std::int32_t DecisionInputHub::enemyCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.vision.enemy_count;
}

std::uint8_t DecisionInputHub::gameProgress() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.game.game_progress;
}

bool DecisionInputHub::supplyZoneGainPoint() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.rfid.supply_zone;
}

bool DecisionInputHub::centerGainPoint() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.rfid.center_gain_point;
}

bool DecisionInputHub::lookupRobotPose(double& x, double& y, double& yaw)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      frame_id_, base_frame_id_, tf2::TimePointZero);
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    yaw = tf2::getYaw(transform.transform.rotation);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "查不到 %s 在 %s 下的位姿: %s", base_frame_id_.c_str(),
      frame_id_.c_str(), ex.what());
    return false;
  }
}

bool DecisionInputHub::lookupRobotPose(double& x, double& y)
{
  double yaw = 0.0;
  return lookupRobotPose(x, y, yaw);
}

}  // namespace guga_decision_common
