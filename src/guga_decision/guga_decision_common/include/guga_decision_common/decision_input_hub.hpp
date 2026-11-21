#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "guga_interfaces/msg/game_status.hpp"
#include "guga_interfaces/msg/rfid_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"
#include "guga_interfaces/msg/vision_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace guga_decision_common
{

struct RobotStatusInput
{
  bool received{false};
  std::int64_t received_at_ns{0};
  std::uint16_t current_hp{0};
  std::uint16_t maximum_hp{0};
  std::uint16_t projectile_allowance{0};
  std::uint16_t heat{0};
  std::uint16_t heat_limit{0};
  std::uint16_t cooling_value{0};
  bool hp_deduced{false};
};

struct GameStatusInput
{
  bool received{false};
  std::int64_t received_at_ns{0};
  std::uint8_t game_progress{0};
  std::int32_t stage_remain_time{0};
};

struct RfidStatusInput
{
  bool received{false};
  std::int64_t received_at_ns{0};
  bool supply_zone{false};
  bool center_gain_point{false};
};

struct VisionInput
{
  bool received{false};
  std::int64_t received_at_ns{0};
  std::int32_t enemy_count{0};
};

struct DecisionInputSnapshot
{
  RobotStatusInput robot;
  GameStatusInput game;
  RfidStatusInput rfid;
  VisionInput vision;
};

struct RobotPoseInput
{
  bool valid{false};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 三套决策方案共用的 ROS 输入缓存。
//
// 该类只负责订阅、缓存和查询位姿，不认识 BehaviorTree.CPP，也不执行决策。
// 同一进程内应只创建一个实例，多个 BT 输入节点共享该实例，避免重复订阅。
class DecisionInputHub : public rclcpp::Node
{
public:
  explicit DecisionInputHub(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  DecisionInputHub(
    const std::string& node_name,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  DecisionInputSnapshot snapshot() const;

  bool hasRobotStatus() const;
  bool hasGameStatus() const;
  bool hasRfidStatus() const;
  bool hasVision() const;

  // Compatibility helpers used by the existing strategy nodes.
  bool hasData() const { return hasRobotStatus(); }
  bool hasRfidData() const { return hasRfidStatus(); }
  std::uint16_t currentHp() const;
  std::uint16_t maximumHp() const;
  std::uint16_t projectileAllowance() const;
  std::uint16_t heat() const;
  std::uint16_t heatLimit() const;
  std::int32_t enemyCount() const;
  std::uint8_t gameProgress() const;
  bool supplyZoneGainPoint() const;
  bool centerGainPoint() const;

  bool lookupRobotPose(double& x, double& y, double& yaw);
  bool lookupRobotPose(double& x, double& y);

  const std::string& frameId() const { return frame_id_; }
  const std::string& baseFrameId() const { return base_frame_id_; }

private:
  using RobotStatusMsg = guga_interfaces::msg::RobotStatus;
  using RfidStatusMsg = guga_interfaces::msg::RfidStatus;
  using GameStatusMsg = guga_interfaces::msg::GameStatus;
  using VisionInfoMsg = guga_interfaces::msg::VisionInfo;

  void setupInputs();

  mutable std::mutex mutex_;
  DecisionInputSnapshot snapshot_;

  rclcpp::Subscription<RobotStatusMsg>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<GameStatusMsg>::SharedPtr game_status_sub_;
  rclcpp::Subscription<RfidStatusMsg>::SharedPtr rfid_status_sub_;
  rclcpp::Subscription<VisionInfoMsg>::SharedPtr vision_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string frame_id_;
  std::string base_frame_id_;
};

}  // namespace guga_decision_common
