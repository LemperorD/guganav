#include "serial_driver/serial_driver_node.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace serial_driver {

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options)
    : Node("serial_driver_node", options)
{
  onConfigure(); // 加载参数

  serial_driver_main_ = std::make_shared<SerialDriverMain>(port_name_, baud_rate_);

  gimbal_vision_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30), [this]() { publishTransformGimbalVision(); });
  referee_rx_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), [this]() { publishRefereeData(); });

  robot_status_pub_ = this->create_publisher<guga_interfaces::msg::RobotStatus>("/referee/robot_status", 10);
  game_status_pub_ = this->create_publisher<guga_interfaces::msg::GameStatus>("/referee/game_status", 10);
  rfid_status_pub_ = this->create_publisher<guga_interfaces::msg::RfidStatus>("/referee/rfid_status", 10);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/serial/gimbal_joint_state", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  bridge_twist_pc_ =
      std::make_shared<RosToSerialBridge<geometry_msgs::msg::Twist>>(
          this, "/cmd_vel",
          [this](const geometry_msgs::msg::Twist& msg) {
            return encodeTwist(msg);
          },
          [this](const uint8_t* data, size_t len) {
            serial_driver_main_->sendDataFrame(data, len);
          });

  bridge_yaw_mcu_ =
      std::make_shared<SerialToRosBridge<std_msgs::msg::Float32>>(
          this, "/serial/Yaw",
          [this](const uint8_t* payload) -> std_msgs::msg::Float32 {
            return decodeYaw(payload);
          },
          [this]() {
            return serial_driver_main_->receiveDataFrameSnapshot();
          });

  bridge_enemy_pos_mcu_ =
      std::make_shared<SerialToRosBridge<geometry_msgs::msg::Point>>(
          this, "/serial/EnemyPos",
          [](const uint8_t* payload) -> geometry_msgs::msg::Point {
            return decodeEnemyPos(payload);
          },
          [this]() {
            return serial_driver_main_->receiveDataFrameSnapshot();
          });
}

SerialDriverNode::~SerialDriverNode() {
  std::cout << "Shutting down SerialDriverNode..." << '\n';

  gimbal_vision_timer_.reset();
  referee_rx_timer_.reset();

  bridge_twist_pc_.reset();
  bridge_yaw_mcu_.reset();
  bridge_enemy_pos_mcu_.reset();

  robot_status_pub_.reset();
  game_status_pub_.reset();
  rfid_status_pub_.reset();

  serial_driver_main_.reset();

  std::cout << "SerialDriverNode shutdown complete." << '\n';
}

// ==================== 参数加载 ====================

void SerialDriverNode::onConfigure() {
  this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<double>("vel_trans_scale", 40.0);

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("vel_trans_scale", vel_trans_scale_);
}

MotionPayload SerialDriverNode::encodeTwist(const geometry_msgs::msg::Twist& msg) {
  const auto vx = static_cast<float>(vel_trans_scale_ * msg.linear.x);
  const auto vy = static_cast<float>(vel_trans_scale_ * msg.linear.y);
  const auto wz = static_cast<float>(msg.angular.z);

  auto vx_smoothed = slidingWindowFilter(vx, vx_buffer_, filter_window_size_);
  auto vy_smoothed = slidingWindowFilter(vy, vy_buffer_, filter_window_size_);

  MotionPayload payload{};
  payload.fill(0);  // 全部初始化为 0

  // 按头文件定义写入三个速度字段（VX=0, VY=4, WZ_NEG=8）
  SerialDriverMain::writeFloatLE(&payload[downlink_offset::VX], vx_smoothed);
  SerialDriverMain::writeFloatLE(&payload[downlink_offset::VY], vy_smoothed);
  SerialDriverMain::writeFloatLE(&payload[downlink_offset::WZ_NEG], wz);

  return payload;
}

std_msgs::msg::Float32 SerialDriverNode::decodeYaw(const uint8_t* payload) {
  std_msgs::msg::Float32 msg;
  msg.data = SerialDriverMain::readFloatLE(&payload[uplink_offset::YAW_DIFF]);
  yaw_diff_ = static_cast<double>(msg.data);

  sensor_msgs::msg::JointState joint_msg;
  joint_msg.name = {
    "gimbal_pitch_joint",
    "gimbal_yaw_joint",
    "gimbal_pitch_odom_joint",
    "gimbal_yaw_odom_joint",
  };
  joint_msg.position = {
    0,
    0,
    0,
    -yaw_diff_,
  };
  joint_state_pub_->publish(joint_msg);
  return msg;
}

geometry_msgs::msg::Point SerialDriverNode::decodeEnemyPos(const uint8_t* payload) {
  geometry_msgs::msg::Point msg;
  msg.x = static_cast<double>(
      SerialDriverMain::readFloatLE(&payload[uplink_offset::ENEMY_X]));
  msg.y = static_cast<double>(
      SerialDriverMain::readFloatLE(&payload[uplink_offset::ENEMY_Y]));
  return msg;
}

// ==================== tf 广播 ====================

void SerialDriverNode::publishTransformGimbalVision() {
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform("odom", "base_footprint",
                                                    tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    std::cerr << "Could not get transform: " << ex.what() << '\n';
    return;
  }

  // 提取 yaw 角
  tf2::Quaternion q(transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z,
                    transform_stamped.transform.rotation.w);

  double roll{};
  double pitch{};
  double yaw{};
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 发布 base_footprint → gimbal_yaw_vision（纯旋转，取反 yaw）
  geometry_msgs::msg::TransformStamped gimbal_tf;
  gimbal_tf.header.stamp = this->get_clock()->now();
  gimbal_tf.header.frame_id = "base_footprint";
  gimbal_tf.child_frame_id = "gimbal_yaw_vision";

  gimbal_tf.transform.translation.x = 0.0;
  gimbal_tf.transform.translation.y = 0.0;
  gimbal_tf.transform.translation.z = 0.0;

  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, -yaw);

  gimbal_tf.transform.rotation.x = q_yaw.x();
  gimbal_tf.transform.rotation.y = q_yaw.y();
  gimbal_tf.transform.rotation.z = q_yaw.z();
  gimbal_tf.transform.rotation.w = q_yaw.w();

  tf_broadcaster_->sendTransform(gimbal_tf);
}

// ==================== 工具方法 ====================

geometry_msgs::msg::Twist SerialDriverNode::transformVelocityToChassis(const geometry_msgs::msg::Twist& twist_in, double yaw_diff) {
  geometry_msgs::msg::Twist out;
  double cos = std::cos(yaw_diff);
  double sin = std::sin(yaw_diff);

  out.linear.x = (twist_in.linear.x * cos) - (twist_in.linear.y * sin);
  out.linear.y = (twist_in.linear.x * sin) + (twist_in.linear.y * cos);
  out.angular.z = twist_in.angular.z;
  return out;
}

// ==================== 裁判系统 ====================

void SerialDriverNode::publishRefereeData() {
  const auto payload_snapshot = serial_driver_main_->takeRefereeFrameSnapshot();
  if (!payload_snapshot.has_value()) {
    return;
  }
  const uint8_t* payload = payload_snapshot->data();

  // ---- 解析比赛进程（payload[0] 的高 4 位） ----
  uint8_t game_progress{};
  std::memcpy(&game_progress, &payload[referee_offset::GAME_PROGRESS],
              sizeof(uint8_t));
  game_progress = (game_progress >> 4);  // 取高 4 位

  // ---- 解析机器人状态 ----
  uint16_t current_hp{};
  uint16_t ammo17{};
  uint16_t heat1{};
  std::memcpy(&current_hp, &payload[referee_offset::CURRENT_HP],
              sizeof(uint16_t));
  std::memcpy(&ammo17, &payload[referee_offset::AMMO_17MM],
              sizeof(uint16_t));
  std::memcpy(&heat1, &payload[referee_offset::BARREL_HEAT],
              sizeof(uint16_t));

  // ---- 解析 RFID ----
  uint32_t rfid{};
  std::memcpy(&rfid, &payload[referee_offset::RFID_STATUS],
              sizeof(uint32_t));

  // ---- 发布机器人状态 ----
  guga_interfaces::msg::RobotStatus robot_status;
  robot_status.current_hp = current_hp;
  robot_status.projectile_allowance_17mm = ammo17;
  robot_status.shooter_17mm_1_barrel_heat = heat1;
  robot_status_pub_->publish(robot_status);

  // ---- 发布比赛状态 ----
  guga_interfaces::msg::GameStatus game_status;
  game_status.game_progress = game_progress;
  game_status_pub_->publish(game_status);

  // ---- 发布 RFID 状态 ----
  guga_interfaces::msg::RfidStatus rfid_status;
  rfid_status = rfid2ros(rfid);
  rfid_status_pub_->publish(rfid_status);
}

guga_interfaces::msg::RfidStatus SerialDriverNode::rfid2ros(uint32_t rfid) {
  guga_interfaces::msg::RfidStatus status;

  // 裁判系统协议 V1.7.0 0x0209: RFID 状态位映射
  status.base_gain_point = (rfid >> 0) & 0x01;
  status.central_highland_gain_point = (rfid >> 1) & 0x01;
  status.enemy_central_highland_gain_point = (rfid >> 2) & 0x01;
  status.friendly_trapezoidal_highland_gain_point = (rfid >> 3) & 0x01;
  status.enemy_trapezoidal_highland_gain_point = (rfid >> 4) & 0x01;
  status.friendly_fly_ramp_front_gain_point = (rfid >> 5) & 0x01;
  status.friendly_fly_ramp_back_gain_point = (rfid >> 6) & 0x01;
  status.enemy_fly_ramp_front_gain_point = (rfid >> 7) & 0x01;
  status.enemy_fly_ramp_back_gain_point = (rfid >> 8) & 0x01;
  status.friendly_central_highland_lower_gain_point = (rfid >> 9) & 0x01;
  status.friendly_central_highland_upper_gain_point = (rfid >> 10) & 0x01;
  status.enemy_central_highland_lower_gain_point = (rfid >> 11) & 0x01;
  status.enemy_central_highland_upper_gain_point = (rfid >> 12) & 0x01;
  status.friendly_highway_lower_gain_point = (rfid >> 13) & 0x01;
  status.friendly_highway_upper_gain_point = (rfid >> 14) & 0x01;
  status.enemy_highway_lower_gain_point = (rfid >> 15) & 0x01;
  status.enemy_highway_upper_gain_point = (rfid >> 16) & 0x01;
  status.friendly_fortress_gain_point = (rfid >> 17) & 0x01;
  status.friendly_outpost_gain_point = (rfid >> 18) & 0x01;
  status.friendly_supply_zone_non_exchange = (rfid >> 19) & 0x01;
  status.friendly_supply_zone_exchange = (rfid >> 20) & 0x01;
  status.friendly_big_resource_island = (rfid >> 21) & 0x01;
  status.enemy_big_resource_island = (rfid >> 22) & 0x01;
  status.center_gain_point = (rfid >> 23) & 0x01;

  return status;
}

}  // namespace serial_driver
