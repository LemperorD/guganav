#ifndef SERIAL_DRIVER_NODE_HPP
#define SERIAL_DRIVER_NODE_HPP

#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "communication/Com.h"
#include "communication/ros_serial_bridge.hpp"

#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"

using json = nlohmann::json;

namespace serial_driver
{

class SerialDriverNode : public rclcpp::Node
{
public: // 构造和析构
  explicit SerialDriverNode(const rclcpp::NodeOptions & options);
  ~SerialDriverNode() override;

public: // 检测雷达是否连接

private: // 编解码函数
  uint8_t* encodeTwist(const geometry_msgs::msg::Twist& msg);
  std_msgs::msg::Float32 decodeYaw(const uint8_t* payload);
  geometry_msgs::msg::Twist decodeTESspeed(const uint8_t* payload);
  geometry_msgs::msg::Point decodeEnemyPos(const uint8_t* payload);

private: // dwa滤波器
  int max_dwa_size_ = 15;
  std::deque<double> dwa_;
  double dwa_filter(double sample);
  geometry_msgs::msg::Twist transformVelocityToChassis(const geometry_msgs::msg::Twist & twist_in, double yaw_diff);

private:
  void publishTransformGimbalVision();
  void publishTransformGimbalYaw(double yaw);

private:
  std::string port_name_ = "/dev/ttyACM0";
  int baud_rate_ = 115200;
  double Yaw_bias_ = 0.0;
  double vel_trans_scale_ = 40.0;
  double yaw_diff_ = 0.0;
  double angle_init_ = 0.0;

  std::shared_ptr<SerialCommunicationClass> com_;

  // 模板类桥接器
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_twist_pc_;
  std::shared_ptr<RosSerialBridge<std_msgs::msg::Float32>> bridge_Yaw_mcu_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_TESspeed_mcu_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Point>> bridge_EnemyPos_mcu_;

  // timer
  rclcpp::TimerBase::SharedPtr gimbal_vision_timer_;
  rclcpp::TimerBase::SharedPtr referee_rx_timer_;

  // tf相关变量
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 多输入/输出的额外sub/pub可在此添加
  // Additional sub/pub for multi-input/output can be added here
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;

  // 多输入/输出所使用的成员变量可在此添加
  // Member variables used for multi-input/output can be added here
  uint8_t chassis_mode_ = chassisFollowed;

  // 判断是否有雷达连接
  bool lidar_connected_ = false;

private: // 裁判系统相关
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  void publishRefereeData();
  pb_rm_interfaces::msg::RfidStatus rfid2ros(uint32_t rfid);
};

}  // namespace serial_driver

#endif // SERIAL_DRIVER_NODE_HPP