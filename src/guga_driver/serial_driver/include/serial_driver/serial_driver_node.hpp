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

#include "serial_driver/serial_driver_main.hpp"
#include "serial_driver/ros_serial_bridge.hpp"

namespace serial_driver
{

class SerialDriverNode : public rclcpp::Node
{
public: // 构造和析构
  explicit SerialDriverNode(const rclcpp::NodeOptions & options);
  ~SerialDriverNode() override;

protected: // 友元
  std::shared_ptr<SerialDriverMain> serial_driver_main_;

private: // 方法
  void onConfigure();
  uint8_t* encodeTwist(const geometry_msgs::msg::Twist& msg);
  void publishTransformGimbalYaw(double yaw);
  void publishTransformGimbalBase(double yaw);
  inline double dwa_filter(double sample);
  inline geometry_msgs::msg::Twist transformVelocityToChassis(
      const geometry_msgs::msg::Twist& twist_in, double yaw_diff);

private: // 成员变量
  std::string port_name_ = "/dev/ttyACM0";
  int baud_rate_ = 115200;
  double Yaw_bias_ = 0.0;
  double vel_trans_scale_ = 40.0;
  double yaw_diff_ = 0.0;
  double angle_init_ = 0.0;

  // tf相关变量
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 多输入/输出的额外sub/pub可在此添加
  // Additional sub/pub for multi-input/output can be added here
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;

  // 多输入/输出所使用的成员变量可在此添加
  // Member variables used for multi-input/output can be added here
};

}  // namespace serial_driver

#endif // SERIAL_DRIVER_NODE_HPP