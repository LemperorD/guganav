#ifndef USB_JOYSTICK_NODE_HPP
#define USB_JOYSTICK_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "usb_joystick_main.hpp"

namespace usbjs_driver
{

class UsbJoystickNode : public rclcpp::Node
{
public: // 构造和析构
  /**
   * @brief 构造函数
   * @param options 节点选项
   */
  explicit UsbJoystickNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
  /**
   * @brief 析构函数
   */
  ~UsbJoystickNode();

private: // 方法
  /**
   * @brief 配置参数
   */
  void onConfigure();

  /**
   * @brief 控制线程函数
   */
  void ctrl_thread();

private: // 成员变量
  std::shared_ptr<UsbJoystickMain> usb_joystick_main_;
  std::string file_name_;
  std::thread ctrl_thread_;
  
  // Remote control state
  std::string output_vel_topic_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_vel_pub_;
  bool remote_mode_ = false;
  bool last_button2_state_ = false;
};

} // namespace usbjs_driver

#endif // USB_JOYSTICK_NODE_HPP