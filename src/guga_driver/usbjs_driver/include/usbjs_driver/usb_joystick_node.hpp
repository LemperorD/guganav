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

  /**
    * @brief 测试线程函数
   */
  void test_thread();

private: // 成员变量
  std::shared_ptr<UsbJoystickMain> usb_joystick_main_; // USB手柄对象
  std::string file_name_; // 设备文件地址

  std::thread ctrl_thread_; // 控制线程
  std::thread test_thread_; // 测试线程(回显部分轴状态和按钮状态)

  std::string output_vel_topic_; // 发布的速度话题名称
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_vel_pub_; // 速度发布器

  bool remote_mode_ = false; // 远程模式标志
  bool last_button2_state_ = false; // 上一次按钮2状态,用于可视化状态变化
};

} // namespace usbjs_driver

#endif // USB_JOYSTICK_NODE_HPP