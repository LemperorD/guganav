#include "usbjs_driver/usb_joystick_node.hpp"

namespace usbjs_driver
{

UsbJoystickNode::UsbJoystickNode(const rclcpp::NodeOptions & options)
  : Node("usb_joystick_node", options)
{
  onConfigure(); // 配置参数

  usb_joystick_main_ = std::make_shared<UsbJoystickMain>(file_name_.c_str()); // 创建USB手柄对象

  output_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    output_vel_topic_, rclcpp::QoS(10)); // 创建发布器

  ctrl_thread_ = std::thread(&UsbJoystickNode::ctrl_thread, this); // 启动控制线程
}

UsbJoystickNode::~UsbJoystickNode()
{
  if (ctrl_thread_.joinable()) ctrl_thread_.join();
  usb_joystick_main_.reset();
  std::cout << "UsbJoystickNode destructor called" << std::endl;
}

void UsbJoystickNode::onConfigure()
{
  this->declare_parameter<std::string>("file_name", "/dev/input/js0");
  this->get_parameter("file_name", file_name_);
  
  this->declare_parameter<std::string>("output_vel_topic", "/cmd_vel");
  this->get_parameter("output_vel_topic", output_vel_topic_);
}

void UsbJoystickNode::ctrl_thread()
{
  while (rclcpp::ok()) {
    if (!usb_joystick_main_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    
    // Read button2 state for mode toggle
    bool button2_pressed = usb_joystick_main_->get_button_state(2);
    
    // Toggle remote_mode on button2 state change (rising edge)
    if (button2_pressed && !last_button2_state_) {
      remote_mode_ = !remote_mode_;
      std::cout << "[UsbJoystick] Remote mode: " << (remote_mode_ ? "ON" : "OFF") << std::endl;
    }
    last_button2_state_ = button2_pressed;
    
    // If in remote mode, read axis values and send joystick command
    if (remote_mode_) {
      int16_t axis1 = usb_joystick_main_->get_axis_state(1);  // Yaw
      int16_t axis2 = usb_joystick_main_->get_axis_state(2);  // Pitch
      
      // Create and publish joystick command
      auto cmd = geometry_msgs::msg::Twist();
      cmd.linear.x = static_cast<float>(axis1) / 32768.0f;
      cmd.angular.z = static_cast<float>(axis2) / 32768.0f;
      
      output_vel_pub_->publish(cmd);

      // Map axis int16 -> normalized position in [-1,1]
      gimbal_cmd.position.yaw = -static_cast<float>(axis1) / 32768.0f;
      gimbal_cmd.position.pitch = static_cast<float>(axis2) / 32768.0f;

      gimbal_cmd_pub_->publish(gimbal_cmd);

      // Debug output
      std::cout << "[RcGimbal] Yaw pos: " << gimbal_cmd.position.yaw 
            << ", Pitch pos: " << gimbal_cmd.position.pitch << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

} // namespace usbjs_driver

// 导出组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usbjs_driver::UsbJoystickNode)

