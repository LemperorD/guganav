#include "rc_gimbal/rc_gimbal_node.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace rc_gimbal
{

RcGimbalNode::RcGimbalNode(const rclcpp::NodeOptions & options)
  : Node("rc_gimbal_node", options)
{
  onConfigure(); // 配置参数
  rc_gimbal_main_ = std::make_shared<RcGimbalMain>(file_name_.c_str());
  
  // Initialize gimbal command publisher
  gimbal_cmd_pub_ = this->create_publisher<simulator::msg::GimbalCmd>(
    gimbal_cmd_topic_, rclcpp::QoS(10));
  
  // Start remote control thread
  ctrl_thread_ = std::thread(&RcGimbalNode::ctrl_thread, this);
}

RcGimbalNode::~RcGimbalNode()
{
  if (ctrl_thread_.joinable()) ctrl_thread_.join();
  rc_gimbal_main_.reset();
  std::cout << "RcGimbalNode destructor called" << std::endl;
}

void RcGimbalNode::onConfigure()
{
  this->declare_parameter<std::string>("file_name", "/dev/input/js0");
  this->get_parameter("file_name", file_name_);
  
  this->declare_parameter<std::string>("gimbal_cmd_topic", "/all_in_one_sensor/robot_base/gimbal_cmd");
  this->get_parameter("gimbal_cmd_topic", gimbal_cmd_topic_);
}

void RcGimbalNode::ctrl_thread()
{
  while (rclcpp::ok()) {
    if (!rc_gimbal_main_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    
    // Read button2 state for mode toggle
    bool button2_pressed = rc_gimbal_main_->get_button_state(2);
    
    // Toggle remote_mode on button2 state change (rising edge)
    if (button2_pressed && !last_button2_state_) {
      remote_mode_ = !remote_mode_;
      std::cout << "[RcGimbal] Remote mode: " << (remote_mode_ ? "ON" : "OFF") << std::endl;
    }
    last_button2_state_ = button2_pressed;
    
    // If in remote mode, read axis values and send gimbal command
    if (remote_mode_) {
      int16_t axis1 = rc_gimbal_main_->get_axis_state(1);  // Yaw
      int16_t axis2 = rc_gimbal_main_->get_axis_state(2);  // Pitch
      
      // Create and publish gimbal command using position control (ABSOLUTE_ANGLE)
      auto gimbal_cmd = simulator::msg::GimbalCmd();
      gimbal_cmd.tid = 0;
      gimbal_cmd.yaw_type = simulator::msg::GimbalCmd::ABSOLUTE_ANGLE;
      gimbal_cmd.pitch_type = simulator::msg::GimbalCmd::ABSOLUTE_ANGLE;

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

} // namespace rc_gimbal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_gimbal::RcGimbalNode)

