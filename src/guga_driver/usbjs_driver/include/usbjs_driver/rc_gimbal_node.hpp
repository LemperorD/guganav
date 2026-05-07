#ifndef RC_GIMBAL_NODE_HPP
#define RC_GIMBAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "simulator/msg/gimbal.hpp"
#include "simulator/msg/gimbal_cmd.hpp"

#include "rc_gimbal_main.hpp"
#include <atomic>

namespace rc_gimbal
{

class RcGimbalNode : public rclcpp::Node
{
public:
  explicit RcGimbalNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RcGimbalNode();

private:
  void onConfigure();
  void ctrl_thread();

private:
  std::shared_ptr<RcGimbalMain> rc_gimbal_main_;
  std::string file_name_;
  std::thread ctrl_thread_;
  
  // Remote control state
  std::string gimbal_cmd_topic_;
  rclcpp::Publisher<simulator::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  bool remote_mode_ = false;
  bool last_button2_state_ = false;
};

} // namespace rc_gimbal

#endif // RC_GIMBAL_NODE_HPP