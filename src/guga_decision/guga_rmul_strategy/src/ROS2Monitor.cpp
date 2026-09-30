#include "ROS2Monitor.hpp"

#include <memory>
#include <utility>

ROS2Monitor::ROS2Monitor(const rclcpp::NodeOptions& options)
: rclcpp::Node("ros2_monitor", options)
{
  const std::string topic =
    declare_parameter<std::string>("robot_status_topic", "referee/robot_status");

  // 回调在 executor 线程执行，tick() 在另一条线程执行，
  // 所以共享的成员一律用 std::atomic，回调里不碰黑板也不做判断。
  subscription_ = create_subscription<RobotStatusMsg>(
    topic, rclcpp::QoS(10),
    [this](RobotStatusMsg::SharedPtr msg) {
      if (!msg) {
        return;
      }
      current_hp_.store(msg->current_hp);
      maximum_hp_.store(msg->maximum_hp);
      has_data_.store(true);
    });

  RCLCPP_INFO(get_logger(), "ROS2Monitor 订阅话题: %s", topic.c_str());
}
