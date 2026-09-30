#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "guga_interfaces/msg/robot_status.hpp"

// ROS 2 侧：只负责订阅裁判系统话题，把最新血量缓存起来。
// 它不认识行为树，也不碰黑板，方便单独测试。
class ROS2Monitor : public rclcpp::Node
{
public:
  explicit ROS2Monitor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // 供行为树节点读取。返回的是最近一次收到的值。
  std::uint16_t currentHp() const { return current_hp_.load(); }
  std::uint16_t maximumHp() const { return maximum_hp_.load(); }

  // 是否已经收到过至少一条消息。没收到时血量是 0，判断前应当先看这个标志。
  bool hasData() const { return has_data_.load(); }

private:
  using RobotStatusMsg = guga_interfaces::msg::RobotStatus;

  rclcpp::Subscription<RobotStatusMsg>::SharedPtr subscription_;
  std::atomic<std::uint16_t> current_hp_{0};
  std::atomic<std::uint16_t> maximum_hp_{0};
  std::atomic<bool> has_data_{false};
};
