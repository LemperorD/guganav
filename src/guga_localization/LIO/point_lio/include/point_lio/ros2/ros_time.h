/**
 * @file ros_time.h
 * @brief ROS2 时间戳转换工具.
 */

#pragma once

#include <cmath>
#include <cstdint>

#include <rclcpp/time.hpp>

/// 秒 (double) → ROS2 Time 消息.
inline rclcpp::Time get_ros_time(double timestamp) {
  const int32_t sec = static_cast<int32_t>(std::floor(timestamp));
  const auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  const uint32_t nanosec = static_cast<uint32_t>(nanosec_d);
  return rclcpp::Time(sec, nanosec);
}
