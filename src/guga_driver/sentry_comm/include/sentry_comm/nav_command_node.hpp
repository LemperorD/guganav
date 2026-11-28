#pragma once

#include "sentry_comm/command_writer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace sentry_comm {

  // 把导航栈的速度与过洞模式写入共享内存，替代旧的串口驱动发送路径。
  class NavCommandNode : public rclcpp::Node {
  public:
    NavCommandNode();
    ~NavCommandNode() override;

  private:
    struct Vector3 {
      double x{};
      double y{};
      double z{};
    };

    struct Quaternion {
      double w{1.0};
      double x{};
      double y{};
      double z{};
    };

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr message);
    void onPitchMode(const std_msgs::msg::UInt8::SharedPtr message);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
    void assembleAndSubmit();
    void logWriterHealth();

    static Vector3 rotate(const Quaternion& orientation, const Vector3& vector);

    std::string shm_name_;
    double write_rate_hz_{};
    double command_timeout_sec_{};
    // 下发速度的缩放：电控侧按 br_protocol 的约定接收放大后的值，
    // 旧 serial_driver 用 vel_trans_scale=40，这里保持一致；wz 单独可调。
    double vel_trans_scale_{40.0};
    double vel_yaw_scale_{1.0};

    std::unique_ptr<NavCommandWriter> writer_;

    std::mutex state_mutex_;
    geometry_msgs::msg::Twist command_{};
    bool has_command_{};
    std::chrono::steady_clock::time_point last_command_time_{};
    std::uint8_t pitch_mode_{};

    Vector3 position_{};
    Vector3 body_velocity_{};
    Quaternion orientation_{};
    bool has_odometry_{};

    std::uint64_t last_failure_count_{};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_subscription_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
        pitch_mode_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;
    rclcpp::TimerBase::SharedPtr assembly_timer_;
  };

}  // namespace sentry_comm
