#include "sentry_comm/nav_command_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace sentry_comm {

  namespace {

    constexpr int ASSEMBLY_PERIOD_MS = 20;

  }  // namespace

  NavCommandNode::NavCommandNode() : Node("sentry_comm_nav_command") {
    shm_name_ = declare_parameter<std::string>("shm_name", "/s2c_serial_comm");
    write_rate_hz_ = declare_parameter<double>("write_rate_hz", 100.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec",
                                                     0.5);
    vel_trans_scale_ = declare_parameter<double>("vel_trans_scale", 40.0);
    vel_yaw_scale_ = declare_parameter<double>("vel_yaw_scale", 1.0);
    const std::string cmd_vel_topic = declare_parameter<std::string>(
        "cmd_vel_topic", "cmd_vel");
    const std::string odometry_topic = declare_parameter<std::string>(
        "odometry_topic", "odometry");
    const std::string pitch_mode_topic = declare_parameter<std::string>(
        "pitch_mode_topic", "/decision/pitch_mode");

    if (!std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("command_timeout_sec 必须为有限正值");
    }

    NavCommandWriter::Config config;
    config.shm_name = shm_name_;
    config.write_rate_hz = write_rate_hz_;
    writer_ = std::make_unique<NavCommandWriter>(config);

    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr message) {
          onCmdVel(message);
        });
    pitch_mode_subscription_ = create_subscription<std_msgs::msg::UInt8>(
        pitch_mode_topic, rclcpp::QoS(10),
        [this](const std_msgs::msg::UInt8::SharedPtr message) {
          onPitchMode(message);
        });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          onOdometry(message);
        });

    assembly_timer_ = create_wall_timer(
        std::chrono::milliseconds(ASSEMBLY_PERIOD_MS), [this] {
          assembleAndSubmit();
          logWriterHealth();
        });

    writer_->start();

    RCLCPP_INFO(get_logger(), "写入共享内存 %s（%.0f Hz），订阅 %s、%s、%s",
                shm_name_.c_str(), write_rate_hz_, cmd_vel_topic.c_str(),
                odometry_topic.c_str(), pitch_mode_topic.c_str());
  }

  NavCommandNode::~NavCommandNode() {
    if (writer_) {
      writer_->stop();
    }
  }

  void NavCommandNode::onCmdVel(
      const geometry_msgs::msg::Twist::SharedPtr message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    command_ = *message;
    has_command_ = true;
    last_command_time_ = std::chrono::steady_clock::now();
  }

  void NavCommandNode::onPitchMode(
      const std_msgs::msg::UInt8::SharedPtr message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 当前按旧协议语义把过洞模式直接写入
    // deform_control，位定义由电控确认后再细分。
    pitch_mode_ = message->data;
  }

  void NavCommandNode::onOdometry(
      const nav_msgs::msg::Odometry::SharedPtr message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    position_.x = message->pose.pose.position.x;
    position_.y = message->pose.pose.position.y;
    position_.z = message->pose.pose.position.z;
    orientation_.w = message->pose.pose.orientation.w;
    orientation_.x = message->pose.pose.orientation.x;
    orientation_.y = message->pose.pose.orientation.y;
    orientation_.z = message->pose.pose.orientation.z;
    body_velocity_.x = message->twist.twist.linear.x;
    body_velocity_.y = message->twist.twist.linear.y;
    body_velocity_.z = message->twist.twist.linear.z;
    has_odometry_ = true;
  }

  void NavCommandNode::assembleAndSubmit() {
    s2c::serial_comm::NavWrite write{};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      write.serial.deform_chassis.deform_control = pitch_mode_;

      const auto now = std::chrono::steady_clock::now();
      const bool is_fresh = has_command_
                            && now - last_command_time_
                                   <= std::chrono::duration<double>(
                                       command_timeout_sec_);
      if (is_fresh) {
        write.serial.deform_chassis.displacement_target_velocity_x =
            static_cast<float>(vel_trans_scale_ * command_.linear.x);
        write.serial.deform_chassis.displacement_target_velocity_y =
            static_cast<float>(vel_trans_scale_ * command_.linear.y);
        write.serial.deform_chassis.displacement_target_velocity_yaw =
            static_cast<float>(vel_yaw_scale_ * command_.angular.z);
      }

      if (has_odometry_) {
        write.nav_to_vision.current_position[0] = static_cast<float>(
            position_.x);
        write.nav_to_vision.current_position[1] = static_cast<float>(
            position_.y);
        write.nav_to_vision.current_position[2] = static_cast<float>(
            position_.z);
        const Vector3 world_velocity = rotate(orientation_, body_velocity_);
        write.nav_to_vision.current_velocity[0] = static_cast<float>(
            world_velocity.x);
        write.nav_to_vision.current_velocity[1] = static_cast<float>(
            world_velocity.y);
        write.nav_to_vision.current_velocity[2] = static_cast<float>(
            world_velocity.z);
        write.nav_to_vision.orientation_quaternion[0] = static_cast<float>(
            orientation_.w);
        write.nav_to_vision.orientation_quaternion[1] = static_cast<float>(
            orientation_.x);
        write.nav_to_vision.orientation_quaternion[2] = static_cast<float>(
            orientation_.y);
        write.nav_to_vision.orientation_quaternion[3] = static_cast<float>(
            orientation_.z);
      }
    }
    writer_->setCommand(write);
  }

  void NavCommandNode::logWriterHealth() {
    const NavCommandWriter::Stats stats = writer_->stats();
    if (stats.failures > last_failure_count_) {
      last_failure_count_ = stats.failures;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "写入共享内存失败，检查串口通信服务端与 shm_name");
    }
  }

  NavCommandNode::Vector3 NavCommandNode::rotate(const Quaternion& orientation,
                                                 const Vector3& vector) {
    const double tx = 2.0
                      * (orientation.y * vector.z - orientation.z * vector.y);
    const double ty = 2.0
                      * (orientation.z * vector.x - orientation.x * vector.z);
    const double tz = 2.0
                      * (orientation.x * vector.y - orientation.y * vector.x);
    return {vector.x + orientation.w * tx
                + (orientation.y * tz - orientation.z * ty),
            vector.y + orientation.w * ty
                + (orientation.z * tx - orientation.x * tz),
            vector.z + orientation.w * tz
                + (orientation.x * ty - orientation.y * tx)};
  }

}  // namespace sentry_comm

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sentry_comm::NavCommandNode>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("sentry_comm_nav_command"), "启动失败: %s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
