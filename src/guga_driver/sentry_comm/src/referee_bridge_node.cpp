#include "sentry_comm/referee_bridge_node.hpp"

#include "referee_mapping.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace sentry_comm {

  RefereeBridgeNode::RefereeBridgeNode() : Node("sentry_comm_referee_bridge") {
    const std::string shm_name = declare_parameter<std::string>(
        "shm_name", "/s2c_serial_comm");
    const double read_rate_hz = declare_parameter<double>("read_rate_hz",
                                                          100.0);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz",
                                                             50.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 1.0);
    const std::string robot_status_topic = declare_parameter<std::string>(
        "robot_status_topic", "/referee/robot_status");
    const std::string game_status_topic = declare_parameter<std::string>(
        "game_status_topic", "/referee/game_status");
    const std::string rfid_status_topic = declare_parameter<std::string>(
        "rfid_status_topic", "/referee/rfid_status");
    const std::string joint_state_topic = declare_parameter<std::string>(
        "joint_state_topic", "/serial/gimbal_joint_state");
    publish_joint_state_ = declare_parameter<bool>("publish_joint_state", true);
    joint_name_ = declare_parameter<std::string>("joint_name",
                                                 "gimbal_yaw_odom_joint");
    invert_joint_angle_ = declare_parameter<bool>("invert_joint_angle", true);

    if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz 必须为有限正值");
    }
    if (!std::isfinite(stale_timeout_sec_) || stale_timeout_sec_ < 0.0) {
      throw std::invalid_argument("stale_timeout_sec 不能为负");
    }

    SnapshotReader::Config config;
    config.shm_name = shm_name;
    config.read_rate_hz = read_rate_hz;
    reader_ = std::make_unique<SnapshotReader>(config);
    reader_->start();

    robot_status_publisher_ =
        create_publisher<guga_interfaces::msg::RobotStatus>(robot_status_topic,
                                                            10);
    game_status_publisher_ = create_publisher<guga_interfaces::msg::GameStatus>(
        game_status_topic, 10);
    rfid_status_publisher_ = create_publisher<guga_interfaces::msg::RfidStatus>(
        rfid_status_topic, 10);
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic, 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));
    publish_timer_ = create_wall_timer(period, [this] { publishReferee(); });

    RCLCPP_INFO(get_logger(),
                "裁判系统桥接：共享内存 %s，读取 %.0f Hz，发布 %.0f Hz",
                shm_name.c_str(), read_rate_hz, publish_rate_hz);
  }

  void RefereeBridgeNode::publishReferee() {
    s2c::serial_comm::AllInfo info{};
    if (!reader_->latestSnapshot(info)) {
      return;
    }
    const double last_mcu_time = info.server_info.last_mcu_receive_time;
    if (last_mcu_time <= 0.0) {
      return;
    }
    const double now = s2c::serial_comm::steadySeconds(
        std::chrono::steady_clock::now());
    if (now - last_mcu_time > stale_timeout_sec_) {
      return;
    }

    const std::uint16_t current_hp = info.mcu_data.robot_status.current_HP;
    robot_status_publisher_->publish(toRobotStatus(
        info.mcu_data, has_previous_hp_ ? previous_hp_ : current_hp));
    game_status_publisher_->publish(toGameStatus(info.mcu_data));
    rfid_status_publisher_->publish(toRfidStatus(info.mcu_data));
    if (publish_joint_state_) {
      joint_state_publisher_->publish(
          toJointState(info.mcu_data, joint_name_, invert_joint_angle_));
    }

    previous_hp_ = current_hp;
    has_previous_hp_ = true;
  }

}  // namespace sentry_comm

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sentry_comm::RefereeBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("sentry_comm_referee_bridge"),
                 "启动失败: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
