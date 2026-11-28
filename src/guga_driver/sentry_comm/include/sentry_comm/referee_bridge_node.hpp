#pragma once

#include "sentry_comm/snapshot_reader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <guga_interfaces/msg/game_status.hpp>
#include <guga_interfaces/msg/rfid_status.hpp>
#include <guga_interfaces/msg/robot_status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cstdint>
#include <memory>

namespace sentry_comm {

  // 从共享内存快照里取电控反馈的裁判系统数据段，发布成决策层使用的话题。
  class RefereeBridgeNode : public rclcpp::Node {
  public:
    RefereeBridgeNode();

  private:
    void publishReferee();

    std::unique_ptr<SnapshotReader> reader_;

    rclcpp::Publisher<guga_interfaces::msg::RobotStatus>::SharedPtr
        robot_status_publisher_;
    rclcpp::Publisher<guga_interfaces::msg::GameStatus>::SharedPtr
        game_status_publisher_;
    rclcpp::Publisher<guga_interfaces::msg::RfidStatus>::SharedPtr
        rfid_status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    double stale_timeout_sec_{};
    std::uint16_t previous_hp_{};
    bool has_previous_hp_{};
    bool publish_joint_state_{};
    bool invert_joint_angle_{};
    std::string joint_name_;
  };

}  // namespace sentry_comm
