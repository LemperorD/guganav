#pragma once

#include "guga_interfaces/msg/game_status.hpp"
#include "guga_interfaces/msg/rfid_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"
#include "serial_comm/SerialCommProtocol.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <cstdint>
#include <string>

namespace sentry_comm {

  // 把电控反馈里的裁判系统数据段转换成决策层使用的话题消息。
  guga_interfaces::msg::RobotStatus toRobotStatus(
      const s2c::com::mcu::ReceivePacket& packet, std::uint16_t previous_hp);

  guga_interfaces::msg::GameStatus toGameStatus(
      const s2c::com::mcu::ReceivePacket& packet);

  guga_interfaces::msg::RfidStatus toRfidStatus(
      const s2c::com::mcu::ReceivePacket& packet);

  // 新模型只有一个云台到底盘的转动关节，取大 yaw 编码器角度。
  // invert 用于对齐模型关节轴方向，旧驱动的 yaw 差与关节轴相反，默认取反。
  sensor_msgs::msg::JointState toJointState(
      const s2c::com::mcu::ReceivePacket& packet,
      const std::string& joint_name, bool invert);

}  // namespace sentry_comm
