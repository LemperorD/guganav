#include "referee_mapping.hpp"

namespace sentry_comm {

  guga_interfaces::msg::RobotStatus toRobotStatus(
      const s2c::com::mcu::ReceivePacket& packet, std::uint16_t previous_hp) {
    guga_interfaces::msg::RobotStatus message;
    message.robot_id = packet.robot_status.robot_id;
    message.robot_level = packet.robot_status.robot_level;
    message.current_hp = packet.robot_status.current_HP;
    message.maximum_hp = packet.robot_status.maximum_HP;
    message.shooter_barrel_cooling_value =
        packet.robot_status.shooter_barrel_cooling_value;
    message.shooter_barrel_heat_limit =
        packet.robot_status.shooter_barrel_heat_limit;
    message.shooter_17mm_1_barrel_heat =
        packet.power_heat_data.shooter_17mm_barrel_heat;
    message.armor_id = packet.hurt_data.armor_id;
    message.hp_deduction_reason = packet.hurt_data.HP_deduction_reason;
    message.is_hp_deduced = packet.robot_status.current_HP < previous_hp;
    // 电控反馈里没有 0x0208（允许发弹量与剩余金币），这两个字段保持 0。
    return message;
  }

  guga_interfaces::msg::GameStatus toGameStatus(
      const s2c::com::mcu::ReceivePacket& packet) {
    guga_interfaces::msg::GameStatus message;
    message.game_progress = packet.game_status.game_progress;
    message.stage_remain_time = packet.game_status.stage_remain_time;
    return message;
  }

  guga_interfaces::msg::RfidStatus toRfidStatus(
      const s2c::com::mcu::ReceivePacket& packet) {
    const std::uint32_t bits = packet.rfid_status.rfid_status;
    guga_interfaces::msg::RfidStatus message;
    message.base_gain_point = (bits >> 0) & 0x01;
    message.central_highland_gain_point = (bits >> 1) & 0x01;
    message.enemy_central_highland_gain_point = (bits >> 2) & 0x01;
    message.friendly_trapezoidal_highland_gain_point = (bits >> 3) & 0x01;
    message.enemy_trapezoidal_highland_gain_point = (bits >> 4) & 0x01;
    message.friendly_fly_ramp_front_gain_point = (bits >> 5) & 0x01;
    message.friendly_fly_ramp_back_gain_point = (bits >> 6) & 0x01;
    message.enemy_fly_ramp_front_gain_point = (bits >> 7) & 0x01;
    message.enemy_fly_ramp_back_gain_point = (bits >> 8) & 0x01;
    message.friendly_central_highland_lower_gain_point = (bits >> 9) & 0x01;
    message.friendly_central_highland_upper_gain_point = (bits >> 10) & 0x01;
    message.enemy_central_highland_lower_gain_point = (bits >> 11) & 0x01;
    message.enemy_central_highland_upper_gain_point = (bits >> 12) & 0x01;
    message.friendly_highway_lower_gain_point = (bits >> 13) & 0x01;
    message.friendly_highway_upper_gain_point = (bits >> 14) & 0x01;
    message.enemy_highway_lower_gain_point = (bits >> 15) & 0x01;
    message.enemy_highway_upper_gain_point = (bits >> 16) & 0x01;
    message.friendly_fortress_gain_point = (bits >> 17) & 0x01;
    message.friendly_outpost_gain_point = (bits >> 18) & 0x01;
    message.friendly_supply_zone_non_exchange = (bits >> 19) & 0x01;
    message.friendly_supply_zone_exchange = (bits >> 20) & 0x01;
    message.friendly_big_resource_island = (bits >> 21) & 0x01;
    message.enemy_big_resource_island = (bits >> 22) & 0x01;
    message.center_gain_point = (bits >> 23) & 0x01;
    return message;
  }

  sensor_msgs::msg::JointState toJointState(
      const s2c::com::mcu::ReceivePacket& packet,
      const std::string& joint_name, bool invert) {
    sensor_msgs::msg::JointState message;
    message.name = {joint_name};
    message.position = {invert ? -packet.yaw_big_position
                               : packet.yaw_big_position};
    return message;
  }

}  // namespace sentry_comm
