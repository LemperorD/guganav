#include "referee_mapping.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

  s2c::com::mcu::ReceivePacket makePacket() {
    s2c::com::mcu::ReceivePacket packet{};
    packet.robot_status.robot_id = 7;
    packet.robot_status.robot_level = 3;
    packet.robot_status.current_HP = 200;
    packet.robot_status.maximum_HP = 400;
    packet.robot_status.shooter_barrel_cooling_value = 60;
    packet.robot_status.shooter_barrel_heat_limit = 240;
    packet.power_heat_data.shooter_17mm_barrel_heat = 120;
    packet.hurt_data.armor_id = 5;
    packet.hurt_data.HP_deduction_reason = 0;
    packet.game_status.game_progress = 4;
    packet.game_status.stage_remain_time = 123;
    return packet;
  }

}  // namespace

TEST(RefereeMappingTest, RobotStatusFields) {
  const s2c::com::mcu::ReceivePacket packet = makePacket();
  const guga_interfaces::msg::RobotStatus message = sentry_comm::toRobotStatus(
      packet, 200);

  EXPECT_EQ(message.robot_id, 7);
  EXPECT_EQ(message.robot_level, 3);
  EXPECT_EQ(message.current_hp, 200);
  EXPECT_EQ(message.maximum_hp, 400);
  EXPECT_EQ(message.shooter_barrel_cooling_value, 60);
  EXPECT_EQ(message.shooter_barrel_heat_limit, 240);
  EXPECT_EQ(message.shooter_17mm_1_barrel_heat, 120);
  EXPECT_EQ(message.armor_id, 5);
  EXPECT_EQ(message.hp_deduction_reason, 0);
  EXPECT_FALSE(message.is_hp_deduced);
  EXPECT_EQ(message.projectile_allowance_17mm, 0);
}

TEST(RefereeMappingTest, HpDeductionComparesPreviousSample) {
  const s2c::com::mcu::ReceivePacket packet = makePacket();
  EXPECT_TRUE(sentry_comm::toRobotStatus(packet, 250).is_hp_deduced);
  EXPECT_FALSE(sentry_comm::toRobotStatus(packet, 200).is_hp_deduced);
  EXPECT_FALSE(sentry_comm::toRobotStatus(packet, 150).is_hp_deduced);
}

TEST(RefereeMappingTest, GameStatusFields) {
  const s2c::com::mcu::ReceivePacket packet = makePacket();
  const guga_interfaces::msg::GameStatus message = sentry_comm::toGameStatus(
      packet);

  EXPECT_EQ(message.game_progress, 4);
  EXPECT_EQ(message.stage_remain_time, 123);
}

TEST(RefereeMappingTest, RfidBits) {
  s2c::com::mcu::ReceivePacket packet = makePacket();
  packet.rfid_status.rfid_status = (1u << 0) | (1u << 3) | (1u << 22)
                                   | (1u << 23);
  const guga_interfaces::msg::RfidStatus message = sentry_comm::toRfidStatus(
      packet);

  EXPECT_TRUE(message.base_gain_point);
  EXPECT_TRUE(message.friendly_trapezoidal_highland_gain_point);
  EXPECT_TRUE(message.enemy_big_resource_island);
  EXPECT_TRUE(message.center_gain_point);
  EXPECT_FALSE(message.central_highland_gain_point);
  EXPECT_FALSE(message.friendly_outpost_gain_point);
}

TEST(RefereeMappingTest, JointStateMapping) {
  s2c::com::mcu::ReceivePacket packet = makePacket();
  packet.yaw_big_position = 1.5;
  const sensor_msgs::msg::JointState inverted = sentry_comm::toJointState(
      packet, "gimbal_yaw_odom_joint", true);
  const sensor_msgs::msg::JointState direct = sentry_comm::toJointState(
      packet, "gimbal_yaw_odom_joint", false);

  ASSERT_EQ(inverted.name.size(), 1u);
  EXPECT_EQ(inverted.name[0], "gimbal_yaw_odom_joint");
  ASSERT_EQ(inverted.position.size(), 1u);
  EXPECT_DOUBLE_EQ(inverted.position[0], -1.5);
  ASSERT_EQ(direct.position.size(), 1u);
  EXPECT_DOUBLE_EQ(direct.position[0], 1.5);
}
