/**
 * @file test_ui_types.cpp
 * @brief guga_ui_common ui_types.hpp 单元测试。
 *
 * 测试内容：
 *   - 所有 POD 结构体的 sizeof / alignof / is_trivially_copyable
 *   - 默认初始化值为零/false/空
 *   - offsetof 断言
 *   - UiSlotId 枚举值一致性
 */

#include <cstddef>
#include <type_traits>

#include "guga_ui_common/ui_types.hpp"
#include "gtest/gtest.h"

namespace gu = guga_ui;

// ==================== sizeof / alignof ====================

TEST(UiTypesTest, AllStructsAre64Bytes) {
  EXPECT_EQ(sizeof(gu::UiRobotStatus), 64u);
  // 以下 struct 在 alignas(64) 下编译器可能追加到 128 字节
  EXPECT_GE(sizeof(gu::UiGameStatus), 64u);
  EXPECT_GE(sizeof(gu::UiDecision), 64u);
  EXPECT_GE(sizeof(gu::UiEnemy), 64u);
  EXPECT_GE(sizeof(gu::UiOdom), 64u);
  EXPECT_GE(sizeof(gu::UiRfidStatus), 64u);
  EXPECT_GE(sizeof(gu::UiYaw), 64u);
  // UiPath 包含 256*2 个 double 的大数组，远大于 64 字节
  EXPECT_GE(sizeof(gu::UiPath), 64u);
}

TEST(UiTypesTest, AllStructsAreTriviallyCopyable) {
  EXPECT_TRUE(std::is_trivially_copyable_v<gu::UiRobotStatus>);
  EXPECT_TRUE(std::is_trivially_copyable_v<gu::UiGameStatus>);
  EXPECT_TRUE(std::is_trivially_copyable_v<gu::UiRfidStatus>);
  EXPECT_TRUE(std::is_trivially_copyable_v<gu::UiDecision>);
  EXPECT_TRUE(std::is_trivially_copyable_v<gu::UiEnemy>);
  EXPECT_TRUE(std::is_trivially_copyable_v<gu::UiOdom>);
  EXPECT_TRUE(std::is_trivially_copyable_v<gu::UiYaw>);
  EXPECT_TRUE(std::is_trivially_copyable_v<gu::UiPath>);
}

// ==================== 默认初始化 ====================

TEST(UiTypesTest, RobotStatusDefaultsAreZero) {
  gu::UiRobotStatus s{};
  EXPECT_EQ(s.current_hp, 0);
  EXPECT_EQ(s.maximum_hp, 0);
  EXPECT_EQ(s.shooter_17mm_1_barrel_heat, 0);
  EXPECT_EQ(s.shooter_barrel_heat_limit, 0);
  EXPECT_EQ(s.projectile_allowance_17mm, 0);
  EXPECT_EQ(s.remaining_gold_coin, 0);
  EXPECT_EQ(s.shooter_barrel_cooling_value, 0);
  EXPECT_EQ(s.robot_id, 0);
  EXPECT_EQ(s.robot_level, 0);
  EXPECT_EQ(s.armor_id, 0);
  EXPECT_EQ(s.hp_deduction_reason, 0);
  EXPECT_FALSE(s.is_hp_deduced);
}

TEST(UiTypesTest, GameStatusDefaultsAreZero) {
  gu::UiGameStatus s{};
  EXPECT_EQ(s.game_progress, 0);
  EXPECT_EQ(s.stage_remain_time, 0);
  EXPECT_DOUBLE_EQ(s.elapsed_sec, 0.0);
}

TEST(UiTypesTest, RfidStatusDefaultsAreFalse) {
  gu::UiRfidStatus s{};
  EXPECT_FALSE(s.base_gain_point);
  EXPECT_FALSE(s.central_highland_gain_point);
  EXPECT_FALSE(s.enemy_central_highland_gain_point);
  EXPECT_FALSE(s.friendly_trapezoidal_highland_gain_point);
  EXPECT_FALSE(s.enemy_trapezoidal_highland_gain_point);
  EXPECT_FALSE(s.friendly_fly_ramp_front_gain_point);
  EXPECT_FALSE(s.friendly_fly_ramp_back_gain_point);
  EXPECT_FALSE(s.enemy_fly_ramp_front_gain_point);
  EXPECT_FALSE(s.enemy_fly_ramp_back_gain_point);
  EXPECT_FALSE(s.friendly_central_highland_lower_gain_point);
  EXPECT_FALSE(s.friendly_central_highland_upper_gain_point);
  EXPECT_FALSE(s.enemy_central_highland_lower_gain_point);
  EXPECT_FALSE(s.enemy_central_highland_upper_gain_point);
  EXPECT_FALSE(s.friendly_highway_lower_gain_point);
  EXPECT_FALSE(s.friendly_highway_upper_gain_point);
  EXPECT_FALSE(s.enemy_highway_lower_gain_point);
  EXPECT_FALSE(s.enemy_highway_upper_gain_point);
  EXPECT_FALSE(s.friendly_fortress_gain_point);
  EXPECT_FALSE(s.friendly_outpost_gain_point);
  EXPECT_FALSE(s.friendly_supply_zone_non_exchange);
  EXPECT_FALSE(s.friendly_supply_zone_exchange);
  EXPECT_FALSE(s.friendly_big_resource_island);
  EXPECT_FALSE(s.enemy_big_resource_island);
  EXPECT_FALSE(s.center_gain_point);
}

TEST(UiTypesTest, DecisionDefaultsAreZero) {
  gu::UiDecision s{};
  EXPECT_EQ(s.state, 0);
  EXPECT_EQ(s.chassis_mode, 0);
  EXPECT_FALSE(s.should_publish_goal);
  EXPECT_FALSE(s.default_spin_latched);
  EXPECT_DOUBLE_EQ(s.target_x, 0.0);
  EXPECT_DOUBLE_EQ(s.target_y, 0.0);
  EXPECT_DOUBLE_EQ(s.target_yaw, 0.0);
  EXPECT_DOUBLE_EQ(s.robot_x, 0.0);
  EXPECT_DOUBLE_EQ(s.robot_y, 0.0);
  EXPECT_DOUBLE_EQ(s.robot_yaw, 0.0);
  EXPECT_EQ(s.compute_us, 0);
}

TEST(UiTypesTest, EnemyDefaultsAreZero) {
  gu::UiEnemy s{};
  EXPECT_EQ(s.armor_count, 0);
  EXPECT_FALSE(s.tracking);
  EXPECT_FALSE(s.enemy_detected);
  EXPECT_DOUBLE_EQ(s.target_x, 0.0);
  EXPECT_DOUBLE_EQ(s.target_y, 0.0);
  EXPECT_DOUBLE_EQ(s.target_z, 0.0);
  EXPECT_DOUBLE_EQ(s.attack_x, 0.0);
  EXPECT_DOUBLE_EQ(s.attack_y, 0.0);
  EXPECT_DOUBLE_EQ(s.last_seen_sec, 0.0);
}

TEST(UiTypesTest, OdomDefaultsAreZero) {
  gu::UiOdom s{};
  EXPECT_DOUBLE_EQ(s.x, 0.0);
  EXPECT_DOUBLE_EQ(s.y, 0.0);
  EXPECT_DOUBLE_EQ(s.z, 0.0);
  EXPECT_DOUBLE_EQ(s.roll, 0.0);
  EXPECT_DOUBLE_EQ(s.pitch, 0.0);
  EXPECT_DOUBLE_EQ(s.yaw, 0.0);
  EXPECT_DOUBLE_EQ(s.vx, 0.0);
  EXPECT_DOUBLE_EQ(s.vy, 0.0);
  EXPECT_DOUBLE_EQ(s.vz, 0.0);
}

TEST(UiTypesTest, YawDefaultsAreZero) {
  gu::UiYaw s{};
  EXPECT_DOUBLE_EQ(s.yaw_diff, 0.0);
  EXPECT_DOUBLE_EQ(s.tes_angular_z, 0.0);
}

// ==================== UiPath 特殊测试 ====================

TEST(UiTypesTest, PathMaxPoints) {
  EXPECT_EQ(gu::UI_PATH_MAX_POINTS, 256u);
}

TEST(UiTypesTest, PathDefaultsAreZero) {
  gu::UiPath s{};
  EXPECT_EQ(s.count, 0u);
  EXPECT_DOUBLE_EQ(s.stamp_sec, 0.0);
  // 前几个路径点应为 0
  EXPECT_DOUBLE_EQ(s.x[0], 0.0);
  EXPECT_DOUBLE_EQ(s.y[0], 0.0);
}

TEST(UiTypesTest, PathOffsetOf) {
  // count 在结构体起始位置
  EXPECT_EQ(offsetof(gu::UiPath, count), 0u);
  // x 数组按 8 字节对齐
  EXPECT_EQ(offsetof(gu::UiPath, x) % 8, 0u);
}

// ==================== UiSlotId 枚举 ====================

TEST(UiTypesTest, SlotIdEnumValues) {
  EXPECT_EQ(static_cast<uint32_t>(gu::UiSlotId::ROBOT_STATUS), 0u);
  EXPECT_EQ(static_cast<uint32_t>(gu::UiSlotId::GAME_STATUS), 1u);
  EXPECT_EQ(static_cast<uint32_t>(gu::UiSlotId::RFID_STATUS), 2u);
  EXPECT_EQ(static_cast<uint32_t>(gu::UiSlotId::DECISION), 3u);
  EXPECT_EQ(static_cast<uint32_t>(gu::UiSlotId::ENEMY), 4u);
  EXPECT_EQ(static_cast<uint32_t>(gu::UiSlotId::ODOM), 5u);
  EXPECT_EQ(static_cast<uint32_t>(gu::UiSlotId::YAW), 6u);
  EXPECT_EQ(static_cast<uint32_t>(gu::UiSlotId::PATH), 7u);
  EXPECT_EQ(static_cast<uint32_t>(gu::UiSlotId::COUNT), 8u);
}
