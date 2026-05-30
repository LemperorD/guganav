#include <memory>
#include <stdexcept>

#include "simple_decision/adapter/transform.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "gtest/gtest.h"
#include "guga_interfaces/msg/game_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"

namespace simple_decision {

  // ═══════════════════════════════════════════════════════
  // Converter tests — covers transform.cpp
  // ═══════════════════════════════════════════════════════

  TEST(ConvertQuaternionTest, CopiesAllFieldsFromRosQuaternion) {
    geometry_msgs::msg::Quaternion ros;
    ros.x = 1.0;
    ros.y = 2.0;
    ros.z = 3.0;
    ros.w = 4.0;

    auto q = ConvertQuaternion(ros);
    EXPECT_DOUBLE_EQ(q.x, 1.0);
    EXPECT_DOUBLE_EQ(q.y, 2.0);
    EXPECT_DOUBLE_EQ(q.z, 3.0);
    EXPECT_DOUBLE_EQ(q.w, 4.0);
  }

  TEST(ConvertPoseTest, CopiesAllFieldsFromRosPose) {
    geometry_msgs::msg::Pose ros;
    ros.position.x = 1.0;
    ros.position.y = 2.0;
    ros.position.z = 3.0;
    ros.orientation.x = 0.0;
    ros.orientation.y = 0.0;
    ros.orientation.z = 0.0;
    ros.orientation.w = 1.0;

    auto p = ConvertPose(ros);
    EXPECT_DOUBLE_EQ(p.position.x, 1.0);
    EXPECT_DOUBLE_EQ(p.position.y, 2.0);
    EXPECT_DOUBLE_EQ(p.position.z, 3.0);
    EXPECT_DOUBLE_EQ(p.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(p.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(p.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(p.orientation.w, 1.0);
  }

  TEST(ConvertPointTest, CopiesAllFieldsFromRosPoint) {
    geometry_msgs::msg::Point ros;
    ros.x = 1.0;
    ros.y = 2.0;
    ros.z = 3.0;

    auto p = ConvertPoint(ros);
    EXPECT_DOUBLE_EQ(p.x, 1.0);
    EXPECT_DOUBLE_EQ(p.y, 2.0);
    EXPECT_DOUBLE_EQ(p.z, 3.0);
  }

  TEST(ConvertRobotStatusTest, CopiesScalarFieldsFromRosMessage) {
    auto ros = std::make_shared<guga_interfaces::msg::RobotStatus>();
    ros->robot_id = 5;
    ros->robot_level = 3;
    ros->current_hp = 400;
    ros->maximum_hp = 500;
    ros->shooter_barrel_cooling_value = 10;
    ros->shooter_barrel_heat_limit = 100;
    ros->shooter_17mm_1_barrel_heat = 20;
    ros->armor_id = 2;
    ros->hp_deduction_reason = 1;
    ros->projectile_allowance_17mm = 50;
    ros->remaining_gold_coin = 100;
    ros->is_hp_deduced = true;

    auto rs = ConvertRobotStatus(ros);
    EXPECT_EQ(rs.robot_id, 5);
    EXPECT_EQ(rs.robot_level, 3);
    EXPECT_EQ(rs.current_hp, 400);
    EXPECT_EQ(rs.maximum_hp, 500);
    EXPECT_EQ(rs.shooter_barrel_cooling_value, 10);
    EXPECT_EQ(rs.shooter_barrel_heat_limit, 100);
    EXPECT_EQ(rs.shooter_17mm_1_barrel_heat, 20);
    EXPECT_EQ(rs.armor_id, 2);
    EXPECT_EQ(rs.hp_deduction_reason, 1);
    EXPECT_EQ(rs.projectile_allowance_17mm, 50);
    EXPECT_EQ(rs.remaining_gold_coin, 100);
    EXPECT_TRUE(rs.is_hp_deduced);
  }

  TEST(ConvertRobotStatusTest, ThrowsOnNullptr) {
    EXPECT_THROW(ConvertRobotStatus(nullptr), std::invalid_argument);
  }

  TEST(ConvertGameStatusTest, CopiesGameProgressFromRosMessage) {
    auto ros = std::make_shared<guga_interfaces::msg::GameStatus>();
    ros->game_progress = guga_interfaces::msg::GameStatus::RUNNING;

    auto gs = ConvertGameStatus(ros);
    EXPECT_EQ(gs.game_progress, GameStatus::RUNNING);
  }

  TEST(ConvertArmorsTest, CopiesAllArmorFields) {
    auto ros = std::make_shared<guga_interfaces::msg::Armors>();
    ros->header.stamp.sec = 10;
    ros->header.stamp.nanosec = 500u;
    ros->header.frame_id = "map";
    guga_interfaces::msg::Armor ra;
    ra.pose.position.x = 1.0;
    ra.pose.position.y = 2.0;
    ra.pose.position.z = 3.0;
    ra.pose.orientation.w = 1.0;
    ros->armors.push_back(ra);

    auto a = ConvertArmors(ros);
    EXPECT_EQ(a.header.stamp.sec, 10);
    EXPECT_EQ(a.header.stamp.nanosec, 500u);
    EXPECT_EQ(a.header.frame_id, "map");
    ASSERT_EQ(a.armors.size(), 1u);
    EXPECT_DOUBLE_EQ(a.armors[0].pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(a.armors[0].pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(a.armors[0].pose.position.z, 3.0);
    EXPECT_DOUBLE_EQ(a.armors[0].pose.orientation.w, 1.0);
  }

  TEST(ConvertArmorsTest, ThrowsOnNullptr) {
    EXPECT_THROW(ConvertArmors(nullptr), std::invalid_argument);
  }

  TEST(ConvertArmorsTest, HandlesEmptyArmorList) {
    auto ros = std::make_shared<guga_interfaces::msg::Armors>();
    auto a = ConvertArmors(ros);
    EXPECT_TRUE(a.armors.empty());
  }

  TEST(ConvertTargetTest, CopiesAllFieldsFromRosMessage) {
    auto ros = std::make_shared<guga_interfaces::msg::Target>();
    ros->header.stamp.sec = 1;
    ros->header.stamp.nanosec = 2u;
    ros->header.frame_id = "odom";
    ros->position.x = 3.0;
    ros->position.y = 4.0;
    ros->position.z = 5.0;
    ros->yaw = 1.57;
    ros->tracking = true;

    auto t = ConvertTarget(ros);
    EXPECT_EQ(t.header.stamp.sec, 1);
    EXPECT_EQ(t.header.stamp.nanosec, 2u);
    EXPECT_EQ(t.header.frame_id, "odom");
    EXPECT_DOUBLE_EQ(t.position.x, 3.0);
    EXPECT_DOUBLE_EQ(t.position.y, 4.0);
    EXPECT_DOUBLE_EQ(t.position.z, 5.0);
    EXPECT_DOUBLE_EQ(t.yaw, 1.57);
    EXPECT_TRUE(t.tracking);
  }

}  // namespace simple_decision
