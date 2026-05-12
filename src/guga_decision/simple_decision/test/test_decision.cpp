#include "simple_decision/core/decision.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

namespace simple_decision {

  class DecisionTest : public testing::Test {
  protected:
    DecisionTest() : dec_(DefaultConfig()) {
    }

    Decision dec_;

    Snapshot SupplyNotRecovered() {
      Snapshot s = HealthyNotAttackedSnapshot();
      s.state = State::SUPPLY;
      s.rs.current_hp = 200;
      s.rs.projectile_allowance_17mm = 50;
      return s;
    }

    Snapshot SupplyRecovered() {
      Snapshot s = HealthyNotAttackedSnapshot();
      s.state = State::SUPPLY;
      s.rs.current_hp = 400;
      s.rs.projectile_allowance_17mm = 100;
      return s;
    }
  };

  // ── computeAction ──
  TEST_F(DecisionTest, ComputeAction_SupplyNotRecovered_HoldsSupply) {
    auto a = dec_.computeAction(SupplyNotRecovered());
    EXPECT_EQ(a.next_state, State::SUPPLY);
    EXPECT_EQ(a.chassis_mode, ChassisMode::CHASSIS_FOLLOWED);
    EXPECT_TRUE(a.should_publish_goal);
  }

  TEST_F(DecisionTest, ComputeAction_SupplyRecoveredAndStatusOk_GoesToDefault) {
    auto a = dec_.computeAction(SupplyRecovered());
    EXPECT_EQ(a.next_state, State::DEFAULT);
  }

  TEST_F(DecisionTest, ComputeAction_StatusBad_EntersSupply) {
    Snapshot s = HealthyNotAttackedSnapshot();
    s.rs.current_hp = 50;

    auto a = dec_.computeAction(s);
    EXPECT_EQ(a.next_state, State::SUPPLY);
  }

  TEST_F(DecisionTest, ComputeAction_LowAmmo_EntersSupply) {
    Snapshot s = HealthyNotAttackedSnapshot();
    s.rs.projectile_allowance_17mm = 0;

    auto a = dec_.computeAction(s);
    EXPECT_EQ(a.next_state, State::SUPPLY);
  }

  TEST_F(DecisionTest, ComputeAction_EnemyRecentNotAttacked_UsesFollowed) {
    Snapshot s = EnemyRecentSnapshot();
    s.armors = Armors{};
    s.target_opt = std::nullopt;

    auto a = dec_.computeAction(s);
    EXPECT_EQ(a.next_state, State::ATTACK);
    EXPECT_EQ(a.chassis_mode, ChassisMode::CHASSIS_FOLLOWED);
    EXPECT_FALSE(a.default_spin_latched);
  }

  TEST_F(DecisionTest, ComputeAction_EnemyRecentAndAttacked_UsesLittleTes) {
    Snapshot s = EnemyRecentSnapshot();
    s.attacked_recent = true;
    s.armors = Armors{};
    s.target_opt = std::nullopt;

    auto a = dec_.computeAction(s);
    EXPECT_EQ(a.next_state, State::ATTACK);
    EXPECT_EQ(a.chassis_mode, ChassisMode::LITTLE_TES);
  }

  TEST_F(DecisionTest,
         ComputeAction_EnemyRecentWithAttackGoal_UsesAttackPosition) {
    Snapshot s = EnemyRecentSnapshot();
    s.armors = ArmorInRange();
    s.has_attack_goal = false;

    auto a = dec_.computeAction(s);
    EXPECT_EQ(a.next_state, State::ATTACK);
    EXPECT_DOUBLE_EQ(a.target_x, s.armors.armors[0].pose.position.x);
    EXPECT_DOUBLE_EQ(a.target_y, s.armors.armors[0].pose.position.y);
  }

  TEST_F(DecisionTest,
         ComputeAction_EnemyRecentWithTrackingTarget_UsesTargetPosition) {
    Snapshot s = EnemyRecentSnapshot();
    s.target_opt = TrackingTarget();
    s.has_attack_goal = false;

    auto a = dec_.computeAction(s);
    EXPECT_EQ(a.next_state, State::ATTACK);
    EXPECT_DOUBLE_EQ(a.target_x, s.target_opt->position.x);
    EXPECT_DOUBLE_EQ(a.target_y, s.target_opt->position.y);
    EXPECT_DOUBLE_EQ(a.target_yaw, s.target_opt->yaw);
  }

  TEST_F(DecisionTest, ComputeAction_Default_AtCenterAndKeepSpin_LatchesSpin) {
    Snapshot s = HealthyNotAttackedSnapshot();
    s.at_center = true;
    s.in_center_keep_spin = true;

    auto a = dec_.computeAction(s);
    EXPECT_EQ(a.next_state, State::DEFAULT);
    EXPECT_EQ(a.chassis_mode, ChassisMode::LITTLE_TES);
    EXPECT_TRUE(a.default_spin_latched);
  }

  TEST_F(DecisionTest, ComputeAction_Default_NotAtCenterInKeepSpin_NoLatch) {
    Snapshot s = HealthyNotAttackedSnapshot();
    s.at_center = false;
    s.in_center_keep_spin = true;

    auto a = dec_.computeAction(s);
    EXPECT_EQ(a.next_state, State::DEFAULT);
    EXPECT_EQ(a.chassis_mode, ChassisMode::CHASSIS_FOLLOWED);
    EXPECT_FALSE(a.default_spin_latched);
  }

  TEST_F(DecisionTest, ComputeAction_Default_OutOfKeepSpin_ClearsLatch) {
    Snapshot s = HealthyNotAttackedSnapshot();
    s.at_center = true;
    s.in_center_keep_spin = false;

    auto a = dec_.computeAction(s);
    EXPECT_FALSE(a.default_spin_latched);
    EXPECT_EQ(a.chassis_mode, ChassisMode::CHASSIS_FOLLOWED);
  }

  TEST_F(DecisionTest, ComputeAction_Default_AttackedRecent_UsesLittleTes) {
    auto a = dec_.computeAction(AttackedRecentSnapshot());
    EXPECT_EQ(a.next_state, State::DEFAULT);
    EXPECT_EQ(a.chassis_mode, ChassisMode::LITTLE_TES);
  }

  TEST_F(DecisionTest, ComputeAction_Default_UsesDefaultCoordinates) {
    Snapshot s = HealthyNotAttackedSnapshot();

    auto a = dec_.computeAction(s);
    EXPECT_DOUBLE_EQ(a.target_x, kDefaultX);
    EXPECT_DOUBLE_EQ(a.target_y, kDefaultY);
    EXPECT_DOUBLE_EQ(a.target_yaw, kDefaultYaw);
  }

  // ── isStatusRecovered ──
  TEST_F(DecisionTest,
         IsStatusRecovered_HpAboveExitAndAmmoAboveMin_ReturnsTrue) {
    EXPECT_TRUE(dec_.isStatusRecovered(HealthyRobotStatus()));
  }

  TEST_F(DecisionTest, IsStatusRecovered_HpBelowExit_ReturnsFalse) {
    RobotStatus rs = HealthyRobotStatus();
    rs.current_hp = 200;
    EXPECT_FALSE(dec_.isStatusRecovered(rs));
  }

  TEST_F(DecisionTest, IsStatusRecovered_AmmoAtMin_ReturnsFalse) {
    RobotStatus rs = HealthyRobotStatus();
    rs.projectile_allowance_17mm = 0;
    EXPECT_FALSE(dec_.isStatusRecovered(rs));
  }

  // ── isStatusBad ──
  TEST_F(DecisionTest, IsStatusBad_Healthy_ReturnsFalse) {
    EXPECT_FALSE(dec_.isStatusBad(HealthyRobotStatus()));
  }

  TEST_F(DecisionTest, IsStatusBad_HpBelowEnter_ReturnsTrue) {
    EXPECT_TRUE(dec_.isStatusBad(LowHpRobotStatus()));
  }

  TEST_F(DecisionTest, IsStatusBad_AmmoAtMin_ReturnsTrue) {
    EXPECT_TRUE(dec_.isStatusBad(LowAmmoRobotStatus()));
  }

  // ── findAttackPosition ──
  TEST_F(DecisionTest, FindAttackPosition_TrackingTarget_UsesTargetPose) {
    auto target = TrackingTarget();
    target.position.x = 1.0;
    target.position.y = 2.0;
    target.position.z = 3.0;
    target.yaw = 1.57;

    auto result = Decision::findAttackPosition(Armors{}, target);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->x, 1.0);
    EXPECT_DOUBLE_EQ(result->y, 2.0);
    EXPECT_DOUBLE_EQ(result->yaw, 1.57);
  }

  TEST_F(DecisionTest, FindAttackPosition_NoTargetWithArmors_PicksClosest) {
    Armors armors;
    Armor far;
    far.pose.position.x = 10.0;
    far.pose.position.y = 0.0;
    far.pose.position.z = 0.0;
    Armor close;
    close.pose.position.x = 2.0;
    close.pose.position.y = 1.0;
    close.pose.position.z = 0.0;
    armors.armors = {far, close};

    auto result = Decision::findAttackPosition(armors, std::nullopt);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->x, 2.0);
    EXPECT_DOUBLE_EQ(result->y, 1.0);
    EXPECT_DOUBLE_EQ(result->yaw, 0.0);
  }

  TEST_F(DecisionTest, FindAttackPosition_NoTargetNoArmors_ReturnsNullopt) {
    auto result = Decision::findAttackPosition(Armors{}, std::nullopt);
    EXPECT_FALSE(result.has_value());
  }

  TEST_F(DecisionTest,
         FindAttackPosition_NonTrackingTargetWithArmors_UsesArmor) {
    Armors armors;
    Armor a;
    a.pose.position.x = 3.0;
    a.pose.position.y = 0.0;
    a.pose.position.z = 0.0;
    armors.armors.push_back(a);

    auto target = NonTrackingTarget();
    target.tracking = false;

    auto result = Decision::findAttackPosition(armors, target);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->x, 3.0);
  }

}  // namespace simple_decision
