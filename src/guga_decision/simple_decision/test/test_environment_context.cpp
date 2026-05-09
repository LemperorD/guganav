#include "simple_decision/core/environment_context.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

namespace simple_decision {

  class EnvironmentContextTest : public testing::Test {
  protected:
    EnvironmentContextTest() : ctx_(DefaultConfig()) {
    }

    EnvironmentContext ctx_;
  };

  // ── Construction ──
  TEST_F(EnvironmentContextTest, DefaultConstructorCreatesEmptyContext) {
    EnvironmentContext default_ctx;
    EXPECT_FALSE(default_ctx.isGameStarted());
    EXPECT_FALSE(default_ctx.isGameOver());
  }

  // ── onRobotStatus ──
  TEST_F(EnvironmentContextTest, OnRobotStatus_StoresItAndSetsHasRs) {
    ctx_.onRobotStatus(HealthyRobotStatus());
    auto snap = ctx_.getSnapshot(MakeStamp(0, 0));
    EXPECT_TRUE(snap.has_rs);
  }

  // ── onGameStatus ──
  TEST_F(EnvironmentContextTest,
         OnGameStatus_NotStartedToRunning_SetsGameStarted) {
    ctx_.onGameStatus(RunningGameStatus(), 123456789LL);
    EXPECT_TRUE(ctx_.isGameStarted());
  }

  TEST_F(EnvironmentContextTest, OnGameStatus_RunningToGameOver_SetsGameOver) {
    ctx_.onGameStatus(RunningGameStatus(), 0);
    GameStatus go;
    go.game_progress = GameStatus::GAME_OVER;
    ctx_.onGameStatus(go, 999LL);

    EXPECT_TRUE(ctx_.isGameOver());
    EXPECT_FALSE(ctx_.isGameStarted());
  }

  TEST_F(EnvironmentContextTest, OnGameStatus_ResetGameOver_ClearsFlag) {
    ctx_.onGameStatus(RunningGameStatus(), 0);
    GameStatus go;
    go.game_progress = GameStatus::GAME_OVER;
    ctx_.onGameStatus(go, 0);
    ASSERT_TRUE(ctx_.isGameOver());

    ctx_.resetGameOver();
    EXPECT_FALSE(ctx_.isGameOver());
  }

  TEST_F(EnvironmentContextTest,
         OnGameStatus_AlreadyRunning_DoesNotSetGameStartedAgain) {
    ctx_.onGameStatus(RunningGameStatus(), 0);
    ASSERT_TRUE(ctx_.isGameStarted());

    ctx_.onGameStatus(RunningGameStatus(), 999LL);
    EXPECT_TRUE(ctx_.isGameStarted());
  }

  // ── onArmors / onTarget ──
  TEST_F(EnvironmentContextTest, OnArmors_StoresArmorData) {
    ctx_.onArmors(ArmorInRange());
    auto snap = ctx_.getSnapshot(MakeStamp(0, 0));
    EXPECT_TRUE(snap.has_armors);
    EXPECT_EQ(snap.armors.armors.size(), 1u);
  }

  TEST_F(EnvironmentContextTest, OnTarget_StoresTargetData) {
    ctx_.onTarget(TrackingTarget());
    auto snap = ctx_.getSnapshot(MakeStamp(0, 0));
    EXPECT_TRUE(snap.target_opt.has_value());
    EXPECT_TRUE(snap.target_opt->tracking);
  }

  // ── detectEnemy ──
  TEST_F(EnvironmentContextTest,
         DetectEnemy_TargetTrackingAndInRange_ReturnsTrue) {
    auto t = TrackingTarget();
    t.position.x = 1.0;
    EXPECT_TRUE(ctx_.detectEnemy(Armors{}, t));
  }

  TEST_F(EnvironmentContextTest,
         DetectEnemy_TargetTrackingButOutOfRange_ReturnsFalse) {
    auto t = TrackingTarget();
    t.position.x = 100.0;
    EXPECT_FALSE(ctx_.detectEnemy(Armors{}, t));
  }

  TEST_F(EnvironmentContextTest, DetectEnemy_NoTargetArmorInRange_ReturnsTrue) {
    EXPECT_TRUE(ctx_.detectEnemy(ArmorInRange(), std::nullopt));
  }

  TEST_F(EnvironmentContextTest,
         DetectEnemy_NoTargetArmorOutOfRange_ReturnsFalse) {
    EXPECT_FALSE(ctx_.detectEnemy(ArmorOutOfRange(), std::nullopt));
  }

  TEST_F(EnvironmentContextTest, DetectEnemy_NoTargetNoArmors_ReturnsFalse) {
    EXPECT_FALSE(ctx_.detectEnemy(Armors{}, std::nullopt));
  }

  TEST_F(EnvironmentContextTest,
         DetectEnemy_NonTrackingTargetWithArmors_IgnoresTarget) {
    EXPECT_TRUE(ctx_.detectEnemy(ArmorInRange(), NonTrackingTarget()));
  }

  // ── isStatusBad ──
  TEST_F(EnvironmentContextTest, IsStatusBad_HpBelowEnter_ReturnsTrue) {
    EXPECT_TRUE(ctx_.isStatusBad(LowHpRobotStatus()));
  }

  TEST_F(EnvironmentContextTest, IsStatusBad_AmmoAtMin_ReturnsTrue) {
    EXPECT_TRUE(ctx_.isStatusBad(LowAmmoRobotStatus()));
  }

  TEST_F(EnvironmentContextTest, IsStatusBad_HpAndAmmoOk_ReturnsFalse) {
    EXPECT_FALSE(ctx_.isStatusBad(HealthyRobotStatus()));
  }

  // ── setState / isStateChanged ──
  TEST_F(EnvironmentContextTest, SetState_DifferentState_SetsChanged) {
    ctx_.setState(State::ATTACK);
    EXPECT_TRUE(ctx_.isStateChanged());
  }

  TEST_F(EnvironmentContextTest, SetState_SameStateTwice_NoChangeOnSecond) {
    ctx_.setState(State::ATTACK);
    ASSERT_TRUE(ctx_.isStateChanged());

    ctx_.setState(State::ATTACK);
    EXPECT_FALSE(ctx_.isStateChanged());
  }

  // ── checkReadiness ──
  TEST_F(EnvironmentContextTest, CheckReadiness_NoRobotStatus_ReturnsNoRs) {
    auto r = ctx_.checkReadiness(0);
    EXPECT_EQ(r.status, Readiness::Status::NO_RS);
  }

  class EnvCtxRequireGameTest : public EnvironmentContextTest {
  protected:
    EnvCtxRequireGameTest() : ctx_req_(RequireGameRunningConfig()) {
    }

    EnvironmentContext ctx_req_;
  };

  TEST_F(EnvCtxRequireGameTest, CheckReadiness_NoGameStatus_ReturnsNoGs) {
    ctx_req_.onRobotStatus(HealthyRobotStatus());
    auto r = ctx_req_.checkReadiness(0);
    EXPECT_EQ(r.status, Readiness::Status::NO_GS);
  }

  TEST_F(EnvCtxRequireGameTest,
         CheckReadiness_GameNotStarted_ReturnsNotStarted) {
    ctx_req_.onRobotStatus(HealthyRobotStatus());
    GameStatus gs;
    gs.game_progress = GameStatus::COUNT_DOWN;
    ctx_req_.onGameStatus(gs, 0);
    auto r = ctx_req_.checkReadiness(1000);
    EXPECT_EQ(r.status, Readiness::Status::NOT_STARTED);
  }

  TEST_F(EnvCtxRequireGameTest,
         CheckReadiness_StartedButInDelay_ReturnsInDelay) {
    ctx_req_.onRobotStatus(HealthyRobotStatus());
    ctx_req_.onGameStatus(RunningGameStatus(), 0);
    auto r = ctx_req_.checkReadiness(0);
    EXPECT_EQ(r.status, Readiness::Status::IN_DELAY);
    EXPECT_GE(r.elapsed, 0.0);
  }

  TEST_F(EnvCtxRequireGameTest, CheckReadiness_StartedAfterDelay_ReturnsReady) {
    ctx_req_.onRobotStatus(HealthyRobotStatus());
    ctx_req_.onGameStatus(RunningGameStatus(), 0);
    auto r = ctx_req_.checkReadiness(
        static_cast<int64_t>(kStartDelaySec * 1e9));
    EXPECT_EQ(r.status, Readiness::Status::READY);
  }

  TEST_F(EnvironmentContextTest,
         CheckReadiness_RequireGameFalseAndHasRs_ReturnsReady) {
    ctx_.onRobotStatus(HealthyRobotStatus());
    auto r = ctx_.checkReadiness(0);
    EXPECT_EQ(r.status, Readiness::Status::READY);
  }

  // ── updatePose / isNearRobotPose ──
  TEST_F(EnvironmentContextTest, IsNearRobotPose_NoPose_ReturnsFalse) {
    EXPECT_FALSE(ctx_.isNearRobotPose(0.0, 0.0, 1.0));
  }

  TEST_F(EnvironmentContextTest, IsNearRobotPose_WithinTolerance_ReturnsTrue) {
    ctx_.updatePose(0.1, 0.1, 0.0);
    EXPECT_TRUE(ctx_.isNearRobotPose(0.0, 0.0, 0.3));
  }

  TEST_F(EnvironmentContextTest, IsNearRobotPose_BeyondTolerance_ReturnsFalse) {
    ctx_.updatePose(1.0, 1.0, 0.0);
    EXPECT_FALSE(ctx_.isNearRobotPose(0.0, 0.0, 0.3));
  }

  TEST_F(EnvironmentContextTest,
         IsNearRobotPose_ExactlyAtTolerance_ReturnsTrue) {
    ctx_.updatePose(0.3, 0.0, 0.0);
    EXPECT_TRUE(ctx_.isNearRobotPose(0.0, 0.0, 0.3));
  }

  // ── getSnapshot ──
  TEST_F(EnvironmentContextTest, GetSnapshot_NoData_ReturnsDefaultsAndNoEnemy) {
    auto snap = ctx_.getSnapshot(MakeStamp(0, 0));
    EXPECT_FALSE(snap.has_rs);
    EXPECT_FALSE(snap.enemy);
    EXPECT_FALSE(snap.enemy_recent);
    EXPECT_FALSE(snap.attacked_recent);
  }

  TEST_F(EnvironmentContextTest,
         GetSnapshot_WithArmorInRange_DetectsEnemyAndSetsRecent) {
    ctx_.onRobotStatus(HealthyRobotStatus());
    ctx_.onArmors(ArmorInRange());

    auto snap = ctx_.getSnapshot(MakeStamp(0, 1000000000u));
    EXPECT_TRUE(snap.enemy);
    EXPECT_TRUE(snap.enemy_recent);
  }

  TEST_F(EnvironmentContextTest,
         GetSnapshot_WithTrackingTargetInRange_DetectsEnemy) {
    ctx_.onRobotStatus(HealthyRobotStatus());
    ctx_.onTarget(TrackingTarget());

    auto snap = ctx_.getSnapshot(MakeStamp(0, 500000000u));
    EXPECT_TRUE(snap.enemy);
    EXPECT_TRUE(snap.enemy_recent);
  }

  TEST_F(EnvironmentContextTest,
         GetSnapshot_AttackedStatus_SetsAttackedRecent) {
    ctx_.onRobotStatus(AttackedRobotStatus());

    auto snap = ctx_.getSnapshot(MakeStamp(0, 500000000u));
    EXPECT_TRUE(snap.attacked_recent);
  }

  TEST_F(EnvironmentContextTest,
         GetSnapshot_HealthyStatus_AttackedRecentFalse) {
    ctx_.onRobotStatus(HealthyRobotStatus());

    auto snap = ctx_.getSnapshot(MakeStamp(0, 0));
    EXPECT_FALSE(snap.attacked_recent);
  }

  TEST_F(EnvironmentContextTest, GetSnapshot_AtCenter_SetsAtCenter) {
    ctx_.onRobotStatus(HealthyRobotStatus());
    ctx_.updatePose(kDefaultX + 0.1, kDefaultY + 0.1, 0.0);

    auto snap = ctx_.getSnapshot(MakeStamp(0, 0));
    EXPECT_TRUE(snap.at_center);
    EXPECT_TRUE(snap.in_center_keep_spin);
  }

  TEST_F(EnvironmentContextTest, GetSnapshot_FarFromCenter_ClearsCenterFlags) {
    ctx_.onRobotStatus(HealthyRobotStatus());
    ctx_.updatePose(10.0, 10.0, 0.0);

    auto snap = ctx_.getSnapshot(MakeStamp(0, 0));
    EXPECT_FALSE(snap.at_center);
    EXPECT_FALSE(snap.in_center_keep_spin);
  }

  TEST_F(EnvironmentContextTest, GetSnapshot_CopiesMatchStartTime) {
    ctx_.onGameStatus(RunningGameStatus(), 5000000000LL);

    auto snap = ctx_.getSnapshot(MakeStamp(0, 0));
    EXPECT_EQ(snap.match_start_time.sec, 5);
    EXPECT_EQ(snap.match_start_time.nanosec, 0u);
  }

}  // namespace simple_decision
