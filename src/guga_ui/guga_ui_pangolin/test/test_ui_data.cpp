/**
 * @file test_ui_data.cpp
 * @brief guga_ui_pangolin UiData / UiDataSource 纯逻辑单元测试。
 *
 * 测试内容：
 *   - UiData 生命周期和默认值
 *   - UiDataSource 方法在未 open 时的行为
 *   - frame count 积累 (无 shm 场景)
 *   - isFresh 边界条件 (无需 shm)
 *
 * 此测试不需要共享内存或 Pangolin / OpenGL 上下文。
 */

#include "data_source.hpp"
#include "gtest/gtest.h"

// ==================== UiData 默认状态 ====================

TEST(UiDataTest, DefaultHasDataFalse) {
  UiData d{};
  EXPECT_FALSE(d.has_data);
}

TEST(UiDataTest, DefaultAgesAreZero) {
  UiData d{};
  EXPECT_EQ(d.robot_status_age, 0u);
  EXPECT_EQ(d.game_status_age, 0u);
  EXPECT_EQ(d.decision_age, 0u);
  EXPECT_EQ(d.enemy_age, 0u);
  EXPECT_EQ(d.odom_age, 0u);
  EXPECT_EQ(d.yaw_age, 0u);
  EXPECT_EQ(d.path_age, 0u);
}

TEST(UiDataTest, DefaultRobotStatusIsZero) {
  UiData d{};
  EXPECT_EQ(d.robot_status.current_hp, 0);
  EXPECT_EQ(d.robot_status.maximum_hp, 0);
  EXPECT_FALSE(d.robot_status.is_hp_deduced);
  EXPECT_EQ(d.robot_status.robot_id, 0);
}

TEST(UiDataTest, DefaultDecisionIsZero) {
  UiData d{};
  EXPECT_EQ(d.decision.state, 0);
  EXPECT_EQ(d.decision.chassis_mode, 0);
  EXPECT_FALSE(d.decision.should_publish_goal);
  EXPECT_DOUBLE_EQ(d.decision.target_x, 0.0);
}

TEST(UiDataTest, DefaultOdomIsZero) {
  UiData d{};
  EXPECT_DOUBLE_EQ(d.odom.x, 0.0);
  EXPECT_DOUBLE_EQ(d.odom.y, 0.0);
  EXPECT_DOUBLE_EQ(d.odom.yaw, 0.0);
}

TEST(UiDataTest, Sizeof) {
  // UiData 是 Pangoin UI 每个渲染帧都要读取的结构体，
  // 验证它的大小是合理的（约 4KB 左右，因为 UiPath 包含 256 个点）
  // 不应过大
  EXPECT_GE(sizeof(UiData), sizeof(guga_ui::UiPath));  // 至少包含 UiPath
  EXPECT_LT(sizeof(UiData), 16384u);                   // 不超过 16KB
}

// ==================== UiDataSource 未打开时的行为 ====================

TEST(UiDataSourceTest, NotOpenIsValidFalse) {
  UiDataSource ds;
  EXPECT_FALSE(ds.isValid());
}

TEST(UiDataSourceTest, NotOpenFrameCountZero) {
  UiDataSource ds;
  EXPECT_EQ(ds.frameCount(), 0u);
}

TEST(UiDataSourceTest, NotOpenDataHasNoData) {
  UiDataSource ds;
  EXPECT_FALSE(ds.data().has_data);
}

TEST(UiDataSourceTest, NotOpenUpdateIsNoop) {
  UiDataSource ds;
  ds.update();
  ds.update();
  ds.update();
  // update() 不应该崩溃
  EXPECT_FALSE(ds.isValid());
  EXPECT_EQ(ds.frameCount(), 0u);
}

// ==================== isFresh 边界条件 (无 shm, 手动设定 ages) ====================

TEST(UiDataSourceTest, IsFreshFrameCountLessThanAge) {
  UiDataSource ds;
  // frame_count_ = 0, age_field = 10 — frame_count < age_field → false
  EXPECT_FALSE(ds.isFresh(10, 60));
}

TEST(UiDataSourceTest, IsFreshZeroAge) {
  UiDataSource ds;
  // frame_count_ = 0, age_field = 0 — distance = 0 ≤ max_age → true
  EXPECT_TRUE(ds.isFresh(0, 60));
}

TEST(UiDataSourceTest, IsFreshDefaultMaxAge) {
  UiDataSource ds;
  // 既然默认 max_age = 60，且 frame_count_ = 0, age_field = 0
  EXPECT_TRUE(ds.isFresh(0));  // 使用默认 max_age=60
}

// ==================== UiDataSource data() 返回常量引用 ====================

TEST(UiDataSourceTest, DataConstRef) {
  UiDataSource ds;
  const UiData& d1 = ds.data();
  const UiData& d2 = ds.data();
  EXPECT_EQ(&d1, &d2);
}
