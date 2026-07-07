/**
 * @file test_data_source.cpp
 * @brief guga_ui_pangolin UiDataSource 单元测试。
 *
 * 测试内容：
 *   - UiDataSource::open() / isValid()
 *   - UiDataSource::update() 数据刷新
 *   - frameCount() 递增
 *   - isFresh() 过期判定
 *   - 多槽位读取 (ROBOT_STATUS, GAME_STATUS, DECISION)
 *   - 无 shm 时的防御行为
 *
 * 此测试使用真实的 ShmWriter + ShmReader，
 * 不需要 Pangolin / OpenGL 上下文。
 */

#include <cstring>
#include <string>

#include "data_source.hpp"
#include "guga_ui_common/shm_writer.hpp"
#include "gtest/gtest.h"

namespace gu = guga_ui;

// ==================== 测试夹具 ====================

class DataSourceTest : public ::testing::Test {
 protected:
  static std::string uniqueName(const char* suffix) {
    return "/test_ds_shm_" + std::string(suffix) + "_" +
           std::to_string(::getpid());
  }

  void TearDown() override {
    // shm 在测试函数作用域内管理，夹具不持有
  }
};

// ==================== open() / isValid() ====================

TEST_F(DataSourceTest, OpenWithValidShm) {
  std::string name = uniqueName("open_valid");

  // 先用 writer 创建 shm
  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::ROBOT_STATUS));

  UiDataSource ds;
  EXPECT_TRUE(ds.open(name));
  EXPECT_TRUE(ds.isValid());

  shm_unlink(name.c_str());
}

TEST_F(DataSourceTest, OpenWithNonexistentShm) {
  UiDataSource ds;
  EXPECT_FALSE(ds.open("/nonexistent_shm_ds_test_12345"));
  EXPECT_FALSE(ds.isValid());
}

TEST_F(DataSourceTest, IsValidFalseBeforeOpen) {
  UiDataSource ds;
  EXPECT_FALSE(ds.isValid());
}

// ==================== update() / frameCount() ====================

TEST_F(DataSourceTest, UpdateSetsHasData) {
  std::string name = uniqueName("hasdata");

  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::ROBOT_STATUS));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));

  // 未 update 之前
  EXPECT_FALSE(ds.data().has_data);

  ds.update();
  EXPECT_TRUE(ds.data().has_data);

  shm_unlink(name.c_str());
}

TEST_F(DataSourceTest, FrameCountIncrements) {
  std::string name = uniqueName("framecnt");

  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::ROBOT_STATUS));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));

  EXPECT_EQ(ds.frameCount(), 0u);
  ds.update();
  EXPECT_EQ(ds.frameCount(), 1u);
  ds.update();
  EXPECT_EQ(ds.frameCount(), 2u);
  ds.update();
  EXPECT_EQ(ds.frameCount(), 3u);

  shm_unlink(name.c_str());
}

TEST_F(DataSourceTest, UpdateNoopWhenNotOpen) {
  UiDataSource ds;
  EXPECT_NO_THROW(ds.update());
  EXPECT_FALSE(ds.data().has_data);
  EXPECT_EQ(ds.frameCount(), 0u);
}

// ==================== 数据读取 ====================

TEST_F(DataSourceTest, UpdateReadsRobotStatus) {
  std::string name = uniqueName("read_rs");

  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::ROBOT_STATUS));

  gu::UiRobotStatus rs{};
  rs.current_hp = 400;
  rs.maximum_hp = 800;
  rs.robot_id = 5;
  writer.write(&rs, sizeof(rs));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));
  ds.update();

  const auto& d = ds.data();
  EXPECT_EQ(d.robot_status.current_hp, 400);
  EXPECT_EQ(d.robot_status.maximum_hp, 800);
  EXPECT_EQ(d.robot_status.robot_id, 5);

  shm_unlink(name.c_str());
}

TEST_F(DataSourceTest, UpdateReadsGameStatus) {
  std::string name = uniqueName("read_gs");

  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::GAME_STATUS));

  gu::UiGameStatus gs{};
  gs.game_progress = 4;  // RUNNING
  gs.stage_remain_time = 300;
  gs.elapsed_sec = 25.5;
  writer.write(&gs, sizeof(gs));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));
  ds.update();

  const auto& d = ds.data();
  EXPECT_EQ(d.game_status.game_progress, 4);
  EXPECT_EQ(d.game_status.stage_remain_time, 300);
  EXPECT_DOUBLE_EQ(d.game_status.elapsed_sec, 25.5);

  shm_unlink(name.c_str());
}

TEST_F(DataSourceTest, UpdateReadsDecision) {
  std::string name = uniqueName("read_dec");

  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::DECISION));

  gu::UiDecision dec{};
  dec.state = 1;  // ATTACK
  dec.chassis_mode = 1;  // LITTLE_TES
  dec.should_publish_goal = true;
  dec.target_x = 7.5;
  dec.robot_x = 3.0;
  dec.compute_us = 150;
  writer.write(&dec, sizeof(dec));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));
  ds.update();

  const auto& d = ds.data();
  EXPECT_EQ(d.decision.state, 1);
  EXPECT_EQ(d.decision.chassis_mode, 1);
  EXPECT_TRUE(d.decision.should_publish_goal);
  EXPECT_DOUBLE_EQ(d.decision.target_x, 7.5);
  EXPECT_DOUBLE_EQ(d.decision.robot_x, 3.0);
  EXPECT_EQ(d.decision.compute_us, 150);

  shm_unlink(name.c_str());
}

// ==================== isFresh() ====================

TEST_F(DataSourceTest, IsFreshReturnsTrueWithinMaxAge) {
  std::string name = uniqueName("fresh_ok");

  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::ROBOT_STATUS));

  gu::UiRobotStatus rs{};
  rs.current_hp = 100;
  writer.write(&rs, sizeof(rs));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));
  ds.update();

  // 刚更新，age = frame_count，distance = 0 ≤ max_age
  EXPECT_TRUE(ds.isFresh(ds.data().robot_status_age, 60));

  // 跑几帧但不超过 max_age
  for (int i = 0; i < 10; ++i) {
    ds.update();
  }
  EXPECT_TRUE(ds.isFresh(ds.data().robot_status_age, 60));

  shm_unlink(name.c_str());
}

TEST_F(DataSourceTest, IsFreshReturnsFalseBeyondMaxAge) {
  std::string name = uniqueName("fresh_stale");

  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::ROBOT_STATUS));

  gu::UiRobotStatus rs{};
  rs.current_hp = 100;
  writer.write(&rs, sizeof(rs));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));
  ds.update();

  const uint64_t age = ds.data().robot_status_age;

  // 跑 100 帧，默认 max_age=60 -> 数据过期
  for (int i = 0; i < 100; ++i) {
    ds.update();
  }

  EXPECT_FALSE(ds.isFresh(age, 60));      // 超过 max_age
  EXPECT_TRUE(ds.isFresh(age, 120));       // 放宽 max_age 则仍新鲜

  shm_unlink(name.c_str());
}

TEST_F(DataSourceTest, IsFreshBoundaryZeroAge) {
  UiDataSource ds;
  // 未调用 update(), frame_count_ = 0, age_field 也是 0
  // frame_count_ < age_field → false (但是这里 frame_count_ == age_field == 0)
  // 所以 0 - 0 = 0 ≤ max_age → true
  EXPECT_TRUE(ds.isFresh(0, 60));
}

// ==================== 数据年龄跟踪 ====================

TEST_F(DataSourceTest, AgeFieldsUpdatedAfterRead) {
  std::string name = uniqueName("age");

  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::ROBOT_STATUS));

  gu::UiRobotStatus rs{};
  rs.current_hp = 200;
  writer.write(&rs, sizeof(rs));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));

  // 第一次 update，robot_status_age 应为 frame_count (1)
  ds.update();
  EXPECT_EQ(ds.data().robot_status_age, 1u);

  // 再 update 一次（无新数据），age 不变
  ds.update();
  EXPECT_EQ(ds.data().robot_status_age, 1u);

  // 写入新数据，age 应更新为当前帧编号
  rs.current_hp = 300;
  writer.write(&rs, sizeof(rs));
  ds.update();
  EXPECT_EQ(ds.data().robot_status_age, 3u);

  shm_unlink(name.c_str());
}

// ==================== 多槽位 ====================

TEST_F(DataSourceTest, MultipleSlotsAgesIndependent) {
  std::string name = uniqueName("multi");

  gu::ShmWriter w_rs;
  gu::ShmWriter w_gs;
  ASSERT_TRUE(w_rs.init(name, gu::UiSlotId::ROBOT_STATUS));
  ASSERT_TRUE(w_gs.init(name, gu::UiSlotId::GAME_STATUS));

  // 先写 RS
  gu::UiRobotStatus rs{};
  rs.current_hp = 500;
  w_rs.write(&rs, sizeof(rs));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));
  ds.update();

  EXPECT_EQ(ds.data().robot_status_age, 1u);
  EXPECT_EQ(ds.data().game_status_age, 0u);  // 未写入 GS

  // 再写 GS
  gu::UiGameStatus gs{};
  gs.game_progress = 3;
  w_gs.write(&gs, sizeof(gs));
  ds.update();

  EXPECT_EQ(ds.data().robot_status_age, 1u);  // RS 未更新
  EXPECT_EQ(ds.data().game_status_age, 2u);    // GS 刚更新

  // 再写 RS
  rs.current_hp = 600;
  w_rs.write(&rs, sizeof(rs));
  ds.update();

  EXPECT_EQ(ds.data().robot_status_age, 3u);
  EXPECT_EQ(ds.data().game_status_age, 2u);  // GS 未更新

  shm_unlink(name.c_str());
}

// ==================== data() 返回引用一致性 ====================

TEST_F(DataSourceTest, DataReturnsConsistentReference) {
  std::string name = uniqueName("dataref");

  gu::ShmWriter writer;
  ASSERT_TRUE(writer.init(name, gu::UiSlotId::ROBOT_STATUS));

  UiDataSource ds;
  ASSERT_TRUE(ds.open(name));
  ds.update();

  const UiData& d1 = ds.data();
  const UiData& d2 = ds.data();
  // 应该返回同一个对象
  EXPECT_EQ(&d1, &d2);

  shm_unlink(name.c_str());
}

// ==================== UiData 默认状态 ====================

TEST_F(DataSourceTest, UiDataDefaults) {
  UiData d{};

  EXPECT_FALSE(d.has_data);

  EXPECT_EQ(d.robot_status_age, 0u);
  EXPECT_EQ(d.game_status_age, 0u);
  EXPECT_EQ(d.decision_age, 0u);
  EXPECT_EQ(d.enemy_age, 0u);
  EXPECT_EQ(d.odom_age, 0u);
  EXPECT_EQ(d.yaw_age, 0u);
  EXPECT_EQ(d.path_age, 0u);

  // 嵌套的结构体都是零初始化的
  EXPECT_EQ(d.robot_status.current_hp, 0);
  EXPECT_EQ(d.odom.x, 0.0);
}
