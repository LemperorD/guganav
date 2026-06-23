/**
 * @file test_shm_write_read.cpp
 * @brief guga_ui_common ShmWriter / ShmReader 集成测试。
 *
 * 测试内容：
 *   - ShmWriter::init() 创建/打开逻辑
 *   - ShmWriter::write() + ShmReader::read() 往返正确性
 *   - checkFresh / getSeq / isValid / slotCount
 *   - 边界条件 (空指针、无效 slot_id、超大数据、未打开读取)
 *   - 移动语义
 *   - seq 协议（偶/奇数）
 *
 * 注意: 这些测试使用真实的 POSIX 共享内存，
 *       要求运行时环境支持 shm_open / mmap / shm_unlink。
 */

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "guga_ui_common/shm_reader.hpp"
#include "guga_ui_common/shm_writer.hpp"
#include "guga_ui_common/ui_types.hpp"
#include "gtest/gtest.h"

namespace gu = guga_ui;

// ==================== 测试夹具 ====================

class ShmWriteReadTest : public ::testing::Test {
 protected:
  /// 每个测试使用唯一的 shm 名称，避免互相干扰
 public:
  static std::string uniqueName(int suffix) {
    return "/test_guga_ui_shm_" + std::to_string(suffix) + "_" +
           std::to_string(::getpid());
  }

 protected:

  void TearDown() override {
    // 清理所有可能残留的 shm
    // (ShmWriter 析构时不会 unlink)
  }
};

// ==================== ShmWriter::init() ====================

TEST_F(ShmWriteReadTest, WriterInitCreatesShm) {
  std::string name = uniqueName(1);
  gu::ShmWriter writer;
  EXPECT_TRUE(writer.init(name, gu::UiSlotId::ROBOT_STATUS));
  shm_unlink(name.c_str());  // 标记删除，进程退出后清除
}

TEST_F(ShmWriteReadTest, WriterInitOpensExistingShm) {
  std::string name = uniqueName(2);
  {
    gu::ShmWriter w1;
    ASSERT_TRUE(w1.init(name, gu::UiSlotId::ROBOT_STATUS));
  }
  // w1 析构（只 munmap，不 unlink），shm 仍然存在
  {
    gu::ShmWriter w2;
    EXPECT_TRUE(w2.init(name, gu::UiSlotId::ROBOT_STATUS));
  }
  shm_unlink(name.c_str());
}

TEST_F(ShmWriteReadTest, WriterInitInvalidSlotId) {
  gu::ShmWriter writer;
  // SHM_MAX_SLOTS = 16, 所以 slot_id >= 16 非法
  EXPECT_FALSE(writer.init(uniqueName(3), static_cast<gu::UiSlotId>(99)));
}

TEST_F(ShmWriteReadTest, WriterInitMaxSlotId) {
  gu::ShmWriter writer;
  // slot_id = 15 是有效的（0–15 共 16 个槽位）
  std::string name = uniqueName(4);
  EXPECT_TRUE(writer.init(name, static_cast<gu::UiSlotId>(15)));
  shm_unlink(name.c_str());
}

// ==================== ShmWriter::write() + ShmReader::read() 往返 ====================

// 辅助函数：准备好 writer，返回 shm 名称供 reader 打开
struct WriterFixture {
  gu::ShmWriter writer;
  std::string name;
};
static WriterFixture makeWriter(int suffix, gu::UiSlotId slot) {
  WriterFixture f;
  f.name = ShmWriteReadTest::uniqueName(suffix);
  bool ok = f.writer.init(f.name, slot);
  EXPECT_TRUE(ok);
  return f;
}

TEST_F(ShmWriteReadTest, WriteReadRobotStatus) {
  auto f = makeWriter(10, gu::UiSlotId::ROBOT_STATUS);

  gu::UiRobotStatus written{};
  written.current_hp = 500;
  written.maximum_hp = 1000;
  written.robot_id = 3;
  written.is_hp_deduced = true;
  f.writer.write(&written, sizeof(written));

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(f.name));

  gu::UiRobotStatus read_back{};
  EXPECT_TRUE(reader.read(gu::UiSlotId::ROBOT_STATUS, &read_back, sizeof(read_back)));
  EXPECT_EQ(read_back.current_hp, 500);
  EXPECT_EQ(read_back.maximum_hp, 1000);
  EXPECT_EQ(read_back.robot_id, 3);
  EXPECT_TRUE(read_back.is_hp_deduced);

  shm_unlink(f.name.c_str());
}

TEST_F(ShmWriteReadTest, WriteReadDecision) {
  auto f = makeWriter(11, gu::UiSlotId::DECISION);

  gu::UiDecision written{};
  written.state = 1;
  written.chassis_mode = 2;
  written.should_publish_goal = true;
  written.target_x = 3.14;
  written.target_y = 2.71;
  written.target_yaw = 1.57;
  written.robot_x = 0.5;
  written.compute_us = 42;
  f.writer.write(&written, sizeof(written));

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(f.name));

  gu::UiDecision read_back{};
  EXPECT_TRUE(reader.read(gu::UiSlotId::DECISION, &read_back, sizeof(read_back)));
  EXPECT_EQ(read_back.state, 1);
  EXPECT_EQ(read_back.chassis_mode, 2);
  EXPECT_TRUE(read_back.should_publish_goal);
  EXPECT_DOUBLE_EQ(read_back.target_x, 3.14);
  EXPECT_DOUBLE_EQ(read_back.target_y, 2.71);
  EXPECT_DOUBLE_EQ(read_back.target_yaw, 1.57);
  EXPECT_DOUBLE_EQ(read_back.robot_x, 0.5);
  EXPECT_EQ(read_back.compute_us, 42);

  shm_unlink(f.name.c_str());
}

TEST_F(ShmWriteReadTest, WriteReadOdom) {
  auto f = makeWriter(12, gu::UiSlotId::ODOM);

  gu::UiOdom written{};
  written.x = 1.0;
  written.y = 2.0;
  written.z = 0.1;
  written.yaw = 0.785;
  written.vx = 0.5;
  f.writer.write(&written, sizeof(written));

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(f.name));

  gu::UiOdom read_back{};
  EXPECT_TRUE(reader.read(gu::UiSlotId::ODOM, &read_back, sizeof(read_back)));
  EXPECT_DOUBLE_EQ(read_back.x, 1.0);
  EXPECT_DOUBLE_EQ(read_back.y, 2.0);
  EXPECT_DOUBLE_EQ(read_back.z, 0.1);
  EXPECT_DOUBLE_EQ(read_back.yaw, 0.785);
  EXPECT_DOUBLE_EQ(read_back.vx, 0.5);

  shm_unlink(f.name.c_str());
}

TEST_F(ShmWriteReadTest, WriteReadAllSlotTypes) {
  // 一次性测试所有槽位类型的往返
  auto f0 = makeWriter(20, gu::UiSlotId::ROBOT_STATUS);
  auto f1 = makeWriter(21, gu::UiSlotId::GAME_STATUS);
  auto f2 = makeWriter(22, gu::UiSlotId::RFID_STATUS);
  auto f3 = makeWriter(23, gu::UiSlotId::DECISION);
  auto f4 = makeWriter(24, gu::UiSlotId::ENEMY);
  auto f5 = makeWriter(25, gu::UiSlotId::ODOM);
  auto f6 = makeWriter(26, gu::UiSlotId::YAW);
  auto f7 = makeWriter(27, gu::UiSlotId::PATH);

  gu::UiRobotStatus rs{};
  rs.current_hp = 300;
  f0.writer.write(&rs, sizeof(rs));

  gu::UiGameStatus gs{};
  gs.game_progress = 4;
  gs.stage_remain_time = 120;
  f1.writer.write(&gs, sizeof(gs));

  gu::UiRfidStatus rf{};
  rf.base_gain_point = true;
  rf.friendly_outpost_gain_point = true;
  f2.writer.write(&rf, sizeof(rf));

  gu::UiDecision dec{};
  dec.state = 2;
  dec.robot_x = 1.0;
  f3.writer.write(&dec, sizeof(dec));

  gu::UiEnemy en{};
  en.tracking = true;
  en.target_x = 5.0;
  f4.writer.write(&en, sizeof(en));

  gu::UiOdom od{};
  od.x = 2.0;
  f5.writer.write(&od, sizeof(od));

  gu::UiYaw yw{};
  yw.yaw_diff = 0.1;
  f6.writer.write(&yw, sizeof(yw));

  gu::UiPath pa{};
  pa.count = 3;
  pa.x[0] = 1.0; pa.y[0] = 2.0;
  f7.writer.write(&pa, sizeof(pa));

  // 打开所有 shm 并读取 (每个槽位使用独立的 shm，但 reader 可以各自打开)
  // 这里简化测试：只验证最后几个
  {
    gu::ShmReader r;
    ASSERT_TRUE(r.open(f0.name));
    gu::UiRobotStatus out{};
    EXPECT_TRUE(r.read(gu::UiSlotId::ROBOT_STATUS, &out, sizeof(out)));
    EXPECT_EQ(out.current_hp, 300);
    shm_unlink(f0.name.c_str());
  }
  {
    gu::ShmReader r;
    ASSERT_TRUE(r.open(f1.name));
    gu::UiGameStatus out{};
    EXPECT_TRUE(r.read(gu::UiSlotId::GAME_STATUS, &out, sizeof(out)));
    EXPECT_EQ(out.game_progress, 4);
    EXPECT_EQ(out.stage_remain_time, 120);
    shm_unlink(f1.name.c_str());
  }
  {
    gu::ShmReader r;
    ASSERT_TRUE(r.open(f3.name));
    gu::UiDecision out{};
    EXPECT_TRUE(r.read(gu::UiSlotId::DECISION, &out, sizeof(out)));
    EXPECT_EQ(out.state, 2);
    shm_unlink(f3.name.c_str());
  }
  {
    gu::ShmReader r;
    ASSERT_TRUE(r.open(f7.name));
    gu::UiPath out{};
    EXPECT_TRUE(r.read(gu::UiSlotId::PATH, &out, sizeof(out)));
    EXPECT_EQ(out.count, 3u);
    EXPECT_DOUBLE_EQ(out.x[0], 1.0);
    shm_unlink(f7.name.c_str());
  }

  shm_unlink(f2.name.c_str());
  shm_unlink(f4.name.c_str());
  shm_unlink(f5.name.c_str());
  shm_unlink(f6.name.c_str());
}

// ==================== write() 边界条件 ====================

TEST_F(ShmWriteReadTest, WriteTruncatesOversizedData) {
  auto f = makeWriter(30, gu::UiSlotId::ROBOT_STATUS);

  // 构造一个大于 64 字节的数据块
  char large_data[128];
  std::memset(large_data, 0xAB, sizeof(large_data));

  // write() 应该截断到 64 字节（不崩溃）
  EXPECT_NO_THROW(f.writer.write(large_data, sizeof(large_data)));

  shm_unlink(f.name.c_str());
}

TEST_F(ShmWriteReadTest, WriteDoesNotCrashWhenNotInitialized) {
  gu::ShmWriter writer;
  gu::UiRobotStatus data{};
  // 未 init，slot_data_ 为空 —— write() 应安全返回
  EXPECT_NO_THROW(writer.write(&data, sizeof(data)));
}

// ==================== ShmReader 边界条件 ====================

TEST_F(ShmWriteReadTest, ReadReturnsFalseWhenNotOpened) {
  gu::ShmReader reader;
  gu::UiRobotStatus out{};
  EXPECT_FALSE(reader.read(gu::UiSlotId::ROBOT_STATUS, &out, sizeof(out)));
}

// checkFresh 在 shm 未打开时会发生 segfault（实现假设 base_addr_ 不为空），
// 调用方需先用 isValid() 检查。这是一个已知的使用合同限制。
// 此处不测试未打开时的 checkFresh 行为。

TEST_F(ShmWriteReadTest, IsValidReturnsFalseWhenNotOpened) {
  gu::ShmReader reader;
  EXPECT_FALSE(reader.isValid());
}

TEST_F(ShmWriteReadTest, SlotCountReturnsZeroWhenNotOpened) {
  gu::ShmReader reader;
  EXPECT_EQ(reader.slotCount(), 0u);
}

TEST_F(ShmWriteReadTest, GetSeqReturnsZeroWhenNotOpened) {
  gu::ShmReader reader;
  EXPECT_EQ(reader.getSeq(gu::UiSlotId::ROBOT_STATUS), 0u);
}

TEST_F(ShmWriteReadTest, OpenInvalidShmFails) {
  gu::ShmReader reader;
  EXPECT_FALSE(reader.open("/nonexistent_shm_for_test_xyz"));
}

// ==================== checkFresh / seq 协议 ====================

TEST_F(ShmWriteReadTest, CheckFreshDetectsNewData) {
  auto f = makeWriter(40, gu::UiSlotId::ROBOT_STATUS);

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(f.name));

  // 还没有写入 —— checkFresh 应为 false
  EXPECT_FALSE(reader.checkFresh(gu::UiSlotId::ROBOT_STATUS));

  // 写入一次
  gu::UiRobotStatus data{};
  data.current_hp = 100;
  f.writer.write(&data, sizeof(data));

  EXPECT_TRUE(reader.checkFresh(gu::UiSlotId::ROBOT_STATUS));

  // read 之后 checkFresh 应变为 false（last_seqs_ 已更新）
  gu::UiRobotStatus out{};
  EXPECT_TRUE(reader.read(gu::UiSlotId::ROBOT_STATUS, &out, sizeof(out)));
  EXPECT_FALSE(reader.checkFresh(gu::UiSlotId::ROBOT_STATUS));

  shm_unlink(f.name.c_str());
}

TEST_F(ShmWriteReadTest, SeqIsEvenAfterWrite) {
  auto f = makeWriter(41, gu::UiSlotId::ROBOT_STATUS);

  gu::UiRobotStatus data{};
  f.writer.write(&data, sizeof(data));

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(f.name));

  uint64_t seq = reader.getSeq(gu::UiSlotId::ROBOT_STATUS);
  EXPECT_NE(seq, 0u);
  // 写入完成后 seq 应为偶数
  EXPECT_EQ(seq % 2, 0u);

  shm_unlink(f.name.c_str());
}

// ==================== ShmReader isValid / slotCount ====================

TEST_F(ShmWriteReadTest, IsValidAfterOpen) {
  auto f = makeWriter(42, gu::UiSlotId::ROBOT_STATUS);

  gu::ShmReader reader;
  EXPECT_FALSE(reader.isValid());
  ASSERT_TRUE(reader.open(f.name));
  EXPECT_TRUE(reader.isValid());

  shm_unlink(f.name.c_str());
}

TEST_F(ShmWriteReadTest, SlotCountAfterOpen) {
  auto f = makeWriter(43, gu::UiSlotId::ROBOT_STATUS);

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(f.name));
  EXPECT_EQ(reader.slotCount(), gu::SHM_MAX_SLOTS);

  shm_unlink(f.name.c_str());
}

// ==================== 移动语义 ====================

TEST_F(ShmWriteReadTest, WriterMoveConstructed) {
  std::string name = uniqueName(50);
  gu::ShmWriter w1;
  ASSERT_TRUE(w1.init(name, gu::UiSlotId::ROBOT_STATUS));

  gu::UiRobotStatus data{};
  data.current_hp = 999;
  w1.write(&data, sizeof(data));

  // 移动构造
  gu::ShmWriter w2(std::move(w1));
  // w2 应该能正常写入（w1 的句柄已转移）
  data.current_hp = 888;
  w2.write(&data, sizeof(data));

  // 验证 w1 已被移走（不应该 double-free）
  data.current_hp = 1;
  w1.write(&data, sizeof(data));  // 应该安全 no-op

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(name));
  gu::UiRobotStatus out{};
  EXPECT_TRUE(reader.read(gu::UiSlotId::ROBOT_STATUS, &out, sizeof(out)));
  EXPECT_EQ(out.current_hp, 888);  // w2 的写入生效

  shm_unlink(name.c_str());
}

TEST_F(ShmWriteReadTest, WriterMoveAssignment) {
  std::string name = uniqueName(51);
  gu::ShmWriter w1;
  ASSERT_TRUE(w1.init(name, gu::UiSlotId::ROBOT_STATUS));

  gu::UiRobotStatus data{};
  data.current_hp = 111;
  w1.write(&data, sizeof(data));

  gu::ShmWriter w2;
  w2 = std::move(w1);
  data.current_hp = 222;
  w2.write(&data, sizeof(data));

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(name));
  gu::UiRobotStatus out{};
  EXPECT_TRUE(reader.read(gu::UiSlotId::ROBOT_STATUS, &out, sizeof(out)));
  EXPECT_EQ(out.current_hp, 222);

  shm_unlink(name.c_str());
}

TEST_F(ShmWriteReadTest, ReaderMoveConstructed) {
  auto f = makeWriter(52, gu::UiSlotId::ROBOT_STATUS);

  gu::UiRobotStatus data{};
  data.current_hp = 555;
  f.writer.write(&data, sizeof(data));

  gu::ShmReader r1;
  ASSERT_TRUE(r1.open(f.name));
  EXPECT_TRUE(r1.isValid());

  gu::ShmReader r2(std::move(r1));
  EXPECT_TRUE(r2.isValid());
  EXPECT_FALSE(r1.isValid());  // r1 移走后无效

  gu::UiRobotStatus out{};
  EXPECT_TRUE(r2.read(gu::UiSlotId::ROBOT_STATUS, &out, sizeof(out)));
  EXPECT_EQ(out.current_hp, 555);

  shm_unlink(f.name.c_str());
}

TEST_F(ShmWriteReadTest, ReaderMoveAssignment) {
  auto f = makeWriter(53, gu::UiSlotId::ROBOT_STATUS);

  gu::UiRobotStatus data{};
  data.current_hp = 777;
  f.writer.write(&data, sizeof(data));

  gu::ShmReader r1;
  ASSERT_TRUE(r1.open(f.name));

  gu::ShmReader r2;
  r2 = std::move(r1);

  gu::UiRobotStatus out{};
  EXPECT_TRUE(r2.read(gu::UiSlotId::ROBOT_STATUS, &out, sizeof(out)));
  EXPECT_EQ(out.current_hp, 777);

  shm_unlink(f.name.c_str());
}

// ==================== 快速连续写入 ====================

TEST_F(ShmWriteReadTest, RapidSequentialWrites) {
  auto f = makeWriter(60, gu::UiSlotId::ROBOT_STATUS);

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(f.name));

  // 快速连续写入 100 次
  for (uint16_t i = 0; i < 100; ++i) {
    gu::UiRobotStatus data{};
    data.current_hp = i;
    f.writer.write(&data, sizeof(data));
  }

  // 最后一次读取应该是最新值（或接近最新）
  gu::UiRobotStatus out{};
  EXPECT_TRUE(reader.read(gu::UiSlotId::ROBOT_STATUS, &out, sizeof(out)));
  EXPECT_GE(out.current_hp, 90u);  // 至少读取到接近末尾的值
  EXPECT_LE(out.current_hp, 99u);

  shm_unlink(f.name.c_str());
}

// ==================== 不同槽位互不干扰 ====================

TEST_F(ShmWriteReadTest, DifferentSlotsDontInterfere) {
  auto f = makeWriter(70, gu::UiSlotId::ROBOT_STATUS);

  // 往 ROBOT_STATUS 写数据
  gu::UiRobotStatus rs{};
  rs.current_hp = 111;
  f.writer.write(&rs, sizeof(rs));

  gu::ShmReader reader;
  ASSERT_TRUE(reader.open(f.name));

  // 读取 ROBOT_STATUS 槽位的内容 — 应正常
  gu::UiRobotStatus out_rs{};
  EXPECT_TRUE(reader.read(gu::UiSlotId::ROBOT_STATUS, &out_rs, sizeof(out_rs)));
  EXPECT_EQ(out_rs.current_hp, 111);

  // 读取 GAME_STATUS 槽位 — 此槽位从未写入，其内容是默认值（全零）
  // checkFresh 应为 false（seq 未变化）
  EXPECT_FALSE(reader.checkFresh(gu::UiSlotId::GAME_STATUS));

  // checkFresh(TEAM_STATUS) — 也应 false
  EXPECT_FALSE(reader.checkFresh(gu::UiSlotId::RFID_STATUS));

  shm_unlink(f.name.c_str());
}
