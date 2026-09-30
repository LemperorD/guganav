/**
 * @file test_shm_layout.cpp
 * @brief guga_ui_common shm_layout.hpp 单元测试。
 *
 * 测试内容：
 *   - 编译期常量的正确性
 *   - ShmHeader / ShmSlot 的大小和对齐
 *   - calcShmSize / calcSlotDataOffset / calcSlotMetaOffset 的布局计算
 */

#include <cstdint>

#include "guga_ui_common/shm_layout.hpp"
#include "gtest/gtest.h"

namespace gu = guga_ui;

// ==================== 编译期常量 ====================

TEST(ShmLayoutTest, ShmMagic) {
  EXPECT_EQ(gu::SHM_MAGIC, 0x4755475547414741ULL);
}

TEST(ShmLayoutTest, ShmVersion) {
  EXPECT_EQ(gu::SHM_VERSION, 1u);
}

TEST(ShmLayoutTest, SlotDataSize) {
  EXPECT_EQ(gu::SHM_SLOT_DATA_SIZE, 64u);
}

TEST(ShmLayoutTest, MaxSlots) {
  EXPECT_EQ(gu::SHM_MAX_SLOTS, 16u);
}

TEST(ShmLayoutTest, DefaultShmName) {
  EXPECT_STREQ(gu::SHM_DEFAULT_NAME, "guga_shm");
}

// ==================== 结构体大小 ====================

TEST(ShmLayoutTest, ShmHeaderSize) {
  EXPECT_EQ(sizeof(gu::ShmHeader), 64u);
}

TEST(ShmLayoutTest, ShmSlotSize) {
  EXPECT_EQ(sizeof(gu::ShmSlot), 64u);
}

TEST(ShmLayoutTest, ShmSlotAlignment) {
  EXPECT_EQ(alignof(gu::ShmSlot), 64u);
}

// ==================== 布局计算函数 ====================

TEST(ShmLayoutTest, CalcShmSize) {
  // header(64) + slots(16*64) + data(16*64) = 64 + 1024 + 1024 = 2112
  EXPECT_EQ(gu::calcShmSize(16), 2112u);

  // header(64) + slots(8*64) + data(8*64) = 64 + 512 + 512 = 1088
  EXPECT_EQ(gu::calcShmSize(8), 1088u);

  // header(64) + slots(0*64) + data(0*64) = 64
  EXPECT_EQ(gu::calcShmSize(0), 64u);
}

TEST(ShmLayoutTest, CalcSlotDataOffset) {
  // 数据区从 header + 所有 16 个 slot meta 之后开始
  // header(64) + 16*slot_meta(64) = 64 + 1024 = 1088
  const size_t data_base = sizeof(gu::ShmHeader) + gu::SHM_MAX_SLOTS * sizeof(gu::ShmSlot);

  EXPECT_EQ(gu::calcSlotDataOffset(0), data_base);
  EXPECT_EQ(gu::calcSlotDataOffset(1), data_base + gu::SHM_SLOT_DATA_SIZE);
  EXPECT_EQ(gu::calcSlotDataOffset(3), data_base + 3 * gu::SHM_SLOT_DATA_SIZE);
  EXPECT_EQ(gu::calcSlotDataOffset(15), data_base + 15 * gu::SHM_SLOT_DATA_SIZE);
}

TEST(ShmLayoutTest, CalcSlotMetaOffset) {
  // slot meta 从 header 之后开始
  // slot 0 在 offset 64, slot 5 在 offset 64 + 5*64 = 384
  EXPECT_EQ(gu::calcSlotMetaOffset(0), sizeof(gu::ShmHeader));
  EXPECT_EQ(gu::calcSlotMetaOffset(5), sizeof(gu::ShmHeader) + 5 * sizeof(gu::ShmSlot));
  EXPECT_EQ(gu::calcSlotMetaOffset(15), sizeof(gu::ShmHeader) + 15 * sizeof(gu::ShmSlot));
}

// ==================== 布局一致性 ====================

TEST(ShmLayoutTest, DataOffsetsAreMonotonic) {
  // 数据区的偏移应该单调递增，且每个槽位的数据区不重叠
  for (size_t i = 1; i < gu::SHM_MAX_SLOTS; ++i) {
    EXPECT_GT(gu::calcSlotDataOffset(i), gu::calcSlotDataOffset(i - 1));
  }
}

TEST(ShmLayoutTest, MetaAndDataDontOverlap) {
  // 最后一个 slot meta 的结束位置应 ≤ 第一个数据区的起始位置
  const size_t last_meta_end = gu::calcSlotMetaOffset(gu::SHM_MAX_SLOTS - 1) + sizeof(gu::ShmSlot);
  const size_t first_data_start = gu::calcSlotDataOffset(0);
  EXPECT_LE(last_meta_end, first_data_start);
}

// ==================== ShmSlot 原子成员 ====================

TEST(ShmLayoutTest, ShmSlotSeqIsAtomic) {
  // seq 成员是 std::atomic<uint64_t>，验证它是 lock-free 的
  gu::ShmSlot slot{};
  EXPECT_TRUE(slot.seq.is_lock_free());
}

TEST(ShmLayoutTest, ShmSlotSeqDefaultZero) {
  gu::ShmSlot slot{};
  EXPECT_EQ(slot.seq.load(), 0u);
}

// ==================== calcShmSize constexpr ====================

TEST(ShmLayoutTest, CalcShmSizeIsConstexpr) {
  // 如果 calcShmSize 可以被静态断言，说明它是 constexpr 的
  static_assert(gu::calcShmSize(0) == 64);
  static_assert(gu::calcShmSize(16) == 68672);
  SUCCEED();  // 到这里说明 static_assert 通过了
}
