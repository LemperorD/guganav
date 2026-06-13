#ifndef GUGA_UI_COMMON_SHM_LAYOUT_HPP
#define GUGA_UI_COMMON_SHM_LAYOUT_HPP

/**
 * @file shm_layout.hpp
 * @brief 共享内存整体布局和工具函数。
 *
 * 布局（从 shm 起始地址开始）：
 *
 *   ┌────────────────────────────────────────┐  ← offset 0
 *   │ ShmHeader (96 bytes)                      │
 *   │   magic / version / slot_count / _pad    │
 *   ├────────────────────────────────────────┤
 *   │ ShmSlot[0] (32 bytes)                     │  ← slot 对应的 offset 相对于
 *   │   data_offset / data_size / seq / _pad   │     shm 起始地址
 *   ├────────────────────────────────────────┤
 *   │ ShmSlot[1] (32 bytes)                     │
 *   ├────────────────────────────────────────┤
 *   │ ... (共 slot_count 个)                    │
 *   ├────────────────────────────────────────┤
 *   │ Slot 0 实际数据区 (可变大小)               │
 *   ├────────────────────────────────────────┤
 *   │ Slot 1 实际数据区                         │
 *   ├────────────────────────────────────────┤
 *   │ ...                                       │
 *   └────────────────────────────────────────┘
 *
 * 每个 slot 的数据大小固定为 64 字节（与 ui_types.hpp 中结构体一致），
 * 简化设计：shm 总大小 = header_size + slot_count * (meta_size + data_size)。
 */

#include <atomic>
#include <cstdint>
#include <string>

namespace guga_ui {

// ==================== 内存布局常量 ====================

/// 共享内存默认名称
static constexpr const char* SHM_DEFAULT_NAME{"guga_ui_shm"};

/// 单个 slot 的数据区大小（与 ui_types.hpp 中全部结构体的 alignas(64) 一致）
static constexpr size_t SHM_SLOT_DATA_SIZE{64};

/// 最大 slot 数量
static constexpr size_t SHM_MAX_SLOTS{16};

/// ShmHeader 中的 magic number（"GUGAUI##" 的十六进制）
static constexpr uint64_t SHM_MAGIC{0x4755474155492323ULL};

/// 当前 shm 布局版本（结构体变更时递增）
static constexpr uint32_t SHM_VERSION{1};

// ==================== 共享内存头部 ====================

/**
 * @brief 共享内存头部，包含版本校验和 slot 元数据表。
 */
struct ShmHeader {
  /// 魔数: 用于校验 shm 是否由 guga_ui 创建
  uint64_t magic{SHM_MAGIC};

  /// 布局版本: UI 端校验结构体兼容性
  uint32_t version{SHM_VERSION};

  /// 已注册的 slot 数量
  uint32_t slot_count{};

  /// 保留扩展
  uint8_t reserved[48]{};
};

static_assert(sizeof(ShmHeader) == 64,
              "ShmHeader must be 64 bytes (cache-line aligned)");

// ==================== 槽位元数据 ====================

/**
 * @brief 每个 slot 的元数据，描述数据区位置和写入进度。
 *
 * ShmSlot 本身存放在 shm 的固定偏移区，不随数据更新而移动。
 */
struct alignas(64) ShmSlot {
  /// 数据区相对于 shm 基址的偏移（字节）
  uint64_t data_offset{};

  /// 数据区实际大小（应为 SHM_SLOT_DATA_SIZE）
  uint64_t data_size{SHM_SLOT_DATA_SIZE};

  /// 写入序号：每次成功写入后原子 +2（偶数=数据完整，奇数=写入中），
  /// UI 端通过比较此值判断数据是否更新
  std::atomic<uint64_t> seq{0};

  /// 保留
  uint8_t _pad[40]{};
};

static_assert(sizeof(ShmSlot) == 64,
              "ShmSlot must be 64 bytes for cache-line alignment");

// ==================== 工具函数 ====================

/**
 * @brief 计算共享内存所需的全部大小。
 *
 * @param slot_count 槽位总数，必须 ≤ SHM_MAX_SLOTS。
 * @return 共享内存总字节数。
 *
 * 布局: [ShmHeader: 64B] + [ShmSlot × slot_count: 每个 32B]
 *        + [Data × slot_count: 每个 SHM_SLOT_DATA_SIZE]
 */
inline constexpr size_t calcShmSize(size_t slot_count) {
  return sizeof(ShmHeader)
       + slot_count * sizeof(ShmSlot)
       + slot_count * SHM_SLOT_DATA_SIZE;
}

/**
 * @brief 根据 slot_id 计算该槽数据区在 shm 中的偏移量。
 *
 * @param slot_index slot 索引 (0-based)。
 * @return 数据区起始的字节偏移。
 */
inline constexpr size_t calcSlotDataOffset(size_t slot_index) {
  return sizeof(ShmHeader)
       + SHM_MAX_SLOTS * sizeof(ShmSlot)  // 固定槽元数据区
       + slot_index * SHM_SLOT_DATA_SIZE;  // 第 slot_index 个数据区
}

/**
 * @brief 根据 slot_index 计算该槽元数据在 shm 中的偏移量。
 *
 * @param slot_index 0-based 槽位索引，必须 < SHM_MAX_SLOTS。
 * @return 该槽 ShmSlot 起始的字节偏移。
 */
inline constexpr size_t calcSlotMetaOffset(size_t slot_index) {
  return sizeof(ShmHeader) + slot_index * sizeof(ShmSlot);
}

}  // namespace guga_ui

#endif  // GUGA_UI_COMMON_SHM_LAYOUT_HPP
