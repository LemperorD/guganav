#ifndef GUGA_COMMON_SHM_WRITER_HPP
#define GUGA_COMMON_SHM_WRITER_HPP
/**
 * @file shm_writer.hpp
 * @brief 共享内存写入端
 *
 * 使用方式（每个算法模块中）：
 *
 *   1. 在类成员中声明 ShmWriter writer_;
 *   2. 构造时调用 writer_.init("guga_shm", SlotId::DECISION);
 *   3. 每次有数据更新时调用 writer_.write(&decision, sizeof(decision));
 *
 * 写入是 lock-free 的：先 memcpy 数据区，再 atomic store 递增 seq。
 * 读取端通过 seq 的奇偶性和前后一致性来判断数据是否完整。
 */

#include <cstdint>
#include <string>

#include "shm_layout.hpp"
#include "shm_types.hpp"

namespace guga_common {

class ShmWriter {
public:
  ShmWriter() = default;

  ~ShmWriter();

  // 禁止拷贝（共享内存资源独占写入指针）
  ShmWriter(const ShmWriter&) = delete;
  ShmWriter& operator=(const ShmWriter&) = delete;

  // 支持移动
  ShmWriter(ShmWriter&& other) noexcept;
  ShmWriter& operator=(ShmWriter&& other) noexcept;

  /**
   * @brief 初始化写入端，创建或打开共享内存。
   *
   * @param name 共享内存名称（如 "guga_ui_shm"）。
   * @param slot_id 要写入的槽位 ID（UiSlotId 枚举值）。
   * @return true 成功，false 失败。
   *
   * 首次调用会创建 shm 并初始化 header；后续调用会打开已有 shm。
   */
  [[nodiscard]] bool init(const std::string& name, SlotId slot_id);

  /**
   * @brief 将数据写入共享内存对应槽位（lock-free）。
   *
   * @param data 源数据指针。
   * @param size 数据大小（必须 ≤ SHM_SLOT_DATA_SIZE）。
   *
   * 写入协议：先写入数据区，再用 release store 递增 seq。
   * 读取端在看到 seq 变化（且低 1 位为 0，即偶数）后读取数据区。
   *
   * 性能：仅 memcpy + atomic store，典型延迟 < 100ns（L1 cache hit 时）。
   */
  void write(const void* data, size_t size);

private:
  /**
   * @brief 按索引获取 slot 元数据指针。
   *
   * @param index 0-based slot 索引。
   * @return 指向该 slot 元数据的指针（位于 shm 头部之后）。
   */
  ShmSlot* slotMetaAt(size_t index);

  std::string name_;
  int shm_fd_{-1};
  void* base_addr_{nullptr};
  size_t shm_size_{};
  uint32_t slot_index_{};

  /// 缓存的本槽数据区指针（避免每帧计算偏移）
  void* slot_data_{nullptr};
  ShmSlot* slot_meta_{nullptr};
};

} // namespace guga_common

#endif // GUGA_COMMON_SHM_WRITER_HPP
