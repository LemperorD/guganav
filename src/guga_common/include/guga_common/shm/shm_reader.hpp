#ifndef GUGA_COMMON_SHM_READER_HPP
#define GUGA_COMMON_SHM_READER_HPP
/**
 * @file shm_reader.hpp
 * @brief 共享内存读取端
 *
 * 使用方式：
 *
 *   1. 构造 ShmReader reader;
 *   2. 调用 reader.open("guga_shm");
 *   3. 每帧读取前调用 reader.checkFresh(slot_id) 判断数据是否更新
 *
 * 读取是 lock-free 的：先用 acquire load 读 seq（偶数），再读数据区，
 * 然后再读一次 seq，如果前后一致则数据完整。
 */

#include <array>
#include <cstdint>
#include <string>

#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "shm_layout.hpp"
#include "shm_types.hpp"

namespace guga_common {

class ShmReader {
public:
  ShmReader() = default;

  ~ShmReader();

  // 禁止拷贝
  ShmReader(const ShmReader&) = delete;
  ShmReader& operator=(const ShmReader&) = delete;

  // 支持移动
  ShmReader(ShmReader&& other) noexcept;
  ShmReader& operator=(ShmReader&& other) noexcept;

  /**
   * @brief 打开已有共享内存（只读模式）。
   *
   * @param name 共享内存名称，与 ShmWriter 使用相同的名称。
   * @return true 成功打开并校验通过，false 失败。
   */
  [[nodiscard]] bool open(const std::string& name);

  /**
   * @brief 查询共享内存是否有效。
   * @return true 表示 shm 已打开且通过了 magic/version 校验。
   */
  [[nodiscard]] bool isValid() const;

  /**
   * @brief 获取已注册的 slot 总数。
   * @return slot 数量，shm 未打开时返回 0。
   */
  [[nodiscard]] uint32_t slotCount() const;

  /**
   * @brief 检查指定槽位是否有新数据写入（自上次 read 以来）。
   *
   * @param slot_id 槽位 ID。
   * @return true 表示 seq 已递增，有新数据。
   */
  [[nodiscard]] bool checkFresh(SlotId slot_id) const;

  /**
   * @brief 从指定槽位读取数据（lock-free 安全读取）。
   *
   * @param slot_id 槽位 ID。
   * @param out 输出缓冲区指针。
   * @param size 输出缓冲区大小（最大复制字节数）。
   * @return true 读取成功，false 失败（槽位无效或读取中数据被覆盖）。
   *
   * 使用 double-check seq 模式：
   *   1. acquire load 读 seq（必须为偶数）
   *   2. memcpy 数据区
   *   3. acquire load 再读 seq，与第 1 步一致 → 数据有效
   *   4. 不一致则重试（最多 3 次）
   */
  [[nodiscard]] bool read(SlotId slot_id, void* out, size_t size) const;

  /**
   * @brief 获取指定槽位的当前 seq 号（用于简单判断是否有更新）。
   * @param slot_id 槽位 ID。
   * @return 当前 seq，槽位无效时返回 0。
   */
  [[nodiscard]] uint64_t getSeq(SlotId slot_id) const;

private:
  /**
   * @brief 按索引获取 slot 元数据指针（const 版本）。
   */
  const ShmSlot* slotMetaAt(size_t index) const;

  std::string name_;         // shm 名称
  int shm_fd_{-1};           // shm_open 返回的文件描述符
  void* base_addr_{nullptr}; // mmap 后的共享内存首地址
  size_t shm_size_{};        // 共享内存大小

  /// 记录每个槽位上次读取时的 seq，用于 checkFresh 比较
  mutable std::array<uint64_t, SHM_MAX_SLOTS> last_seqs_{};
};

} // namespace guga_common

#endif // GUGA_COMMON_SHM_READER_HPP
