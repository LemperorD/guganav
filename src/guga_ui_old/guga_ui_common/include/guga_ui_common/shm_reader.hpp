#ifndef GUGA_UI_COMMON_SHM_READER_HPP
#define GUGA_UI_COMMON_SHM_READER_HPP
/**
 * @file shm_reader.hpp
 * @brief 共享内存读取端，供 UI 进程读取算法模块写入的数据。
 *
 * 使用方式（在 Pangolin UI 进程中）：
 *
 *   1. 构造 ShmReader reader;
 *   2. 调用 reader.open("guga_ui_shm");
 *   3. 每帧渲染前调用 reader.checkFresh(slot_id) 判断数据是否更新
 *   4. 调用 reader.read(slot_id, &ui_struct) 读取最新数据
 *
 * 读取是 lock-free 的：先用 acquire load 读 seq（偶数），再读数据区，
 * 然后再读一次 seq，如果前后一致则数据完整。
 */

#include <atomic>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "shm_layout.hpp"
#include "ui_types.hpp"

namespace guga_ui {

class ShmReader {
public:
  ShmReader() = default;

  ~ShmReader() {
    if (base_addr_ != nullptr) {
      munmap(base_addr_, shm_size_);
    }
    if (shm_fd_ >= 0) {
      close(shm_fd_);
    }
  }

  // 禁止拷贝
  ShmReader(const ShmReader&) = delete;
  ShmReader& operator=(const ShmReader&) = delete;

  // 支持移动
  ShmReader(ShmReader&& other) noexcept
      : shm_fd_(other.shm_fd_),
        base_addr_(other.base_addr_),
        shm_size_(other.shm_size_),
        last_seqs_(std::move(other.last_seqs_)) {
    other.shm_fd_ = -1;
    other.base_addr_ = nullptr;
  }

  ShmReader& operator=(ShmReader&& other) noexcept {
    if (this != &other) {
      if (base_addr_) munmap(base_addr_, shm_size_);
      shm_fd_ = other.shm_fd_;
      base_addr_ = other.base_addr_;
      shm_size_ = other.shm_size_;
      last_seqs_ = std::move(other.last_seqs_);
      other.shm_fd_ = -1;
      other.base_addr_ = nullptr;
    }
    return *this;
  }

  /**
   * @brief 打开已有共享内存（只读模式）。
   *
   * @param name 共享内存名称，与 ShmWriter 使用相同的名称。
   * @return true 成功打开并校验通过，false 失败。
   */
  [[nodiscard]] bool open(const std::string& name) {
    name_ = name;

    shm_fd_ = shm_open(name_.c_str(), O_RDONLY, 0666);
    if (shm_fd_ < 0) {
      std::cerr << "[ShmReader] shm_open failed (maybe no writer yet): "
                << strerror(errno) << std::endl;
      return false;
    }

    // 获取 shm 大小
    struct stat sb;
    if (fstat(shm_fd_, &sb) != 0) {
      std::cerr << "[ShmReader] fstat failed: " << strerror(errno)
                << std::endl;
      close(shm_fd_);
      shm_fd_ = -1;
      return false;
    }
    shm_size_ = static_cast<size_t>(sb.st_size);

    // 预期的 shm 大小
    const size_t expected_size = calcShmSize(SHM_MAX_SLOTS);
    if (shm_size_ < expected_size) {
      std::cerr << "[ShmReader] shm size too small: " << shm_size_
                << " < " << expected_size << std::endl;
      close(shm_fd_);
      shm_fd_ = -1;
      return false;
    }

    // mmap 只读
    base_addr_ = mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED,
                      shm_fd_, 0);
    if (base_addr_ == MAP_FAILED) {
      std::cerr << "[ShmReader] mmap failed: " << strerror(errno)
                << std::endl;
      close(shm_fd_);
      shm_fd_ = -1;
      base_addr_ = nullptr;
      return false;
    }

    // 校验 magic 和 version
    auto* header = static_cast<const ShmHeader*>(base_addr_);
    if (header->magic != SHM_MAGIC) {
      std::cerr << "[ShmReader] shm magic mismatch: 0x" << std::hex
                << header->magic << " != 0x" << SHM_MAGIC << std::dec
                << std::endl;
      munmap(base_addr_, shm_size_);
      close(shm_fd_);
      base_addr_ = nullptr;
      shm_fd_ = -1;
      return false;
    }
    if (header->version != SHM_VERSION) {
      std::cerr << "[ShmReader] shm version mismatch: " << header->version
                << " != " << SHM_VERSION << std::endl;
      munmap(base_addr_, shm_size_);
      close(shm_fd_);
      base_addr_ = nullptr;
      shm_fd_ = -1;
      return false;
    }

    // 初始化 seq 缓存
    last_seqs_.fill(0);

    std::cout << "[ShmReader] opened shm \"" << name_ << "\", "
              << header->slot_count << " slots" << std::endl;

    return true;
  }

  /**
   * @brief 查询共享内存是否有效。
   * @return true 表示 shm 已打开且通过了 magic/version 校验。
   */
  [[nodiscard]] bool isValid() const {
    if (base_addr_ == nullptr) return false;
    auto* header = static_cast<const ShmHeader*>(base_addr_);
    return (header->magic == SHM_MAGIC && header->version == SHM_VERSION);
  }

  /**
   * @brief 获取已注册的 slot 总数。
   * @return slot 数量，shm 未打开时返回 0。
   */
  [[nodiscard]] uint32_t slotCount() const {
    if (!isValid()) return 0;
    auto* header = static_cast<const ShmHeader*>(base_addr_);
    return header->slot_count;
  }

  /**
   * @brief 检查指定槽位是否有新数据写入（自上次 read 以来）。
   *
   * @param slot_id 槽位 ID。
   * @return true 表示 seq 已递增，有新数据。
   */
  [[nodiscard]] bool checkFresh(UiSlotId slot_id) const {
    uint32_t idx = static_cast<uint32_t>(slot_id);
    if (idx >= SHM_MAX_SLOTS) return false;

    const auto* slot = slotMetaAt(idx);
    uint64_t current_seq = slot->seq.load(std::memory_order_acquire);
    return (current_seq != last_seqs_[idx]);
  }

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
  [[nodiscard]] bool read(UiSlotId slot_id, void* out, size_t size) const {
    uint32_t idx = static_cast<uint32_t>(slot_id);
    if (idx >= SHM_MAX_SLOTS) return false;
    if (base_addr_ == nullptr) return false;

    const auto* slot = slotMetaAt(idx);
    const auto* data = static_cast<const uint8_t*>(base_addr_)
                     + slot->data_offset;

    // 最多重试 3 次，防止在极端竞争下死循环
    for (int retry = 0; retry < 3; ++retry) {
      uint64_t seq1 = slot->seq.load(std::memory_order_acquire);

      // seq 为奇数 → 写入进行中，等一下重试
      if (seq1 & 1ULL) {
        continue;
      }

      // 复制数据
      const size_t copy_size = (size > slot->data_size) ? slot->data_size
                                                          : size;
      std::memcpy(out, data, copy_size);

      uint64_t seq2 = slot->seq.load(std::memory_order_acquire);

      if (seq1 == seq2) {
        // 数据完整
        last_seqs_[idx] = seq2;
        return true;
      }
      // seq 变了→写入刚好发生在 memcpy 之间，重试
    }

    return false;  // 重试耗尽
  }

  /**
   * @brief 获取指定槽位的当前 seq 号（用于简单判断是否有更新）。
   * @param slot_id 槽位 ID。
   * @return 当前 seq，槽位无效时返回 0。
   */
  [[nodiscard]] uint64_t getSeq(UiSlotId slot_id) const {
    uint32_t idx = static_cast<uint32_t>(slot_id);
    if (idx >= SHM_MAX_SLOTS || base_addr_ == nullptr) return 0;
    const auto* slot = slotMetaAt(idx);
    return slot->seq.load(std::memory_order_acquire);
  }

private:
  /**
   * @brief 按索引获取 slot 元数据指针（const 版本）。
   */
  const ShmSlot* slotMetaAt(size_t index) const {
    auto* base = static_cast<const uint8_t*>(base_addr_);
    auto offset = calcSlotMetaOffset(index);
    return reinterpret_cast<const ShmSlot*>(base + offset);
  }

  std::string name_;
  int shm_fd_{-1};
  void* base_addr_{nullptr};
  size_t shm_size_{};

  /// 记录每个槽位上次读取时的 seq，用于 checkFresh 比较
  mutable std::array<uint64_t, SHM_MAX_SLOTS> last_seqs_{};
};

}  // namespace guga_ui

#endif  // GUGA_UI_COMMON_SHM_READER_HPP
