#ifndef GUGA_UI_COMMON_SHM_WRITER_HPP
#define GUGA_UI_COMMON_SHM_WRITER_HPP

/**
 * @file shm_writer.hpp
 * @brief 共享内存写入端，供算法模块在 publish 旁路调用。
 *
 * 使用方式（每个算法模块中）：
 *
 *   1. 在类成员中声明 ShmWriter writer_;
 *   2. 构造时调用 writer_.init("guga_ui_shm", UiSlotId::DECISION);
 *   3. 每次有数据更新时调用 writer_.write(&ui_decision, sizeof(ui_decision));
 *
 * 写入是 lock-free 的：先 memcpy 数据区，再 atomic store 递增 seq。
 * 读取端通过 seq 的奇偶性和前后一致性来判断数据是否完整。
 */

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "guga_ui_common/shm_layout.hpp"
#include "guga_ui_common/ui_types.hpp"

namespace guga_ui {

class ShmWriter {
 public:
  ShmWriter() = default;

  ~ShmWriter() {
    // 不 unlink shm——由最后一次使用它的进程负责
    // 只 munmap
    if (base_addr_ != nullptr) {
      munmap(base_addr_, shm_size_);
    }
  }

  // 禁止拷贝（共享内存资源独占写入指针）
  ShmWriter(const ShmWriter&) = delete;
  ShmWriter& operator=(const ShmWriter&) = delete;
  ShmWriter(ShmWriter&& other) noexcept
      : shm_fd_(other.shm_fd_),
        base_addr_(other.base_addr_),
        shm_size_(other.shm_size_),
        slot_data_(other.slot_data_),
        slot_meta_(other.slot_meta_) {
    other.shm_fd_ = -1;
    other.base_addr_ = nullptr;
    other.slot_data_ = nullptr;
    other.slot_meta_ = nullptr;
  }

  ShmWriter& operator=(ShmWriter&& other) noexcept {
    if (this != &other) {
      if (base_addr_) munmap(base_addr_, shm_size_);
      shm_fd_ = other.shm_fd_;
      base_addr_ = other.base_addr_;
      shm_size_ = other.shm_size_;
      slot_data_ = other.slot_data_;
      slot_meta_ = other.slot_meta_;
      other.shm_fd_ = -1;
      other.base_addr_ = nullptr;
      other.slot_data_ = nullptr;
      other.slot_meta_ = nullptr;
    }
    return *this;
  }

  /**
   * @brief 初始化写入端，创建或打开共享内存。
   *
   * @param name 共享内存名称（如 "guga_ui_shm"）。
   * @param slot_id 要写入的槽位 ID（UiSlotId 枚举值）。
   * @return true 成功，false 失败。
   *
   * 首次调用会创建 shm 并初始化 header；后续调用会打开已有 shm。
   */
  [[nodiscard]] bool init(const std::string& name, UiSlotId slot_id) {
    name_ = name;
    slot_index_ = static_cast<uint32_t>(slot_id);

    if (slot_index_ >= SHM_MAX_SLOTS) {
      std::cerr << "[ShmWriter] slot_id " << slot_index_
                << " exceeds SHM_MAX_SLOTS=" << SHM_MAX_SLOTS << std::endl;
      return false;
    }

    shm_size_ = calcShmSize(SHM_MAX_SLOTS);

    // 尝试打开已有 shm
    bool created{false};
    shm_fd_ = shm_open(name_.c_str(), O_RDWR, 0666);
    if (shm_fd_ < 0) {
      // 不存在则创建
      shm_fd_ = shm_open(name_.c_str(),
                         O_RDWR | O_CREAT | O_EXCL, 0666);
      if (shm_fd_ < 0) {
        std::cerr << "[ShmWriter] shm_open failed: " << strerror(errno)
                  << std::endl;
        return false;
      }
      created = true;
    }

    // 设置大小
    if (created && ftruncate(shm_fd_, static_cast<off_t>(shm_size_)) != 0) {
      std::cerr << "[ShmWriter] ftruncate failed: " << strerror(errno)
                << std::endl;
      close(shm_fd_);
      shm_fd_ = -1;
      return false;
    }

    // mmap 整个共享内存
    base_addr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE,
                      MAP_SHARED, shm_fd_, 0);
    if (base_addr_ == MAP_FAILED) {
      std::cerr << "[ShmWriter] mmap failed: " << strerror(errno)
                << std::endl;
      close(shm_fd_);
      shm_fd_ = -1;
      base_addr_ = nullptr;
      return false;
    }

    // 定位 header
    auto* header = static_cast<ShmHeader*>(base_addr_);

    if (created) {
      // 首次创建：初始化 header 和所有 slot meta
      header->magic = SHM_MAGIC;
      header->version = SHM_VERSION;
      header->slot_count = SHM_MAX_SLOTS;

      for (size_t i = 0; i < SHM_MAX_SLOTS; ++i) {
        auto* slot = slotMetaAt(i);
        slot->data_offset = calcSlotDataOffset(i);
        slot->data_size = SHM_SLOT_DATA_SIZE;
        slot->seq = 0;
      }
    } else {
      // 验证已有 shm 的兼容性
      if (header->magic != SHM_MAGIC) {
        std::cerr << "[ShmWriter] shm magic mismatch: 0x" << std::hex
                  << header->magic << " != 0x" << SHM_MAGIC << std::dec
                  << std::endl;
        munmap(base_addr_, shm_size_);
        close(shm_fd_);
        base_addr_ = nullptr;
        shm_fd_ = -1;
        return false;
      }
      if (header->version != SHM_VERSION) {
        std::cerr << "[ShmWriter] shm version mismatch: " << header->version
                  << " != " << SHM_VERSION << std::endl;
        munmap(base_addr_, shm_size_);
        close(shm_fd_);
        base_addr_ = nullptr;
        shm_fd_ = -1;
        return false;
      }
    }

    // 缓存本槽的数据区和元数据区指针
    slot_meta_ = slotMetaAt(slot_index_);
    slot_data_ = static_cast<uint8_t*>(base_addr_)
               + slot_meta_->data_offset;

    return true;
  }

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
  void write(const void* data, size_t size) {
    if (slot_data_ == nullptr || slot_meta_ == nullptr) {
      return;
    }

    const size_t copy_size = (size > SHM_SLOT_DATA_SIZE) ? SHM_SLOT_DATA_SIZE
                                                          : size;

    // 1. 将当前 seq 递增为奇数，表示"正在写入"
    //    读取端看到奇数 → 数据不可靠，跳过
    uint64_t seq = slot_meta_->seq.load(std::memory_order_acquire);
    slot_meta_->seq.store(seq | 1ULL, std::memory_order_release);

    // 2. 写入数据
    std::memcpy(slot_data_, data, copy_size);

    // 3. 将 seq 递增为偶数，表示"写入完成"
    //    release store 保证 memcpy 对读取端可见
    slot_meta_->seq.store(seq + 2ULL, std::memory_order_release);
  }

 private:
  /**
   * @brief 按索引获取 slot 元数据指针。
   *
   * @param index 0-based slot 索引。
   * @return 指向该 slot 元数据的指针（位于 shm 头部之后）。
   */
  ShmSlot* slotMetaAt(size_t index) {
    auto* base = static_cast<uint8_t*>(base_addr_);
    auto offset = calcSlotMetaOffset(index);
    return reinterpret_cast<ShmSlot*>(base + offset);
  }

  std::string name_;
  int shm_fd_{-1};
  void* base_addr_{nullptr};
  size_t shm_size_{};
  uint32_t slot_index_{};

  // 缓存的本槽数据区指针（避免每帧计算偏移）
  void* slot_data_{nullptr};
  ShmSlot* slot_meta_{nullptr};
};

}  // namespace guga_ui

#endif  // GUGA_UI_COMMON_SHM_WRITER_HPP
