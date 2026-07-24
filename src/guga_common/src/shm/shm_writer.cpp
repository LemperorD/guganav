#include "guga_common/shm/shm_writer.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace guga_common {

ShmWriter::ShmWriter(ShmWriter&& other) noexcept
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

ShmWriter::~ShmWriter() {
  // 不 unlink shm——由最后一次使用它的进程负责
  if (base_addr_ != nullptr) {
    munmap(base_addr_, shm_size_);
  }
}

ShmWriter& ShmWriter::operator=(ShmWriter&& other) noexcept {
  if (this != &other) {
    if (base_addr_ != nullptr) {
      munmap(base_addr_, shm_size_);
    }
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

bool ShmWriter::init(const std::string& name, SlotId slot_id) {
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
    shm_fd_ = shm_open(name_.c_str(), O_RDWR | O_CREAT | O_EXCL, 0666);
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
  }

  // 缓存本槽的数据区和元数据区指针
  slot_meta_ = slotMetaAt(slot_index_);
  slot_data_ = static_cast<uint8_t*>(base_addr_)
               + slot_meta_->data_offset;

  return true;
}

void ShmWriter::write(const void* data, size_t size) {
  if (slot_data_ == nullptr || slot_meta_ == nullptr) {
    return;
  }

  const size_t copy_size = (size > SHM_SLOT_DATA_SIZE) ? SHM_SLOT_DATA_SIZE
                                                        : size;

  // 1. 将当前 seq 递增为奇数，表示"正在写入"
  uint64_t seq = slot_meta_->seq.load(std::memory_order_acquire);
  slot_meta_->seq.store(seq | 1ULL, std::memory_order_release);

  // 2. 写入数据
  std::memcpy(slot_data_, data, copy_size);

  // 3. 将 seq 递增为偶数，表示"写入完成"
  slot_meta_->seq.store(seq + 2ULL, std::memory_order_release);
}

ShmSlot* ShmWriter::slotMetaAt(size_t index) {
  auto* base = static_cast<uint8_t*>(base_addr_);
  auto offset = calcSlotMetaOffset(index);
  return reinterpret_cast<ShmSlot*>(base + offset);
}

} // namespace guga_common
