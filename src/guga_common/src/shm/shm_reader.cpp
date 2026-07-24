#include "guga_common/shm/shm_reader.hpp"

namespace guga_common {

ShmReader::ShmReader(ShmReader&& other) noexcept
  : shm_fd_(other.shm_fd_),
    base_addr_(other.base_addr_),
    shm_size_(other.shm_size_),
    last_seqs_(std::move(other.last_seqs_))
{
  other.shm_fd_ = -1;
  other.base_addr_ = nullptr;
}

ShmReader::~ShmReader() {
  if (base_addr_ != nullptr) {
    munmap(base_addr_, shm_size_);
  }
  if (shm_fd_ >= 0) {
    close(shm_fd_);
  }
}

ShmReader& ShmReader::operator=(ShmReader&& other) noexcept {
  if (this != &other) {
    if (base_addr_ != nullptr) {
      munmap(base_addr_, shm_size_);
    }
    shm_fd_ = other.shm_fd_;
    base_addr_ = other.base_addr_;
    shm_size_ = other.shm_size_;
    last_seqs_ = std::move(other.last_seqs_);
    other.shm_fd_ = -1;
    other.base_addr_ = nullptr;
  }
  return *this;
}

bool ShmReader::open(const std::string& name) {
  name_ = name;

  shm_fd_ = shm_open(name_.c_str(), O_RDONLY, 0666);
  if (shm_fd_ < 0) {
    std::cerr << "[ShmReader] shm_open failed (maybe no writer yet): "
              << strerror(errno) << std::endl;
    return false;
  }

  // 获取 shm 大小
  struct stat sb {};
  if (fstat(shm_fd_, &sb) != 0) {
    std::cerr << "[ShmReader] fstat failed: "
              << strerror(errno) << std::endl;
    close(shm_fd_);
    shm_fd_ = -1;
    return false;
  }
  shm_size_ = static_cast<size_t>(sb.st_size);

  // 校验 shm 大小
  const size_t expected_size = calcShmSize(SHM_MAX_SLOTS);
  if (shm_size_ < expected_size) {
    std::cerr << "[ShmReader] shm size too small: " << shm_size_
              << " < " << expected_size << std::endl;
    close(shm_fd_);
    shm_fd_ = -1;
    return false;
  }

  // mmap 只读
  base_addr_ = mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
  if (base_addr_ == MAP_FAILED) {
    std::cerr << "[ShmReader] mmap failed: " << strerror(errno)
              << std::endl;
    close(shm_fd_);
    shm_fd_ = -1;
    base_addr_ = nullptr;
    return false;
  }

  // 校验 magic
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

  // 初始化 seq 缓存
  last_seqs_.fill(0);

  std::cout << "[ShmReader] opened shm \"" << name_ << "\", "
            << header->slot_count << " slots" << std::endl;

  return true;
}

bool ShmReader::isValid() const {
  if (base_addr_ == nullptr) {
    return false;
  }
  auto* header = static_cast<const ShmHeader*>(base_addr_);
  return (header->magic == SHM_MAGIC);
}

uint32_t ShmReader::slotCount() const {
  if (!isValid()) {
    return 0;
  }
  auto* header = static_cast<const ShmHeader*>(base_addr_);
  return header->slot_count;
}

bool ShmReader::checkFresh(SlotId slot_id) const {
  uint32_t idx = static_cast<uint32_t>(slot_id);
  if (idx >= SHM_MAX_SLOTS) {
    return false;
  }

  const auto* slot = slotMetaAt(idx);
  uint64_t current_seq = slot->seq.load(std::memory_order_acquire);
  return (current_seq != last_seqs_[idx]);
}

bool ShmReader::read(SlotId slot_id, void* out, size_t size) const {
  uint32_t idx = static_cast<uint32_t>(slot_id);
  if (idx >= SHM_MAX_SLOTS) {
    return false;
  }
  if (base_addr_ == nullptr) {
    return false;
  }

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
    // seq 变了 → 写入刚好发生在 memcpy 之间，重试
  }

  return false;  // 重试耗尽
}

uint64_t ShmReader::getSeq(SlotId slot_id) const {
  uint32_t idx = static_cast<uint32_t>(slot_id);
  if (idx >= SHM_MAX_SLOTS || base_addr_ == nullptr) {
    return 0;
  }
  const auto* slot = slotMetaAt(idx);
  return slot->seq.load(std::memory_order_acquire);
}

const ShmSlot* ShmReader::slotMetaAt(size_t index) const {
  auto* base = static_cast<const uint8_t*>(base_addr_);
  auto offset = calcSlotMetaOffset(index);
  return reinterpret_cast<const ShmSlot*>(base + offset);
}

} // namespace guga_common
