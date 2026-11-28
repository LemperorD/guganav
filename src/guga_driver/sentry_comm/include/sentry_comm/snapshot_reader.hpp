#pragma once

#include "SerialCommClient.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace sentry_comm {

  // 按固定频率读取共享内存快照，缓存最近一份供调用方取用。
  class SnapshotReader {
  public:
    static constexpr double DEFAULT_READ_RATE_HZ = 100.0;

    struct Config {
      std::string shm_name{s2c::serial_comm::kDefaultShmName};
      double read_rate_hz{DEFAULT_READ_RATE_HZ};
    };

    struct Stats {
      std::uint64_t attempts{};
      std::uint64_t successes{};
      std::uint64_t failures{};
      double last_success_time{};
    };

    SnapshotReader();
    explicit SnapshotReader(Config config);
    ~SnapshotReader();

    SnapshotReader(const SnapshotReader&) = delete;
    SnapshotReader& operator=(const SnapshotReader&) = delete;

    void start();
    void stop();

    [[nodiscard]] bool hasSnapshot() const;
    [[nodiscard]] bool latestSnapshot(
        s2c::serial_comm::AllInfo& snapshot) const;
    [[nodiscard]] Stats stats() const;

  private:
    void loop();

    Config config_;
    double read_period_seconds_{};
    s2c::serial_comm::SerialCommClient client_;

    mutable std::mutex snapshot_mutex_;
    s2c::serial_comm::AllInfo latest_snapshot_{};
    bool has_snapshot_{};

    std::atomic<std::uint64_t> attempts_{};
    std::atomic<std::uint64_t> successes_{};
    std::atomic<std::uint64_t> failures_{};
    std::atomic<double> last_success_time_{};

    std::thread thread_;
    std::atomic<bool> is_running_{};
  };

}  // namespace sentry_comm
