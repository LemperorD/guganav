#include "sentry_comm/snapshot_reader.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace sentry_comm {

  namespace {

    double checkedPeriodSeconds(double read_rate_hz) {
      if (!std::isfinite(read_rate_hz) || read_rate_hz <= 0.0) {
        throw std::invalid_argument("读取频率必须为有限正值");
      }
      return 1.0 / read_rate_hz;
    }

  }  // namespace

  SnapshotReader::SnapshotReader() : SnapshotReader(Config{}) {
  }

  SnapshotReader::SnapshotReader(Config config)
      : config_(std::move(config)),
        read_period_seconds_(checkedPeriodSeconds(config_.read_rate_hz)),
        client_(config_.shm_name) {
  }

  SnapshotReader::~SnapshotReader() {
    stop();
  }

  void SnapshotReader::start() {
    if (thread_.joinable()) {
      throw std::logic_error("SnapshotReader::start: 读取线程已在运行");
    }
    is_running_.store(true, std::memory_order_release);
    try {
      thread_ = std::thread(&SnapshotReader::loop, this);
    } catch (...) {
      is_running_.store(false, std::memory_order_release);
      throw;
    }
  }

  void SnapshotReader::stop() {
    is_running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  bool SnapshotReader::hasSnapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return has_snapshot_;
  }

  bool SnapshotReader::latestSnapshot(
      s2c::serial_comm::AllInfo& snapshot) const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!has_snapshot_) {
      return false;
    }
    snapshot = latest_snapshot_;
    return true;
  }

  SnapshotReader::Stats SnapshotReader::stats() const {
    Stats result;
    result.attempts = attempts_.load(std::memory_order_relaxed);
    result.successes = successes_.load(std::memory_order_relaxed);
    result.failures = failures_.load(std::memory_order_relaxed);
    result.last_success_time = last_success_time_.load(
        std::memory_order_relaxed);
    return result;
  }

  void SnapshotReader::loop() {
    const auto period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(read_period_seconds_));
    while (is_running_.load(std::memory_order_acquire)) {
      // 每轮重新取起点，误差不跨轮累积。
      const auto start = std::chrono::steady_clock::now();
      s2c::serial_comm::AllInfo snapshot{};
      if (client_.vis_get_all_info(snapshot)) {
        {
          std::lock_guard<std::mutex> lock(snapshot_mutex_);
          latest_snapshot_ = snapshot;
          has_snapshot_ = true;
        }
        last_success_time_.store(
            s2c::serial_comm::steadySeconds(std::chrono::steady_clock::now()),
            std::memory_order_relaxed);
        successes_.fetch_add(1, std::memory_order_relaxed);
      } else {
        failures_.fetch_add(1, std::memory_order_relaxed);
      }
      attempts_.fetch_add(1, std::memory_order_relaxed);
      std::this_thread::sleep_until(start + period);
    }
  }

}  // namespace sentry_comm
