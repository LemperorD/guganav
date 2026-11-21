#include "sentry_comm/command_writer.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sentry_comm {

  namespace {

    double checkedPeriodSeconds(double write_rate_hz) {
      if (!std::isfinite(write_rate_hz) || write_rate_hz <= 0.0) {
        throw std::invalid_argument("写入频率必须为有限正值");
      }
      return 1.0 / write_rate_hz;
    }

    bool sendCommand(s2c::serial_comm::SerialCommClient& client,
                     const s2c::serial_comm::VisWrite& command) {
      return client.vis_write(command);
    }

    bool sendCommand(s2c::serial_comm::SerialCommClient& client,
                     const s2c::serial_comm::NavWrite& command) {
      return client.nav_write(command);
    }

  }  // namespace

  template <typename PayloadT>
  CommandWriter<PayloadT>::CommandWriter() : CommandWriter(Config{}) {
  }

  template <typename PayloadT>
  CommandWriter<PayloadT>::CommandWriter(Config config)
      : config_(std::move(config)),
        write_period_seconds_(checkedPeriodSeconds(config_.write_rate_hz)),
        client_(config_.shm_name) {
  }

  template <typename PayloadT>
  CommandWriter<PayloadT>::~CommandWriter() {
    stop();
  }

  template <typename PayloadT>
  void CommandWriter<PayloadT>::setCommand(const PayloadT& command) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = command;
    has_command_ = true;
  }

  template <typename PayloadT>
  void CommandWriter<PayloadT>::update(
      const std::function<void(PayloadT&)>& mutator) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    mutator(command_);
    has_command_ = true;
  }

  template <typename PayloadT>
  bool CommandWriter<PayloadT>::hasCommand() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return has_command_;
  }

  template <typename PayloadT>
  bool CommandWriter<PayloadT>::command(PayloadT& command) const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!has_command_) {
      return false;
    }
    command = command_;
    return true;
  }

  template <typename PayloadT>
  void CommandWriter<PayloadT>::start() {
    if (thread_.joinable()) {
      throw std::logic_error("CommandWriter::start: 写入线程已在运行");
    }
    is_running_.store(true, std::memory_order_release);
    try {
      thread_ = std::thread(&CommandWriter::loop, this);
    } catch (...) {
      is_running_.store(false, std::memory_order_release);
      throw;
    }
  }

  template <typename PayloadT>
  void CommandWriter<PayloadT>::stop() {
    is_running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  template <typename PayloadT>
  typename CommandWriter<PayloadT>::Stats CommandWriter<PayloadT>::stats()
      const {
    Stats result;
    result.attempts = attempts_.load(std::memory_order_relaxed);
    result.successes = successes_.load(std::memory_order_relaxed);
    result.failures = failures_.load(std::memory_order_relaxed);
    result.last_success_time = last_success_time_.load(
        std::memory_order_relaxed);
    return result;
  }

  template <typename PayloadT>
  void CommandWriter<PayloadT>::loop() {
    const auto period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(write_period_seconds_));
    while (is_running_.load(std::memory_order_acquire)) {
      // 每轮重新取起点，误差不跨轮累积。
      const auto cycle_start = std::chrono::steady_clock::now();
      PayloadT command{};
      bool should_send = false;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        should_send = has_command_;
        if (should_send) {
          command = command_;
        }
      }
      if (should_send) {
        attempts_.fetch_add(1, std::memory_order_relaxed);
        if (sendCommand(client_, command)) {
          successes_.fetch_add(1, std::memory_order_relaxed);
          last_success_time_.store(
              s2c::serial_comm::steadySeconds(std::chrono::steady_clock::now()),
              std::memory_order_relaxed);
        } else {
          failures_.fetch_add(1, std::memory_order_relaxed);
        }
      }
      std::this_thread::sleep_until(cycle_start + period);
    }
  }

  template class CommandWriter<s2c::serial_comm::VisWrite>;
  template class CommandWriter<s2c::serial_comm::NavWrite>;

}  // namespace sentry_comm
