#pragma once

#include "SerialCommClient.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace sentry_comm {

  // 按固定频率把最近一次提交的命令写入共享内存，供串口通信服务端取走。
  template <typename PayloadT>
  class CommandWriter {
  public:
    static constexpr double DEFAULT_WRITE_RATE_HZ = 100.0;

    struct Config {
      std::string shm_name{s2c::serial_comm::kDefaultShmName};
      double write_rate_hz{DEFAULT_WRITE_RATE_HZ};
    };

    struct Stats {
      std::uint64_t attempts{};
      std::uint64_t successes{};
      std::uint64_t failures{};
      double last_success_time{};
    };

    CommandWriter();
    explicit CommandWriter(Config config);
    ~CommandWriter();

    CommandWriter(const CommandWriter&) = delete;
    CommandWriter& operator=(const CommandWriter&) = delete;

    void setCommand(const PayloadT& command);
    void update(const std::function<void(PayloadT&)>& mutator);
    [[nodiscard]] bool hasCommand() const;
    [[nodiscard]] bool command(PayloadT& command) const;

    void start();
    void stop();

    [[nodiscard]] Stats stats() const;

  private:
    void loop();

    Config config_;
    double write_period_seconds_{};
    s2c::serial_comm::SerialCommClient client_;

    mutable std::mutex command_mutex_;
    PayloadT command_{};
    bool has_command_{};

    std::atomic<std::uint64_t> attempts_{};
    std::atomic<std::uint64_t> successes_{};
    std::atomic<std::uint64_t> failures_{};
    std::atomic<double> last_success_time_{};

    std::thread thread_;
    std::atomic<bool> is_running_{};
  };

  using VisCommandWriter = CommandWriter<s2c::serial_comm::VisWrite>;
  using NavCommandWriter = CommandWriter<s2c::serial_comm::NavWrite>;

}  // namespace sentry_comm
