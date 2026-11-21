#include "sentry_comm/snapshot_reader.hpp"

#include "ipc/IpcServer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace {

  using namespace std::chrono_literals;

  std::string makeShmName(const std::string& suffix) {
    return "/sentry_comm_test_" + std::to_string(::getpid()) + "_" + suffix;
  }

  template <typename Predicate>
  bool waitFor(Predicate predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return predicate();
  }

  TEST(SnapshotReaderTest, RejectsInvalidRate) {
    EXPECT_THROW(
        sentry_comm::SnapshotReader(
            sentry_comm::SnapshotReader::Config{"/sentry_comm_invalid", 0.0}),
        std::invalid_argument);
    EXPECT_THROW(
        sentry_comm::SnapshotReader(
            sentry_comm::SnapshotReader::Config{"/sentry_comm_invalid", -1.0}),
        std::invalid_argument);
    EXPECT_THROW(
        sentry_comm::SnapshotReader(sentry_comm::SnapshotReader::Config{
            "/sentry_comm_invalid", std::numeric_limits<double>::quiet_NaN()}),
        std::invalid_argument);
  }

  TEST(SnapshotReaderTest, CountsFailuresWithoutServer) {
    sentry_comm::SnapshotReader reader(
        {makeShmName("missing"),
         sentry_comm::SnapshotReader::DEFAULT_READ_RATE_HZ});
    reader.start();

    ASSERT_TRUE(
        waitFor([&reader] { return reader.stats().failures >= 3; }, 2000ms));

    s2c::serial_comm::AllInfo snapshot{};
    EXPECT_FALSE(reader.hasSnapshot());
    EXPECT_FALSE(reader.latestSnapshot(snapshot));
    EXPECT_EQ(reader.stats().successes, 0u);

    reader.stop();
  }

  TEST(SnapshotReaderTest, ReadsLatestPublishedSnapshot) {
    const std::string shm_name = makeShmName("publish");
    s2c::ipc::IpcServer server(shm_name);

    s2c::serial_comm::AllInfo published{};
    published.mcu_data.mark = 42;
    server.publish(s2c::serial_comm::kAllInfoChannel, published);

    sentry_comm::SnapshotReader reader({shm_name, 200.0});
    reader.start();

    s2c::serial_comm::AllInfo snapshot{};
    ASSERT_TRUE(waitFor(
        [&reader, &snapshot] {
          return reader.latestSnapshot(snapshot)
                 && snapshot.mcu_data.mark == 42;
        },
        2000ms));
    EXPECT_TRUE(reader.hasSnapshot());
    EXPECT_GT(reader.stats().successes, 0u);
    EXPECT_GT(reader.stats().last_success_time, 0.0);

    s2c::serial_comm::AllInfo updated = published;
    updated.mcu_data.mark = 43;
    server.publish(s2c::serial_comm::kAllInfoChannel, updated);

    ASSERT_TRUE(waitFor(
        [&reader, &snapshot] {
          return reader.latestSnapshot(snapshot)
                 && snapshot.mcu_data.mark == 43;
        },
        2000ms));

    reader.stop();
    EXPECT_TRUE(reader.latestSnapshot(snapshot));
    EXPECT_EQ(snapshot.mcu_data.mark, 43u);
  }

  TEST(SnapshotReaderTest, ReadsAtConfiguredRate) {
    sentry_comm::SnapshotReader reader({makeShmName("rate"), 100.0});
    reader.start();
    std::this_thread::sleep_for(500ms);
    reader.stop();

    const sentry_comm::SnapshotReader::Stats stats = reader.stats();
    EXPECT_GE(stats.attempts, 25u);
    EXPECT_LE(stats.attempts, 75u);
  }

  TEST(SnapshotReaderTest, RejectsSecondStart) {
    sentry_comm::SnapshotReader reader({makeShmName("twice"), 100.0});
    reader.start();
    EXPECT_THROW(reader.start(), std::logic_error);
    reader.stop();
  }

}  // namespace
