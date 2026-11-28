#include "sentry_comm/command_writer.hpp"

#include "ipc/IpcServer.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace {

  using namespace std::chrono_literals;

  std::string makeShmName(const std::string& suffix) {
    return "/sentry_comm_writer_test_" + std::to_string(::getpid()) + "_"
           + suffix;
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

  TEST(CommandWriterTest, RejectsInvalidRate) {
    EXPECT_THROW(sentry_comm::NavCommandWriter(
                     sentry_comm::NavCommandWriter::Config{"/invalid", 0.0}),
                 std::invalid_argument);
    EXPECT_THROW(sentry_comm::NavCommandWriter(
                     sentry_comm::NavCommandWriter::Config{"/invalid", -1.0}),
                 std::invalid_argument);
    EXPECT_THROW(
        sentry_comm::VisCommandWriter(sentry_comm::VisCommandWriter::Config{
            "/invalid", std::numeric_limits<double>::quiet_NaN()}),
        std::invalid_argument);
  }

  TEST(CommandWriterTest, SendsNothingBeforeCommandIsSet) {
    const std::string shm_name = makeShmName("idle");
    s2c::ipc::IpcServer server(shm_name);
    std::atomic<int> vis_count{};
    server.setHandler<s2c::serial_comm::VisWrite>(
        s2c::serial_comm::kVisWriteChannel,
        [&vis_count](const s2c::serial_comm::VisWrite&) { ++vis_count; });
    server.start();

    sentry_comm::VisCommandWriter writer({shm_name, 200.0});
    writer.start();
    std::this_thread::sleep_for(100ms);

    EXPECT_EQ(vis_count.load(), 0);
    EXPECT_EQ(writer.stats().attempts, 0u);

    writer.stop();
  }

  TEST(CommandWriterTest, DeliversLatestVisCommand) {
    const std::string shm_name = makeShmName("vis");
    s2c::ipc::IpcServer server(shm_name);
    std::mutex latest_mutex;
    s2c::serial_comm::VisWrite latest{};
    std::atomic<int> vis_count{};
    server.setHandler<s2c::serial_comm::VisWrite>(
        s2c::serial_comm::kVisWriteChannel,
        [&latest_mutex, &latest,
         &vis_count](const s2c::serial_comm::VisWrite& write) {
          std::lock_guard<std::mutex> lock(latest_mutex);
          latest = write;
          ++vis_count;
        });
    server.start();

    s2c::serial_comm::VisWrite command{};
    command.serial.fire_control.fire = 1;
    command.serial.axes.pitch_mode = s2c::com::mcu::AXIS_MODE_TORQUE_PLUS_PID;
    command.serial.axes.pitch_torque = 0.5f;

    sentry_comm::VisCommandWriter writer({shm_name, 200.0});
    writer.setCommand(command);
    writer.start();

    EXPECT_TRUE(waitFor(
        [&] {
          std::lock_guard<std::mutex> lock(latest_mutex);
          return vis_count.load() > 0 && latest.serial.axes.pitch_torque == 0.5f
                 && latest.serial.fire_control.fire == 1;
        },
        2000ms));

    writer.update([](s2c::serial_comm::VisWrite& write) {
      write.serial.axes.pitch_torque = -0.25f;
    });

    EXPECT_TRUE(waitFor(
        [&] {
          std::lock_guard<std::mutex> lock(latest_mutex);
          return latest.serial.axes.pitch_torque == -0.25f
                 && latest.serial.fire_control.fire == 1;
        },
        2000ms));

    writer.stop();
    EXPECT_GT(writer.stats().successes, 0u);
    EXPECT_EQ(writer.stats().failures, 0u);
  }

  TEST(CommandWriterTest, NavWriterUsesNavChannelOnly) {
    const std::string shm_name = makeShmName("nav");
    s2c::ipc::IpcServer server(shm_name);
    std::atomic<int> vis_count{};
    std::atomic<int> nav_count{};
    std::atomic<float> nav_velocity{};
    server.setHandler<s2c::serial_comm::VisWrite>(
        s2c::serial_comm::kVisWriteChannel,
        [&vis_count](const s2c::serial_comm::VisWrite&) { ++vis_count; });
    server.setHandler<s2c::serial_comm::NavWrite>(
        s2c::serial_comm::kNavWriteChannel,
        [&nav_count, &nav_velocity](const s2c::serial_comm::NavWrite& write) {
          nav_velocity.store(
              write.serial.deform_chassis.displacement_target_velocity_x);
          ++nav_count;
        });
    server.start();

    s2c::serial_comm::NavWrite command{};
    command.serial.deform_chassis.displacement_target_velocity_x = 0.3f;

    sentry_comm::NavCommandWriter writer({shm_name, 200.0});
    writer.setCommand(command);
    writer.start();

    EXPECT_TRUE(waitFor(
        [&] { return nav_count.load() > 0 && nav_velocity.load() == 0.3f; },
        2000ms));
    EXPECT_EQ(vis_count.load(), 0);

    writer.stop();
  }

  TEST(CommandWriterTest, CountsFailuresWithoutServer) {
    sentry_comm::NavCommandWriter writer({makeShmName("missing"), 200.0});
    writer.setCommand(s2c::serial_comm::NavWrite{});
    writer.start();

    EXPECT_TRUE(
        waitFor([&writer] { return writer.stats().failures >= 3; }, 2000ms));
    EXPECT_EQ(writer.stats().successes, 0u);

    writer.stop();
  }

  TEST(CommandWriterTest, RejectsSecondStart) {
    sentry_comm::NavCommandWriter writer({makeShmName("twice"), 100.0});
    writer.start();
    EXPECT_THROW(writer.start(), std::logic_error);
    writer.stop();
  }

}  // namespace
