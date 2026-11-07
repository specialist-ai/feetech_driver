#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "fake_servo.hpp"
#include "feetech/bus.hpp"

using fake::Bytes;
using feetech::Error;

class BusTest : public ::testing::Test {
 protected:
  void SetUp() override {
    servos.add(1, 1000);
    servos.add(2, -500);
    servos.add(3, 2048);
    ASSERT_TRUE(bus.open(pty.name, 1000000)) << bus.last_error();
  }
  fake::Pty pty;
  fake::Servos servos{pty.master};
  feetech::Bus bus;
};

TEST_F(BusTest, PingsReadsAndWrites) {
  EXPECT_TRUE(bus.is_open());
  EXPECT_TRUE(bus.ping(1)) << bus.last_error();
  std::vector<std::uint8_t> data;
  ASSERT_TRUE(bus.read_registers(2, feetech::reg::kPresentPosition, 2, data));
  EXPECT_EQ(data, (Bytes{0xF4, 0x81}));  // -500 in sign-magnitude
  ASSERT_TRUE(bus.write_registers(3, feetech::reg::kTorqueLimit, {0x10, 0x02}));
  EXPECT_EQ(servos.u16(3, feetech::reg::kTorqueLimit), 0x0210);
  EXPECT_EQ(bus.last_error_code(), Error::kNone);
  EXPECT_TRUE(bus.last_error().empty());
}

TEST_F(BusTest, SyncReadsFeedbackAndSyncWritesGoals) {
  std::vector<std::optional<feetech::Feedback>> out;
  ASSERT_EQ(bus.sync_read_feedback({3, 1, 2}, out), 3u) << bus.last_error();
  EXPECT_EQ(out[0]->position_counts, 2048);
  EXPECT_EQ(out[1]->position_counts, 1000);
  EXPECT_EQ(out[2]->position_counts, -500);
  EXPECT_DOUBLE_EQ(out[2]->voltage_volts, 12.0);
  // Id 9 is silent. The replies after it still reach their ids.
  EXPECT_EQ(bus.sync_read_feedback({1, 9, 2}, out), 2u);
  EXPECT_TRUE(out[0] && !out[1] && out[2]);
  EXPECT_EQ(bus.last_error_code(), Error::kTimeout);
  ASSERT_TRUE(bus.sync_write_goals(
      {{1, feetech::encode_goal_block(100)}, {2, feetech::encode_goal_block(-200)}}));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(servos.u16(1, feetech::reg::kGoalPosition), 100);
  EXPECT_EQ(servos.u16(2, feetech::reg::kGoalPosition), 0x8000 | 200);
  EXPECT_TRUE(bus.sync_write_goals({}));
}

TEST_F(BusTest, ReportsTheServoErrorByte) {
  servos.set_error(feetech::servo_error::kOverload);
  EXPECT_TRUE(bus.ping(1));
  EXPECT_EQ(bus.last_servo_error(), feetech::servo_error::kOverload);
  std::vector<std::optional<feetech::Feedback>> out;
  EXPECT_EQ(bus.sync_read_feedback({1, 2}, out), 2u);
  EXPECT_EQ(bus.last_servo_error(), feetech::servo_error::kOverload);
}

TEST_F(BusTest, TimesOutAfterTheTimeoutPlusWireTime) {
  bus.set_timeout_ms(30);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(bus.ping(9));
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_EQ(bus.last_error_code(), Error::kTimeout);
  EXPECT_GE(elapsed, std::chrono::milliseconds(30));
  EXPECT_LT(elapsed, std::chrono::milliseconds(300));
  bus.set_timeout_ms(0);
  EXPECT_EQ(bus.last_error_code(), Error::kInvalidArgument);
  EXPECT_EQ(bus.timeout_ms(), 30);
}

TEST_F(BusTest, RejectsABadChecksumAWrongIdAndNoise) {
  servos.set_hook([](const Bytes&) {
    Bytes reply = fake::status(1, 0, {});
    reply.back() ^= 0x01;
    return reply;
  });
  EXPECT_FALSE(bus.ping(1));
  EXPECT_EQ(bus.last_error_code(), Error::kMalformed);
  servos.set_hook([](const Bytes&) { return fake::status(7, 0, {}); });
  EXPECT_FALSE(bus.ping(1));
  EXPECT_EQ(bus.last_error_code(), Error::kWrongId);
  servos.set_hook([](const Bytes&) { return Bytes{0x00, 0x13, 0xFF, 0x37}; });
  EXPECT_FALSE(bus.ping(1));
  EXPECT_EQ(bus.last_error_code(), Error::kMalformed);
  // A valid reply after noise still counts.
  servos.set_hook([](const Bytes&) {
    Bytes reply{0x00, 0x42};
    const Bytes status = fake::status(1, 0, {});
    reply.insert(reply.end(), status.begin(), status.end());
    return reply;
  });
  EXPECT_TRUE(bus.ping(1)) << bus.last_error();
}

TEST_F(BusTest, ALateReplyDoesNotBecomeTheNextReply) {
  bool first = true;
  servos.set_hook([&first](const Bytes&) -> std::optional<Bytes> {
    if (!std::exchange(first, false)) return std::nullopt;
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    return fake::status(1, 0, {0xAA, 0xBB});
  });
  std::vector<std::uint8_t> data;
  EXPECT_FALSE(bus.read_registers(1, feetech::reg::kPresentPosition, 2, data));
  EXPECT_EQ(bus.last_error_code(), Error::kTimeout);
  ASSERT_TRUE(bus.read_registers(1, feetech::reg::kPresentPosition, 2, data)) << bus.last_error();
  EXPECT_EQ(data, (Bytes{0xE8, 0x03}));
}

TEST_F(BusTest, ASecondBusOnThePortIsBusy) {
  feetech::Bus other;
  EXPECT_FALSE(other.open(pty.name, 1000000));
  EXPECT_EQ(other.last_error_code(), Error::kPortBusy);
  bus.close();
  EXPECT_FALSE(bus.is_open());
  EXPECT_TRUE(other.open(pty.name, 1000000)) << other.last_error();
}

TEST_F(BusTest, TwoThreadsShareOneBus) {
  const auto loop = [this](std::uint8_t id, int& failures) {
    std::vector<std::optional<feetech::Feedback>> out;
    for (int i = 0; i < 200; ++i) {
      failures += bus.sync_read_feedback({id, 3}, out) != 2;
      failures += !bus.ping(id);
    }
  };
  int failures_1 = 0;
  int failures_2 = 0;
  std::thread a(loop, 1, std::ref(failures_1));
  std::thread b(loop, 2, std::ref(failures_2));
  a.join();
  b.join();
  EXPECT_EQ(failures_1 + failures_2, 0) << bus.last_error();
  EXPECT_EQ(servos.requests(), 800);
}

TEST(Bus, RefusesBadArgumentsAndAClosedPort) {
  feetech::Bus bus;
  EXPECT_FALSE(bus.ping(1));
  EXPECT_EQ(bus.last_error_code(), Error::kNotOpen);
  std::vector<std::uint8_t> data;
  EXPECT_FALSE(bus.read_registers(1, 0, 0, data));
  EXPECT_EQ(bus.last_error_code(), Error::kInvalidArgument);
  EXPECT_FALSE(bus.ping(feetech::kBroadcastId));
  EXPECT_EQ(bus.last_error_code(), Error::kInvalidArgument);
  EXPECT_FALSE(bus.sync_write_goals({{1, {}}, {1, {}}}));
  EXPECT_EQ(bus.last_error_code(), Error::kInvalidArgument);
  EXPECT_FALSE(bus.open("/nonexistent-tty", 12345));
  EXPECT_EQ(bus.last_error_code(), Error::kUnsupportedBaud);
  EXPECT_FALSE(bus.open("/nonexistent-tty", 1000000));
  EXPECT_EQ(bus.last_error_code(), Error::kIo);
  feetech::Bus moved = std::move(bus);
  EXPECT_FALSE(bus.is_open());
}

TEST(Bus, AHungUpPortIsDisconnected) {
  fake::Pty pty;
  feetech::Bus bus;
  ASSERT_TRUE(bus.open(pty.name, 1000000)) << bus.last_error();
  ::close(pty.master);
  pty.master = -1;
  EXPECT_FALSE(bus.ping(1));
  EXPECT_EQ(bus.last_error_code(), Error::kDisconnected) << bus.last_error();
}
