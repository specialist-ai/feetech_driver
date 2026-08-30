// The golden byte vectors come from the Feetech manual, and an STS3215
// (model 777, firmware 3.10) answered them on the bench.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "feetech/protocol.hpp"

// --- Sign-magnitude codec --------------------------------------------------

TEST(FeetechProtocol, SignMagnitudeIsNotTwosComplement)
{
  // -100 as position: magnitude in the low bits, bit 15 as the sign. Two's
  // complement would give 0xFF9C, which the servo reads as a huge positive.
  EXPECT_EQ(feetech::encode_sign_magnitude(-100, feetech::kSignBitPosition), 0x8064);
  EXPECT_EQ(feetech::encode_sign_magnitude(100, feetech::kSignBitPosition), 0x0064);
  EXPECT_EQ(feetech::decode_sign_magnitude(0x8064, feetech::kSignBitPosition), -100);
  EXPECT_EQ(feetech::decode_sign_magnitude(0x0064, feetech::kSignBitPosition), 100);
}

TEST(FeetechProtocol, SignMagnitudeRoundTripsAcrossZero)
{
  for (int value = -2047; value <= 2047; ++value) {
    const auto raw = feetech::encode_sign_magnitude(value, feetech::kSignBitPosition);
    EXPECT_EQ(feetech::decode_sign_magnitude(raw, feetech::kSignBitPosition), value)
      << "failed at " << value;
  }
}

TEST(FeetechProtocol, LoadUsesBitTenAsSign)
{
  // Present_Load is permille with the sign at bit 10, not bit 15.
  EXPECT_EQ(feetech::decode_sign_magnitude(0x00C8, feetech::kSignBitLoad), 200);
  EXPECT_EQ(feetech::decode_sign_magnitude(0x04C8, feetech::kSignBitLoad), -200);
}

// --- Byte order ------------------------------------------------------------

TEST(FeetechProtocol, SixteenBitFieldsAreLittleEndian)
{
  // The SCS/SCSCL series is big-endian; SMS/STS is not. Getting this backwards
  // turns 2048 counts into 8 counts.
  std::array<std::uint8_t, 2> bytes{};
  feetech::write_le_u16(bytes.data(), 2048);
  EXPECT_EQ(bytes[0], 0x00);
  EXPECT_EQ(bytes[1], 0x08);
  EXPECT_EQ(feetech::read_le_u16(bytes.data()), 2048);
}

// --- Packet builders -------------------------------------------------------

TEST(FeetechProtocol, BuildPingMatchesGoldenBytes)
{
  std::vector<std::uint8_t> packet;
  feetech::build_ping(packet, 1);
  const std::vector<std::uint8_t> expected{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  EXPECT_EQ(packet, expected);
}

TEST(FeetechProtocol, BuildReadFeedbackBlockMatchesGoldenBytes)
{
  // READ 15 bytes from Present_Position (56) on id 1 -- the transaction the
  // bench probe used.
  std::vector<std::uint8_t> packet;
  feetech::build_read(packet, 1, feetech::reg::kPresentPosition, feetech::kFeedbackBlockSize);
  const std::vector<std::uint8_t> expected{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x0F, 0xB1};
  EXPECT_EQ(packet, expected);
}

TEST(FeetechProtocol, BuildSyncReadMatchesGoldenBytes)
{
  std::vector<std::uint8_t> packet;
  feetech::build_sync_read(
    packet, feetech::reg::kPresentPosition, feetech::kFeedbackBlockSize, {1, 2});
  const std::vector<std::uint8_t> expected{
    0xFF, 0xFF, 0xFE, 0x06, 0x82, 0x38, 0x0F, 0x01, 0x02, 0x2F};
  EXPECT_EQ(packet, expected);
}

TEST(FeetechProtocol, BuildSyncWriteGoalsMatchesGoldenBytes)
{
  std::vector<feetech::GoalEntry> entries{{1, feetech::encode_goal_block(2048)}};
  std::vector<std::uint8_t> packet;
  feetech::build_sync_write_goals(packet, entries);
  const std::vector<std::uint8_t> expected{
    0xFF, 0xFF, 0xFE, 0x0C, 0x83, 0x29, 0x07, 0x01,
    0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x39};
  EXPECT_EQ(packet, expected);
}

TEST(FeetechProtocol, SyncWriteGrowsByNineBytesPerServo)
{
  // One id byte plus the 7-byte goal block. Confirms the batching claim the
  // I/O-thread design rests on: a second servo costs wire time, not a second
  // transaction.
  std::vector<std::uint8_t> one;
  std::vector<std::uint8_t> two;
  feetech::build_sync_write_goals(one, {{1, feetech::encode_goal_block(0)}});
  feetech::build_sync_write_goals(
    two, {{1, feetech::encode_goal_block(0)}, {2, feetech::encode_goal_block(0)}});
  EXPECT_EQ(two.size(), one.size() + feetech::kGoalBlockSize + 1);
}

TEST(FeetechProtocol, BuilderReusesTheCallerBuffer)
{
  // Every builder clear()s first, so a control loop can pass one buffer back
  // and never reallocate.
  std::vector<std::uint8_t> packet;
  feetech::build_ping(packet, 1);
  const std::size_t ping_size = packet.size();
  feetech::build_ping(packet, 2);
  EXPECT_EQ(packet.size(), ping_size);
  EXPECT_EQ(packet[2], 0x02);
}

// --- Goal block ------------------------------------------------------------

TEST(FeetechProtocol, GoalBlockHoldsThePositionWithTheProfileOff)
{
  const auto block = feetech::encode_goal_block(-1000);
  const std::array<std::uint8_t, feetech::kGoalBlockSize> expected{0, 0xE8, 0x83, 0, 0, 0, 0};
  EXPECT_EQ(block, expected);  // acceleration, position (sign-magnitude), time, velocity
}

// --- Status parsing --------------------------------------------------------

TEST(FeetechProtocol, ParseStatusAcceptsAValidPingReply)
{
  const std::vector<std::uint8_t> reply{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  auto status = feetech::parse_status(reply.data(), reply.size());
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->id, 1);
  EXPECT_TRUE(status->data.empty());
}

TEST(FeetechProtocol, ParseStatusRejectsABadChecksum)
{
  const std::vector<std::uint8_t> reply{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFD};
  EXPECT_FALSE(feetech::parse_status(reply.data(), reply.size()).has_value());
}

TEST(FeetechProtocol, ParseStatusRejectsAShortBuffer)
{
  const std::vector<std::uint8_t> reply{0xFF, 0xFF, 0x01, 0x11, 0x00};
  EXPECT_FALSE(feetech::parse_status(reply.data(), reply.size()).has_value());
}

TEST(FeetechProtocol, StatusPacketSizeMatchesTheFeedbackReply)
{
  // FF FF ID LEN ERR <15 data> CHK
  EXPECT_EQ(feetech::status_packet_size(feetech::kFeedbackBlockSize), 21u);
  EXPECT_EQ(feetech::status_packet_size(0), 6u);
}

// --- Feedback decoding -----------------------------------------------------

TEST(FeetechProtocol, DecodeFeedbackReadsEveryField)
{
  // regs 56..70: pos=1000, vel=-50, load=200, 12.2 V, 27 C, status 0, moving,
  // current 100 ticks.
  const std::array<std::uint8_t, feetech::kFeedbackBlockSize> block{
    0xE8, 0x03,   // Present_Position  = 1000
    0x32, 0x80,   // Present_Velocity  = -50 (sign-magnitude)
    0xC8, 0x00,   // Present_Load      = 200
    0x7A,         // Present_Voltage   = 122 -> 12.2 V
    0x1B,         // Present_Temperature = 27 C
    0x00,         // reg 64
    0x00,         // Status (65)
    0x01,         // Moving (66)
    0x00, 0x00,   // regs 67, 68
    0x64, 0x00};  // Present_Current   = 100 ticks

  auto feedback = feetech::decode_feedback(block.data(), block.size());
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(feedback->position_counts, 1000);
  EXPECT_EQ(feedback->velocity_counts_per_sec, -50);
  EXPECT_EQ(feedback->load_permille, 200);
  EXPECT_DOUBLE_EQ(feedback->voltage_volts, 12.2);
  EXPECT_EQ(feedback->temperature_celsius, 27);
  EXPECT_EQ(feedback->status, 0);
  EXPECT_TRUE(feedback->moving);
  EXPECT_NEAR(feedback->current_amps, 0.65, 1e-9);
}

TEST(FeetechProtocol, DecodeFeedbackRejectsAShortBlock)
{
  const std::array<std::uint8_t, 4> block{0xE8, 0x03, 0x00, 0x00};
  EXPECT_FALSE(feetech::decode_feedback(block.data(), block.size()).has_value());
}
