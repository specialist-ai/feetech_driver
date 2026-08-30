#include <gtest/gtest.h>

#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <tuple>

#include "feetech/protocol.hpp"

using feetech::decode_sign_magnitude;
using feetech::encode_sign_magnitude;

TEST(Protocol, SignMagnitudeEncodesAndSaturates) {
  EXPECT_EQ(encode_sign_magnitude(-100, 15), 0x8064);
  EXPECT_EQ(encode_sign_magnitude(100, 15), 0x0064);
  EXPECT_EQ(encode_sign_magnitude(40000, 15), 0x7FFF);
  EXPECT_EQ(encode_sign_magnitude(INT_MIN, 15), 0xFFFF);
  EXPECT_EQ(decode_sign_magnitude(0x8064, 15), -100);
  EXPECT_EQ(decode_sign_magnitude(0x04C8, feetech::kSignBitLoad), -200);
  for (int value = -2047; value <= 2047; ++value) {
    EXPECT_EQ(decode_sign_magnitude(encode_sign_magnitude(value, 15), 15), value);
  }
  std::array<std::uint8_t, 2> bytes{};
  feetech::write_le_u16(bytes.data(), 2048);
  EXPECT_EQ(bytes, (std::array<std::uint8_t, 2>{0x00, 0x08}));
  EXPECT_EQ(feetech::read_le_u16(bytes.data()), 2048);
}

TEST(Protocol, GoalBlockHoldsTheSaturatedPosition) {
  EXPECT_EQ(feetech::encode_goal_block(-1000),
            (std::array<std::uint8_t, feetech::kGoalBlockSize>{0, 0xE8, 0x83, 0, 0, 0, 0}));
  EXPECT_EQ(feetech::read_le_u16(&feetech::encode_goal_block(40000)[1]), 0x7FFF);
}

TEST(Protocol, DecodeFeedbackReadsEveryField) {
  // Position 1000, velocity -50, load 200, 12.2 V, 27 C, moving, current 100 counts.
  const std::array<std::uint8_t, feetech::kFeedbackBlockSize> block{
      0xE8, 0x03, 0x32, 0x80, 0xC8, 0x00, 0x7A, 0x1B, 0x00, 0x05, 0x01, 0x00, 0x00, 0x64, 0x00};
  const auto feedback = feetech::decode_feedback(block.data(), block.size());
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(std::tie(feedback->position_counts, feedback->velocity_counts_per_sec,
                     feedback->load_permille, feedback->temperature_celsius, feedback->status,
                     feedback->moving),
            std::make_tuple(1000, -50, 200, 27, 5, true));
  EXPECT_DOUBLE_EQ(feedback->voltage_volts, 12.2);
  EXPECT_NEAR(feedback->current_amps, 0.65, 1e-9);
  EXPECT_FALSE(feetech::decode_feedback(block.data(), 4).has_value());
  EXPECT_DOUBLE_EQ(feetech::kRadiansPerCount * feetech::kCountsPerTurn, 2 * M_PI);
}
