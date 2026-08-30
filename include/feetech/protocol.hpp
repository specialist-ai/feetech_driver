// Feetech SMS/STS register map and data encoding. 16-bit registers are little-endian, and
// signed values are sign-magnitude. This header needs C++17.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace feetech {

constexpr std::uint8_t kBroadcastId = 0xFE, kMaxServoId = 0xFD;
constexpr int kCountsPerTurn = 4096;
constexpr double kRadiansPerCount = 2 * 3.141592653589793238462643383279502884 / kCountsPerTurn;
constexpr int kMaxTorqueLimit = 1000;      // Torque_Limit is in permille of the stall torque.
constexpr std::uint8_t kPositionMode = 0;  // an Operating_Mode value

// The STS/SMS control table, as in FTServo SMS_STS.h. A /* 2 */ marks a 2-byte register.
namespace reg {
// EEPROM: a write needs Lock 0 and wears the flash.
constexpr std::uint8_t kFirmwareMajor = 0, kFirmwareMinor = 1, kModel = 3 /* 2 */, kId = 5,
                       kBaudRate = 6, kReturnDelay = 7, kResponseStatusLevel = 8,
                       kMinAngleLimit = 9 /* 2 */, kMaxAngleLimit = 11 /* 2 */,
                       kMaxTemperatureLimit = 13, kMaxInputVoltage = 14, kMinInputVoltage = 15,
                       kMaxTorque = 16 /* 2 */, kPhase = 18, kUnloadingCondition = 19,
                       kLedAlarmCondition = 20, kPCoefficient = 21, kDCoefficient = 22,
                       kICoefficient = 23, kMinimumStartupForce = 24 /* 2 */, kCwDeadZone = 26,
                       kCcwDeadZone = 27, kProtectionCurrent = 28 /* 2 */, kAngularResolution = 30,
                       kHomingOffset = 31 /* 2, sign bit 11 */, kOperatingMode = 33,
                       kProtectiveTorque = 34, kProtectionTime = 35, kOverloadTorque = 36,
                       kVelocityPCoefficient = 37, kOverCurrentProtectionTime = 38,
                       kVelocityICoefficient = 39;
// RAM: a power cycle resets it.
constexpr std::uint8_t kTorqueEnable = 40, kAcceleration = 41, kGoalPosition = 42 /* 2 */,
                       kGoalTime = 44 /* 2 */, kGoalSpeed = 46 /* 2 */, kTorqueLimit = 48 /* 2 */,
                       kLock = 55, kPresentPosition = 56 /* 2 */, kPresentSpeed = 58 /* 2 */,
                       kPresentLoad = 60 /* 2, sign bit 10 */, kPresentVoltage = 62,
                       kPresentTemperature = 63, kStatus = 65, kMoving = 66,
                       kPresentCurrent = 69 /* 2 */;
}  // namespace reg

// Bits of the ERR byte of a status reply and of the Status register.
namespace servo_error {
constexpr std::uint8_t kVoltage = 0x01, kSensor = 0x02, kTemperature = 0x04, kCurrent = 0x08,
                       kOverload = 0x20;
}  // namespace servo_error

constexpr int kSignBitPosition = 15, kSignBitVelocity = 15, kSignBitLoad = 10, kSignBitCurrent = 15;
constexpr std::size_t kGoalBlockSize = 7;       // registers 41..47
constexpr std::size_t kFeedbackBlockSize = 15;  // registers 56..70

constexpr int decode_sign_magnitude(std::uint16_t raw, int sign_bit) {
  const int magnitude = raw & ((1 << sign_bit) - 1);
  return (raw & (1 << sign_bit)) ? -magnitude : magnitude;
}

// The magnitude saturates at (1 << sign_bit) - 1.
constexpr std::uint16_t encode_sign_magnitude(int value, int sign_bit) {
  const std::int64_t wide = value;
  const std::int64_t magnitude =
      std::min(wide < 0 ? -wide : wide, (std::int64_t{1} << sign_bit) - 1);
  return static_cast<std::uint16_t>(wide < 0 ? magnitude | (std::int64_t{1} << sign_bit)
                                             : magnitude);
}

constexpr std::uint16_t read_le_u16(const std::uint8_t* src) {
  return static_cast<std::uint16_t>(src[0] | (src[1] << 8));
}

constexpr void write_le_u16(std::uint8_t* dst, std::uint16_t value) {
  dst[0] = static_cast<std::uint8_t>(value & 0xFF);
  dst[1] = static_cast<std::uint8_t>(value >> 8);
}

struct GoalEntry {
  std::uint8_t id{0};
  std::array<std::uint8_t, kGoalBlockSize> block{};
};

// Acceleration, time and speed are 0, so the servo goes to the position at full speed.
inline std::array<std::uint8_t, kGoalBlockSize> encode_goal_block(int position_counts) {
  std::array<std::uint8_t, kGoalBlockSize> block{};
  write_le_u16(&block[1], encode_sign_magnitude(position_counts, kSignBitPosition));
  return block;
}

struct Feedback {
  int position_counts{0};
  int velocity_counts_per_sec{0};
  int load_permille{0};  // a signed PWM duty, not a torque
  double voltage_volts{0.0};
  int temperature_celsius{0};
  std::uint8_t status{0};  // servo_error bits
  bool moving{false};
  double current_amps{0.0};
};

inline std::optional<Feedback> decode_feedback(const std::uint8_t* bytes, std::size_t length) {
  if (length < kFeedbackBlockSize) return std::nullopt;
  const auto signed_at = [bytes](std::size_t offset, int sign_bit) {
    return decode_sign_magnitude(read_le_u16(bytes + offset), sign_bit);
  };
  return Feedback{signed_at(0, kSignBitPosition),
                  signed_at(2, kSignBitVelocity),
                  signed_at(4, kSignBitLoad),
                  bytes[6] / 10.0,
                  bytes[7],
                  bytes[9],
                  bytes[10] != 0,
                  signed_at(13, kSignBitCurrent) * 0.0065};
}

}  // namespace feetech
