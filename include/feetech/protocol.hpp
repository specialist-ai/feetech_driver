// Feetech SMS/STS wire protocol: packets, the control table and the feedback block.
//
// The framing is Dynamixel Protocol 1.0 (FF FF ID LEN INST PARAMS CHK, with
// CHK = ~sum(ID..PARAMS)). SMS/STS differs in two details, and each one
// corrupts data without an error: 16-bit fields are little-endian, and signed
// values are sign-magnitude. The addresses agree with FTServo_Linux SMS_STS.h
// and LeRobot feetech/tables.py, and an STS3215 (model 777, firmware 3.10)
// read them back.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <vector>

namespace feetech
{

constexpr std::uint8_t kBroadcastId = 0xFE;
constexpr std::uint8_t kMaxServoId = 0xFD;
constexpr int kSts3215Model = 777;
constexpr int kCountsPerTurn = 4096;
constexpr double kRadiansPerCount = 2 * std::numbers::pi / kCountsPerTurn;
// Torque_Limit is in permille of the stall torque.
constexpr int kMaxTorqueLimit = 1000;

namespace instruction
{
constexpr std::uint8_t kPing = 0x01;
constexpr std::uint8_t kRead = 0x02;
constexpr std::uint8_t kWrite = 0x03;
constexpr std::uint8_t kSyncRead = 0x82;
constexpr std::uint8_t kSyncWrite = 0x83;
}  // namespace instruction

namespace reg
{
// EEPROM. A write takes effect only while Lock is 0, and each write wears the flash.
constexpr std::uint8_t kModel = 3;
constexpr std::uint8_t kId = 5;
constexpr std::uint8_t kPCoefficient = 21;
constexpr std::uint8_t kDCoefficient = 22;
constexpr std::uint8_t kICoefficient = 23;
constexpr std::uint8_t kMinimumStartupForce = 24;
constexpr std::uint8_t kCwDeadZone = 26;
constexpr std::uint8_t kCcwDeadZone = 27;
constexpr std::uint8_t kOperatingMode = 33;
// RAM. A power cycle resets it.
constexpr std::uint8_t kTorqueEnable = 40;
constexpr std::uint8_t kAcceleration = 41;     // first register of the goal block
constexpr std::uint8_t kTorqueLimit = 48;
constexpr std::uint8_t kLock = 55;
constexpr std::uint8_t kPresentPosition = 56;  // first register of the feedback block
}  // namespace reg

constexpr std::uint8_t kPositionMode = 0;  // Operating_Mode

// The sign bit of each sign-magnitude register.
constexpr int kSignBitPosition = 15;
constexpr int kSignBitVelocity = 15;
constexpr int kSignBitLoad = 10;

constexpr std::size_t kGoalBlockSize = 7;       // registers 41..47
constexpr std::size_t kFeedbackBlockSize = 15;  // registers 56..70

constexpr int decode_sign_magnitude(std::uint16_t raw, int sign_bit)
{
  const int magnitude = raw & ((1 << sign_bit) - 1);
  return (raw & (1 << sign_bit)) ? -magnitude : magnitude;
}

constexpr std::uint16_t encode_sign_magnitude(int value, int sign_bit)
{
  const int magnitude = (value < 0 ? -value : value) & ((1 << sign_bit) - 1);
  return static_cast<std::uint16_t>(value < 0 ? magnitude | (1 << sign_bit) : magnitude);
}

constexpr std::uint16_t read_le_u16(const std::uint8_t * src)
{
  return static_cast<std::uint16_t>(src[0] | (src[1] << 8));
}

constexpr void write_le_u16(std::uint8_t * dst, std::uint16_t value)
{
  dst[0] = static_cast<std::uint8_t>(value & 0xFF);
  dst[1] = static_cast<std::uint8_t>(value >> 8);
}

// The bitwise NOT of the byte sum between the FF FF header and the checksum.
inline std::uint8_t checksum(const std::uint8_t * body, std::size_t length)
{
  unsigned sum = 0;
  for (std::size_t i = 0; i < length; ++i) {
    sum += body[i];
  }
  return static_cast<std::uint8_t>(~sum);
}

// --- Packet builders. Each one clears `out` first, so a loop can reuse one buffer.

inline void begin_packet(std::vector<std::uint8_t> & out, std::uint8_t id, std::uint8_t inst)
{
  out.assign({0xFF, 0xFF, id, 0, inst});  // LEN is set by end_packet
}

inline void end_packet(std::vector<std::uint8_t> & out)
{
  out[3] = static_cast<std::uint8_t>(out.size() - 3);  // INST + params + CHK
  out.push_back(checksum(out.data() + 2, out.size() - 2));
}

inline void build_ping(std::vector<std::uint8_t> & out, std::uint8_t id)
{
  begin_packet(out, id, instruction::kPing);
  end_packet(out);
}

inline void build_read(
  std::vector<std::uint8_t> & out, std::uint8_t id, std::uint8_t address, std::uint8_t length)
{
  begin_packet(out, id, instruction::kRead);
  out.insert(out.end(), {address, length});
  end_packet(out);
}

inline void build_write(
  std::vector<std::uint8_t> & out, std::uint8_t id, std::uint8_t address,
  const std::uint8_t * data, std::size_t length)
{
  begin_packet(out, id, instruction::kWrite);
  out.push_back(address);
  out.insert(out.end(), data, data + length);
  end_packet(out);
}

// Each servo replies in the order of `ids`. The older SCS series has no SYNC_READ.
inline void build_sync_read(
  std::vector<std::uint8_t> & out, std::uint8_t address, std::uint8_t length,
  const std::vector<std::uint8_t> & ids)
{
  begin_packet(out, kBroadcastId, instruction::kSyncRead);
  out.insert(out.end(), {address, length});
  out.insert(out.end(), ids.begin(), ids.end());
  end_packet(out);
}

struct GoalEntry
{
  std::uint8_t id;
  std::array<std::uint8_t, kGoalBlockSize> block;
};

// A broadcast gets no reply, so it costs about 1/8 of an addressed WRITE, even for one servo.
inline void build_sync_write_goals(
  std::vector<std::uint8_t> & out, const std::vector<GoalEntry> & entries)
{
  begin_packet(out, kBroadcastId, instruction::kSyncWrite);
  out.insert(out.end(), {reg::kAcceleration, static_cast<std::uint8_t>(kGoalBlockSize)});
  for (const auto & entry : entries) {
    out.push_back(entry.id);
    out.insert(out.end(), entry.block.begin(), entry.block.end());
  }
  end_packet(out);
}

struct StatusPacket
{
  std::uint8_t id;
  std::vector<std::uint8_t> data;
};

// Parse FF FF ID LEN ERR DATA... CHK. Return nullopt for a short buffer or a bad checksum.
inline std::optional<StatusPacket> parse_status(const std::uint8_t * bytes, std::size_t length)
{
  if (length < 6 || bytes[0] != 0xFF || bytes[1] != 0xFF) {return std::nullopt;}
  const std::size_t packet_length = bytes[3];
  if (packet_length < 2 || length < packet_length + 4) {return std::nullopt;}
  const std::size_t end = packet_length + 3;  // the checksum index
  if (checksum(bytes + 2, end - 2) != bytes[end]) {return std::nullopt;}
  return StatusPacket{bytes[2], {bytes + 5, bytes + end}};
}

constexpr std::size_t status_packet_size(std::size_t data_length)
{
  return data_length + 6;
}

// Registers 41..47 with the motion profile off (0): a position stream at the
// control rate needs no profile in the servo.
inline std::array<std::uint8_t, kGoalBlockSize> encode_goal_block(int position_counts)
{
  std::array<std::uint8_t, kGoalBlockSize> block{};
  write_le_u16(&block[1], encode_sign_magnitude(position_counts, kSignBitPosition));
  return block;
}

struct Feedback
{
  int position_counts;
  int velocity_counts_per_sec;
  int load_permille;  // a signed PWM duty, not a torque
  double voltage_volts;
  int temperature_celsius;
  std::uint8_t status;
  bool moving;
  double current_amps;
};

// Decode registers 56..70.
inline std::optional<Feedback> decode_feedback(const std::uint8_t * bytes, std::size_t length)
{
  if (length < kFeedbackBlockSize) {return std::nullopt;}
  return Feedback{
    decode_sign_magnitude(read_le_u16(bytes + 0), kSignBitPosition),
    decode_sign_magnitude(read_le_u16(bytes + 2), kSignBitVelocity),
    decode_sign_magnitude(read_le_u16(bytes + 4), kSignBitLoad),
    bytes[6] / 10.0,
    bytes[7],
    bytes[9],
    bytes[10] != 0,
    read_le_u16(bytes + 13) * 0.0065,  // 6.5 mA a count
  };
}

}  // namespace feetech
