// A half-duplex serial bus of Feetech SMS/STS servos (Linux). The adapter must not echo TX.
// Threads: every member is safe from any thread. A mutex runs the calls one at a time.
// Processes: open() takes a flock, so only one Bus in all processes holds a port.
// Errors: a failed call returns false or a short count and sets last_error() and
// last_error_code(). A success clears them. The ERR byte of the replies goes to
// last_servo_error(). A call waits timeout_ms() plus the wire time of its packets.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "feetech/protocol.hpp"

namespace feetech {

enum class Error : std::uint8_t {
  kNone,
  kNotOpen,
  kInvalidArgument,  // nothing was sent
  kUnsupportedBaud,
  kPortBusy,  // another Bus, in this or another process, holds the port
  kIo,
  kDisconnected,  // the adapter was unplugged; close() and open() again
  kTimeout,
  kMalformed,  // bytes arrived but no valid reply
  kWrongId,
};

class Bus {
 public:
  Bus();
  ~Bus();
  Bus(Bus&& other) noexcept;
  Bus& operator=(Bus&& other) noexcept;

  /// `baud`: 1000000, 500000, 115200, 57600, 38400 or 19200.
  [[nodiscard]] bool open(const std::string& port, int baud);
  void close();
  bool is_open() const;

  [[nodiscard]] bool ping(std::uint8_t id);
  [[nodiscard]] bool read_registers(std::uint8_t id, std::uint8_t address, std::uint8_t length,
                                    std::vector<std::uint8_t>& out);
  /// kBroadcastId gets no reply. A write of reg::kId accepts the reply from the new id.
  [[nodiscard]] bool write_registers(std::uint8_t id, std::uint8_t address,
                                     const std::uint8_t* data, std::size_t length);
  [[nodiscard]] bool write_registers(std::uint8_t id, std::uint8_t address,
                                     const std::vector<std::uint8_t>& data);
  /// One SYNC_READ of registers 56..70. Returns how many ids replied; `out` holds nullopt
  /// for the others.
  std::size_t sync_read_feedback(const std::vector<std::uint8_t>& ids,
                                 std::vector<std::optional<Feedback>>& out);
  /// One SYNC_WRITE of registers 41..47. No servo replies, so true means sent.
  [[nodiscard]] bool sync_write_goals(const std::vector<GoalEntry>& entries);

  std::string last_error() const;
  Error last_error_code() const;
  std::uint8_t last_servo_error() const;

  /// 20 ms by default. A value of 0 or less is ignored and sets kInvalidArgument.
  void set_timeout_ms(int timeout_ms);
  int timeout_ms() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace feetech
