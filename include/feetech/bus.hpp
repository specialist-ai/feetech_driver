// Half-duplex serial transport for a Feetech SMS/STS servo bus.
//
// Each call is one blocking transaction with a timeout, so keep a Bus off a
// control loop. A 15-byte SYNC_READ takes about 1 ms, and the USB overhead
// comes once per transaction for any number of servos.
//
// One tty is one bus in a process: every Bus that names the same device (by
// its resolved path) shares one fd and one transaction mutex, so two threads
// take turns by transaction. An advisory flock refuses a second process. The
// adapter must not echo TX onto RX: each request first flushes the input.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "feetech/protocol.hpp"

namespace feetech
{

/// The processes that hold `port` open, as "comm (pid N)". Processes of
/// another user are invisible without privilege.
std::vector<std::string> port_holders(const std::string & port);

class Bus
{
public:
  Bus() = default;
  ~Bus();

  Bus(const Bus &) = delete;
  Bus & operator=(const Bus &) = delete;

  /// Open the tty in raw 8N1 mode. `baud` is one of 1000000, 500000, 115200,
  /// 57600, 38400 or 19200.
  bool open(const std::string & port, int baud);
  void close();
  bool is_open() const {return fd() >= 0;}

  bool ping(std::uint8_t id);
  bool read_registers(
    std::uint8_t id, std::uint8_t address, std::uint8_t length, std::vector<std::uint8_t> & out);
  /// An EEPROM register keeps the value only while the Lock register is 0.
  bool write_registers(
    std::uint8_t id, std::uint8_t address, const std::uint8_t * data, std::size_t length);

  /// Read the feedback block of each id in one SYNC_READ. `out` gets one entry
  /// for each id, nullopt where no valid reply came. Return how many replied.
  std::size_t sync_read_feedback(
    const std::vector<std::uint8_t> & ids, std::vector<std::optional<Feedback>> & out);
  /// Broadcast the goal blocks in one SYNC_WRITE. No servo replies.
  bool sync_write_goals(const std::vector<GoalEntry> & entries);

  /// The reason the last call returned false.
  const std::string & last_error() const {return last_error_;}
  void set_timeout_ms(int timeout_ms) {timeout_ms_ = timeout_ms;}

private:
  struct Port
  {
    int fd{-1};
    int baud{0};
    std::mutex transaction;  // held for a request and its reply
    ~Port();
  };

  static std::shared_ptr<Port> acquire(const std::string & port, int baud, std::string & error);
  int fd() const {return port_ ? port_->fd : -1;}
  bool send(const std::vector<std::uint8_t> & packet);
  /// Send tx_ and receive the status reply of `id`, with `data_length` bytes of data.
  bool request(std::uint8_t id, std::size_t data_length, std::vector<std::uint8_t> * data);
  bool receive_exact(std::uint8_t * buffer, std::size_t count);
  std::optional<StatusPacket> receive_status(std::size_t data_length);

  std::shared_ptr<Port> port_;
  int timeout_ms_{20};
  std::string last_error_;
  std::vector<std::uint8_t> tx_;  // reused, so a steady loop does not allocate
  std::vector<std::uint8_t> rx_;
};

}  // namespace feetech
