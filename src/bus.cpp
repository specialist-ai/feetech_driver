#include "feetech/bus.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iterator>
#include <mutex>
#include <utility>

namespace feetech {
namespace {

using Clock = std::chrono::steady_clock;
using Bytes = std::vector<std::uint8_t>;

constexpr int kDefaultTimeoutMs = 20;
// The packet limit of the Feetech SDK. The header, id, length and checksum take 6 bytes.
constexpr std::size_t kMaxPacketSize = 250;
constexpr std::size_t kMaxReadLength = kMaxPacketSize - 6;
constexpr std::size_t kMaxWriteLength = kMaxPacketSize - 7;
constexpr std::size_t kMaxSyncReadIds = kMaxPacketSize - 8;
constexpr std::size_t kMaxGoals = (kMaxPacketSize - 8) / (kGoalBlockSize + 1);
constexpr std::uint8_t kPing = 0x01, kRead = 0x02, kWrite = 0x03, kSyncRead = 0x82,
                       kSyncWrite = 0x83;

constexpr std::pair<int, speed_t> kBaudRates[] = {{1000000, B1000000}, {500000, B500000},
                                                  {115200, B115200},   {57600, B57600},
                                                  {38400, B38400},     {19200, B19200}};

std::uint8_t checksum(const std::uint8_t* bytes, std::size_t length) {
  unsigned sum = 0;
  for (std::size_t i = 0; i < length; ++i) sum += bytes[i];
  return static_cast<std::uint8_t>(~sum);
}

void begin_packet(Bytes& tx, std::uint8_t id, std::uint8_t instruction) {
  tx.assign({0xFF, 0xFF, id, 0, instruction});
}

void end_packet(Bytes& tx) {
  tx[3] = static_cast<std::uint8_t>(tx.size() - 3);
  tx.push_back(checksum(tx.data() + 2, tx.size() - 2));
}

// A byte is 10 bits on the wire: start, 8 data, stop.
Clock::duration wire_time(std::size_t bytes, int baud) {
  const auto rate = static_cast<std::size_t>(baud);
  return std::chrono::microseconds(static_cast<long>((bytes * 10'000'000 + rate - 1) / rate));
}

bool is_hangup(int error) { return error == EIO || error == ENXIO || error == ENODEV; }

std::string describe(const char* call, const std::string& port, int error) {
  return std::string(call) + "('" + port + "'): " + std::strerror(error);
}

}  // namespace

struct Bus::Impl {
  std::mutex mutex;  // held for a whole call; guards every field below
  int fd{-1};
  int baud{0};
  std::string port;
  int timeout{kDefaultTimeoutMs};
  Error code{Error::kNone};
  std::string message;
  std::uint8_t servo_error{0};
  Bytes tx;
  Bytes rx;  // received bytes that are not parsed yet
  Clock::time_point deadline;
  bool garbled{false};   // the call skipped bytes that were not a valid reply
  bool wrong_id{false};  // the call got a valid reply from an unexpected id
  std::uint8_t reply_id{0};
  std::uint8_t reply_error{0};
  Bytes reply_data;
  int os_error{0};  // the errno of the last kIo

  ~Impl() { close_port(); }

  // Reads one field under the mutex. A moved-from Bus has no Impl and gets `fallback`.
  template <typename T>
  static T get(const std::unique_ptr<Impl>& impl, T Impl::* field, T fallback) {
    if (!impl) return fallback;
    const std::lock_guard<std::mutex> lock(impl->mutex);
    return (*impl).*field;
  }

  std::unique_lock<std::mutex> start() {
    std::unique_lock<std::mutex> lock(mutex);
    code = Error::kNone;
    message.clear();
    servo_error = 0;
    return lock;
  }

  bool fail(Error error, std::string text) {
    code = error;
    message = std::move(text);
    return false;
  }

  void close_port() {
    if (fd < 0) return;
    ::tcflush(fd, TCIOFLUSH);  // so close() does not wait for the output
    ::close(fd);
    fd = -1;
  }

  bool valid_id(std::uint8_t id, bool broadcast) {
    if (id <= kMaxServoId || (broadcast && id == kBroadcastId)) return true;
    return fail(Error::kInvalidArgument, "Id " + std::to_string(id) + " is not a servo id.");
  }

  // Ids must be unique servo ids, and at most `limit` of them.
  template <typename Items, typename IdOf>
  bool valid_ids(const Items& items, IdOf id_of, std::size_t limit) {
    if (items.size() > limit) {
      return fail(Error::kInvalidArgument,
                  "One packet holds at most " + std::to_string(limit) + " servos.");
    }
    std::array<bool, 256> seen{};
    for (const auto& item : items) {
      const std::uint8_t id = id_of(item);
      if (!valid_id(id, false)) return false;
      if (std::exchange(seen[id], true)) {
        return fail(Error::kInvalidArgument, "Id " + std::to_string(id) + " appears twice.");
      }
    }
    return true;
  }

  Error io_error() {
    os_error = errno;
    return Error::kIo;
  }

  bool ready() { return fd >= 0 || fail(Error::kNotOpen, "The bus is not open."); }

  // Waits until the port is ready for `events`. It returns kNone, kTimeout, kDisconnected or kIo.
  Error wait(short events, Clock::time_point until) {
    for (;;) {
      const auto now = Clock::now();
      if (now >= until) return Error::kTimeout;
      pollfd pfd{fd, events, 0};
      const auto ms = std::chrono::ceil<std::chrono::milliseconds>(until - now).count();
      const int ready = ::poll(&pfd, 1, static_cast<int>(ms));
      if (ready < 0 && errno != EINTR) return io_error();
      if (ready <= 0) continue;
      if ((pfd.revents & events) != 0) return Error::kNone;
      if ((pfd.revents & POLLNVAL) == 0) return Error::kDisconnected;  // POLLHUP or POLLERR
      errno = EBADF;
      return io_error();
    }
  }

  // Sends tx. The deadline allows `reply_bytes` more on the wire.
  bool send(std::size_t reply_bytes) {
    deadline = Clock::now() + std::chrono::milliseconds(timeout) +
               wire_time(tx.size() + reply_bytes, baud);
    rx.clear();
    garbled = wrong_id = false;
    ::tcflush(fd, TCIFLUSH);  // bytes from before this call are not its reply
    for (std::size_t done = 0; done < tx.size();) {
      const ssize_t n = ::write(fd, tx.data() + done, tx.size() - done);
      const int error = errno;
      if (n > 0) {
        done += static_cast<std::size_t>(n);
      } else if (error == EAGAIN) {
        if (const Error result = wait(POLLOUT, deadline); result != Error::kNone) {
          return fail(result, "Writing to '" + port + "' failed.");
        }
      } else if (error != EINTR) {
        return fail(is_hangup(error) ? Error::kDisconnected : Error::kIo,
                    describe("write", port, error));
      }
    }
    return true;
  }

  // Reads the next status packet with `length` data bytes: FF FF ID LEN ERR DATA CHK, with
  // LEN = length + 2. It skips bytes that do not start such a packet and bad checksums.
  Error receive(std::size_t length) {
    const std::size_t size = length + 6;
    for (;;) {
      while (!rx.empty() &&
             !(rx[0] == 0xFF && (rx.size() < 2 || rx[1] == 0xFF) &&
               (rx.size() < 3 || rx[2] != 0xFF) && (rx.size() < 4 || rx[3] == length + 2))) {
        rx.erase(rx.begin());
        garbled = true;
      }
      if (rx.size() >= size) {
        if (checksum(&rx[2], size - 3) == rx[size - 1]) {
          reply_id = rx[2];
          reply_error = rx[4];
          reply_data.assign(rx.begin() + 5, rx.begin() + 5 + static_cast<long>(length));
          rx.erase(rx.begin(), rx.begin() + static_cast<long>(size));
          return Error::kNone;
        }
        rx.erase(rx.begin());
        garbled = true;
        continue;
      }
      if (const Error error = wait(POLLIN, deadline); error != Error::kNone) return error;
      std::uint8_t chunk[256];
      const ssize_t n = ::read(fd, chunk, sizeof chunk);
      if (n > 0) {
        rx.insert(rx.end(), chunk, chunk + n);
      } else if (n == 0 || is_hangup(errno)) {
        return Error::kDisconnected;
      } else if (errno != EAGAIN && errno != EINTR) {
        return io_error();
      }
    }
  }

  // Names why servo `id` sent no valid reply. Then it discards input until the line is quiet
  // for the timeout, so a late reply does not become the reply of the next call.
  bool receive_failed(Error error, std::uint8_t id) {
    if (error == Error::kDisconnected) {
      return fail(error, "'" + port + "' hung up. Close the bus and open it again.");
    }
    const std::string servo = "Servo " + std::to_string(id);
    if (error == Error::kIo) {
      fail(error, describe("read", port, os_error));
    } else if (wrong_id) {
      fail(Error::kWrongId, "A reply came from another id than " + std::to_string(id) + ".");
    } else if (garbled) {
      fail(Error::kMalformed, servo + " sent no valid reply.");
    } else {
      fail(Error::kTimeout, servo + " did not reply within " + std::to_string(timeout) + " ms.");
    }
    const auto quiet = std::chrono::milliseconds(timeout);
    const auto limit = Clock::now() + 3 * quiet;  // a line that never goes quiet stops it too
    std::uint8_t chunk[256];
    while (wait(POLLIN, std::min(Clock::now() + quiet, limit)) == Error::kNone &&
           ::read(fd, chunk, sizeof chunk) > 0) {
    }
    ::tcflush(fd, TCIFLUSH);
    return false;
  }

  // Sends tx and receives one reply with `length` data bytes from `id` or `alternate_id`.
  bool request(std::uint8_t id, std::uint8_t alternate_id, std::size_t length) {
    if (!send(length + 6)) return false;
    for (;;) {
      if (const Error error = receive(length); error != Error::kNone) {
        return receive_failed(error, id);
      }
      if (reply_id == id || reply_id == alternate_id) break;
      wrong_id = true;  // a late reply to an earlier call, or an echo
    }
    servo_error = reply_error;
    return true;
  }
};

Bus::Bus() : impl_(std::make_unique<Impl>()) {}
Bus::~Bus() = default;
Bus::Bus(Bus&& other) noexcept = default;
Bus& Bus::operator=(Bus&& other) noexcept = default;

bool Bus::open(const std::string& port, int baud) {
  if (!impl_) impl_ = std::make_unique<Impl>();
  Impl& d = *impl_;
  const auto lock = d.start();
  const auto rate = std::find_if(std::begin(kBaudRates), std::end(kBaudRates),
                                 [baud](const auto& entry) { return entry.first == baud; });
  if (rate == std::end(kBaudRates)) {
    return d.fail(Error::kUnsupportedBaud, "Baud " + std::to_string(baud) + " is not supported.");
  }
  if (port.empty()) return d.fail(Error::kInvalidArgument, "The port name is empty.");
  d.close_port();
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    return d.fail(errno == EBUSY ? Error::kPortBusy : Error::kIo, describe("open", port, errno));
  }
  const auto fail = [&](Error error, std::string text) {
    ::close(fd);
    return d.fail(error, std::move(text));
  };
  // Lock before the line settings change, so a holder keeps its baud.
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    if (errno != EWOULDBLOCK) return fail(Error::kIo, describe("flock", port, errno));
    return fail(Error::kPortBusy, "Another Bus or another process holds '" + port + "'.");
  }
  termios tty{};
  if (::tcgetattr(fd, &tty) != 0) return fail(Error::kIo, describe("tcgetattr", port, errno));
  ::cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | CRTSCTS);
  tty.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  ::cfsetispeed(&tty, rate->second);
  ::cfsetospeed(&tty, rate->second);
  if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
    return fail(Error::kIo, describe("tcsetattr", port, errno));
  }
  termios applied{};
  if (::tcgetattr(fd, &applied) != 0 || ::cfgetospeed(&applied) != rate->second) {
    return fail(Error::kUnsupportedBaud,
                "'" + port + "' did not take baud " + std::to_string(baud) + ".");
  }
  ::tcflush(fd, TCIOFLUSH);
  d.fd = fd;
  d.baud = baud;
  d.port = port;
  return true;
}

void Bus::close() {
  if (!impl_) return;
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->close_port();
}

bool Bus::ping(std::uint8_t id) {
  if (!impl_) return false;
  Impl& d = *impl_;
  const auto lock = d.start();
  if (!d.valid_id(id, false) || !d.ready()) return false;
  begin_packet(d.tx, id, kPing);
  end_packet(d.tx);
  return d.request(id, id, 0);
}

bool Bus::read_registers(std::uint8_t id, std::uint8_t address, std::uint8_t length,
                         std::vector<std::uint8_t>& out) {
  if (!impl_) return false;
  Impl& d = *impl_;
  const auto lock = d.start();
  if (!d.valid_id(id, false)) return false;
  if (length == 0 || length > kMaxReadLength) {
    return d.fail(Error::kInvalidArgument,
                  "A read takes 1 to " + std::to_string(kMaxReadLength) + " bytes.");
  }
  if (!d.ready()) return false;
  begin_packet(d.tx, id, kRead);
  d.tx.insert(d.tx.end(), {address, length});
  end_packet(d.tx);
  if (!d.request(id, id, length)) return false;
  out = d.reply_data;
  return true;
}

bool Bus::write_registers(std::uint8_t id, std::uint8_t address, const std::uint8_t* data,
                          std::size_t length) {
  if (!impl_) return false;
  Impl& d = *impl_;
  const auto lock = d.start();
  if (!d.valid_id(id, true)) return false;
  if (data == nullptr || length == 0 || length > kMaxWriteLength) {
    return d.fail(Error::kInvalidArgument,
                  "A write takes 1 to " + std::to_string(kMaxWriteLength) + " bytes.");
  }
  // The servo replies to a write of its id from the new id.
  const bool writes_id = address <= reg::kId && std::size_t{reg::kId} < address + length;
  const std::uint8_t reply_id = writes_id ? data[reg::kId - address] : id;
  if (!d.valid_id(reply_id, false) || !d.ready()) return false;
  begin_packet(d.tx, id, kWrite);
  d.tx.push_back(address);
  d.tx.insert(d.tx.end(), data, data + length);
  end_packet(d.tx);
  return id == kBroadcastId ? d.send(0) : d.request(id, reply_id, 0);
}

bool Bus::write_registers(std::uint8_t id, std::uint8_t address,
                          const std::vector<std::uint8_t>& data) {
  return write_registers(id, address, data.data(), data.size());
}

std::size_t Bus::sync_read_feedback(const std::vector<std::uint8_t>& ids,
                                    std::vector<std::optional<Feedback>>& out) {
  out.assign(ids.size(), std::nullopt);
  if (!impl_) return 0;
  Impl& d = *impl_;
  const auto lock = d.start();
  const auto id_of = [](std::uint8_t id) { return id; };
  if (ids.empty() || !d.valid_ids(ids, id_of, kMaxSyncReadIds) || !d.ready()) return 0;
  begin_packet(d.tx, kBroadcastId, kSyncRead);
  d.tx.insert(d.tx.end(), {reg::kPresentPosition, static_cast<std::uint8_t>(kFeedbackBlockSize)});
  d.tx.insert(d.tx.end(), ids.begin(), ids.end());
  end_packet(d.tx);
  if (!d.send(ids.size() * (kFeedbackBlockSize + 6))) return 0;
  // Servos reply in the order of `ids`. A silent servo leaves a gap.
  std::size_t next = 0;
  std::size_t answered = 0;
  Error error = Error::kNone;
  while (next < ids.size() && (error = d.receive(kFeedbackBlockSize)) == Error::kNone) {
    const auto slot = static_cast<std::size_t>(
        std::find(ids.begin() + static_cast<long>(next), ids.end(), d.reply_id) - ids.begin());
    if (slot == ids.size()) {
      d.wrong_id = true;
      continue;
    }
    next = slot + 1;
    d.servo_error |= d.reply_error;
    out[slot] = decode_feedback(d.reply_data.data(), d.reply_data.size());
    ++answered;
  }
  if (answered < ids.size()) {
    const auto missing = std::find(out.begin(), out.end(), std::nullopt) - out.begin();
    d.receive_failed(error == Error::kNone ? Error::kTimeout : error, ids[missing]);
  }
  return answered;
}

bool Bus::sync_write_goals(const std::vector<GoalEntry>& entries) {
  if (!impl_) return false;
  Impl& d = *impl_;
  const auto lock = d.start();
  const auto id_of = [](const GoalEntry& entry) { return entry.id; };
  if (entries.empty()) return true;
  if (!d.valid_ids(entries, id_of, kMaxGoals) || !d.ready()) return false;
  begin_packet(d.tx, kBroadcastId, kSyncWrite);
  d.tx.insert(d.tx.end(), {reg::kAcceleration, static_cast<std::uint8_t>(kGoalBlockSize)});
  for (const auto& entry : entries) {
    d.tx.push_back(entry.id);
    d.tx.insert(d.tx.end(), entry.block.begin(), entry.block.end());
  }
  end_packet(d.tx);
  return d.send(0);
}

std::string Bus::last_error() const {
  return Impl::get(impl_, &Impl::message, std::string("The Bus was moved from."));
}
Error Bus::last_error_code() const { return Impl::get(impl_, &Impl::code, Error::kNotOpen); }
std::uint8_t Bus::last_servo_error() const {
  return Impl::get(impl_, &Impl::servo_error, std::uint8_t{0});
}
int Bus::timeout_ms() const { return Impl::get(impl_, &Impl::timeout, kDefaultTimeoutMs); }
bool Bus::is_open() const { return Impl::get(impl_, &Impl::fd, -1) >= 0; }

void Bus::set_timeout_ms(int timeout_ms) {
  if (!impl_) return;
  const auto lock = impl_->start();
  if (timeout_ms > 0) {
    impl_->timeout = timeout_ms;
  } else {
    impl_->fail(Error::kInvalidArgument, "The timeout must be at least 1 ms.");
  }
}

}  // namespace feetech
