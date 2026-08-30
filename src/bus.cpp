#include "feetech/bus.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace feetech
{

namespace
{

// The STS table also has 250000 and 128000, which need termios2 and no hardware here uses.
speed_t baud_constant(int baud)
{
  switch (baud) {
    case 1000000: return B1000000;
    case 500000: return B500000;
    case 115200: return B115200;
    case 57600: return B57600;
    case 38400: return B38400;
    case 19200: return B19200;
    default: return 0;
  }
}

// The resolved path, so /dev/ttyUSB0 and a /dev/serial/by-id link to it are one device.
std::string canonical(const std::string & port)
{
  char * resolved = ::realpath(port.c_str(), nullptr);
  if (resolved == nullptr) {return port;}
  std::string result(resolved);
  std::free(resolved);
  return result;
}

std::int64_t now_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

std::vector<std::string> port_holders(const std::string & port)
{
  const std::string device = canonical(port);
  const std::string self = std::to_string(::getpid());
  std::vector<std::string> holders;

  DIR * proc = ::opendir("/proc");
  if (proc == nullptr) {return holders;}
  while (const dirent * entry = ::readdir(proc)) {
    const std::string pid = entry->d_name;
    if (std::isdigit(static_cast<unsigned char>(pid[0])) == 0 || pid == self) {continue;}
    DIR * descriptors = ::opendir(("/proc/" + pid + "/fd").c_str());
    if (descriptors == nullptr) {continue;}
    while (const dirent * descriptor = ::readdir(descriptors)) {
      char target[PATH_MAX];
      const std::string link = "/proc/" + pid + "/fd/" + descriptor->d_name;
      const ssize_t length = ::readlink(link.c_str(), target, sizeof(target) - 1);
      if (length <= 0) {continue;}
      target[length] = '\0';
      if (device != target) {continue;}
      std::string name = "?";
      std::ifstream comm("/proc/" + pid + "/comm");
      std::getline(comm, name);
      holders.push_back(name + " (pid " + pid + ")");
      break;
    }
    ::closedir(descriptors);
  }
  ::closedir(proc);
  return holders;
}

Bus::~Bus()
{
  close();
}

Bus::Port::~Port()
{
  if (fd >= 0) {::close(fd);}
}

bool Bus::open(const std::string & port, int baud)
{
  close();
  port_ = acquire(port, baud, last_error_);
  if (!port_) {return false;}
  last_error_.clear();
  return true;
}

void Bus::close()
{
  // The last holder closes the fd (~Port). The registry holds a weak_ptr only.
  port_.reset();
}

std::shared_ptr<Bus::Port> Bus::acquire(const std::string & port, int baud, std::string & error)
{
  static std::mutex registry_mutex;
  static std::map<std::string, std::weak_ptr<Port>> registry;

  const speed_t speed = baud_constant(baud);
  if (speed == 0) {
    error = "unsupported baud " + std::to_string(baud) +
      " (supported: 1000000/500000/115200/57600/38400/19200)";
    return nullptr;
  }

  const std::string device = canonical(port);
  const std::lock_guard<std::mutex> guard(registry_mutex);
  if (auto shared = registry[device].lock()) {
    if (shared->baud != baud) {
      error = "'" + port + "' is already open at " + std::to_string(shared->baud) +
        " baud, cannot also open it at " + std::to_string(baud) +
        " (every servo on one wire runs at one rate)";
      return nullptr;
    }
    return shared;
  }

  auto opened = std::make_shared<Port>();
  opened->baud = baud;
  opened->fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (opened->fd < 0) {
    error = "open('" + port + "'): " + std::strerror(errno);
    return nullptr;
  }

  // Lock before the line settings change, so a program that holds the bus keeps its baud.
  // flock, not TIOCEXCL: TIOCEXCL belongs to the tty and stays set while any
  // descriptor is open, so a killed driver can block the port. The kernel drops
  // a flock with its holder. pyserial's exclusive=True takes the same lock.
  if (::flock(opened->fd, LOCK_EX | LOCK_NB) != 0) {
    error = "'" + port + "' is already open by another process";
    const auto holders = port_holders(port);
    for (std::size_t i = 0; i < holders.size(); ++i) {
      error += (i == 0 ? ": " : ", ") + holders[i];
    }
    error += ". Two programs on one half-duplex bus corrupt each other's frames. "
      "Stop the other program first.";
    return nullptr;
  }
  termios tty{};
  if (::tcgetattr(opened->fd, &tty) != 0) {
    error = "tcgetattr('" + port + "'): " + std::strerror(errno);
    return nullptr;
  }
  ::cfmakeraw(&tty);
  ::cfsetispeed(&tty, speed);
  ::cfsetospeed(&tty, speed);
  tty.c_cc[VMIN] = 0;   // read() returns what is ready, and poll() waits
  tty.c_cc[VTIME] = 0;
  if (::tcsetattr(opened->fd, TCSANOW, &tty) != 0) {
    error = "tcsetattr('" + port + "'): " + std::strerror(errno);
    return nullptr;
  }
  ::tcflush(opened->fd, TCIOFLUSH);
  registry[device] = opened;
  return opened;
}

bool Bus::send(const std::vector<std::uint8_t> & packet)
{
  ::tcflush(fd(), TCIFLUSH);  // drop the rest of a transaction that timed out

  // The fd is non-blocking, so a tty that stops draining would spin here forever.
  const std::int64_t deadline = now_ms() + timeout_ms_;
  std::size_t written = 0;
  while (written < packet.size()) {
    if (now_ms() > deadline) {
      last_error_ = "write timeout after " + std::to_string(written) + "/" +
        std::to_string(packet.size()) + " bytes";
      return false;
    }
    const ssize_t n = ::write(fd(), packet.data() + written, packet.size() - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
    } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
      last_error_ = std::string("write: ") + std::strerror(errno);
      return false;
    }
  }
  return true;
}

bool Bus::receive_exact(std::uint8_t * buffer, std::size_t count)
{
  const std::int64_t deadline = now_ms() + timeout_ms_;
  std::size_t filled = 0;
  while (filled < count) {
    const std::int64_t remaining = deadline - now_ms();
    if (remaining <= 0) {
      last_error_ = "timeout after " + std::to_string(filled) + "/" + std::to_string(count) +
        " bytes";
      return false;
    }
    pollfd pfd{fd(), POLLIN, 0};
    const int ready = ::poll(&pfd, 1, static_cast<int>(remaining));
    if (ready < 0 && errno != EINTR) {
      last_error_ = std::string("poll: ") + std::strerror(errno);
      return false;
    }
    if (ready <= 0) {continue;}
    const ssize_t n = ::read(fd(), buffer + filled, count - filled);
    if (n > 0) {
      filled += static_cast<std::size_t>(n);
    } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
      last_error_ = std::string("read: ") + std::strerror(errno);
      return false;
    }
  }
  return true;
}

std::optional<StatusPacket> Bus::receive_status(std::size_t data_length)
{
  rx_.resize(status_packet_size(data_length));
  if (!receive_exact(rx_.data(), rx_.size())) {return std::nullopt;}
  auto status = parse_status(rx_.data(), rx_.size());
  if (!status || status->data.size() != data_length) {
    last_error_ = "malformed status packet";
    return std::nullopt;
  }
  return status;
}

bool Bus::request(std::uint8_t id, std::size_t data_length, std::vector<std::uint8_t> * data)
{
  if (fd() < 0) {
    last_error_ = "bus not open";
    return false;
  }
  const std::lock_guard<std::mutex> guard(port_->transaction);
  if (!send(tx_)) {return false;}
  auto status = receive_status(data_length);
  if (!status) {return false;}
  if (status->id != id) {
    last_error_ = "reply from id " + std::to_string(status->id) + ", expected " +
      std::to_string(id);
    return false;
  }
  if (data != nullptr) {*data = std::move(status->data);}
  return true;
}

bool Bus::ping(std::uint8_t id)
{
  build_ping(tx_, id);
  return request(id, 0, nullptr);
}

bool Bus::read_registers(
  std::uint8_t id, std::uint8_t address, std::uint8_t length, std::vector<std::uint8_t> & out)
{
  build_read(tx_, id, address, length);
  return request(id, length, &out);
}

bool Bus::write_registers(
  std::uint8_t id, std::uint8_t address, const std::uint8_t * data, std::size_t length)
{
  // An addressed write gets a status reply (Response_Status_Level 1, the default).
  build_write(tx_, id, address, data, length);
  return request(id, 0, nullptr);
}

std::size_t Bus::sync_read_feedback(
  const std::vector<std::uint8_t> & ids, std::vector<std::optional<Feedback>> & out)
{
  out.assign(ids.size(), std::nullopt);
  if (fd() < 0) {
    last_error_ = "bus not open";
    return 0;
  }
  build_sync_read(tx_, reg::kPresentPosition, kFeedbackBlockSize, ids);
  const std::lock_guard<std::mutex> guard(port_->transaction);
  if (!send(tx_)) {return 0;}

  // A silent servo stalls the replies after it, so the first timeout ends the
  // batch rather than assign the next reply to the wrong id.
  std::size_t answered = 0;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    auto status = receive_status(kFeedbackBlockSize);
    if (!status) {break;}
    if (status->id != ids[i]) {
      last_error_ = "sync_read reply from id " + std::to_string(status->id) + ", expected " +
        std::to_string(ids[i]);
      break;
    }
    out[i] = decode_feedback(status->data.data(), status->data.size());
    ++answered;
  }
  return answered;
}

bool Bus::sync_write_goals(const std::vector<GoalEntry> & entries)
{
  if (fd() < 0) {
    last_error_ = "bus not open";
    return false;
  }
  if (entries.empty()) {return true;}
  build_sync_write_goals(tx_, entries);
  const std::lock_guard<std::mutex> guard(port_->transaction);
  return send(tx_);
}

}  // namespace feetech
