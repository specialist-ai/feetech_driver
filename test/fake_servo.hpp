// A pty stands in for the adapter, and a thread on its master side plays the servos.

#pragma once

#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "feetech/protocol.hpp"

namespace fake {

using Bytes = std::vector<std::uint8_t>;

inline std::uint8_t checksum(const Bytes& packet) {
  unsigned sum = 0;
  for (std::size_t i = 2; i < packet.size(); ++i) sum += packet[i];
  return static_cast<std::uint8_t>(~sum);
}

inline Bytes status(std::uint8_t id, std::uint8_t error, const Bytes& data) {
  Bytes packet{0xFF, 0xFF, id, static_cast<std::uint8_t>(data.size() + 2), error};
  packet.insert(packet.end(), data.begin(), data.end());
  packet.push_back(checksum(packet));
  return packet;
}

// The Bus opens `name`. The slave fd stays open, so the master does not see a hangup.
struct Pty {
  Pty() {
    char path[256] = {};
    termios raw{};
    ::cfmakeraw(&raw);
    if (::openpty(&master, &slave, path, &raw, nullptr) == 0) name = path;
  }
  ~Pty() {
    if (master >= 0) ::close(master);
    ::close(slave);
  }
  int master{-1};
  int slave{-1};
  std::string name;
};

// Servos with a 256-byte control table each. PING, READ and WRITE get a reply from the
// addressed id. SYNC_WRITE gets none. SYNC_READ gets one reply per listed servo, in order.
class Servos {
 public:
  // A hook sees each request first. It returns the bytes to send instead of the replies.
  using Hook = std::function<std::optional<Bytes>(const Bytes& request)>;

  explicit Servos(int master) : master_(master), thread_([this] { run(); }) {}
  ~Servos() {
    stop_ = true;
    thread_.join();
  }
  void add(std::uint8_t id, int position) {
    std::array<std::uint8_t, 256> table{};
    table[feetech::reg::kId] = id;
    feetech::write_le_u16(&table[feetech::reg::kPresentPosition],
                          feetech::encode_sign_magnitude(position, feetech::kSignBitPosition));
    table[feetech::reg::kPresentVoltage] = 120;
    const std::lock_guard<std::mutex> lock(mutex_);
    tables_[id] = table;
  }
  int u16(std::uint8_t id, std::uint8_t address) {
    const std::lock_guard<std::mutex> lock(mutex_);
    return feetech::read_le_u16(&tables_.at(id)[address]);
  }
  void set_error(std::uint8_t error) { error_ = error; }
  void set_hook(Hook hook) {
    const std::lock_guard<std::mutex> lock(mutex_);
    hook_ = std::move(hook);
  }
  int requests() const { return requests_; }

 private:
  void run() {
    Bytes buffer;
    while (!stop_) {
      pollfd pfd{master_, POLLIN, 0};
      std::uint8_t chunk[512];
      if (::poll(&pfd, 1, 5) <= 0) continue;
      const ssize_t n = ::read(master_, chunk, sizeof chunk);
      if (n <= 0) continue;
      buffer.insert(buffer.end(), chunk, chunk + n);
      while (buffer.size() >= 4 && buffer.size() >= buffer[3] + 4u) {
        const Bytes request(buffer.begin(), buffer.begin() + buffer[3] + 4);
        buffer.erase(buffer.begin(), buffer.begin() + buffer[3] + 4);
        ++requests_;
        const Bytes reply = respond(request);
        if (!reply.empty() && ::write(master_, reply.data(), reply.size()) < 0) return;
      }
    }
  }
  Bytes respond(const Bytes& request) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (hook_) {
      if (auto reply = hook_(request)) return *reply;
    }
    const std::uint8_t id = request[2];
    const Bytes p(request.begin() + 5, request.end() - 1);
    const auto read = [&](std::uint8_t from, std::uint8_t address, std::uint8_t length) {
      const auto& table = tables_.at(from);
      return status(from, error_, {table.begin() + address, table.begin() + address + length});
    };
    Bytes reply;
    switch (request[4]) {
      case 0x01:  // PING
        if (tables_.count(id) != 0) reply = status(id, error_, {});
        break;
      case 0x02:  // READ
        if (tables_.count(id) != 0) reply = read(id, p[0], p[1]);
        break;
      case 0x03:  // WRITE
        if (tables_.count(id) == 0) break;
        std::copy(p.begin() + 1, p.end(), tables_[id].begin() + p[0]);
        reply = status(id, error_, {});
        break;
      case 0x82:  // SYNC_READ
        for (std::size_t i = 2; i < p.size(); ++i) {
          if (tables_.count(p[i]) == 0) continue;
          const Bytes one = read(p[i], p[0], p[1]);
          reply.insert(reply.end(), one.begin(), one.end());
        }
        break;
      case 0x83:  // SYNC_WRITE
        for (std::size_t i = 2; i + 1 + p[1] <= p.size(); i += 1 + p[1]) {
          if (tables_.count(p[i]) != 0) std::copy_n(&p[i + 1], p[1], &tables_[p[i]][p[0]]);
        }
        break;
      default:
        break;
    }
    return reply;
  }

  int master_;
  std::mutex mutex_;
  std::map<std::uint8_t, std::array<std::uint8_t, 256>> tables_;
  std::atomic<std::uint8_t> error_{0};
  std::atomic<int> requests_{0};
  Hook hook_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace fake
