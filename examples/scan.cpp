// Ping each id on a bus and print the servos that reply. Read only.
//
//   feetech_scan_example /dev/ttyUSB0 [baud]

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "feetech/bus.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <port> [baud]\n", argv[0]);
    return 2;
  }
  const std::string port = argv[1];
  const int baud = (argc > 2) ? std::atoi(argv[2]) : 1000000;

  feetech::Bus bus;
  if (!bus.open(port, baud)) {
    std::fprintf(stderr, "open %s: %s\n", port.c_str(), bus.last_error().c_str());
    return 1;
  }
  std::printf("scanning %s at %d baud\n", port.c_str(), baud);

  std::vector<std::uint8_t> found;
  for (std::uint8_t id = 0; id <= feetech::kMaxServoId; ++id) {
    if (bus.ping(id)) {found.push_back(id);}
  }
  if (found.empty()) {
    std::printf("  no servo replied\n");
    return 0;
  }

  std::vector<std::optional<feetech::Feedback>> feedback;
  bus.sync_read_feedback(found, feedback);
  for (std::size_t i = 0; i < found.size(); ++i) {
    if (!feedback[i]) {
      std::printf("  id %u: replied to the ping but not to the read\n", found[i]);
      continue;
    }
    std::printf(
      "  id %u: %d counts, %.1f V, %d C\n", found[i], feedback[i]->position_counts,
      feedback[i]->voltage_volts, feedback[i]->temperature_celsius);
  }
  return 0;
}
