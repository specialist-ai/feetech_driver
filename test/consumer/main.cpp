// Use the installed headers and the compiled library, so a missing export fails the link.

#include <cstdio>
#include <vector>

#include "feetech/bus.hpp"
#include "feetech/protocol.hpp"

int main()
{
  std::vector<std::uint8_t> packet;
  feetech::build_ping(packet, 18);
  if (packet.size() != 6) {
    std::fprintf(stderr, "a PING is 6 bytes on the wire, got %zu\n", packet.size());
    return 1;
  }

  if (!feetech::port_holders("/nonexistent-tty").empty()) {
    std::fprintf(stderr, "a device that does not exist cannot have holders\n");
    return 1;
  }

  feetech::Bus bus;
  if (bus.is_open()) {
    std::fprintf(stderr, "a default-constructed Bus is not open\n");
    return 1;
  }
  std::printf("consumer linked and ran against the installed package\n");
  return 0;
}
