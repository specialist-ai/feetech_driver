// A failed Bus call returns False, None or a short count and sets last_error. Only open()
// raises, a RuntimeError. The GIL is released only around calls that touch the port.

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstdint>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "feetech/bus.hpp"
#include "feetech/protocol.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using Bytes = std::vector<std::uint8_t>;
// It releases the GIL around the call only. nanobind converts the result after it.
using Release = nb::call_guard<nb::gil_scoped_release>;
using feetech::Bus;

namespace {

template <class Call>
auto without_gil(Call&& call) {
  nb::gil_scoped_release release;
  return call();
}

template <class Container>
nb::bytes as_bytes(const Container& data) {
  return nb::bytes(reinterpret_cast<const char*>(data.data()), data.size());
}

void open_or_raise(Bus& bus, const std::string& port, int baud) {
  if (!without_gil([&] { return bus.open(port, baud); }))
    throw std::runtime_error(bus.last_error());
}

std::string feedback_repr(const feetech::Feedback& f) {
  char text[256];
  std::snprintf(text, sizeof text,
                "Feedback(position_counts=%d, velocity_counts_per_sec=%d, load_permille=%d, "
                "voltage_volts=%.1f, temperature_celsius=%d, status=%u, moving=%s, "
                "current_amps=%.4f)",
                f.position_counts, f.velocity_counts_per_sec, f.load_permille, f.voltage_volts,
                f.temperature_celsius, static_cast<unsigned>(f.status), f.moving ? "True" : "False",
                f.current_amps);
  return text;
}

}  // namespace

NB_MODULE(_core, m) {
  namespace ft = feetech;
  m.attr("__version__") = FEETECH_VERSION;
  m.attr("BROADCAST_ID") = ft::kBroadcastId;
  m.attr("MAX_SERVO_ID") = ft::kMaxServoId;
  m.attr("COUNTS_PER_TURN") = ft::kCountsPerTurn;
  m.attr("RADIANS_PER_COUNT") = ft::kRadiansPerCount;
  m.attr("MAX_TORQUE_LIMIT") = ft::kMaxTorqueLimit;

  nb::enum_<ft::Error> error(m, "Error", nb::is_arithmetic(), "Why the last Bus call failed.");
  const char* names[] = {"NONE",      "NOT_OPEN", "INVALID_ARGUMENT", "UNSUPPORTED_BAUD",
                         "PORT_BUSY", "IO",       "DISCONNECTED",     "TIMEOUT",
                         "MALFORMED", "WRONG_ID"};
  for (int i = 0; i < 10; ++i) error.value(names[i], static_cast<ft::Error>(i));

  m.def(
      "encode_goal_block", [](int position) { return as_bytes(ft::encode_goal_block(position)); },
      "position_counts"_a, "Registers 41 to 47. The position saturates at +-32767.");

  nb::class_<ft::Feedback>(m, "Feedback", "Registers 56 to 70 of one servo, decoded.")
      .def(nb::init<>())
      .def_ro("position_counts", &ft::Feedback::position_counts)
      .def_ro("velocity_counts_per_sec", &ft::Feedback::velocity_counts_per_sec)
      .def_ro("load_permille", &ft::Feedback::load_permille, "A signed PWM duty, not a torque.")
      .def_ro("voltage_volts", &ft::Feedback::voltage_volts)
      .def_ro("temperature_celsius", &ft::Feedback::temperature_celsius)
      .def_ro("status", &ft::Feedback::status, "servo_error bits.")
      .def_ro("moving", &ft::Feedback::moving)
      .def_ro("current_amps", &ft::Feedback::current_amps)
      .def("__repr__", &feedback_repr);

  nb::class_<ft::GoalEntry>(m, "GoalEntry", "One servo of a sync_write_goals call.")
      .def(
          "__init__",
          [](ft::GoalEntry* self, std::uint8_t id, int position) {
            new (self) ft::GoalEntry{id, ft::encode_goal_block(position)};
          },
          "id"_a, "position_counts"_a)
      .def_rw("id", &ft::GoalEntry::id)
      .def_prop_ro("block", [](const ft::GoalEntry& e) { return as_bytes(e.block); });

  nb::class_<Bus>(m, "Bus", "A failed call returns False or None and sets last_error.")
      .def(
          "__init__",
          [](Bus* self, std::optional<std::string> port, int baud) {
            new (self) Bus();
            if (port) open_or_raise(*self, *port, baud);
          },
          "port"_a = nb::none(), "baud"_a = 1000000, "Opens `port` when it is given.")
      .def("open", &open_or_raise, "port"_a, "baud"_a = 1000000, "Raises RuntimeError.")
      .def("close", &Bus::close, Release())
      .def("__enter__", [](nb::handle self) { return self; })
      .def("__exit__", [](Bus& self, nb::args) { without_gil([&] { self.close(); }); })
      .def_prop_ro("is_open", &Bus::is_open)
      .def_prop_ro("last_error", &Bus::last_error, "Empty after a successful call.")
      .def_prop_ro("last_error_code", &Bus::last_error_code)
      .def_prop_ro("last_servo_error", &Bus::last_servo_error, "The ERR bits of the replies.")
      .def_prop_rw("timeout_ms", &Bus::timeout_ms, &Bus::set_timeout_ms)
      .def("ping", &Bus::ping, "servo_id"_a, Release())
      .def(
          "read_registers",
          [](Bus& self, std::uint8_t id, std::uint8_t address, std::uint8_t length) {
            Bytes data;
            const bool ok =
                without_gil([&] { return self.read_registers(id, address, length, data); });
            return ok ? std::optional(as_bytes(data)) : std::nullopt;
          },
          "servo_id"_a, "address"_a, "length"_a, "None if the servo did not reply.")
      .def(
          "write_registers",
          [](Bus& self, std::uint8_t id, std::uint8_t address, nb::bytes data) {
            const auto* begin = static_cast<const std::uint8_t*>(data.data());
            const Bytes bytes(begin, begin + data.size());
            return without_gil([&] { return self.write_registers(id, address, bytes); });
          },
          "servo_id"_a, "address"_a, "data"_a)
      .def(
          "sync_read_feedback",
          [](Bus& self, const Bytes& ids) {
            std::vector<std::optional<ft::Feedback>> out;
            self.sync_read_feedback(ids, out);
            return out;
          },
          "ids"_a, Release(), "One entry per id, None where no reply came.")
      .def("sync_write_goals", &Bus::sync_write_goals, "entries"_a, Release(),
           "True means sent: no servo replies.");
}
