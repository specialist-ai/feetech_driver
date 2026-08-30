// A C++ call that returns false raises RuntimeError with last_error().

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "feetech/bus.hpp"
#include "feetech/protocol.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using Blocking = nb::call_guard<nb::gil_scoped_release>;

NB_MODULE(_core, m)
{
  m.attr("MAX_SERVO_ID") = feetech::kMaxServoId;
  m.attr("STS3215_MODEL") = feetech::kSts3215Model;
  m.attr("COUNTS_PER_TURN") = feetech::kCountsPerTurn;
  m.attr("RADIANS_PER_COUNT") = feetech::kRadiansPerCount;
  m.attr("MAX_TORQUE_LIMIT") = feetech::kMaxTorqueLimit;

  nb::module_ reg = m.def_submodule("reg", "Control-table addresses.");
  reg.attr("MODEL") = feetech::reg::kModel;
  reg.attr("ID") = feetech::reg::kId;
  reg.attr("P_COEFFICIENT") = feetech::reg::kPCoefficient;
  reg.attr("D_COEFFICIENT") = feetech::reg::kDCoefficient;
  reg.attr("I_COEFFICIENT") = feetech::reg::kICoefficient;
  reg.attr("MINIMUM_STARTUP_FORCE") = feetech::reg::kMinimumStartupForce;
  reg.attr("CW_DEAD_ZONE") = feetech::reg::kCwDeadZone;
  reg.attr("CCW_DEAD_ZONE") = feetech::reg::kCcwDeadZone;
  reg.attr("TORQUE_ENABLE") = feetech::reg::kTorqueEnable;
  reg.attr("ACCELERATION") = feetech::reg::kAcceleration;
  reg.attr("TORQUE_LIMIT") = feetech::reg::kTorqueLimit;
  reg.attr("LOCK") = feetech::reg::kLock;

  m.def(
    "encode_goal_block",
    [](int position_counts) {
      const auto block = feetech::encode_goal_block(position_counts);
      return nb::bytes(block.data(), block.size());
    },
    "position_counts"_a, "The goal block, registers 41 to 47. Write it at reg.ACCELERATION.");

  nb::class_<feetech::Feedback>(m, "Feedback")
  .def_ro("position_counts", &feetech::Feedback::position_counts)
  .def_ro("velocity_counts_per_sec", &feetech::Feedback::velocity_counts_per_sec)
  .def_ro("load_permille", &feetech::Feedback::load_permille, "A signed PWM duty, not a torque.")
  .def_ro("voltage_volts", &feetech::Feedback::voltage_volts)
  .def_ro("temperature_celsius", &feetech::Feedback::temperature_celsius)
  .def_ro("status", &feetech::Feedback::status)
  .def_ro("moving", &feetech::Feedback::moving)
  .def_ro("current_amps", &feetech::Feedback::current_amps);

  m.def("port_holders", &feetech::port_holders, "port"_a);

  nb::class_<feetech::Bus>(m, "Bus")
  .def(nb::init<>())
  .def(
    "open",
    [](feetech::Bus & self, const std::string & port, int baud) {
      if (!self.open(port, baud)) {throw std::runtime_error(self.last_error());}
    },
    "port"_a, "baud"_a = 1000000, Blocking())
  .def("close", &feetech::Bus::close, Blocking())
  .def_prop_ro("is_open", &feetech::Bus::is_open)
  .def_prop_ro("last_error", &feetech::Bus::last_error)
  .def("ping", &feetech::Bus::ping, "servo_id"_a, Blocking())
  .def(
    "read_registers",
    [](feetech::Bus & self, std::uint8_t servo_id, std::uint8_t address, std::uint8_t length)
    -> std::optional<nb::bytes> {
      std::vector<std::uint8_t> data;
      bool ok;
      {
        nb::gil_scoped_release release;
        ok = self.read_registers(servo_id, address, length, data);
      }
      if (!ok) {return std::nullopt;}
      return nb::bytes(data.data(), data.size());
    },
    "servo_id"_a, "address"_a, "length"_a, "None if the servo did not reply.")
  .def(
    "write_registers",
    [](feetech::Bus & self, std::uint8_t servo_id, std::uint8_t address,
    const std::vector<std::uint8_t> & data) {
      return self.write_registers(servo_id, address, data.data(), data.size());
    },
    "servo_id"_a, "address"_a, "data"_a, Blocking())
  .def(
    "read_feedback",
    [](feetech::Bus & self, std::uint8_t servo_id) {
      std::vector<std::optional<feetech::Feedback>> out;
      self.sync_read_feedback({servo_id}, out);
      return out[0];
    },
    "servo_id"_a, "None if the servo did not reply.", Blocking());
}
