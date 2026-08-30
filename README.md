# feetech_driver

C++ driver for Feetech SMS/STS serial-bus servos (STS3215 and its relatives),
with Python bindings and two bench tools. The library does not depend on ROS.

It is the one implementation of the wire protocol and the serial transport, so a
plain C++ program, a Python script and a ROS 2 hardware component all put the
same bytes on the wire.

## Layers

| Header | What it holds | Depends on |
|---|---|---|
| `feetech/protocol.hpp` | framing, the sign-magnitude codec, the control table, the feedback block | nothing |
| `feetech/bus.hpp` | half-duplex serial transport, blocking transactions | POSIX |

`protocol.hpp` is pure logic. `test/test_protocol.cpp` pins every packet the
driver emits to a golden byte vector.

The library stops at the encoder frame. A joint frame is a sign and an offset on
top of it. That is calibration, it differs per machine, and it belongs to the
consumer.

## Build

```bash
pixi run build          # the library, the tests and the example
pixi run test           # the C++ tests, no hardware needed

uv sync                 # the Python extension and the package
uv run pytest python/tests -q
```

The build produces a static library by default. Pass
`-DBUILD_SHARED_LIBS=ON` for a shared one, which is what a distribution package
wants. See "One tty, one bus" below for why that choice matters.

A ROS 2 workspace builds this package with colcon: `package.xml` declares
`<build_type>cmake</build_type>`, so no ROS dependency is added to the library.

## Use

```cpp
#include "feetech/bus.hpp"

feetech::Bus bus;
if (!bus.open("/dev/ttyUSB0", 1000000)) {
  std::fprintf(stderr, "%s\n", bus.last_error().c_str());
  return 1;
}

std::vector<std::optional<feetech::Feedback>> feedback;
bus.sync_read_feedback({18, 19}, feedback);          // one transaction, both servos

std::vector<feetech::GoalEntry> goals;
goals.push_back({18, feetech::encode_goal_block(2048)});
bus.sync_write_goals(goals);                         // one broadcast frame
```

`examples/scan.cpp` is the same API as a runnable program.

From Python, through the same core:

```python
import feetech_driver as feetech

bus = feetech.Bus()
bus.open('/dev/ttyUSB0', 1000000)
servo = feetech.Servo(bus, 18)
print(servo.feedback().position_counts)
servo.write_goal(2048)
bus.close()
```

## Tools

```bash
feetech-scan   --port /dev/ttyUSB0          # which servos answer, and what they report
feetech-set-id --port /dev/ttyUSB0 --to 18  # give a servo its id (writes EEPROM)
```

`feetech-scan` only reads. Run it first when a bus does not work: it separates a
servo that does not answer, which is a power, a wiring or a port problem, from a
servo that answers on the wrong id, which is a provisioning problem.

`feetech-set-id` is the only part of this package that writes EEPROM. Point
`--port` at one adapter at a time. Two factory-fresh servos both answer to id 1,
so an id written on a shared wire reaches both.

Each tool refuses to start while another process holds the port. Two programs on
one half-duplex bus corrupt each other's frames, and the result reads as
unreliable hardware rather than as a mistake.

## Why not the Dynamixel SDK

The framing is Dynamixel Protocol 1.0 (`FF FF ID LEN INST … CHK`, checksum
`= ~sum(ID..params)`), but three details differ, and each one corrupts data
silently rather than failing loudly:

1. **16-bit fields are little-endian** on SMS/STS. The older SCS/SCSCL series is
   big-endian, and most sample code does not say which it targets.
2. **Signed values are sign-magnitude, not two's complement.** The sign bit is
   position 15 for position and velocity, 11 for homing offset, 10 for load.
   Two's-complement decoding works for positive values and breaks for negative.
3. **P/D/I gains are one byte each and live in EEPROM** behind the `Lock`
   register, so re-applying gains at every bring-up is not an option. `Bus`
   never writes EEPROM. `feetech-set-id` does, deliberately and on request.

Register addresses were cross-checked against FTServo_Linux's `SMS_STS.h` and
LeRobot's `feetech/tables.py`, then read back from an STS3215 (model 777,
firmware 3.10) on the bench.

## Timing

Keep `Bus` off a control loop. One `SYNC_READ` round trip costs ~1 ms with a p95
near 1.6 ms, which is far too much jitter to sit inside one. Run it on its own
I/O thread and let the loop read the last sample. Measured on the bench (STS3215
firmware 3.10, CH343 adapter in `cdc_acm` mode, 1 Mbit):

| Transaction | median | p95 |
|---|---|---|
| `SYNC_READ`, 15-byte feedback block | 1.00 ms | 1.05 ms |
| `SYNC_WRITE`, 7-byte goal block | 0.12 ms | 0.58 ms |
| full cycle (read + write) | 1.00 ms | 1.58 ms |

Only ~450 µs of that is wire time. The rest is USB round-trip overhead, and it is
paid **once per read transaction regardless of servo count**. That is why the hot
path is exactly one `SYNC_READ` plus one `SYNC_WRITE` per cycle, and why
`sync_write_goals` is used even for a single servo: it is broadcast, so it is
never acknowledged, which makes it ~8x cheaper than an addressed `WRITE`.

## One tty, one bus

Two devices may share one USB adapter, each driven by its own thread, so two
`Bus` objects open the same tty.

`Bus::open` therefore resolves through a process-wide registry keyed on the
device name. Both callers get one fd and one transaction mutex, so their threads
interleave whole transactions instead of bytes. Two independent opens would
corrupt the half-duplex framing, which reads as flaky hardware rather than as a
wiring mistake. The fd belongs to the last holder, not the first, so tearing one
device down leaves the other running. `test/test_bus_sharing.cpp` pins that
against a pty.

The key is the **resolved** device path, so `/dev/ttyUSB0` and a
`/dev/serial/by-id` symlink to it are one entry rather than two opens of one
wire.

The registry is a process-wide static, so **link the library once per process**.
A static build puts a separate copy in every library that links it, which splits
the sharing silently. Build shared (`-DBUILD_SHARED_LIBS=ON`) anywhere two
consumers can end up in one process. The static build is for the Python wheel,
which is one consumer by construction.

## Two programs, one bus

A registry cannot reach across processes, and a second *program* on the wire
corrupts in the same way, also silently. Reads steal each other's bytes and
checksums fail, which a driver reports as a broken servo. So `open()` takes an
advisory `flock`:

```
'/dev/ttyUSB0' is already open by another process: ros2_control_node (pid 4213).
Two programs on one half-duplex bus corrupt each other's frames. Stop the other
program first.
```

`port_holders()` supplies the name. It walks `/proc` and compares the resolved
target of every descriptor, so it finds any holder rather than a known list of
program names. Processes owned by another user are invisible without privilege.

**flock, not `TIOCEXCL`.** `TIOCEXCL` would also stop a program that checks for
no lock at all, but it is a property of the tty rather than of our file
description. It stays set while *any* descriptor on the device is open, so a
killed driver can leave the port unopenable by everyone until some unrelated
holder closes. The kernel drops a `flock` when the holder dies.

## Stable device names

`/dev/ttyUSB0` and `/dev/ttyACM0` are assigned in enumeration order, so two
adapters can swap between boots and a configuration naming one then drives the
other device. Give every adapter a udev rule that names it after its serial
number, and name that path instead.

## Where this is used

```
feetech_driver/    this repository -- the servo bus
specialist_sdk/    the Specialist gripper: joint model, calibration, safety guards
specialist_ros2/   the ros2_control hardware component
```

A consumer adds the joint frame, the calibration file and the safety policy on
top. Nothing of that is here.
