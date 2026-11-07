# feetech_driver

A minimal Linux driver for Feetech SMS/STS serial-bus servos (STS3215 and relatives): a C++
library and Python bindings. It pings, reads and writes registers, reads the feedback of many
servos in one SYNC_READ and writes their goal positions in one SYNC_WRITE.

## Build

```sh
pixi run test                                   # C++ library and tests, from pixi.lock
cmake -S . -B build && cmake --build build      # needs GTest; add -DFEETECH_BUILD_TESTING=OFF without it
pip install .                                   # the Python module, feetech_driver
uv sync && uv run pytest                        # the Python tests
```

CMake consumers link `feetech::feetech`, from `find_package(feetech_driver)` or `add_subdirectory`.

## C++

```cpp
#include <feetech/bus.hpp>

feetech::Bus bus;
if (!bus.open("/dev/ttyACM0", 1000000)) std::cerr << bus.last_error() << "\n";
std::vector<std::optional<feetech::Feedback>> feedback;
bus.sync_read_feedback({1, 2}, feedback);
if (feedback[0]) std::cout << feedback[0]->position_counts * feetech::kRadiansPerCount << "\n";
const std::uint8_t on = 1;
if (!bus.write_registers(1, feetech::reg::kTorqueEnable, &on, 1)) std::cerr << bus.last_error();
if (!bus.sync_write_goals({{1, feetech::encode_goal_block(2048)}})) std::cerr << bus.last_error();
```

## Python

```python
import feetech_driver as feetech

with feetech.Bus("/dev/ttyACM0", 1000000) as bus:  # raises RuntimeError if it cannot open
    for servo_id, feedback in zip([1, 2], bus.sync_read_feedback([1, 2])):
        print(servo_id, feedback and feedback.position_counts * feetech.RADIANS_PER_COUNT)
    if not bus.write_registers(1, 40, b"\x01"):  # Torque_Enable
        print(bus.last_error_code, bus.last_error)
    bus.sync_write_goals([feetech.GoalEntry(1, 2048)])
```

## Threads and processes

Use one `Bus` per port. Every `Bus` call is safe from any thread: a mutex runs the calls one
at a time, so many threads can share that one `Bus`. `open()` takes a `flock` on the port, so
a second `Bus`, in this process or in another one, fails with `kPortBusy`.

## Errors

A failed call returns `false` (Python: `False` or `None`) or, for `sync_read_feedback`, a
count smaller than the number of ids. It sets `last_error()`, a sentence, and
`last_error_code()`, an `Error`. A successful call clears both. Only Python's `open()` raises.
A reply with ERR bits succeeds, and `last_servo_error()` holds the `servo_error` bits.
A call waits `timeout_ms()` (20 ms by default) plus the wire time of its packets. After a
missing or garbled reply, the bus discards input until the line is quiet, so the next call
starts clean. `kDisconnected` means that the adapter was unplugged: `close()`, then `open()`.
