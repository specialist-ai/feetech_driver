"""Give a new or a replaced servo its id. A new STS3215 has id 1.

The tool writes the id only (EEPROM), to one addressed servo, so the other
servos keep their ids. Connect one adapter at a time: two new servos both have
id 1. Stop each program that uses the bus first.

    feetech-set-id --port /dev/ttyUSB0 --to 18
    feetech-set-id --port /dev/ttyUSB0 --from 18 --to 28
"""

import argparse

from feetech_driver import _core
from feetech_driver.servo import Servo
from feetech_driver.tools import assign_id, open_bus


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", required=True, help="the adapter to write on")
    parser.add_argument("--baud", type=int, default=1000000, help="bus rate")
    parser.add_argument(
        "--from", dest="from_id", type=int, default=1, help="the id of the servo now"
    )
    parser.add_argument("--to", dest="to_id", type=int, required=True, help="the new id")
    args = parser.parse_args()

    bus = open_bus(args.port, args.baud)
    try:
        model = Servo(bus, args.from_id).read_u16(_core.reg.MODEL)
        if model is not None and model != _core.STS3215_MODEL:
            print(
                f"note: id {args.from_id} reports model {model}, not the STS3215 ({_core.STS3215_MODEL})."
            )
        print(assign_id(bus, args.from_id, args.to_id))
    except ValueError as reason:
        raise SystemExit(f"error: {reason}.") from None
    finally:
        bus.close()


if __name__ == "__main__":
    main()
