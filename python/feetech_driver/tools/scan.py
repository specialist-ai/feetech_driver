"""Show the servos that reply on a bus, and what they report. The tool only reads.

A servo that does not reply has a power, wiring or port problem. A servo that
replies at the wrong id has a provisioning problem. Stop each program that
uses the bus first.

    feetech-scan --port /dev/ttyUSB0
"""

import argparse

from feetech_driver import _core
from feetech_driver.tools import check_id, open_bus, survey


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", required=True, help="serial device")
    parser.add_argument(
        "--baud", type=int, default=1000000, help="bus rate (the factory rate is 1000000)"
    )
    parser.add_argument("--max-id", type=int, default=_core.MAX_SERVO_ID, help="highest id to ping")
    args = parser.parse_args()
    try:
        check_id("--max-id", args.max_id)
    except ValueError as reason:
        raise SystemExit(f"error: {reason}") from None

    bus = open_bus(args.port, args.baud)
    try:
        print(f"scanning {args.port} at {args.baud} baud, ids 0 to {args.max_id}")
        for line in survey(bus, {}, args.max_id):
            print(f"  {line}")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
