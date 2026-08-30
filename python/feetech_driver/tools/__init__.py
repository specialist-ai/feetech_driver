"""Bench tools for a Feetech servo bus."""

from feetech_driver import _core
from feetech_driver.servo import Servo, scan


def open_bus(port: str, baud: int) -> _core.Bus:
    """Open the bus, or stop the program with the reason (for example, the program that holds it)."""
    bus = _core.Bus()
    try:
        bus.open(port, baud)
    except RuntimeError as exc:
        raise SystemExit(f"error: {exc}") from None
    return bus


def check_id(flag: str, value: int) -> None:
    # The broadcast id is MAX_SERVO_ID + 1. An EEPROM write to it changes each servo on the wire.
    if not 0 <= value <= _core.MAX_SERVO_ID:
        raise ValueError(f"{flag} {value} is outside 0 to {_core.MAX_SERVO_ID}")


def assign_id(bus: _core.Bus, from_id: int, to_id: int) -> str:
    """Give the servo at `from_id` the id `to_id` (one EEPROM write). Return a report line.

    Raise ValueError with the reason when the change is not safe or fails.
    """
    check_id("--from", from_id)
    check_id("--to", to_id)
    source, target = Servo(bus, from_id), Servo(bus, to_id)
    if not source.ping():
        raise ValueError(f"no servo replied at id {from_id}")
    if from_id == to_id:
        return f"the servo already has id {to_id}. Nothing was written."
    if target.ping():
        raise ValueError(
            f"id {to_id} is already on this adapter. Two servos with one id make the bus unusable"
        )

    source.set_torque(False)
    if not source.unlock_eeprom():
        raise ValueError(f"the Lock register of id {from_id} did not clear")
    # The servo replies at the new id as soon as the write completes.
    source.write_u8(_core.reg.ID, to_id)
    if not target.ping():
        raise ValueError(
            f"no servo replied at id {to_id}. The servo can still have id {from_id}, unlocked"
        )
    if not target.lock_eeprom():
        raise ValueError(
            f"the id is {to_id}, but the Lock register did not close. Cycle the power and try again"
        )
    return f"the servo now has id {to_id}."


def survey(bus: _core.Bus, expected: dict[int, str], max_id: int = _core.MAX_SERVO_ID) -> list[str]:
    """Ping each id and describe the servos that reply. `expected` maps an id to a name.

    An expected id that does not reply is reported as missing.
    """
    found = scan(bus, max_id)
    lines = []
    for servo_id in found:
        servo = Servo(bus, servo_id)
        model, sample, torque_limit = (
            servo.read_u16(_core.reg.MODEL),
            servo.feedback(),
            servo.read_u16(_core.reg.TORQUE_LIMIT),
        )
        label = (
            f"id {servo_id} = {expected[servo_id]}" if servo_id in expected else f"id {servo_id}"
        )
        if None in (model, sample, torque_limit):
            lines.append(
                f"{label}: replied to the ping but not to each read. The bus is unreliable."
            )
            continue
        lines.append(
            f"{label}: model {model}, {sample.position_counts} counts, {sample.voltage_volts:.1f} V, "
            f"{sample.temperature_celsius} C, torque_limit {torque_limit}"
        )
    lines += [
        f"MISSING: id {i} ({name}) did not reply."
        for i, name in sorted(expected.items())
        if i not in found
    ]
    return lines or ["no servo replied."]
