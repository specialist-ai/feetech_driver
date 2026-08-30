import time

from feetech_driver import _core

# A flash write keeps the servo busy. Wait this long before the next transaction.
EEPROM_WRITE_SETTLE_S = 0.02


class Servo:
    """Register access for one servo id. A read returns None when the servo does not reply."""

    def __init__(self, bus: _core.Bus, servo_id: int):
        self.bus = bus
        self.id = servo_id

    def ping(self) -> bool:
        return self.bus.ping(self.id)

    def read_u8(self, address: int) -> int | None:
        data = self.bus.read_registers(self.id, address, 1)
        return None if data is None else data[0]

    def read_u16(self, address: int) -> int | None:
        data = self.bus.read_registers(self.id, address, 2)
        return None if data is None else int.from_bytes(data, "little")

    def write_u8(self, address: int, value: int) -> bool:
        return self.bus.write_registers(self.id, address, [value & 0xFF])

    def write_u16(self, address: int, value: int) -> bool:
        return self.bus.write_registers(self.id, address, list(value.to_bytes(2, "little")))

    def feedback(self) -> _core.Feedback | None:
        return self.bus.read_feedback(self.id)

    def set_torque(self, enabled: bool) -> bool:
        return self.write_u8(_core.reg.TORQUE_ENABLE, int(enabled))

    def write_goal(self, position_counts: int) -> bool:
        return self.bus.write_registers(
            self.id, _core.reg.ACCELERATION, list(_core.encode_goal_block(position_counts))
        )

    def write_eeprom(self, address: int, value: int, width: int = 1) -> bool:
        """Write an EEPROM register and confirm it with a read.

        A flash write keeps the servo busy, so it can miss its own status reply.
        Only the read tells if the write took effect. The Lock register must be 0.
        """
        if width == 1:
            self.write_u8(address, value)
        else:
            self.write_u16(address, value)
        time.sleep(EEPROM_WRITE_SETTLE_S)
        read = self.read_u8(address) if width == 1 else self.read_u16(address)
        return read == value

    def unlock_eeprom(self) -> bool:
        return self.write_u8(_core.reg.LOCK, 0)

    def lock_eeprom(self) -> bool:
        return self.write_u8(_core.reg.LOCK, 1)


def scan(bus: _core.Bus, max_id: int = _core.MAX_SERVO_ID) -> list[int]:
    """Ping each id in turn and return the ids that reply.

    A broadcast ping makes each servo reply at once, and the collisions look like bad checksums.
    """
    return [servo_id for servo_id in range(max_id + 1) if bus.ping(servo_id)]
