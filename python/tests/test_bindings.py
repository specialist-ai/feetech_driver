import math
import os
import select
import threading
import tty

import pytest

import feetech_driver as feetech


def status(sid: int, data: bytes = b"") -> bytes:
    body = bytes([sid, len(data) + 2, 0]) + data
    return b"\xff\xff" + body + bytes([~sum(body) & 0xFF])


def answer(tables: dict[int, bytearray], sid: int, inst: int, p: bytes) -> bytes:
    if inst == 0x82:  # SYNC_READ
        return b"".join(status(i, tables[i][p[0] : p[0] + p[1]]) for i in p[2:] if i in tables)
    if inst == 0x83:  # SYNC_WRITE
        for k in range(2, len(p), p[1] + 1):
            tables[p[k]][p[0] : p[0] + p[1]] = p[k + 1 : k + 1 + p[1]]
    if sid not in tables or inst not in (1, 2, 3):
        return b""
    if inst == 3:
        tables[sid][p[0] : p[0] + len(p) - 1] = p[1:]
    return status(sid, bytes(tables[sid][p[0] : p[0] + p[1]]) if inst == 2 else b"")


@pytest.fixture
def fake():
    """Servos 1 and 2 at position 2048 on the master side of a pty. No hardware is opened."""
    master, slave = os.openpty()
    tty.setraw(slave)
    port, tables, stop = os.ttyname(slave), {1: bytearray(256), 2: bytearray(256)}, []
    os.close(slave)
    for table in tables.values():
        table[56:58] = (2048).to_bytes(2, "little")

    def run() -> None:
        buf = b""
        while not stop:
            try:
                buf += os.read(master, 4096) if select.select([master], [], [], 0.05)[0] else b""
            except OSError:  # EIO until the Bus opens the slave
                continue
            while len(buf) >= 4 and len(buf) >= 4 + buf[3]:
                pkt, buf = buf[: 4 + buf[3]], buf[4 + buf[3] :]
                os.write(master, answer(tables, pkt[2], pkt[4], pkt[5:-1]))

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    yield port, tables
    stop.append(True)
    thread.join()
    os.close(master)


def test_constants_and_value_types():
    assert feetech.RADIANS_PER_COUNT * feetech.COUNTS_PER_TURN == pytest.approx(2 * math.pi)
    assert feetech.encode_goal_block(-1000) == b"\x00\xe8\x83" + bytes(4)
    assert feetech.GoalEntry(3, 2048).block == feetech.encode_goal_block(2048)
    assert "position_counts=0" in repr(feetech.Feedback())
    bus = feetech.Bus()
    assert not bus.is_open and not bus.ping(1) and bus.last_error_code == feetech.Error.NOT_OPEN
    with pytest.raises(RuntimeError, match="/nonexistent-tty"):
        bus.open("/nonexistent-tty")


def test_bus_against_the_fake(fake):
    port, tables = fake
    with feetech.Bus(port) as bus:
        assert bus.ping(1) and not bus.ping(5) and bus.last_error_code == feetech.Error.TIMEOUT
        assert bus.read_registers(2, 56, 2) == b"\x00\x08"
        assert bus.write_registers(1, 48, b"\xe8\x03") and tables[1][48:50] == b"\xe8\x03"
        feedback = bus.sync_read_feedback([1, 7, 2])
        assert [f and f.position_counts for f in feedback] == [2048, None, 2048]
        assert bus.sync_write_goals([feetech.GoalEntry(1, -5), feetech.GoalEntry(2, 9)])
        assert bus.ping(1)  # the fake handled the SYNC_WRITE before this PING
        assert tables[1][42:44] == b"\x05\x80" and tables[2][42:44] == b"\x09\x00"
        with pytest.raises(RuntimeError, match="holds"):
            feetech.Bus(port)
    assert not bus.is_open
