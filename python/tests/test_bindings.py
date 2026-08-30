import math

import pytest

import feetech_driver as feetech


def test_the_goal_block_holds_a_sign_magnitude_position():
    assert bytes(feetech.encode_goal_block(-1000)) == b"\x00\xe8\x83\x00\x00\x00\x00"


def test_a_count_is_one_4096th_of_a_turn():
    assert feetech.RADIANS_PER_COUNT * feetech.COUNTS_PER_TURN == pytest.approx(2 * math.pi)


def test_bus_calls_fail_before_open():
    bus = feetech.Bus()
    assert not bus.is_open
    assert not bus.ping(1)
    assert feetech.Servo(bus, 1).read_u16(feetech.reg.MODEL) is None
    assert bus.read_feedback(1) is None


def test_opening_a_missing_device_raises_with_the_reason():
    assert feetech.port_holders("/nonexistent-tty") == []
    with pytest.raises(RuntimeError, match="/nonexistent-tty"):
        feetech.Bus().open("/nonexistent-tty", 1000000)
