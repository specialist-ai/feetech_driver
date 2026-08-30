"""Driver for Feetech SMS/STS serial-bus servos."""

from feetech_driver._core import (
    BROADCAST_ID,
    COUNTS_PER_TURN,
    MAX_SERVO_ID,
    MAX_TORQUE_LIMIT,
    RADIANS_PER_COUNT,
    Bus,
    Error,
    Feedback,
    GoalEntry,
    __version__,
    encode_goal_block,
)

__all__ = [
    "BROADCAST_ID",
    "COUNTS_PER_TURN",
    "MAX_SERVO_ID",
    "MAX_TORQUE_LIMIT",
    "RADIANS_PER_COUNT",
    "Bus",
    "Error",
    "Feedback",
    "GoalEntry",
    "__version__",
    "encode_goal_block",
]
