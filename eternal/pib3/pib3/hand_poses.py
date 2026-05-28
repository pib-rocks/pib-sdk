"""Hand pose presets for PIB robot.

Deprecated: Use ``HandPose`` enum from ``pib3.types`` instead::

    >>> from pib3 import HandPose
    >>> robot.set_joints(HandPose.LEFT_OPEN)

These module-level aliases are kept for backward compatibility.
"""

from .types import HandPose, LEFT_HAND_JOINTS, RIGHT_HAND_JOINTS

LEFT_HAND_OPEN = HandPose.LEFT_OPEN.value
LEFT_HAND_CLOSED = HandPose.LEFT_CLOSED.value
RIGHT_HAND_OPEN = HandPose.RIGHT_OPEN.value
RIGHT_HAND_CLOSED = HandPose.RIGHT_CLOSED.value

__all__ = [
    "LEFT_HAND_OPEN",
    "LEFT_HAND_CLOSED",
    "RIGHT_HAND_OPEN",
    "RIGHT_HAND_CLOSED",
    "LEFT_HAND_JOINTS",
    "RIGHT_HAND_JOINTS",
]
