"""Named robot poses and pose sequences, sourced from pib-backend/Cerebra.

In Cerebra, a user poses the robot by hand and saves it under a name;
pib-backend stores the resulting joint values and hands them back over its
REST API (see :class:`pib_sdk.backend.BackendClient`). There's no single
"move to pose" call on the robot -- Cerebra itself assembles one by fetching
a pose's stored joint values and then issuing one batched joint-trajectory
command. This module does the same, driving the robot via
:class:`pib_sdk.control.Write`.

Example
-------
    from pib_sdk.backend import BackendClient
    from pib_sdk.control import Write
    from pib_sdk.features.poses import set_pose, play_pose_sequence

    backend = BackendClient(host="localhost")
    writer = Write(host="localhost")

    set_pose(writer, backend, name="wave_hello")

    play_pose_sequence(writer, backend, [("wave_hello", 1.5), ("rest", 0.0)])

:func:`save_current_pose` runs the other direction -- reading the robot's
current commanded joint values via :class:`pib_sdk.telemetry.Telemetry` and
saving them to Cerebra as a new named pose.
"""

from __future__ import annotations

import time
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from pib_sdk.backend import BackendClient
from pib_sdk.control import Write
from pib_sdk.telemetry import Telemetry

# pib-backend stores each pose's joint values in the same internal units as
# the `/apply_joint_trajectory` ROS service (hundredths of a degree) -- see
# pib-backend's startup_pose_executor.py, which forwards a pose's stored
# `position` straight into a JointTrajectoryPoint with no scaling.
_INTERNAL_UNITS_PER_DEGREE = 100.0


@dataclass(frozen=True)
class PoseSummary:
    """A saved pose's identity, without its joint values."""

    pose_id: str
    name: str
    deletable: bool


@dataclass(frozen=True)
class Pose:
    """A saved pose's joint values, in degrees, keyed by motor name."""

    pose_id: str
    name: str
    deletable: bool
    motor_angles_deg: dict[str, float]


def list_poses(backend: BackendClient) -> list[PoseSummary]:
    """List every pose saved in Cerebra, without fetching joint values."""
    return [
        PoseSummary(pose_id=item["poseId"], name=item["name"], deletable=item["deletable"])
        for item in backend.list_poses()
    ]


def get_pose(
    backend: BackendClient, *, name: str | None = None, pose_id: str | None = None
) -> Pose:
    """Fetch a saved pose's joint values, identified by ``name`` or ``pose_id``."""
    if (name is None) == (pose_id is None):
        raise ValueError("Provide exactly one of `name` or `pose_id`")

    if name is not None:
        data = backend.get_pose_by_name(name)
        pose_id = data["poseId"]
        deletable = data["deletable"]
        raw_positions = data["motorPositions"]
    else:
        summary = next(
            (item for item in backend.list_poses() if item["poseId"] == pose_id), None
        )
        if summary is None:
            raise ValueError(f"No pose with id {pose_id!r}")
        name = summary["name"]
        deletable = summary["deletable"]
        raw_positions = backend.get_motor_positions(pose_id)

    motor_angles_deg = {
        item["motorName"]: item["position"] / _INTERNAL_UNITS_PER_DEGREE
        for item in raw_positions
    }
    return Pose(pose_id=pose_id, name=name, deletable=deletable, motor_angles_deg=motor_angles_deg)


def apply_pose(writer: Write, pose: Pose) -> bool:
    """Move every motor in ``pose`` to its saved angle, in one batched command."""
    args: list[str | float] = []
    for motor_name, angle_deg in pose.motor_angles_deg.items():
        args.extend((motor_name, angle_deg))
    return writer.move(*args)


def set_pose(
    writer: Write, backend: BackendClient, *, name: str | None = None, pose_id: str | None = None
) -> Pose:
    """Fetch a saved pose by ``name`` or ``pose_id`` and move the robot to it."""
    pose = get_pose(backend, name=name, pose_id=pose_id)
    apply_pose(writer, pose)
    return pose


def play_pose_sequence(
    writer: Write,
    backend: BackendClient,
    sequence: Sequence[tuple[str, float]],
    *,
    by: str = "name",
) -> None:
    """Move through a sequence of saved poses, holding for each given duration.

    ``sequence`` is ``[(pose_identifier, hold_seconds), ...]``; ``by`` selects
    whether ``pose_identifier`` is a pose name (default) or a ``pose_id``.
    There's no native pose-sequence concept in pib-backend -- Cerebra only
    offers this via chained "move to pose" + "wait" Blockly blocks -- so this
    simply fetches and applies each pose in order.
    """
    if by not in ("name", "pose_id"):
        raise ValueError("`by` must be 'name' or 'pose_id'")
    for identifier, hold_seconds in sequence:
        kwargs = {"name": identifier} if by == "name" else {"pose_id": identifier}
        set_pose(writer, backend, **kwargs)
        if hold_seconds > 0:
            time.sleep(hold_seconds)


def save_current_pose(
    telemetry: Telemetry, backend: BackendClient, name: str, motor_names: Iterable[str]
) -> Pose:
    """Read ``motor_names``' current commanded angles and save them to Cerebra as ``name``.

    "Current" means the same thing :class:`pib_sdk.telemetry.Telemetry` always
    means it: each motor's last *commanded* position, not a live sensor
    reading -- see that module's docstring. ``motor_names`` is any iterable
    of motor-name strings, e.g. a chain's ``.motor_names`` from
    :mod:`pib_sdk.robot_model`.
    """
    angles_deg = telemetry.get_positions_deg(motor_names)
    motor_positions = [
        {"motorName": motor_name, "position": round(angle_deg * _INTERNAL_UNITS_PER_DEGREE)}
        for motor_name, angle_deg in angles_deg.items()
    ]
    created = backend.create_pose(name, motor_positions)
    return Pose(
        pose_id=created["poseId"],
        name=created["name"],
        deletable=created["deletable"],
        motor_angles_deg=angles_deg,
    )
