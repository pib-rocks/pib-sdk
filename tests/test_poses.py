"""Offline tests for pib_sdk.features.poses (fake backend/writer, no network)."""

from __future__ import annotations

import pytest

import pib_sdk.features.poses as poses_module
from conftest import patch_roslibpy
from pib_sdk import control
from pib_sdk.control import Write
from pib_sdk.features.poses import (
    Pose,
    apply_pose,
    get_pose,
    list_poses,
    play_pose_sequence,
    play_pose_sequence_timed,
    save_current_pose,
    set_pose,
)


class _FakeBackend:
    def __init__(self, poses: dict[str, dict]):
        self._poses = poses
        self.created_pose: dict | None = None

    def list_poses(self):
        return [
            {"poseId": pose["poseId"], "name": pose["name"], "deletable": pose["deletable"]}
            for pose in self._poses.values()
        ]

    def get_pose_by_name(self, name):
        for pose in self._poses.values():
            if pose["name"] == name:
                return pose
        raise KeyError(name)

    def get_motor_positions(self, pose_id):
        return self._poses[pose_id]["motorPositions"]

    def create_pose(self, name, motor_positions):
        self.created_pose = {"name": name, "motorPositions": motor_positions}
        return {"poseId": "new-id", "name": name, "deletable": True}


class _FakeWriter:
    def __init__(self):
        self.calls: list[tuple] = []
        self.timed_calls: list[tuple[list[str], list[tuple[list[float], float]]]] = []

    def move(self, *args):
        self.calls.append(args)
        return True

    def send_timed_trajectory(self, joint_names, waypoints):
        self.timed_calls.append((list(joint_names), list(waypoints)))
        return True


class _FakeTelemetry:
    def __init__(self, angles_deg: dict[str, float]):
        self._angles_deg = angles_deg
        self.requested: list | None = None

    def get_positions_deg(self, motor_names):
        self.requested = list(motor_names)
        return {name: self._angles_deg[name] for name in self.requested}


def _backend_with(*poses):
    return _FakeBackend({pose["poseId"]: pose for pose in poses})


WAVE = {
    "poseId": "p1",
    "name": "wave",
    "deletable": True,
    "motorPositions": [{"motorName": "elbow_right", "position": 4500}],
}
REST = {
    "poseId": "p2",
    "name": "rest",
    "deletable": False,
    "motorPositions": [{"motorName": "elbow_right", "position": 0}],
}
REACH = {
    "poseId": "p3",
    "name": "reach",
    "deletable": True,
    "motorPositions": [
        {"motorName": "elbow_right", "position": 0},
        {"motorName": "wrist_right", "position": -1000},
    ],
}


def test_list_poses_returns_summaries_without_motor_positions():
    backend = _backend_with(WAVE, REST)
    summaries = list_poses(backend)
    assert {summary.name for summary in summaries} == {"wave", "rest"}
    assert all(
        hasattr(summary, "pose_id") and not hasattr(summary, "motor_angles_deg")
        for summary in summaries
    )


def test_get_pose_by_name_converts_internal_units_to_degrees():
    backend = _backend_with(WAVE)
    pose = get_pose(backend, name="wave")
    assert pose.pose_id == "p1"
    assert pose.motor_angles_deg == {"elbow_right": 45.0}


def test_get_pose_by_id_looks_up_name_from_the_listing():
    backend = _backend_with(WAVE, REST)
    pose = get_pose(backend, pose_id="p2")
    assert pose.name == "rest"
    assert pose.motor_angles_deg == {"elbow_right": 0.0}


def test_get_pose_requires_exactly_one_selector():
    backend = _backend_with(WAVE)
    with pytest.raises(ValueError):
        get_pose(backend)
    with pytest.raises(ValueError):
        get_pose(backend, name="wave", pose_id="p1")


def test_get_pose_by_id_raises_for_unknown_id():
    backend = _backend_with(WAVE)
    with pytest.raises(ValueError, match="p999"):
        get_pose(backend, pose_id="p999")


def test_apply_pose_sends_one_vector_move_covering_every_motor():
    pose = Pose(
        pose_id="p1",
        name="wave",
        deletable=True,
        motor_angles_deg={"elbow_right": 45.0, "wrist_right": -10.0},
    )
    writer = _FakeWriter()

    apply_pose(writer, pose)

    assert writer.calls == [("elbow_right", 45.0, "wrist_right", -10.0)]


def test_set_pose_fetches_then_applies():
    backend = _backend_with(WAVE)
    writer = _FakeWriter()

    pose = set_pose(writer, backend, name="wave")

    assert pose.name == "wave"
    assert writer.calls == [("elbow_right", 45.0)]


def test_play_pose_sequence_applies_each_pose_and_sleeps_between(monkeypatch):
    sleeps: list[float] = []
    monkeypatch.setattr(poses_module.time, "sleep", lambda seconds: sleeps.append(seconds))
    backend = _backend_with(WAVE, REST)
    writer = _FakeWriter()

    play_pose_sequence(writer, backend, [("wave", 1.5), ("rest", 0.0)])

    assert writer.calls == [("elbow_right", 45.0), ("elbow_right", 0.0)]
    assert sleeps == [1.5]  # a zero-second hold shouldn't call sleep(0)


def test_play_pose_sequence_rejects_unknown_by_value():
    backend = _backend_with(WAVE)
    writer = _FakeWriter()
    with pytest.raises(ValueError):
        play_pose_sequence(writer, backend, [("wave", 0.0)], by="pose_name")


def test_play_pose_sequence_timed_accumulates_per_leg_seconds():
    backend = _backend_with(WAVE, REST)
    writer = _FakeWriter()

    play_pose_sequence_timed(writer, backend, [("wave", 1.5), ("rest", 0.5)])

    assert len(writer.timed_calls) == 1
    joint_names, waypoints = writer.timed_calls[0]
    assert joint_names == ["elbow_right"]
    assert len(waypoints) == 2
    assert waypoints[0] == ([4500.0], 1.5)
    assert waypoints[1] == ([0.0], 2.0)
    assert writer.calls == []  # must not use the naive stop-per-pose path


def test_play_pose_sequence_timed_unions_joints_and_backfills():
    backend = _backend_with(WAVE, REACH)
    writer = _FakeWriter()

    play_pose_sequence_timed(writer, backend, [("wave", 1.0), ("reach", 0.5)])

    joint_names, waypoints = writer.timed_calls[0]
    assert joint_names == ["elbow_right", "wrist_right"]
    assert waypoints[0][0] == [4500.0, -1000.0]  # wrist backfilled from reach
    assert waypoints[1][0] == [0.0, -1000.0]
    assert waypoints[0][1] == 1.0
    assert waypoints[1][1] == 1.5


def test_play_pose_sequence_timed_carries_forward_missing_later_motors():
    backend = _backend_with(REACH, WAVE)
    writer = _FakeWriter()

    play_pose_sequence_timed(writer, backend, [("reach", 0.25), ("wave", 0.75)])

    joint_names, waypoints = writer.timed_calls[0]
    assert joint_names == ["elbow_right", "wrist_right"]
    assert waypoints[0][0] == [0.0, -1000.0]
    assert waypoints[1][0] == [4500.0, -1000.0]  # wrist carried forward
    assert waypoints[0][1] == 0.25
    assert waypoints[1][1] == 1.0


def test_play_pose_sequence_timed_rejects_unknown_by_value():
    backend = _backend_with(WAVE)
    writer = _FakeWriter()
    with pytest.raises(ValueError):
        play_pose_sequence_timed(writer, backend, [("wave", 1.0)], by="pose_name")


def test_play_pose_sequence_timed_emits_software_trajectory_shape(monkeypatch):
    topics, services = patch_roslibpy(monkeypatch, control)
    writer = Write(host="localhost")
    services["/apply_joint_trajectory"].default_response = {"successful": True}
    backend = _backend_with(WAVE, REACH)

    play_pose_sequence_timed(writer, backend, [("wave", 1.0), ("reach", 0.5)])

    calls = services["/apply_joint_trajectory"].calls
    assert len(calls) == 1
    trajectory = calls[0]["joint_trajectory"]
    points = trajectory["points"]
    assert trajectory["joint_names"] == ["elbow_right", "wrist_right"]
    assert len(points) == 2
    assert [point["positions"] for point in points] == [
        [4500.0, -1000.0],
        [0.0, -1000.0],
    ]
    assert all(len(point["positions"]) == len(trajectory["joint_names"]) for point in points)
    assert points[0]["time_from_start"] == {"sec": 1, "nanosec": 0}
    assert points[1]["time_from_start"] == {"sec": 1, "nanosec": 500_000_000}
    # Software-trajectory shape: multi-point, positions-per-joint, non-zero times.
    assert len(points) > 1
    assert all(point["time_from_start"] != {"sec": 0, "nanosec": 0} for point in points)


def test_save_current_pose_reads_telemetry_and_creates_a_pose():
    backend = _backend_with(WAVE)
    telemetry = _FakeTelemetry({"elbow_right": 45.0, "wrist_right": -10.0})

    pose = save_current_pose(telemetry, backend, "new_pose", ["elbow_right", "wrist_right"])

    assert telemetry.requested == ["elbow_right", "wrist_right"]
    assert backend.created_pose == {
        "name": "new_pose",
        "motorPositions": [
            {"motorName": "elbow_right", "position": 4500},
            {"motorName": "wrist_right", "position": -1000},
        ],
    }
    assert pose.pose_id == "new-id"
    assert pose.motor_angles_deg == {"elbow_right": 45.0, "wrist_right": -10.0}
