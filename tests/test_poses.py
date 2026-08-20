"""Offline tests for pib_sdk.features.poses (fake backend/writer, no network)."""

from __future__ import annotations

import pytest

import pib_sdk.features.poses as poses_module
from pib_sdk.features.poses import (
    Pose,
    apply_pose,
    get_pose,
    list_poses,
    play_pose_sequence,
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

    def move(self, *args):
        self.calls.append(args)
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
