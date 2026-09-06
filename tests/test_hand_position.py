"""Offline tests for get_hand_position_xyz (injectable position source)."""

import pytest

from pib_sdk.kinematics import ArmKinematics, fk, get_hand_position_xyz

KNOWN_Q_DEG = {
    "right": [10.0, 20.0, -5.0, 15.0, 30.0, 0.0],
    "left": [8.0, -12.0, 4.0, -18.0, 22.0, 6.0],
}


class FakeTelemetry:
    """Minimal position source: only get_positions_deg is used by the helper."""

    def __init__(self, positions: dict[str, float]):
        self.positions = dict(positions)
        self.requested_names: list | None = None

    def get_positions_deg(self, names):
        self.requested_names = list(names)
        return dict(self.positions)


def _positions_for(side: str, q_deg: list[float]) -> dict[str, float]:
    names = ArmKinematics(side).motor_names
    return dict(zip(names, q_deg, strict=True))


def test_returned_xyz_matches_fk():
    side = "right"
    q = KNOWN_Q_DEG[side]
    fake = FakeTelemetry(_positions_for(side, q))
    xyz = get_hand_position_xyz(side, telemetry=fake)
    assert xyz == tuple(fk(side, q).translation)


def test_uses_motor_names_in_correct_order():
    side = "right"
    names = ArmKinematics(side).motor_names
    fake = FakeTelemetry(_positions_for(side, KNOWN_Q_DEG[side]))
    get_hand_position_xyz(side, telemetry=fake)
    assert fake.requested_names == names


def test_raises_on_missing_motor():
    side = "right"
    names = list(ArmKinematics(side).motor_names)
    missing = names[-1]
    positions = _positions_for(side, KNOWN_Q_DEG[side])
    del positions[missing]
    fake = FakeTelemetry(positions)
    with pytest.raises(KeyError, match=missing):
        get_hand_position_xyz(side, telemetry=fake)


def test_left_and_right_both_work():
    results = {}
    for side in ("left", "right"):
        q = KNOWN_Q_DEG[side]
        fake = FakeTelemetry(_positions_for(side, q))
        xyz = get_hand_position_xyz(side, telemetry=fake)
        assert xyz == tuple(fk(side, q).translation)
        results[side] = xyz

    zero_right = get_hand_position_xyz(
        "right", telemetry=FakeTelemetry(_positions_for("right", [0.0] * 6))
    )
    zero_left = get_hand_position_xyz(
        "left", telemetry=FakeTelemetry(_positions_for("left", [0.0] * 6))
    )
    assert zero_right != zero_left
