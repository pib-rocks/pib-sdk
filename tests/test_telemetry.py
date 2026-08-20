"""Offline tests for pib_sdk.telemetry (roslibpy is faked; no rosbridge)."""

from __future__ import annotations

import pytest

import pib_sdk.telemetry as telemetry_module
from conftest import patch_roslibpy
from pib_sdk.telemetry import Telemetry


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, telemetry_module)


def _diagnostic_status(motor_name: str, current_ma: int) -> dict:
    return {
        "level": 0,
        "name": motor_name,
        "message": "",
        "values": [{"key": motor_name, "value": str(current_ma)}],
    }


def test_get_position_deg_converts_internal_units_to_degrees(fake_roslibpy):
    _topics, services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    services["get_joint_position"].responses["elbow_right"] = {
        "successful": True,
        "position": 4500,
        "message": "",
    }

    assert telemetry.get_position_deg("elbow_right") == 45.0


def test_get_position_deg_raises_on_unknown_joint(fake_roslibpy):
    _topics, services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    services["get_joint_position"].responses["bogus"] = {
        "successful": False,
        "message": "unknown joint name 'bogus'",
    }

    with pytest.raises(ValueError, match="unknown joint name"):
        telemetry.get_position_deg("bogus")


def test_get_positions_deg_batches_several_motors(fake_roslibpy):
    _topics, services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    services["get_joint_position"].responses["a"] = {"successful": True, "position": 100}
    services["get_joint_position"].responses["b"] = {"successful": True, "position": -200}

    result = telemetry.get_positions_deg(["a", "b"])

    assert result == {"a": 1.0, "b": -2.0}


def test_get_current_ma_reads_from_the_cache_after_a_publish(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")

    topics["motor_current"].publish_fake(_diagnostic_status("elbow_right", 1234))

    assert telemetry.get_current_ma("elbow_right") == 1234


def test_get_current_ma_times_out_for_a_motor_never_seen(fake_roslibpy):
    telemetry = Telemetry(host="localhost")

    with pytest.raises(TimeoutError, match="elbow_right"):
        telemetry.get_current_ma("elbow_right", timeout=0.05)


def test_subscribe_current_streams_readings_until_unsubscribed(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    received: list[tuple[str, int]] = []

    def _on_current(motor_name, current_ma):
        received.append((motor_name, current_ma))

    telemetry.subscribe_current(_on_current)
    topics["motor_current"].publish_fake(_diagnostic_status("wrist_left", 42))
    topics["motor_current"].publish_fake(_diagnostic_status("wrist_left", 43))

    telemetry.unsubscribe_current(_on_current)
    topics["motor_current"].publish_fake(_diagnostic_status("wrist_left", 44))

    assert received == [("wrist_left", 42), ("wrist_left", 43)]
    assert telemetry.get_current_ma("wrist_left") == 44  # cache still updates either way


def test_on_motor_current_ignores_malformed_messages(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")

    topics["motor_current"].publish_fake({"name": "x", "values": []})
    topics["motor_current"].publish_fake({"values": [{"key": "x", "value": "1"}]})

    with pytest.raises(TimeoutError):
        telemetry.get_current_ma("x", timeout=0.05)
