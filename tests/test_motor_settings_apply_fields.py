"""A partial ``Write.set()`` must name the fields it changes (PR-2319).

The settings message cannot express "not provided": an omitted field arrives on the
robot as 0/false and used to be written to the motor, which zeroed the pulse widths,
rotation ranges and acceleration and disabled the motor. The SDK now sends
``apply_fields`` so the motors node merges the request onto the current settings.
"""

import pytest

from conftest import patch_roslibpy
from pib_sdk import control
from pib_sdk.control import Write

OK_RESPONSE = {"settings_applied": True, "settings_persisted": True}


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, control)


def _writer_with_service(fake_roslibpy):
    """Construct the client first: services are registered as they are built."""
    writer = Write(host="localhost")
    _topics, services = fake_roslibpy
    service = services["/apply_motor_settings"]
    service.default_response = OK_RESPONSE
    return writer, service


def test_partial_set_names_only_the_field_it_changes(fake_roslibpy):
    writer, service = _writer_with_service(fake_roslibpy)

    assert writer.set("pinky_left_stretch", velocity=6000) is True

    request = service.calls[0]
    assert request["apply_fields"] == ["velocity"]
    assert request["motor_settings"] == {
        "motor_name": "pinky_left_stretch",
        "velocity": 6000,
    }


def test_two_settings_are_both_named(fake_roslibpy):
    writer, service = _writer_with_service(fake_roslibpy)

    writer.set("elbow_right", velocity=6000, acceleration=8000)

    request = service.calls[0]
    assert request["apply_fields"] == ["acceleration", "velocity"]
    assert request["motor_settings"]["acceleration"] == 8000


def test_default_settings_name_every_field_they_send(fake_roslibpy):
    writer, service = _writer_with_service(fake_roslibpy)

    writer.set("elbow_left", control.default)

    request = service.calls[0]
    sent = sorted(key for key in request["motor_settings"] if key != "motor_name")
    assert request["apply_fields"] == sent
    assert set(sent) == set(control.DEFAULT_SETTINGS)


def test_the_motor_name_is_never_an_apply_field(fake_roslibpy):
    writer, service = _writer_with_service(fake_roslibpy)

    writer.set("pinky_left_stretch", velocity=6000)

    assert "motor_name" not in service.calls[0]["apply_fields"]
