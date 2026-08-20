"""Offline tests for pib_sdk.robot (roslibpy is faked across three modules)."""

from __future__ import annotations

import pytest

import pib_sdk.control as control_module
import pib_sdk.speech as speech_module
import pib_sdk.telemetry as telemetry_module
from conftest import FakeRos, patch_roslibpy
from pib_sdk.backend import BackendClient
from pib_sdk.control import Write
from pib_sdk.robot import Robot
from pib_sdk.speech import Speak
from pib_sdk.telemetry import Telemetry

_TERMINATE_CALLS: list[str] = []


class _TrackedFakeRos(FakeRos):
    def terminate(self):
        _TERMINATE_CALLS.append(f"{self.host}:{self.port}")


@pytest.fixture
def fake_roslibpy(monkeypatch):
    _TERMINATE_CALLS.clear()
    patch_roslibpy(monkeypatch, control_module, speech_module, telemetry_module)
    for module in (control_module, speech_module, telemetry_module):
        monkeypatch.setattr(module.roslibpy, "Ros", _TrackedFakeRos)


def test_robot_bundles_all_four_clients(fake_roslibpy):
    robot = Robot(host="pib.local", rosbridge_port=9090, backend_port=5000)

    assert isinstance(robot.write, Write)
    assert isinstance(robot.speak, Speak)
    assert isinstance(robot.telemetry, Telemetry)
    assert isinstance(robot.backend, BackendClient)


def test_robot_points_every_rosbridge_client_at_the_same_host_and_port(fake_roslibpy):
    robot = Robot(host="pib.local", rosbridge_port=9191)

    assert robot.write.ros.host == "pib.local"
    assert robot.write.ros.port == 9191
    assert robot.speak._ros.host == "pib.local"
    assert robot.speak._ros.port == 9191
    assert robot.telemetry.ros.host == "pib.local"
    assert robot.telemetry.ros.port == 9191


def test_robot_backend_uses_its_own_port(fake_roslibpy):
    robot = Robot(host="pib.local", backend_port=5050)

    assert robot.backend.base_url == "http://pib.local:5050"


def test_robot_close_closes_every_rosbridge_connection(fake_roslibpy):
    robot = Robot(host="pib.local")

    robot.close()

    assert len(_TERMINATE_CALLS) == 3


def test_robot_context_manager_closes_on_exit(fake_roslibpy):
    with Robot(host="pib.local"):
        pass

    assert len(_TERMINATE_CALLS) == 3
