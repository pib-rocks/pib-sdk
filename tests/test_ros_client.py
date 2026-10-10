"""Offline tests for pib_sdk.ros_client: one shared rosbridge client per process (PR-1882).

The fake below models roslibpy's twisted reactor: it can be started once, and
once a client's ``terminate()`` has stopped it, the next ``run()`` fails the way
the real one does (``ReactorNotRestartable`` in the reactor thread, surfacing as
``RosTimeoutError: Failed to connect to ROS``).
"""

from __future__ import annotations

import pytest
import roslibpy
from roslibpy.core import RosTimeoutError

import pib_sdk.control as control_module
import pib_sdk.telemetry as telemetry_module
from conftest import FakeRos, patch_roslibpy
from pib_sdk.control import Write
from pib_sdk.features.assistant import Assistant
from pib_sdk.features.camera import Camera
from pib_sdk.features.display import Display
from pib_sdk.features.imu import IMU
from pib_sdk.features.programs import Programs
from pib_sdk.features.relay import Relay
from pib_sdk.models import Models
from pib_sdk.robot import Robot
from pib_sdk.ros_client import get_ros
from pib_sdk.speech import Speak
from pib_sdk.telemetry import Telemetry


class _Reactor:
    def __init__(self) -> None:
        self.running = False
        self.stopped = False
        self.run_calls = 0
        self.connect_calls = 0
        self.terminate_calls = 0
        self.clients: list[FakeRos] = []


@pytest.fixture
def reactor(monkeypatch):
    state = _Reactor()

    class ReactorFakeRos(FakeRos):
        def __init__(self, host=None, port=None):
            super().__init__(host=host, port=port)
            state.clients.append(self)

        def run(self):
            state.run_calls += 1
            if state.stopped:
                self.is_connected = False
                raise RosTimeoutError("Failed to connect to ROS")
            state.running = True

        def connect(self):
            state.connect_calls += 1

        def terminate(self):
            state.terminate_calls += 1
            self.is_connected = False
            if state.running:
                state.running = False
                state.stopped = True

    patch_roslibpy(monkeypatch, control_module, telemetry_module)
    monkeypatch.setattr(roslibpy, "Ros", ReactorFakeRos)
    return state


def test_get_ros_returns_one_client_per_endpoint(reactor):
    first = get_ros("pib.local", 9090)
    again = get_ros("pib.local", 9090)
    other_port = get_ros("pib.local", 9191)

    assert again is first
    assert other_port is not first
    assert (other_port.host, other_port.port) == ("pib.local", 9191)
    assert reactor.run_calls == 1
    assert reactor.connect_calls == 1


def test_constructing_every_feature_starts_the_reactor_once(reactor):
    clients = [
        Write(host="pib.local"),
        Speak(host="pib.local"),
        Telemetry(host="pib.local"),
        Models(host="pib.local"),
        Camera(host="pib.local"),
        Relay(host="pib.local"),
        IMU(host="pib.local"),
        Display(host="pib.local"),
        Assistant(host="pib.local"),
        Programs(host="pib.local"),
    ]
    robot = Robot(host="pib.local")

    connections = {id(getattr(client, "ros", None) or client._ros) for client in clients}
    connections |= {id(robot.write.ros), id(robot.speak._ros), id(robot.telemetry.ros)}
    assert reactor.run_calls == 1
    assert len(reactor.clients) == 1
    assert connections == {id(reactor.clients[0])}


def test_telemetry_block_then_write_block_in_one_process(reactor):
    with Telemetry(host="pib.local") as telemetry:
        telemetry_ros = telemetry.ros

    with Write(host="pib.local") as write:
        write_ros = write.ros

    assert write_ros is telemetry_ros
    assert reactor.run_calls == 1


def test_close_and_exit_leave_the_shared_client_connected(reactor):
    with Telemetry(host="pib.local") as telemetry:
        pass
    write = Write(host="pib.local")
    write.close()

    assert telemetry.ros.is_connected
    assert reactor.terminate_calls == 0
    assert get_ros("pib.local", 9090) is telemetry.ros
