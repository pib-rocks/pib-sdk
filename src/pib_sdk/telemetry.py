"""Live-ish motor telemetry over rosbridge: commanded position and current.

:class:`Telemetry` mirrors :class:`pib_sdk.control.Write`'s connection
pattern, but for *reading* rather than commanding. It wraps the only two
motor values pib-backend actually publishes:

* **Position** -- via the ``get_joint_position`` service. This is the last
  position **commanded**, not a live sensor reading: pib-backend's own
  physical-position reader (``Motor.get_current_position()``, backed by the
  Tinkerforge Servo Bricklet's encoder) is never published over rosbridge or
  REST -- it's only used internally by the robot's own startup-pose logic.
  So this tells you "what did I last tell this motor to do", not "where is
  it right now" -- it won't detect a stall or a hand-moved joint.
* **Current** -- electrical current draw in milliamps (Tinkerforge Servo
  Bricklet units), pushed continuously (~4 Hz by default) on the
  ``motor_current`` topic for every *connected* motor. A motor with no
  bricklet wired up never appears on this topic at all -- that's the
  steady state for it, not a timeout bug.

There is no current or true-position telemetry beyond these two; this module
doesn't invent any.

Example
-------
    from pib_sdk.telemetry import Telemetry

    telemetry = Telemetry(host="localhost")
    print(telemetry.get_position_deg("elbow_right"))   # last commanded angle
    print(telemetry.get_current_ma("elbow_right"))      # latest current draw
    telemetry.close()
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Iterable
from typing import Any

import roslibpy

_INTERNAL_UNITS_PER_DEGREE = 100.0


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


class Telemetry:
    """Reads a motor's last commanded position and its live current draw.

    See the module docstring for exactly what each value means -- both
    mirror what pib-backend actually publishes, not idealized sensor feedback.
    """

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        get_position_service: str = "get_joint_position",
        motor_current_topic: str = "motor_current",
    ) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros)

        self._get_position_service = roslibpy.Service(
            self.ros, get_position_service, "datatypes/GetJointPosition"
        )
        self._motor_current_topic = roslibpy.Topic(
            self.ros, motor_current_topic, "diagnostic_msgs/DiagnosticStatus"
        )

        self._current_lock = threading.Lock()
        self._latest_current_ma: dict[str, int] = {}
        self._current_subscribers: list[Callable[[str, int], None]] = []
        self._motor_current_topic.subscribe(self._on_motor_current)

    def __enter__(self) -> Telemetry:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        """Close the underlying rosbridge connection."""
        try:
            if self.ros and self.ros.is_connected:
                self.ros.terminate()
        except Exception:
            pass

    # -- position (last commanded target, not live sensor feedback) ------ #
    def get_position_deg(self, motor_name: str, timeout: float = 5.0) -> float:
        """Return ``motor_name``'s last **commanded** target position, in degrees.

        Raises :class:`ValueError` if the backend doesn't recognise
        ``motor_name`` (the same names used by :class:`pib_sdk.control.Write`).
        """
        request = roslibpy.ServiceRequest({"joint_name": motor_name})
        response = self._get_position_service.call(request, timeout=timeout)
        if not response.get("successful", False):
            raise ValueError(
                f"get_joint_position({motor_name!r}) failed: {response.get('message', '')}"
            )
        return response["position"] / _INTERNAL_UNITS_PER_DEGREE

    def get_positions_deg(
        self, motor_names: Iterable[str], timeout: float = 5.0
    ) -> dict[str, float]:
        """Convenience: :meth:`get_position_deg` for several motors at once.

        ``motor_names`` is any iterable of motor-name strings -- e.g. a
        chain's ``.motor_names`` from :mod:`pib_sdk.robot_model`.
        """
        return {name: self.get_position_deg(name, timeout=timeout) for name in motor_names}

    # -- current (electrical, milliamps) ---------------------------------- #
    def get_current_ma(self, motor_name: str, timeout: float = 2.0) -> int:
        """Return ``motor_name``'s most recently observed current, in milliamps.

        Blocks until a reading for ``motor_name`` has been seen on the
        ``motor_current`` topic (it publishes ~4 times/second for every
        connected motor) or ``timeout`` elapses. Raises :class:`TimeoutError`
        if none arrives -- the steady state for a motor with no bricklet
        connected, not necessarily a transient glitch.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._current_lock:
                if motor_name in self._latest_current_ma:
                    return self._latest_current_ma[motor_name]
            time.sleep(0.02)
        raise TimeoutError(
            f"No current reading for {motor_name!r} within {timeout:.1f}s "
            "(it may not have a bricklet connected)"
        )

    def subscribe_current(self, callback: Callable[[str, int], None]) -> None:
        """Call ``callback(motor_name, current_ma)`` for every future reading."""
        with self._current_lock:
            self._current_subscribers.append(callback)

    def unsubscribe_current(self, callback: Callable[[str, int], None]) -> None:
        with self._current_lock:
            if callback in self._current_subscribers:
                self._current_subscribers.remove(callback)

    def _on_motor_current(self, message: dict[str, Any]) -> None:
        motor_name = message.get("name")
        values = message.get("values") or []
        if not motor_name or not values:
            return
        try:
            current_ma = int(values[0]["value"])
        except (KeyError, ValueError, TypeError):
            return
        with self._current_lock:
            self._latest_current_ma[motor_name] = current_ma
            subscribers = list(self._current_subscribers)
        for subscriber in subscribers:
            subscriber(motor_name, current_ma)
