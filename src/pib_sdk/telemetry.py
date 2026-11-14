"""Motor telemetry over rosbridge: commanded position, measured joint states, current.

:class:`Telemetry` mirrors :class:`pib_sdk.control.Write`'s connection
pattern, but for *reading* rather than commanding. It wraps the three motor
values pib-backend publishes:

* **Commanded position** -- via the ``get_joint_position`` service
  (:meth:`Telemetry.get_position_deg`). This is the last position
  **commanded**, not a sensor reading. It tells you "what did I last tell this
  motor to do", not "where is it right now", so it won't detect a stall or a
  hand-moved joint. Kept unchanged for backward compatibility.
* **Measured joint states** -- ``sensor_msgs/JointState`` on ``/joint_states``
  (:meth:`Telemetry.get_measured_position_deg`, :meth:`Telemetry.get_joint_state`,
  :meth:`Telemetry.subscribe_joint_states`). pib-backend's ``motor_control``
  node publishes it from a bus sweep at ``joint_state_publish_rate_hz``
  (10 Hz by default), positions in **radians**, ``header.stamp`` set at
  publish time on the robot's clock. What a position *is* depends on the
  actuator family (pib-backend PR-2114 "position kind"): **Feetech** serial
  servos report a real **measured** encoder position; the **Tinkerforge Servo
  Bricklet** has no position feedback, so on a bricklet-only robot the topic
  carries the last **commanded target** instead. On a robot with any Feetech
  bus, bricklet joints are absent from the topic. The message itself does not
  say which kind it is.
* **Current** -- electrical current draw in milliamps, pushed on the
  ``motor_current`` topic (once per second by default) for every *connected*
  motor. A motor with no current reading never appears on this topic at all
  -- that's the steady state for it, not a timeout bug.

Example
-------
    from pib_sdk.telemetry import Telemetry

    telemetry = Telemetry(host="localhost")
    print(telemetry.get_position_deg("elbow_right"))           # last commanded angle
    print(telemetry.get_measured_position_deg("elbow_right"))  # from /joint_states
    print(telemetry.get_current_ma("elbow_right"))             # latest current draw
    telemetry.close()
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import Any

import roslibpy

from pib_sdk.ros_client import get_ros, release_ros

_INTERNAL_UNITS_PER_DEGREE = 100.0

#: Default maximum age of a measured joint position before it counts as stale.
DEFAULT_JOINT_STATE_MAX_AGE_S = 0.5


class StaleJointStateError(RuntimeError):
    """The newest ``/joint_states`` reading for a joint is older than ``max_age``."""


@dataclass(frozen=True)
class JointStateSample:
    """One ``sensor_msgs/JointState`` message as received over rosbridge.

    ``stamp_s`` is the message's ``header.stamp`` in seconds (robot clock,
    ``None`` if the header was empty). ``received_s`` is the local
    ``time.time()`` at arrival and ``received_monotonic_s`` the local
    ``time.monotonic()`` at arrival; staleness checks use the latter, so they
    do not depend on the robot's and this host's clocks agreeing.
    ``positions_rad`` maps joint name to position in radians, exactly as
    published; only joints present in this message are included.
    """

    stamp_s: float | None
    received_s: float
    received_monotonic_s: float
    positions_rad: dict[str, float]

    @property
    def positions_deg(self) -> dict[str, float]:
        """``positions_rad`` converted to degrees."""
        return {name: math.degrees(rad) for name, rad in self.positions_rad.items()}

    @property
    def age_s(self) -> float:
        """Seconds since this sample arrived locally."""
        return time.monotonic() - self.received_monotonic_s


def _stamp_seconds(header: Any) -> float | None:
    """``header.stamp`` in seconds; accepts ROS 2 (sec/nanosec) and ROS 1 (secs/nsecs) keys."""
    if not isinstance(header, dict):
        return None
    stamp = header.get("stamp")
    if not isinstance(stamp, dict):
        return None
    sec = stamp.get("sec", stamp.get("secs"))
    nanosec = stamp.get("nanosec", stamp.get("nsecs", 0))
    try:
        value = float(sec) + float(nanosec) * 1e-9
    except (TypeError, ValueError):
        return None
    if value == 0.0:
        return None
    return value


def _parse_joint_state(message: dict[str, Any]) -> JointStateSample | None:
    names = message.get("name") or []
    positions = message.get("position") or []
    if not isinstance(names, list) or not isinstance(positions, list):
        return None
    parsed: dict[str, float] = {}
    for name, value in zip(names, positions):
        if not isinstance(name, str) or not name:
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        if not math.isfinite(value):
            continue
        parsed[name] = float(value)
    if not parsed:
        return None
    return JointStateSample(
        stamp_s=_stamp_seconds(message.get("header")),
        received_s=time.time(),
        received_monotonic_s=time.monotonic(),
        positions_rad=parsed,
    )


class Telemetry:
    """Reads commanded positions, measured joint states and current draw.

    See the module docstring for exactly what each value means -- all of them
    mirror what pib-backend actually publishes, not idealized sensor feedback.
    """

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        get_position_service: str = "get_joint_position",
        motor_current_topic: str = "motor_current",
        joint_states_topic: str = "/joint_states",
    ) -> None:
        self.ros = get_ros(host, port)

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

        # /joint_states arrives at ~10 Hz with every joint, so it is subscribed
        # only on first use rather than for every Telemetry instance.
        self._joint_states_topic = roslibpy.Topic(
            self.ros, joint_states_topic, "sensor_msgs/JointState"
        )
        self._joint_state_lock = threading.Lock()
        self._joint_states_subscribed = False
        self._latest_joint_state: JointStateSample | None = None
        self._latest_joint_by_name: dict[str, JointStateSample] = {}
        self._joint_state_subscribers: list[Callable[[JointStateSample], None]] = []

    def __enter__(self) -> Telemetry:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        """Release the shared rosbridge connection; it stays open for other clients."""
        with self._joint_state_lock:
            subscribed = self._joint_states_subscribed
            self._joint_states_subscribed = False
        if subscribed:
            self._joint_states_topic.unsubscribe(self._on_joint_state)
        release_ros(self.ros)

    # -- position (last commanded target, not live sensor feedback) ------ #
    def get_position_deg(self, motor_name: str, timeout: float = 5.0) -> float:
        """Return ``motor_name``'s last **commanded** target position, in degrees.

        This is not a measurement; use :meth:`get_measured_position_deg` for
        the position published on ``/joint_states``. Raises
        :class:`ValueError` if the backend doesn't recognise ``motor_name``
        (the same names used by :class:`pib_sdk.control.Write`).
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

    # -- measured joint states (/joint_states, radians on the wire) -------- #
    def get_joint_state(
        self, max_age: float | None = DEFAULT_JOINT_STATE_MAX_AGE_S, timeout: float = 2.0
    ) -> JointStateSample:
        """Return the newest whole ``/joint_states`` message.

        Waits up to ``timeout`` seconds for the first message (raises
        :class:`TimeoutError` if none arrives). Raises
        :class:`StaleJointStateError` if the newest message is older than
        ``max_age`` seconds; pass ``max_age=None`` to get it regardless and
        check :attr:`JointStateSample.age_s` yourself.
        """
        self._ensure_joint_states_subscribed()
        sample = self._wait_for(lambda: self._latest_joint_state, timeout, "any joint")
        _check_fresh(sample, max_age, "/joint_states")
        return sample

    def get_measured_position_deg(
        self,
        motor_name: str,
        max_age: float | None = DEFAULT_JOINT_STATE_MAX_AGE_S,
        timeout: float = 2.0,
    ) -> float:
        """Return ``motor_name``'s newest position from ``/joint_states``, in degrees.

        On Feetech servos this is a measured encoder position; on a
        bricklet-only robot it is the commanded target (see the module
        docstring). Raises :class:`TimeoutError` if no reading for
        ``motor_name`` arrives within ``timeout`` (the steady state for a joint
        the backend does not publish), and :class:`StaleJointStateError` if
        its newest reading is older than ``max_age`` seconds (``None``
        disables the check).
        """
        return self.get_measured_positions_deg([motor_name], max_age=max_age, timeout=timeout)[
            motor_name
        ]

    def get_measured_positions_deg(
        self,
        motor_names: Iterable[str],
        max_age: float | None = DEFAULT_JOINT_STATE_MAX_AGE_S,
        timeout: float = 2.0,
    ) -> dict[str, float]:
        """:meth:`get_measured_position_deg` for several motors, one wait for all.

        Each joint keeps its own newest reading, so a joint the last sweep
        missed still returns its previous value as long as it is younger than
        ``max_age``. Use :meth:`get_joint_state` when you need positions that
        all come from the same message and stamp.
        """
        names = list(motor_names)
        self._ensure_joint_states_subscribed()

        def _all_seen() -> dict[str, JointStateSample] | None:
            found = {name: self._latest_joint_by_name.get(name) for name in names}
            if any(sample is None for sample in found.values()):
                return None
            return found  # type: ignore[return-value]

        missing_label = ", ".join(repr(name) for name in names)
        samples = self._wait_for(_all_seen, timeout, missing_label)
        result: dict[str, float] = {}
        for name in names:
            sample = samples[name]
            _check_fresh(sample, max_age, repr(name))
            result[name] = math.degrees(sample.positions_rad[name])
        return result

    def subscribe_joint_states(self, callback: Callable[[JointStateSample], None]) -> None:
        """Call ``callback(sample)`` for every future ``/joint_states`` message.

        Each :class:`JointStateSample` carries ``header.stamp`` and the local
        arrival time, so the callback can log joint states with timestamps.
        Callbacks run on roslibpy's receive thread; keep them short.
        """
        with self._joint_state_lock:
            self._joint_state_subscribers.append(callback)
        self._ensure_joint_states_subscribed()

    def unsubscribe_joint_states(self, callback: Callable[[JointStateSample], None]) -> None:
        """Stop calling ``callback``; no error if it was not subscribed."""
        with self._joint_state_lock:
            if callback in self._joint_state_subscribers:
                self._joint_state_subscribers.remove(callback)

    def record_joint_states(self, duration: float) -> list[JointStateSample]:
        """Block for ``duration`` seconds and return every message received meanwhile."""
        recorded: list[JointStateSample] = []
        self.subscribe_joint_states(recorded.append)
        try:
            time.sleep(duration)
        finally:
            self.unsubscribe_joint_states(recorded.append)
        return list(recorded)

    def _ensure_joint_states_subscribed(self) -> None:
        with self._joint_state_lock:
            if self._joint_states_subscribed:
                return
            self._joint_states_subscribed = True
        self._joint_states_topic.subscribe(self._on_joint_state)

    def _wait_for(self, read: Callable[[], Any], timeout: float, label: str) -> Any:
        deadline = time.monotonic() + timeout
        while True:
            with self._joint_state_lock:
                value = read()
            if value is not None:
                return value
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"No /joint_states reading for {label} within {timeout:.1f}s "
                    "(the backend may not publish it, or motor power is off)"
                )
            time.sleep(0.01)

    def _on_joint_state(self, message: dict[str, Any]) -> None:
        sample = _parse_joint_state(message)
        if sample is None:
            return
        with self._joint_state_lock:
            self._latest_joint_state = sample
            for name in sample.positions_rad:
                self._latest_joint_by_name[name] = sample
            subscribers = list(self._joint_state_subscribers)
        for subscriber in subscribers:
            subscriber(sample)

    # -- current (electrical, milliamps) ---------------------------------- #
    def get_current_ma(self, motor_name: str, timeout: float = 2.0) -> int:
        """Return ``motor_name``'s most recently observed current, in milliamps.

        Blocks until a reading for ``motor_name`` has been seen on the
        ``motor_current`` topic (it publishes once per second by default for
        every connected motor) or ``timeout`` elapses. Raises :class:`TimeoutError`
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


def _check_fresh(sample: JointStateSample, max_age: float | None, label: str) -> None:
    if max_age is None:
        return
    age = sample.age_s
    if age > max_age:
        raise StaleJointStateError(
            f"/joint_states reading for {label} is {age:.3f}s old (max_age={max_age:.3f}s)"
        )
