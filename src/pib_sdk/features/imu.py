"""Live IMU samples over rosbridge.

pib-backend publishes ``sensor_msgs/Imu`` on ``/imu``. This client keeps the
most recent well-formed message and returns it from :meth:`IMU.latest`. There
is no callback API and no assumption that samples arrive at a regular
interval — measured periods on the current hardware are not uniform.

Orientation is a sensor_msgs field, not a guaranteed measurement. When
``orientation_covariance[0] == -1.0`` (the sensor_msgs convention for "do
not use"), :attr:`IMUData.orientation` is ``None`` and
:attr:`IMUData.orientation_available` is ``False``. This hardware release
publishes that sentinel; the SDK does not invent a quaternion.

Example
-------
    from pib_sdk.features.imu import IMU

    imu = IMU(host="localhost")
    sample = imu.latest()
    if sample is not None:
        print(sample.acceleration_m_s2, sample.age_s, sample.orientation_available)
    imu.close()
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any

import roslibpy

_ORIENTATION_UNAVAILABLE = -1.0


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


@dataclass(frozen=True)
class Vector3:
    """Cartesian triple in the units of the enclosing :class:`IMUData` field."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Quaternion:
    """Hamilton quaternion ``(x, y, z, w)``. Unused while orientation is unavailable."""

    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class IMUData:
    """One IMU sample captured on the host at receive time.

    ``timestamp_s`` is ``time.time()`` when the rosbridge message arrived, not
    the sensor's header stamp. ``age_s`` is ``time.time() - timestamp_s`` at
    the :meth:`IMU.latest` call that built this snapshot.
    """

    acceleration_m_s2: Vector3
    angular_velocity_rad_s: Vector3
    orientation: Quaternion | None
    orientation_available: bool
    orientation_covariance: list[float]
    timestamp_s: float
    age_s: float


def _as_vector3(value: Any) -> Vector3 | None:
    if not isinstance(value, dict):
        return None
    try:
        return Vector3(x=float(value["x"]), y=float(value["y"]), z=float(value["z"]))
    except (KeyError, TypeError, ValueError):
        return None


def _as_quaternion(value: Any) -> Quaternion | None:
    if not isinstance(value, dict):
        return None
    try:
        return Quaternion(
            x=float(value["x"]),
            y=float(value["y"]),
            z=float(value["z"]),
            w=float(value["w"]),
        )
    except (KeyError, TypeError, ValueError):
        return None


def _as_covariance(value: Any) -> list[float]:
    if not isinstance(value, (list, tuple)):
        return []
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError):
        return []


def _orientation_available(covariance: list[float]) -> bool:
    return bool(covariance) and covariance[0] != _ORIENTATION_UNAVAILABLE


class IMU:
    """Caches the latest ``/imu`` sample. :meth:`latest` never blocks for one."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        imu_topic: str = "/imu",
        imu_message_type: str = "sensor_msgs/Imu",
    ) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros)

        self._lock = threading.Lock()
        self._acceleration_m_s2: Vector3 | None = None
        self._angular_velocity_rad_s: Vector3 | None = None
        self._orientation: Quaternion | None = None
        self._orientation_available = False
        self._orientation_covariance: list[float] = []
        self._timestamp_s: float | None = None

        self._imu_topic = roslibpy.Topic(self.ros, imu_topic, imu_message_type)
        self._imu_topic.subscribe(self._on_imu)

    def __enter__(self) -> IMU:
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

    def latest(self) -> IMUData | None:
        """Return the most recent sample, or ``None`` if none has arrived yet.

        Does not wait and does not assume a publishing period. Intervals on
        the current hardware are not uniform, so callers should use
        :attr:`IMUData.age_s` rather than a 100 ms rule.
        """
        with self._lock:
            if (
                self._acceleration_m_s2 is None
                or self._angular_velocity_rad_s is None
                or self._timestamp_s is None
            ):
                return None
            timestamp_s = self._timestamp_s
            return IMUData(
                acceleration_m_s2=self._acceleration_m_s2,
                angular_velocity_rad_s=self._angular_velocity_rad_s,
                orientation=self._orientation,
                orientation_available=self._orientation_available,
                orientation_covariance=list(self._orientation_covariance),
                timestamp_s=timestamp_s,
                age_s=time.time() - timestamp_s,
            )

    def _on_imu(self, message: dict[str, Any]) -> None:
        acceleration = _as_vector3(message.get("linear_acceleration"))
        angular_velocity = _as_vector3(message.get("angular_velocity"))
        if acceleration is None or angular_velocity is None:
            return
        covariance = _as_covariance(message.get("orientation_covariance"))
        available = _orientation_available(covariance)
        orientation = _as_quaternion(message.get("orientation")) if available else None
        if available and orientation is None:
            available = False
        received_at = time.time()
        with self._lock:
            self._acceleration_m_s2 = acceleration
            self._angular_velocity_rad_s = angular_velocity
            self._orientation = orientation
            self._orientation_available = available
            self._orientation_covariance = covariance
            self._timestamp_s = received_at
