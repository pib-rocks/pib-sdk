"""Camera snapshots and depth frames over rosbridge.

pib-backend exposes JPEG snapshots via ``get_camera_image`` and depth via
``get_depth_frame`` / ``get_distance_at_px``. There's no live video streaming,
and no on-board vision/AI here. Camera *settings* (resolution, refresh rate,
quality) are REST configuration, already on :class:`pib_sdk.backend.BackendClient`
(``get_camera_settings`` / ``update_camera_settings``) -- not part of this
module.

Example
-------
    from pib_sdk.features.camera import Camera

    camera = Camera(host="localhost")
    camera.save_snapshot("frame.jpg")
    depth_mm = camera.get_depth_frame()  # uint16 (H, W), millimetres
    distance_mm = camera.get_distance_at_px(320, 240)
    camera.close()
"""

from __future__ import annotations

import base64
import time
from pathlib import Path
from typing import TYPE_CHECKING

import roslibpy

if TYPE_CHECKING:
    import numpy as np


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


def _require_numpy():
    """Lazy numpy import so this module stays importable without it.

    numpy is a core pib-sdk dependency; depth methods still import it only when
    used, matching how :mod:`pib_sdk.features.drawing` lazy-imports Pillow.
    """
    try:
        import numpy as np
    except ImportError as error:
        raise ImportError(
            "Camera depth methods require numpy; install it with `pip install numpy`."
        ) from error
    return np


class Camera:
    """Fetches JPEG snapshots and depth from pib's camera over rosbridge."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        get_image_service: str = "get_camera_image",
        get_depth_frame_service: str = "get_depth_frame",
        get_distance_at_px_service: str = "get_distance_at_px",
    ) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros)
        self._get_image_service = roslibpy.Service(
            self.ros, get_image_service, "datatypes/GetCameraImage"
        )
        self._get_depth_frame_service = roslibpy.Service(
            self.ros, get_depth_frame_service, "datatypes/GetDepthFrame"
        )
        self._get_distance_at_px_service = roslibpy.Service(
            self.ros, get_distance_at_px_service, "datatypes/GetDistanceAtPx"
        )

    def __enter__(self) -> Camera:
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

    def get_snapshot_bytes(self, timeout: float = 10.0) -> bytes:
        """Return one JPEG frame as raw bytes."""
        response = self._get_image_service.call(roslibpy.ServiceRequest({}), timeout=timeout)
        return base64.b64decode(response["image_base64"])

    def save_snapshot(self, path: str | Path, timeout: float = 10.0) -> None:
        """Fetch one JPEG frame and write it to ``path``."""
        Path(path).write_bytes(self.get_snapshot_bytes(timeout=timeout))

    def get_depth_frame(self, timeout: float = 10.0) -> np.ndarray | None:
        """Return the current depth image as millimetres, or ``None`` on failure.

        Calls the ``get_depth_frame`` ROS2 service and decodes ``depth_base64``
        as little-endian uint16 millimetres. Shape is ``(height, width)``;
        ``0`` means invalid / no reading. Encoding from the service is
        ``16UC1``.

        Requires numpy (lazy import; install with ``pip install numpy``).
        """
        np = _require_numpy()
        try:
            response = self._get_depth_frame_service.call(
                roslibpy.ServiceRequest({}), timeout=timeout
            )
        except Exception:
            return None
        if not response:
            return None
        depth_base64 = response.get("depth_base64") or ""
        if not depth_base64:
            return None
        try:
            width = int(response["width"])
            height = int(response["height"])
            raw = base64.b64decode(depth_base64)
            depth = np.frombuffer(raw, dtype="<u2").reshape((height, width))
        except (KeyError, TypeError, ValueError):
            return None
        return depth.copy()

    def get_distance_at_px(self, x: int, y: int, timeout: float = 10.0) -> float:
        """Return depth at pixel ``(x, y)`` of the current camera image, in millimetres.

        Calls the ``get_distance_at_px`` ROS2 service. ``0.0`` means missing
        cache, out of bounds, or an invalid reading.
        """
        try:
            response = self._get_distance_at_px_service.call(
                roslibpy.ServiceRequest({"x": int(x), "y": int(y)}),
                timeout=timeout,
            )
        except Exception:
            return 0.0
        if not response:
            return 0.0
        try:
            return float(response["distance_mm"])
        except (KeyError, TypeError, ValueError):
            return 0.0

    def save_depth(self, path: str | Path, timeout: float = 10.0) -> None:
        """Fetch a depth frame (uint16 millimetres) and write it as a ``.npy`` file.

        ``0`` pixels are invalid. Requires numpy (lazy import).
        """
        np = _require_numpy()
        depth = self.get_depth_frame(timeout=timeout)
        if depth is None:
            raise RuntimeError("No depth frame available to save")
        np.save(path, depth)
