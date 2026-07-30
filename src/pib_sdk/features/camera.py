"""Camera snapshots over rosbridge.

pib-backend exposes exactly one thing over the camera's ROS interface: a
single-frame JPEG snapshot via the ``get_camera_image`` service. There's no
live video streaming, and no on-board vision/AI here -- this is a plain
snapshot camera. Camera *settings* (resolution, refresh rate, quality) are
REST configuration, already on :class:`pib_sdk.backend.BackendClient`
(``get_camera_settings`` / ``update_camera_settings``) -- not part of this
module.

Example
-------
    from pib_sdk.features.camera import Camera

    camera = Camera(host="localhost")
    camera.save_snapshot("frame.jpg")
    camera.close()
"""

from __future__ import annotations

import base64
import time
from pathlib import Path

import roslibpy


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


class Camera:
    """Fetches single-frame JPEG snapshots from pib's camera over rosbridge."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        get_image_service: str = "get_camera_image",
    ) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros)
        self._get_image_service = roslibpy.Service(
            self.ros, get_image_service, "datatypes/GetCameraImage"
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
