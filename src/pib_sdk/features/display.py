"""Push images to pib's physical display, over rosbridge.

pib has a small screen driven by the ``display_image`` topic -- fire-and-
forget, publish and it shows, no request/response. There's one built-in
animation (pib's default animated eyes) plus support for a custom PNG/JPEG/
animated GIF you supply as bytes.

Example
-------
    from pathlib import Path
    from pib_sdk.features.display import Display, ImageFormat

    display = Display(host="localhost")
    display.show_animated_eyes()
    display.show_custom(Path("logo.png").read_bytes(), format=ImageFormat.PNG)
    display.clear()
    display.close()
"""

from __future__ import annotations

import base64
import time
from enum import IntEnum

import roslibpy

# Mirrors pib-backend's ImageId.msg -- PIB_EYES_ANIMATED is the one built-in image.
_IMAGE_ID_NONE = 0
_IMAGE_ID_CUSTOM = 1
_IMAGE_ID_PIB_EYES_ANIMATED = 2


class ImageFormat(IntEnum):
    """Mirrors pib-backend's ImageFormat.msg."""

    ANIMATED_GIF = 0
    PNG = 1
    JPEG = 2


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


class Display:
    """Publishes images to pib's screen over rosbridge."""

    def __init__(
        self, host: str = "localhost", port: int = 9090, display_image_topic: str = "display_image"
    ) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros)
        self._display_image_topic = roslibpy.Topic(
            self.ros, display_image_topic, "datatypes/DisplayImage"
        )

    def __enter__(self) -> Display:
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

    def show_animated_eyes(self) -> None:
        """Show pib's built-in animated-eyes image."""
        self._publish(image_id=_IMAGE_ID_PIB_EYES_ANIMATED, image_format=ImageFormat.ANIMATED_GIF)

    def show_custom(self, image_bytes: bytes, *, format: ImageFormat) -> None:
        """Show a custom image. ``format`` must match ``image_bytes``' actual encoding."""
        self._publish(image_id=_IMAGE_ID_CUSTOM, image_format=format, data=image_bytes)

    def clear(self) -> None:
        """Show nothing."""
        self._publish(image_id=_IMAGE_ID_NONE, image_format=ImageFormat.PNG)

    def _publish(
        self, *, image_id: int, image_format: ImageFormat, data: bytes = b""
    ) -> None:
        # pib-backend's `byte[] data` field follows rosbridge's convention of
        # transmitting uint8[]/byte[] arrays as base64-encoded strings over JSON.
        message = {
            "id": {"value": image_id},
            "format": {"value": int(image_format)},
            "data": base64.b64encode(data).decode("ascii"),
        }
        self._display_image_topic.publish(roslibpy.Message(message))
