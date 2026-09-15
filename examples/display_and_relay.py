#!/usr/bin/env python3
"""Live robot: update the display and optionally pulse the relay."""

import argparse
import time
from pathlib import Path

from pib_sdk.features.display import Display, ImageFormat
from pib_sdk.features.relay import Relay


def image_format(path: Path) -> ImageFormat:
    suffix = path.suffix.lower()
    if suffix == ".png":
        return ImageFormat.PNG
    if suffix in {".jpg", ".jpeg"}:
        return ImageFormat.JPEG
    if suffix == ".gif":
        return ImageFormat.ANIMATED_GIF
    raise ValueError("image extension must be .png, .jpg/.jpeg, or .gif")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="pib.local")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--enable-relay", action="store_true")
    args = parser.parse_args()

    with Display(args.host) as display:
        if args.image:
            display.show_custom(args.image.read_bytes(), format=image_format(args.image))
        else:
            display.show_animated_eyes()

    if args.enable_relay:
        with Relay(args.host) as relay:
            if not relay.set(True):
                raise RuntimeError("relay-on command was rejected")
            time.sleep(1)
            if not relay.set(False):
                raise RuntimeError("relay-off command was rejected")


if __name__ == "__main__":
    main()
