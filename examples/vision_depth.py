#!/usr/bin/env python3
"""Live robot: save a color snapshot and uint16 millimetre depth frame."""

import argparse
from pathlib import Path

from pib_sdk.features.camera import Camera


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="pib.local")
    parser.add_argument("--output-dir", type=Path, default=Path("."))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    with Camera(args.host) as camera:
        camera.save_snapshot(args.output_dir / "frame.jpg")
        depth = camera.get_depth_frame()
        if depth is None:
            raise RuntimeError("no depth frame is available")
        valid = depth[depth != 0]
        print("shape:", depth.shape, "valid pixels:", valid.size)
        print("nearest millimetres:", int(valid.min()) if valid.size else "none")
        x, y = depth.shape[1] // 2, depth.shape[0] // 2
        print("center millimetres:", camera.get_distance_at_px(x, y))
        camera.save_depth(args.output_dir / "depth.npy")


if __name__ == "__main__":
    main()
