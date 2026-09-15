#!/usr/bin/env python3
"""Live robot: trace line art and draw it on a configured surface."""

import argparse
from pathlib import Path

from pib_sdk import Write, pose_from_xyz_rpy
from pib_sdk.features.drawing import DrawingSurface, image_to_sketch, sketch_to_trajectory


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--host", default="pib.local")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--origin", nargs=3, type=float, default=[-350.0, 100.0, 850.0])
    parser.add_argument("--width", type=float, default=120.0)
    parser.add_argument("--height", type=float, default=120.0)
    parser.add_argument("--rate", type=float, default=6.0)
    args = parser.parse_args()

    sketch = image_to_sketch(args.image)
    surface = DrawingSurface(
        pose=pose_from_xyz_rpy(args.origin),
        width_mm=args.width,
        height_mm=args.height,
    )
    trajectory = sketch_to_trajectory(sketch, surface, args.side, points_per_stroke=20)
    print(sketch.point_count, "source points;", len(trajectory), "motor waypoints")
    with Write(args.host) as writer:
        trajectory.play(writer, rate_hz=args.rate)


if __name__ == "__main__":
    main()
