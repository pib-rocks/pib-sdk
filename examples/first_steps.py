#!/usr/bin/env python3
"""Live robot: inspect telemetry and move one motor cautiously."""

import argparse
import time

from pib_sdk.robot import Robot


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="pib.local")
    parser.add_argument("--motor", default="elbow_right")
    parser.add_argument("--angle", type=float, default=10.0)
    args = parser.parse_args()

    with Robot(host=args.host) as robot:
        print("last commanded degrees:", robot.telemetry.get_position_deg(args.motor))
        try:
            print("current milliamps:", robot.telemetry.get_current_ma(args.motor))
        except TimeoutError as error:
            print(error)
        if not robot.write.set(args.motor, default=True, velocity=3000):
            raise RuntimeError("motor settings were rejected")
        if not robot.write.move(args.motor, args.angle):
            raise RuntimeError("move was rejected")
        time.sleep(1)
        if not robot.write.move(args.motor, 0.0):
            raise RuntimeError("return-to-zero move was rejected")


if __name__ == "__main__":
    main()
