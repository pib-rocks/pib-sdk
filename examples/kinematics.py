#!/usr/bin/env python3
"""Offline FK/IK example for one pib arm."""

import argparse

from pib_sdk import ArmKinematics


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--xyz", nargs=3, type=float, default=[-400.0, 100.0, 900.0])
    args = parser.parse_args()

    arm = ArmKinematics(args.side)
    lower, upper = arm.joint_limits_deg
    q_deg = arm.inverse(args.xyz)
    reached = arm.forward(q_deg).translation

    print("motor order:", arm.motor_names)
    print("limits (degrees):", list(zip(lower, upper)))
    print("solution (degrees):", q_deg)
    print("reached XYZ (millimetres):", reached)


if __name__ == "__main__":
    main()
