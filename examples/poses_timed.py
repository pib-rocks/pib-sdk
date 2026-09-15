#!/usr/bin/env python3
"""Live robot: optionally save a pose, then play a smooth timed sequence."""

import argparse

from pib_sdk import Write
from pib_sdk.backend import BackendClient
from pib_sdk.features.poses import play_pose_sequence_timed, save_current_pose
from pib_sdk.robot_model import get_arm_model
from pib_sdk.telemetry import Telemetry


def scheduled_pose(value: str) -> tuple[str, float]:
    name, separator, seconds = value.rpartition(":")
    if not separator:
        raise argparse.ArgumentTypeError("expected POSE_NAME:LEG_SECONDS")
    return name, float(seconds)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("schedule", nargs="+", type=scheduled_pose)
    parser.add_argument("--host", default="pib.local")
    parser.add_argument("--save-name")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    args = parser.parse_args()

    backend = BackendClient(args.host)
    if args.save_name:
        with Telemetry(args.host) as telemetry:
            pose = save_current_pose(
                telemetry, backend, args.save_name, get_arm_model(args.side).motor_names
            )
            print("saved", pose.name, pose.pose_id)

    with Write(args.host) as writer:
        play_pose_sequence_timed(writer, backend, args.schedule)


if __name__ == "__main__":
    main()
