#!/usr/bin/env python3
"""
Read actual servo calibration from Tinkerforge bricklets on the robot.

Connects directly to Tinkerforge Brick Daemon (bypassing the Robot class)
to read the hardware-configured degree ranges, pulse widths, and motion
configuration for each servo channel.

This avoids the auto-configure step that would overwrite the values.

Usage:
    python read_servo_calibration.py --host 172.26.34.149

The output can be used to update pib3/resources/joint_limits_robot.yaml.
"""

import argparse
import math
import time
from typing import Dict, List, Tuple

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_servo_v2 import BrickletServoV2

# Import channel mapping from pib3
from pib3.backends.robot import PIB_SERVO_CHANNELS


def read_bricklet_config(
    servo: BrickletServoV2,
    uid: str,
    label: str,
    channels: List[int],
) -> Dict[int, dict]:
    """Read configuration from all channels of a servo bricklet."""
    print(f"\n  Servo Bricklet: {uid} ({label})")
    print(f"  {'Ch':>3}  {'Degree Min':>10}  {'Degree Max':>10}  {'PW Min':>7}  {'PW Max':>7}  {'Velocity':>8}  {'Accel':>8}  {'Decel':>8}  {'Enabled':>7}  {'Position':>8}")
    print(f"  {'-'*3}  {'-'*10}  {'-'*10}  {'-'*7}  {'-'*7}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*7}  {'-'*8}")

    results = {}
    for ch in channels:
        try:
            degree = servo.get_degree(ch)
            pw = servo.get_pulse_width(ch)
            motion = servo.get_motion_configuration(ch)
            enabled = servo.get_enabled(ch)
            position = servo.get_position(ch)

            results[ch] = {
                "degree_min": degree.min,
                "degree_max": degree.max,
                "pw_min": pw.min,
                "pw_max": pw.max,
                "velocity": motion.velocity,
                "acceleration": motion.acceleration,
                "deceleration": motion.deceleration,
                "enabled": enabled,
                "position": position,
            }

            print(
                f"  {ch:>3}  {degree.min:>10}  {degree.max:>10}  "
                f"{pw.min:>7}  {pw.max:>7}  "
                f"{motion.velocity:>8}  {motion.acceleration:>8}  {motion.deceleration:>8}  "
                f"{'ON' if enabled else 'OFF':>7}  {position:>8}"
            )
        except Exception as e:
            print(f"  {ch:>3}  ERROR: {e}")

    return results


def build_joint_limits(
    bricklet_configs: Dict[str, Dict[int, dict]],
    servo_uids: Dict[str, str],
) -> Dict[str, dict]:
    """Map bricklet channel configs to motor names with degree ranges.

    Args:
        bricklet_configs: {uid: {channel: config_dict}}
        servo_uids: {"servo1": uid, "servo2": uid, "servo3": uid}

    Returns:
        {motor_name: {"degree_min": ..., "degree_max": ..., "min_rad": ..., "max_rad": ...}}
    """
    # Build reverse mapping: motor_name -> (bricklet_key, channel)
    motor_to_bricklet = {}
    servo1 = servo_uids["servo1"]
    servo2 = servo_uids["servo2"]
    servo3 = servo_uids["servo3"]

    for motor_name, (bricklet_num, channel) in PIB_SERVO_CHANNELS.items():
        if bricklet_num == 1:
            uid = servo1
        elif bricklet_num == 2:
            uid = servo2
        elif bricklet_num == 3:
            uid = servo3
        else:
            continue

        motor_to_bricklet[motor_name] = (uid, channel)

    # Build joint limits from hardware config
    joint_limits = {}
    for motor_name, (uid, channel) in motor_to_bricklet.items():
        config = bricklet_configs.get(uid, {}).get(channel)
        if config is None:
            continue

        degree_min = config["degree_min"]  # centidegrees
        degree_max = config["degree_max"]  # centidegrees

        min_rad = math.radians(degree_min / 100.0)
        max_rad = math.radians(degree_max / 100.0)

        joint_limits[motor_name] = {
            "degree_min_centideg": degree_min,
            "degree_max_centideg": degree_max,
            "min_deg": degree_min / 100.0,
            "max_deg": degree_max / 100.0,
            "min_rad": min_rad,
            "max_rad": max_rad,
        }

    return joint_limits


def print_yaml_update(joint_limits: Dict[str, dict]):
    """Print the joint limits in YAML format for joint_limits_robot.yaml."""
    print("\n" + "=" * 60)
    print("  Suggested joint_limits_robot.yaml update")
    print("=" * 60)
    print()
    print("joints:")

    # Group by category
    categories = {
        "Head joints": ["turn_head_motor", "tilt_forward_motor"],
        "Left arm joints": [
            "shoulder_vertical_left", "shoulder_horizontal_left",
            "upper_arm_left_rotation", "elbow_left",
            "lower_arm_left_rotation", "wrist_left",
        ],
        "Left hand joints": [
            "thumb_left_opposition", "thumb_left_stretch",
            "index_left_stretch", "middle_left_stretch",
            "ring_left_stretch", "pinky_left_stretch",
        ],
        "Right arm joints": [
            "shoulder_vertical_right", "shoulder_horizontal_right",
            "upper_arm_right_rotation", "elbow_right",
            "lower_arm_right_rotation",
        ],
        "Right hand joints": [
            "thumb_right_opposition", "thumb_right_stretch",
            "index_right_stretch", "middle_right_stretch",
            "ring_right_stretch", "pinky_right_stretch",
        ],
    }

    for category, motor_names in categories.items():
        print(f"  # {category}")
        for name in motor_names:
            if name in joint_limits:
                lim = joint_limits[name]
                min_deg = lim["min_deg"]
                max_deg = lim["max_deg"]
                print(f"  {name}:")
                print(f"    min: {lim['min_rad']:.16f}  # {min_deg:.0f} degrees")
                print(f"    max: {lim['max_rad']:.16f}  # {max_deg:.0f} degrees")
            else:
                print(f"  # {name}: NOT READ (not in bricklet mapping)")
        print()


def main():
    parser = argparse.ArgumentParser(
        description="Read servo calibration from Tinkerforge bricklets"
    )
    parser.add_argument(
        "--host",
        default="172.26.34.149",
        help="Robot IP address (default: 172.26.34.149)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=4223,
        help="Tinkerforge Brick Daemon port (default: 4223)",
    )
    parser.add_argument(
        "--servo1-uid",
        default="2cPP",
        help="UID of servo bricklet 1 (right arm). Default: 2cPP",
    )
    parser.add_argument(
        "--servo2-uid",
        default="2cPm",
        help="UID of servo bricklet 2 (shoulders). Default: 2cPm",
    )
    parser.add_argument(
        "--servo3-uid",
        default="2cPQ",
        help="UID of servo bricklet 3 (left arm). Default: 2cPQ",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  Servo Calibration Readout")
    print("=" * 60)
    print(f"Host: {args.host}:{args.port}")
    print(f"Servo 1 (right arm): {args.servo1_uid}")
    print(f"Servo 2 (shoulders): {args.servo2_uid}")
    print(f"Servo 3 (left arm):  {args.servo3_uid}")

    # Connect directly to Tinkerforge (no Robot class, no auto-configure)
    ipcon = IPConnection()
    try:
        ipcon.connect(args.host, args.port)
        print(f"\n[OK] Connected to Tinkerforge at {args.host}:{args.port}")
    except Exception as e:
        print(f"\n[ERROR] Cannot connect to Tinkerforge: {e}")
        return 1

    time.sleep(0.2)

    bricklet_configs = {}

    # Read all 10 channels (0-9) from each bricklet
    all_channels = list(range(10))

    bricklets = [
        (args.servo1_uid, "Servo 1 - right arm"),
        (args.servo2_uid, "Servo 2 - shoulders"),
        (args.servo3_uid, "Servo 3 - left arm"),
    ]

    for uid, label in bricklets:
        try:
            servo = BrickletServoV2(uid, ipcon)
            config = read_bricklet_config(servo, uid, label, all_channels)
            bricklet_configs[uid] = config
        except Exception as e:
            print(f"\n  [ERROR] Cannot read bricklet {uid} ({label}): {e}")

    # Map to motor names
    servo_uids = {
        "servo1": args.servo1_uid,
        "servo2": args.servo2_uid,
        "servo3": args.servo3_uid,
    }
    joint_limits = build_joint_limits(bricklet_configs, servo_uids)

    # Print per-motor summary
    print("\n" + "=" * 60)
    print("  Per-Motor Degree Ranges (from hardware)")
    print("=" * 60)
    print(f"\n  {'Motor':<30}  {'Min (°)':>8}  {'Max (°)':>8}  {'Range (°)':>9}  {'Min (rad)':>12}  {'Max (rad)':>12}")
    print(f"  {'-'*30}  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*12}  {'-'*12}")

    for name in sorted(joint_limits.keys()):
        lim = joint_limits[name]
        range_deg = lim["max_deg"] - lim["min_deg"]
        print(
            f"  {name:<30}  {lim['min_deg']:>8.1f}  {lim['max_deg']:>8.1f}  "
            f"{range_deg:>9.1f}  {lim['min_rad']:>12.6f}  {lim['max_rad']:>12.6f}"
        )

    # Print YAML update
    print_yaml_update(joint_limits)

    ipcon.disconnect()
    print("[OK] Done")
    return 0


if __name__ == "__main__":
    exit(main())
