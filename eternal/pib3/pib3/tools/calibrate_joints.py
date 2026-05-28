#!/usr/bin/env python3
"""
Joint Calibration Tool for PIB Robot.

This script guides you through calibrating the min/max positions for each joint.
Use Cerebra or another control interface to manually move joints to their limits,
then this script reads and records the values.

Usage:
    python -m pib3.tools.calibrate_joints --host 172.26.34.149

    # Calibrate only specific joints:
    python -m pib3.tools.calibrate_joints --joints elbow_left wrist_left

    # Calibrate only left hand:
    python -m pib3.tools.calibrate_joints --group left_hand
"""

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import yaml


# Joint groups for convenient selection
JOINT_GROUPS = {
    "head": [
        "turn_head_motor",
        "tilt_forward_motor",
    ],
    "left_arm": [
        "shoulder_vertical_left",
        "shoulder_horizontal_left",
        "upper_arm_left_rotation",
        "elbow_left",
        "lower_arm_left_rotation",
        "wrist_left",
    ],
    "left_hand": [
        "thumb_left_opposition",
        "thumb_left_stretch",
        "index_left_stretch",
        "middle_left_stretch",
        "ring_left_stretch",
        "pinky_left_stretch",
    ],
    "right_arm": [
        "shoulder_vertical_right",
        "shoulder_horizontal_right",
        "upper_arm_right_rotation",
        "elbow_right",
        "lower_arm_right_rotation",
        "wrist_right",
    ],
    "right_hand": [
        "thumb_right_opposition",
        "thumb_right_stretch",
        "index_right_stretch",
        "middle_right_stretch",
        "ring_right_stretch",
        "pinky_right_stretch",
    ],
}

ALL_JOINTS = (
    JOINT_GROUPS["head"]
    + JOINT_GROUPS["left_arm"]
    + JOINT_GROUPS["left_hand"]
    + JOINT_GROUPS["right_arm"]
    + JOINT_GROUPS["right_hand"]
)


def get_config_path() -> Path:
    """Get path to the joint_limits_robot.yaml config file."""
    return Path(__file__).parent.parent / "resources" / "joint_limits_robot.yaml"


def load_existing_config() -> Dict:
    """Load existing joint limits config."""
    config_path = get_config_path()
    if config_path.exists():
        with open(config_path, "r") as f:
            return yaml.safe_load(f) or {}
    return {}


def save_config(config: Dict) -> None:
    """Save joint limits config to file and clear cache."""
    config_path = get_config_path()

    # Build the YAML content with comments
    lines = [
        "# PIB Robot Joint Limits for Real Robot",
        "# Calibrated using: python -m pib3.tools.calibrate_joints",
        "#",
        "# These values define the mapping from percentage (0-100%) to radians.",
        "# 0% = min (radians), 100% = max (radians)",
        "#",
        "# Values are in radians. Joints with 'null' are not calibrated.",
        "",
        "joints:",
    ]

    joints = config.get("joints", {})

    # Group joints by category for readability
    categories = [
        ("Head joints", JOINT_GROUPS["head"]),
        ("Left arm joints", JOINT_GROUPS["left_arm"]),
        ("Left hand joints", JOINT_GROUPS["left_hand"]),
        ("Right arm joints", JOINT_GROUPS["right_arm"]),
        ("Right hand joints", JOINT_GROUPS["right_hand"]),
    ]

    for category_name, joint_list in categories:
        lines.append(f"  # {category_name}")
        for joint in joint_list:
            if joint in joints:
                lim = joints[joint]
                min_val = lim.get("min", 0.0)
                max_val = lim.get("max", 0.0)
                # Format with appropriate precision
                if min_val is None:
                    min_str = "null  # Not calibrated"
                else:
                    min_str = f"{min_val:.4f}"
                if max_val is None:
                    max_str = "null  # Not calibrated"
                else:
                    max_str = f"{max_val:.4f}"
                lines.append(f"  {joint}:")
                lines.append(f"    min: {min_str}")
                lines.append(f"    max: {max_str}")
            else:
                lines.append(f"  {joint}:")
                lines.append(f"    min: null  # Not calibrated")
                lines.append(f"    max: null  # Not calibrated")
        lines.append("")

    with open(config_path, "w") as f:
        f.write("\n".join(lines))

    # Clear the joint limits cache so new values are used immediately
    try:
        from pib3.backends.base import clear_joint_limits_cache
        clear_joint_limits_cache()
    except ImportError:
        pass

    print(f"\nConfig saved to: {config_path}")


def wait_for_position_reading(
    robot,
    joint_name: str,
    prompt: str,
    timeout: float = 60.0,
) -> Optional[float]:
    """
    Wait for user to position joint and press Enter, then read position.

    Args:
        robot: Connected RealRobotBackend instance.
        joint_name: Name of joint to read.
        prompt: Instructions to show user.
        timeout: Max time to wait for stable reading.

    Returns:
        Position in radians, or None if cancelled/failed.
    """
    print(f"\n{prompt}")
    print(f"  Joint: {joint_name}")
    print("\n  [Press Enter when ready, or 's' to skip this joint]")

    response = input("  > ").strip().lower()
    if response == 's':
        print("  Skipping...")
        return None

    # Take multiple readings and average for stability
    readings = []
    print("  Reading position", end="", flush=True)
    for _ in range(5):
        pos = robot._get_joint_radians(joint_name)
        if pos is not None:
            readings.append(pos)
        print(".", end="", flush=True)
        time.sleep(0.2)
    print()

    if not readings:
        print(f"  WARNING: Could not read position for {joint_name}")
        return None

    avg_pos = sum(readings) / len(readings)
    print(f"  Read: {avg_pos:.4f} rad ({avg_pos * 180 / math.pi:.1f} deg)")
    return avg_pos


def calibrate_joint(
    robot,
    joint_name: str,
    existing_min: Optional[float] = None,
    existing_max: Optional[float] = None,
) -> Dict[str, Optional[float]]:
    """
    Calibrate a single joint's min/max positions.

    Returns:
        Dict with 'min' and 'max' keys (values may be None if skipped).
    """
    print("\n" + "=" * 60)
    print(f"  CALIBRATING: {joint_name}")
    print("=" * 60)

    if existing_min is not None or existing_max is not None:
        print(f"  Current values: min={existing_min}, max={existing_max}")
        print("  [Press Enter to recalibrate, or 'k' to keep existing values]")
        response = input("  > ").strip().lower()
        if response == 'k':
            print("  Keeping existing values.")
            return {"min": existing_min, "max": existing_max}

    # Calibrate minimum
    min_val = wait_for_position_reading(
        robot,
        joint_name,
        "Move joint to MINIMUM position using Cerebra.",
    )

    # Calibrate maximum
    max_val = wait_for_position_reading(
        robot,
        joint_name,
        "Move joint to MAXIMUM position using Cerebra.",
    )

    # Validate
    if min_val is not None and max_val is not None:
        if min_val > max_val:
            print(f"\n  NOTE: min ({min_val:.4f}) > max ({max_val:.4f})")
            print("  This is expected for some joints (e.g. finger joints).")
            print("  Keeping values as recorded.")

    return {"min": min_val, "max": max_val}


def run_calibration(
    host: str,
    port: int,
    joints_to_calibrate: List[str],
) -> None:
    """Run the interactive calibration process."""

    # Import here to avoid import errors if roslibpy not installed
    try:
        from pib3.backends import RealRobotBackend
    except ImportError as e:
        print(f"Error: {e}")
        print("Install with: pip install pib3[robot]")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("  PIB Robot Joint Calibration Tool")
    print("=" * 60)
    print(f"\nConnecting to robot at {host}:{port}...")

    robot = RealRobotBackend(host=host, port=port)
    try:
        robot.connect()
    except Exception as e:
        print(f"Error connecting to robot: {e}")
        sys.exit(1)

    if not robot.is_connected:
        print("Failed to connect to robot.")
        sys.exit(1)

    print("Connected!")
    print("\nWaiting for motor position data...")
    time.sleep(2.0)

    # Load existing config
    config = load_existing_config()
    if "joints" not in config:
        config["joints"] = {}

    print(f"\nJoints to calibrate: {len(joints_to_calibrate)}")
    print("For each joint, you will:")
    print("  1. Move it to MINIMUM position using Cerebra")
    print("  2. Press Enter to record")
    print("  3. Move it to MAXIMUM position using Cerebra")
    print("  4. Press Enter to record")
    print("\nYou can press 's' to skip any joint.")
    print("\n[Press Enter to begin]")
    input()

    # Calibrate each joint
    calibrated = {}
    try:
        for i, joint_name in enumerate(joints_to_calibrate, 1):
            print(f"\n[{i}/{len(joints_to_calibrate)}]")

            existing = config["joints"].get(joint_name, {})
            existing_min = existing.get("min")
            existing_max = existing.get("max")

            result = calibrate_joint(
                robot,
                joint_name,
                existing_min,
                existing_max,
            )
            calibrated[joint_name] = result

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted.")
    finally:
        robot.disconnect()

    # Show summary
    print("\n" + "=" * 60)
    print("  CALIBRATION SUMMARY")
    print("=" * 60)

    for joint_name, values in calibrated.items():
        min_val = values.get("min")
        max_val = values.get("max")
        min_str = f"{min_val:.4f}" if min_val is not None else "N/A"
        max_str = f"{max_val:.4f}" if max_val is not None else "N/A"
        range_str = ""
        if min_val is not None and max_val is not None:
            range_deg = (max_val - min_val) * 180 / math.pi
            range_str = f" (range: {range_deg:.1f} deg)"
        print(f"  {joint_name}: min={min_str}, max={max_str}{range_str}")

    # Ask to save
    if calibrated:
        print("\n" + "-" * 60)
        print("Save calibrated values to config file?")
        print("  [y] Yes, save")
        print("  [n] No, discard")
        response = input("  > ").strip().lower()

        if response == 'y':
            # Merge calibrated values into config
            for joint_name, values in calibrated.items():
                if joint_name not in config["joints"]:
                    config["joints"][joint_name] = {}
                if values.get("min") is not None:
                    config["joints"][joint_name]["min"] = values["min"]
                if values.get("max") is not None:
                    config["joints"][joint_name]["max"] = values["max"]

            save_config(config)
            print("Calibration complete!")
        else:
            print("Changes discarded.")
    else:
        print("\nNo joints were calibrated.")


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate PIB robot joint limits interactively.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Calibrate all joints
  python -m pib3.tools.calibrate_joints --host 172.26.34.149

  # Calibrate only left hand
  python -m pib3.tools.calibrate_joints --host 172.26.34.149 --group left_hand

  # Calibrate specific joints
  python -m pib3.tools.calibrate_joints --host 172.26.34.149 --joints elbow_left wrist_left

  # List available joints and groups
  python -m pib3.tools.calibrate_joints --list
        """,
    )

    parser.add_argument(
        "--host",
        default="172.26.34.149",
        help="Robot IP address (default: 172.26.34.149)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9090,
        help="Rosbridge port (default: 9090)",
    )
    parser.add_argument(
        "--joints",
        nargs="+",
        help="Specific joints to calibrate",
    )
    parser.add_argument(
        "--group",
        choices=list(JOINT_GROUPS.keys()),
        help="Calibrate a joint group (head, left_arm, left_hand, right_arm, right_hand)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Calibrate all joints",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available joints and groups, then exit",
    )

    args = parser.parse_args()

    # Handle --list
    if args.list:
        print("\nAvailable joint groups:")
        for group_name, joints in JOINT_GROUPS.items():
            print(f"\n  {group_name}:")
            for joint in joints:
                print(f"    - {joint}")
        print(f"\nTotal joints: {len(ALL_JOINTS)}")
        return

    # Determine which joints to calibrate
    if args.joints:
        joints_to_calibrate = args.joints
        # Validate joint names
        invalid = [j for j in joints_to_calibrate if j not in ALL_JOINTS]
        if invalid:
            print(f"Error: Unknown joints: {invalid}")
            print("Use --list to see available joints.")
            sys.exit(1)
    elif args.group:
        joints_to_calibrate = JOINT_GROUPS[args.group]
    elif args.all:
        joints_to_calibrate = ALL_JOINTS
    else:
        # Default: show help
        parser.print_help()
        print("\n\nQuick start:")
        print(f"  python -m pib3.tools.calibrate_joints --host {args.host} --group left_hand")
        return

    run_calibration(args.host, args.port, joints_to_calibrate)


if __name__ == "__main__":
    main()
