#!/usr/bin/env python3
"""
Basic Joint Control Example

This script demonstrates fundamental joint control operations:
1. Reading joint positions
2. Setting single joint positions
3. Setting multiple joints simultaneously
4. Synchronous vs asynchronous movement
5. Saving and restoring poses

Requirements:
    pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"

Usage:
    python basic_joint_control.py --host 172.26.34.149
    python basic_joint_control.py --demo read
    python basic_joint_control.py --demo write
    python basic_joint_control.py --demo pose

Note:
    This example works with both the physical robot and Webots simulation.
    For Webots, use the appropriate host/port for the simulation environment.
"""

import argparse
import json
import time
from pathlib import Path

try:
    from pib3 import Robot, Joint
    HAS_PIB3 = True
except ImportError:
    HAS_PIB3 = False


def demo_read_joints(robot):
    """Demonstrate reading joint positions."""
    print("\n=== Reading Joint Positions ===")

    # Read a single joint
    print("\n1. Reading single joint (TURN_HEAD):")
    head_pos = robot.get_joint(Joint.TURN_HEAD)
    if head_pos is not None:
        print(f"   Head position: {head_pos:.1f}%")
    else:
        print("   Could not read head position (timeout)")

    # Read in radians
    print("\n2. Reading same joint in radians:")
    head_rad = robot.get_joint(Joint.TURN_HEAD, unit="rad")
    if head_rad is not None:
        print(f"   Head position: {head_rad:.3f} rad")

    # Read multiple specific joints
    print("\n3. Reading multiple joints:")
    arm_joints = [Joint.SHOULDER_VERTICAL_LEFT, Joint.SHOULDER_HORIZONTAL_LEFT, Joint.ELBOW_LEFT]
    positions = robot.get_joints(arm_joints)
    for name, pos in positions.items():
        print(f"   {name}: {pos:.1f}%")

    # Read all joints
    print("\n4. Reading ALL joints:")
    all_joints = robot.get_joints()
    print(f"   Found {len(all_joints)} joints:")
    for name, pos in sorted(all_joints.items()):
        print(f"   {name}: {pos:.1f}%")


def demo_write_joints(robot):
    """Demonstrate setting joint positions."""
    print("\n=== Setting Joint Positions ===")

    # Set a single joint (async - returns immediately)
    print("\n1. Moving head left (async):")
    robot.set_joint(Joint.TURN_HEAD, 70.0)
    print("   Command sent! (not waiting for completion)")
    time.sleep(1.0)

    # Set a single joint (sync - waits for completion)
    print("\n2. Moving head right (sync - waiting for completion):")
    success = robot.set_joint(Joint.TURN_HEAD, 30.0, async_=False, timeout=3.0)
    if success:
        print("   Joint reached target position!")
    else:
        print("   Timeout waiting for joint to reach position")

    # Center head
    print("\n3. Centering head:")
    robot.set_joint(Joint.TURN_HEAD, 50.0, async_=False, timeout=2.0)
    print("   Head centered")

    # Set multiple joints at once
    print("\n4. Moving multiple joints simultaneously:")
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 40.0,
        Joint.ELBOW_LEFT: 60.0,
    })
    print("   Commands sent for shoulder and elbow")
    time.sleep(1.5)

    # Return to neutral
    print("\n5. Returning joints to neutral (50%):")
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 50.0,
        Joint.ELBOW_LEFT: 50.0,
    }, async_=False, timeout=3.0)
    print("   Done!")


def demo_pose_management(robot):
    """Demonstrate saving and restoring poses."""
    print("\n=== Pose Management ===")

    pose_file = Path("saved_pose.json")

    # Save current pose
    print("\n1. Saving current pose to file:")
    current_pose = robot.get_joints()
    with open(pose_file, "w") as f:
        json.dump(current_pose, f, indent=2)
    print(f"   Saved {len(current_pose)} joint positions to {pose_file}")

    # Move to a different pose
    print("\n2. Moving to a test pose:")
    robot.set_joints({
        Joint.TURN_HEAD: 70.0,
        Joint.SHOULDER_VERTICAL_LEFT: 30.0,
        Joint.ELBOW_LEFT: 70.0,
    }, async_=False, timeout=3.0)
    print("   Moved to test pose")
    time.sleep(1.0)

    # Restore saved pose
    print("\n3. Restoring saved pose from file:")
    with open(pose_file) as f:
        saved_pose = json.load(f)

    # Only restore the joints we moved (file uses string keys)
    joints_to_restore = {
        k: v for k, v in saved_pose.items()
        if k in [Joint.TURN_HEAD.value, Joint.SHOULDER_VERTICAL_LEFT.value, Joint.ELBOW_LEFT.value]
    }
    robot.set_joints(joints_to_restore, async_=False, timeout=3.0)
    print("   Pose restored!")

    # Clean up
    pose_file.unlink(missing_ok=True)
    print(f"\n   Cleaned up {pose_file}")


def demo_wave(robot):
    """Fun demo: make the robot wave."""
    print("\n=== Wave Demo ===")
    print("Making the robot wave!")

    # Raise arm
    print("   Raising arm...")
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 20.0,
        Joint.SHOULDER_HORIZONTAL_LEFT: 30.0,
        Joint.ELBOW_LEFT: 30.0,
    }, async_=False, timeout=2.0)

    # Wave motion
    print("   Waving...")
    for i in range(3):
        robot.set_joint(Joint.ELBOW_LEFT, 20.0, async_=False, timeout=1.0)
        robot.set_joint(Joint.ELBOW_LEFT, 40.0, async_=False, timeout=1.0)

    # Return to neutral
    print("   Returning to neutral...")
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 50.0,
        Joint.SHOULDER_HORIZONTAL_LEFT: 50.0,
        Joint.ELBOW_LEFT: 50.0,
    }, async_=False, timeout=2.0)

    print("   Done waving!")


def main():
    parser = argparse.ArgumentParser(
        description="Basic Joint Control Example",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python basic_joint_control.py --host 172.26.34.149
  python basic_joint_control.py --demo read
  python basic_joint_control.py --demo write
  python basic_joint_control.py --demo pose
  python basic_joint_control.py --demo wave
        """
    )
    parser.add_argument(
        "--host",
        default="172.26.34.149",
        help="Robot IP address (default: 172.26.34.149)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9090,
        help="Rosbridge port (default: 9090)"
    )
    parser.add_argument(
        "--demo",
        choices=["read", "write", "pose", "wave", "all"],
        default="all",
        help="Which demo to run (default: all)"
    )
    args = parser.parse_args()

    if not HAS_PIB3:
        print("Error: pib3 not installed.")
        print("Install with: pip install 'pib3[robot] @ git+https://github.com/mamrehn/pib3.git'")
        return

    print(f"Connecting to robot at {args.host}:{args.port}...")

    try:
        with Robot(host=args.host, port=args.port) as robot:
            print(f"Connected: {robot.is_connected}")

            if args.demo in ("read", "all"):
                demo_read_joints(robot)

            if args.demo in ("write", "all"):
                demo_write_joints(robot)

            if args.demo in ("pose", "all"):
                demo_pose_management(robot)

            if args.demo in ("wave", "all"):
                demo_wave(robot)

            print("\n=== All demos complete ===")

    except ConnectionError as e:
        print(f"Connection failed: {e}")
        print("\nTroubleshooting:")
        print("1. Check robot is powered on and connected to network")
        print("2. Verify rosbridge_server is running on the robot")
        print(f"3. Confirm IP address is correct: {args.host}")
    except KeyboardInterrupt:
        print("\nInterrupted by user")


if __name__ == "__main__":
    main()
