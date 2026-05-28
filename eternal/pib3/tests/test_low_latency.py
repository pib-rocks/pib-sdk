#!/usr/bin/env python3
"""
Test direct Tinkerforge motor control (low-latency mode).

Tests writing and reading servo positions via Tinkerforge bricklets,
which are auto-discovered on connect. Optionally monitors whether
ROS topics receive updates (they should not — direct mode bypasses ROS).

Usage:
    python test_low_latency.py --host 172.26.34.149
    python test_low_latency.py --host 172.26.34.149 --motors elbows
    python test_low_latency.py --host 172.26.34.149 --motors hands-left
    python test_low_latency.py --host 172.26.34.149 --motors all
"""

import argparse
import time
import threading
from typing import Dict, List, Optional

from pib3 import Robot


# Motor groups (safe subsets for targeted testing)
MOTOR_GROUPS = {
    "elbows": ["elbow_left", "elbow_right"],
    "elbows-left": ["elbow_left"],
    "elbows-right": ["elbow_right"],
    "hands": [
        "thumb_left_opposition", "thumb_left_stretch",
        "index_left_stretch", "middle_left_stretch",
        "ring_left_stretch", "pinky_left_stretch",
        "thumb_right_opposition", "thumb_right_stretch",
        "index_right_stretch", "middle_right_stretch",
        "ring_right_stretch", "pinky_right_stretch",
    ],
    "hands-left": [
        "thumb_left_opposition", "thumb_left_stretch",
        "index_left_stretch", "middle_left_stretch",
        "ring_left_stretch", "pinky_left_stretch",
    ],
    "hands-right": [
        "thumb_right_opposition", "thumb_right_stretch",
        "index_right_stretch", "middle_right_stretch",
        "ring_right_stretch", "pinky_right_stretch",
    ],
    "all": [
        "elbow_left", "elbow_right",
        "thumb_left_opposition", "thumb_left_stretch",
        "index_left_stretch", "middle_left_stretch",
        "ring_left_stretch", "pinky_left_stretch",
        "thumb_right_opposition", "thumb_right_stretch",
        "index_right_stretch", "middle_right_stretch",
        "ring_right_stretch", "pinky_right_stretch",
    ],
}


class ROSTopicMonitor:
    """Monitor /joint_trajectory ROS topic for position updates."""

    def __init__(self, robot: Robot, motors: List[str]):
        self.robot = robot
        self.motors = set(motors)
        self.ros_updates: List[Dict] = []
        self.lock = threading.Lock()
        self._subscriber = None

    def start(self):
        try:
            import roslibpy
            topic = roslibpy.Topic(
                self.robot._client,
                "/joint_trajectory",
                "trajectory_msgs/JointTrajectory",
            )
            topic.subscribe(self._on_message)
            self._subscriber = topic
            print("[ROS Monitor] Subscribed to /joint_trajectory")
        except Exception as e:
            print(f"[ROS Monitor] Failed to subscribe: {e}")

    def stop(self):
        if self._subscriber:
            try:
                self._subscriber.unsubscribe()
            except Exception:
                pass
            self._subscriber = None

    def _on_message(self, msg: dict):
        with self.lock:
            joint_names = msg.get("joint_names", [])
            points = msg.get("points", [])
            positions = {}
            for i, name in enumerate(joint_names):
                if name in self.motors:
                    if points and len(points[0].get("positions", [])) > i:
                        positions[name] = points[0]["positions"][i]
            if positions:
                self.ros_updates.append({
                    "timestamp": time.time(),
                    "positions": positions,
                })

    def get_updates_since(self, since: float) -> List[Dict]:
        with self.lock:
            return [u for u in self.ros_updates if u["timestamp"] > since]

    def clear(self):
        with self.lock:
            self.ros_updates.clear()


def print_separator(title: str = ""):
    if title:
        print(f"\n{'='*60}")
        print(f"  {title}")
        print(f"{'='*60}")
    else:
        print("-" * 60)


def check_servo_power_relay(robot: Robot, relay_uid: str) -> bool:
    """Check and enable the solid-state relay that powers the servos."""
    print_separator("Checking Servo Power Relay")

    if robot._tinkerforge_conn is None:
        print("[WARN] Tinkerforge not connected, cannot check relay.")
        return False

    try:
        from tinkerforge.bricklet_solid_state_relay_v2 import BrickletSolidStateRelayV2

        ssr = BrickletSolidStateRelayV2(relay_uid, robot._tinkerforge_conn)
        state = ssr.get_state()
        print(f"  Relay UID: {relay_uid}")
        print(f"  Relay state: {'ON' if state else 'OFF'}")

        if not state:
            print("  [ACTION] Servo power is OFF. Turning ON...")
            ssr.set_state(True)
            time.sleep(0.5)
            state = ssr.get_state()
            print(f"  Relay state after enable: {'ON' if state else 'OFF'}")
            if not state:
                print("  [FAIL] Could not enable servo power relay!")
                return False

        print("  [OK] Servo power is ON")
        return True

    except Exception as e:
        print(f"  [WARN] Could not check relay {relay_uid}: {e}")
        return False


def test_write(
    robot: Robot,
    motor: str,
    positions: List[float],
    monitor: Optional[ROSTopicMonitor] = None,
) -> bool:
    """Write a sequence of positions to a motor and verify."""
    print(f"\n  Testing write to {motor}...")

    for pos in positions:
        try:
            if monitor:
                monitor.clear()
            before = time.time()

            reached = robot.set_joint(motor, pos, unit="percent")
            current = robot.get_joint(motor, unit="percent")
            current_str = f"{current:.1f}" if current is not None else "None"
            status = "OK" if reached else "TIMEOUT"

            ros_status = ""
            if monitor:
                time.sleep(0.1)
                updates = monitor.get_updates_since(before)
                ros_positions = {}
                for u in updates:
                    ros_positions.update(u.get("positions", {}))
                if motor in ros_positions:
                    ros_status = f" | ROS: {ros_positions[motor]:.1f}"
                else:
                    ros_status = " | ROS: no update"

            print(f"    Set {pos:.0f}% -> Tinkerforge: {current_str}% [{status}]{ros_status}")
        except Exception as e:
            print(f"    [ERROR] {e}")
            return False

    return True


def test_read(robot: Robot, motors: List[str]) -> Dict[str, float]:
    """Read positions from motors and report latency."""
    print(f"\n  Reading {len(motors)} motor(s) via Tinkerforge...")

    start = time.perf_counter()
    positions = robot.get_joints(motors, unit="percent")
    elapsed = (time.perf_counter() - start) * 1000

    print(f"    Read completed in {elapsed:.1f}ms")
    for name, pos in positions.items():
        pos_str = f"{pos:.1f}" if pos is not None else "None"
        print(f"    {name}: {pos_str}%")

    return positions


def test_ros_topic_population(
    robot: Robot,
    monitor: ROSTopicMonitor,
    motor: str,
) -> bool:
    """Verify that direct motor commands do NOT propagate to ROS topics."""
    print(f"\n  Testing ROS topic population for {motor}...")

    monitor.clear()
    before = time.time()

    robot.set_joint(motor, 30.0, unit="percent")
    robot.set_joint(motor, 70.0, unit="percent")

    time.sleep(0.2)
    updates = monitor.get_updates_since(before)

    if updates:
        print(f"    [WARN] ROS topic received {len(updates)} update(s) — may cause double commands")
        for u in updates:
            if motor in u["positions"]:
                print(f"           {motor} = {u['positions'][motor]}")
        return True
    else:
        print(f"    [OK] No ROS topic updates (direct mode bypasses ROS)")
        return False


def run_motor_tests(
    robot: Robot,
    motors: List[str],
    monitor: Optional[ROSTopicMonitor] = None,
):
    """Run write/read tests on a list of motors."""
    # Move all to neutral first
    for motor in motors:
        print(f"\n  Moving {motor} to 50% (neutral)...")
        robot.set_joint(motor, 50.0, unit="percent")

    # Write test: full range sweep
    for motor in motors:
        print_separator(f"Testing {motor}")

        current = robot.get_joint(motor, unit="percent")
        current_str = f"{current:.1f}" if current is not None else "None"
        print(f"\n  Current position: {current_str}%")

        success = test_write(robot, motor, [0.0, 100.0, 50.0], monitor)
        print(f"  [{'OK' if success else 'FAIL'}] Write test for {motor}")

    # Batch read test
    print_separator("Batch Read")
    positions = test_read(robot, motors)
    print(f"  [{'OK' if positions else 'FAIL'}] Read test ({len(positions)} motors)")


def main():
    parser = argparse.ArgumentParser(
        description="Test direct Tinkerforge motor control (low-latency mode)"
    )
    parser.add_argument(
        "--host",
        default="172.26.34.149",
        help="Robot IP address (default: 172.26.34.149)",
    )
    parser.add_argument(
        "--motors",
        default="hands",
        choices=list(MOTOR_GROUPS.keys()),
        help="Motor group to test (default: hands)",
    )
    parser.add_argument(
        "--relay-uid",
        default="27Po",
        help="UID of solid state relay bricklet (servo power). Default: 27Po",
    )
    parser.add_argument(
        "--skip-ros-check",
        action="store_true",
        help="Skip ROS topic population test",
    )
    args = parser.parse_args()

    motors = MOTOR_GROUPS[args.motors]

    print_separator(f"Direct Motor Control Test — {args.motors}")
    print(f"Robot host: {args.host}")
    print(f"Motors: {', '.join(motors)}")

    try:
        with Robot(host=args.host) as robot:
            print(f"[OK] Connected to robot at {args.host}")

            # Check servo power
            check_servo_power_relay(robot, args.relay_uid)

            # Show discovered bricklets (already discovered during connect)
            print_separator("Bricklet Discovery")
            uids = robot.discovered_servo_uids
            if uids:
                roles = ["servo1 (right arm)", "servo2 (shoulders/head)", "servo3 (left arm)"]
                print(f"[OK] Found {len(uids)} servo bricklet(s):")
                for role, uid in zip(roles, uids):
                    print(f"  {role}: {uid}")
            else:
                print("[WARN] No servo bricklets found.")

            # Status
            print_separator("Motor Control Status")
            print(f"  low_latency_available: {robot.low_latency_available}")
            print(f"  low_latency_enabled: {robot.low_latency_enabled}")

            # Start ROS monitor
            monitor = None
            if not args.skip_ros_check:
                monitor = ROSTopicMonitor(robot, motors)
                monitor.start()
                time.sleep(0.5)

            # Run tests
            run_motor_tests(robot, motors, monitor)

            # ROS topic population test
            if monitor:
                print_separator("ROS Topic Population Check")
                print("Verifying direct commands do NOT appear in ROS topics...\n")
                ros_populated = test_ros_topic_population(robot, monitor, motors[0])
                monitor.stop()

                if ros_populated:
                    print("\n  [WARN] ROS topics were updated during direct mode.")
                    print("         This may cause double motor commands!")
                else:
                    print("\n  [OK] Direct mode correctly bypasses ROS topics.")

            # Cleanup: return to neutral
            print_separator("Cleanup")
            print("Returning motors to 50%...")
            for motor in motors:
                try:
                    robot.set_joint(motor, 50.0, unit="percent")
                except Exception:
                    pass
            time.sleep(0.5)
            print("[OK] Done")

    except Exception as e:
        print(f"[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
