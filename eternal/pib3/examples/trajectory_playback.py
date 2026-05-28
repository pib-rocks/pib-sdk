#!/usr/bin/env python3
"""
Trajectory Playback Example

This script demonstrates trajectory generation and playback:
1. Generating a trajectory from an image
2. Previewing trajectory information
3. Playing back a trajectory on the robot
4. Using progress callbacks
5. Saving and loading trajectories

Requirements:
    pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"

Usage:
    python trajectory_playback.py --host 172.26.34.149
    python trajectory_playback.py --image my_drawing.png
    python trajectory_playback.py --trajectory saved_trajectory.json

Note:
    This example works with both the physical robot and Webots simulation.
    For Webots, use the appropriate host/port for the simulation environment.
"""

import argparse
import json
import time
from pathlib import Path

try:
    from pib3 import Robot, Joint, generate_trajectory, TrajectoryConfig, Trajectory
    HAS_PIB3 = True
except ImportError:
    HAS_PIB3 = False


def demo_generate_trajectory(image_path: Path):
    """Demonstrate generating a trajectory from an image."""
    print("\n=== Generating Trajectory ===")

    if not image_path.exists():
        print(f"Error: Image not found: {image_path}")
        return None

    print(f"Processing image: {image_path}")

    # Create configuration
    config = TrajectoryConfig()

    # Adjust paper settings (optional)
    config.paper.size = 0.10  # 10cm drawing area
    config.paper.start_x = 0.32
    config.paper.center_y = -0.22
    config.paper.height_z = 0.83

    # Generate trajectory
    print("Generating trajectory (this may take a moment)...")
    start_time = time.time()
    trajectory = generate_trajectory(str(image_path), config=config)
    elapsed = time.time() - start_time

    print(f"Trajectory generated in {elapsed:.1f}s")
    print(f"  Waypoints: {len(trajectory)}")

    return trajectory


def demo_trajectory_info(trajectory):
    """Display information about a trajectory."""
    print("\n=== Trajectory Information ===")

    if trajectory is None:
        print("No trajectory loaded")
        return

    print(f"Total waypoints: {len(trajectory)}")

    # Count strokes (pen up/down transitions)
    if hasattr(trajectory, 'strokes'):
        print(f"Total strokes: {len(trajectory.strokes)}")

    # Show first few waypoints
    print("\nFirst 5 waypoints:")
    for i, wp in enumerate(trajectory[:5]):
        if isinstance(wp, dict):
            joints = list(wp.keys())[:3]
            print(f"  [{i}] {joints}...")
        else:
            print(f"  [{i}] {wp}")


def demo_playback(robot, trajectory, rate_hz: float = 20.0):
    """Demonstrate playing a trajectory on the robot."""
    print("\n=== Trajectory Playback ===")

    if trajectory is None:
        print("No trajectory to play")
        return

    total_waypoints = len(trajectory)
    print(f"Playing {total_waypoints} waypoints at {rate_hz} Hz")
    print(f"Estimated duration: {total_waypoints / rate_hz:.1f}s")

    # Progress callback
    last_percent = -1

    def on_progress(current, total):
        nonlocal last_percent
        percent = int(100 * current / total)
        if percent != last_percent and percent % 10 == 0:
            print(f"  Progress: {percent}% ({current}/{total})")
            last_percent = percent

    print("\nStarting playback...")
    start_time = time.time()

    try:
        success = robot.run_trajectory(
            trajectory,
            rate_hz=rate_hz,
            progress_callback=on_progress
        )

        elapsed = time.time() - start_time
        if success:
            print(f"\nPlayback complete! ({elapsed:.1f}s)")
        else:
            print(f"\nPlayback stopped ({elapsed:.1f}s)")

    except KeyboardInterrupt:
        print("\nPlayback interrupted by user")


def demo_save_load(trajectory, output_path: Path):
    """Demonstrate saving and loading trajectories."""
    print("\n=== Save/Load Trajectory ===")

    if trajectory is None:
        print("No trajectory to save")
        return None

    # Save trajectory
    print(f"Saving trajectory to {output_path}...")

    if hasattr(trajectory, 'to_json'):
        trajectory.to_json(str(output_path))
    else:
        # Fallback for raw list of waypoints
        with open(output_path, 'w') as f:
            json.dump(list(trajectory), f, indent=2)

    print(f"  Saved {len(trajectory)} waypoints")

    # Load it back
    print(f"Loading trajectory from {output_path}...")

    if hasattr(Trajectory, 'from_json'):
        loaded = Trajectory.from_json(str(output_path))
    else:
        with open(output_path) as f:
            loaded = json.load(f)

    print(f"  Loaded {len(loaded)} waypoints")

    return loaded


def create_simple_trajectory():
    """Create a simple test trajectory without an image."""
    print("\n=== Creating Simple Test Trajectory ===")

    # A simple trajectory that moves the head back and forth
    waypoints = []

    # Generate smooth head movement
    import math
    for i in range(100):
        t = i / 99.0  # 0 to 1
        head_pos = 50 + 20 * math.sin(t * 2 * math.pi)  # 30 to 70
        waypoints.append({
            Joint.TURN_HEAD: head_pos,
        })

    print(f"Created test trajectory with {len(waypoints)} waypoints")
    return waypoints


def main():
    parser = argparse.ArgumentParser(
        description="Trajectory Playback Example",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate and play trajectory from image
  python trajectory_playback.py --host 172.26.34.149 --image pib3_logo.png

  # Play a saved trajectory file
  python trajectory_playback.py --host 172.26.34.149 --trajectory saved.json

  # Generate trajectory only (no playback)
  python trajectory_playback.py --image pib3_logo.png --no-play

  # Use a simple test trajectory (no image needed)
  python trajectory_playback.py --host 172.26.34.149 --test
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
        "--image",
        type=Path,
        help="Image file to convert to trajectory"
    )
    parser.add_argument(
        "--trajectory",
        type=Path,
        help="Pre-existing trajectory JSON file to play"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("trajectory_output.json"),
        help="Output file for saving trajectory (default: trajectory_output.json)"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Playback rate in Hz (default: 20)"
    )
    parser.add_argument(
        "--no-play",
        action="store_true",
        help="Generate/load trajectory but don't play it"
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Use a simple test trajectory (no image needed)"
    )
    args = parser.parse_args()

    if not HAS_PIB3:
        print("Error: pib3 not installed.")
        print("Install with: pip install 'pib3[robot] @ git+https://github.com/mamrehn/pib3.git'")
        return

    trajectory = None

    # Load or generate trajectory
    if args.trajectory:
        # Load existing trajectory
        print(f"Loading trajectory from {args.trajectory}...")
        if hasattr(Trajectory, 'from_json'):
            trajectory = Trajectory.from_json(str(args.trajectory))
        else:
            with open(args.trajectory) as f:
                trajectory = json.load(f)
        print(f"Loaded {len(trajectory)} waypoints")

    elif args.image:
        # Generate from image
        trajectory = demo_generate_trajectory(args.image)

        if trajectory:
            # Save the generated trajectory
            demo_save_load(trajectory, args.output)

    elif args.test:
        # Create simple test trajectory
        trajectory = create_simple_trajectory()

    else:
        # Default: use sample image if exists
        sample_image = Path(__file__).parent / "pib3_logo.png"
        if sample_image.exists():
            print(f"Using sample image: {sample_image}")
            trajectory = demo_generate_trajectory(sample_image)
        else:
            print("No image specified. Use --image, --trajectory, or --test")
            print("Run with --help for usage information")
            return

    # Show trajectory info
    if trajectory:
        demo_trajectory_info(trajectory)

    # Play on robot
    if trajectory and not args.no_play:
        print(f"\nConnecting to robot at {args.host}:{args.port}...")

        try:
            with Robot(host=args.host, port=args.port) as robot:
                print(f"Connected: {robot.is_connected}")
                demo_playback(robot, trajectory, args.rate)

        except ConnectionError as e:
            print(f"Connection failed: {e}")
            print("Use --no-play to generate trajectory without robot connection")
        except KeyboardInterrupt:
            print("\nInterrupted by user")

    print("\n=== Done ===")


if __name__ == "__main__":
    main()
