#!/usr/bin/env python3
"""
IMU Data Visualization Example

This script demonstrates real-time IMU data visualization:
1. Reading accelerometer data
2. Reading gyroscope data
3. Full IMU data stream
4. Real-time plotting with matplotlib

Requirements:
    pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"
    pip install matplotlib numpy

Usage:
    python imu_visualization.py --host 172.26.34.149
    python imu_visualization.py --demo accel
    python imu_visualization.py --demo gyro
    python imu_visualization.py --demo plot

Note:
    This example requires the physical robot with OAK-D Lite camera (which has the IMU).
    IMU features are not available in Webots simulation.
"""

import argparse
import time
import threading
from collections import deque

import numpy as np

try:
    from pib3 import Robot
    HAS_PIB3 = True
except ImportError:
    HAS_PIB3 = False

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Note: matplotlib not installed. Plotting features disabled.")
    print("Install with: pip install matplotlib")


def demo_accelerometer(robot, duration: float = 5.0, frequency: int = 100):
    """Demonstrate accelerometer data streaming."""
    print("\n=== Accelerometer Demo ===")

    robot.set_imu_frequency(frequency)
    print(f"Set IMU frequency to {frequency} Hz")

    samples = []
    start_time = None

    def on_accel(data):
        nonlocal start_time
        if start_time is None:
            start_time = time.time()

        vec = data.get('vector', {})
        samples.append({
            'time': time.time() - start_time,
            'x': vec.get('x', 0),
            'y': vec.get('y', 0),
            'z': vec.get('z', 0),
        })

        # Print every 25th sample (~4 times per second at 100Hz)
        if len(samples) % 25 == 0:
            print(f"  Sample {len(samples)}: "
                  f"x={vec.get('x', 0):7.3f}, "
                  f"y={vec.get('y', 0):7.3f}, "
                  f"z={vec.get('z', 0):7.3f} m/s²")

    print(f"Reading accelerometer for {duration}s...")
    sub = robot.subscribe_imu(on_accel, data_type="accelerometer")
    time.sleep(duration)
    sub.unsubscribe()

    # Statistics
    if samples:
        x_vals = [s['x'] for s in samples]
        y_vals = [s['y'] for s in samples]
        z_vals = [s['z'] for s in samples]

        actual_duration = samples[-1]['time']
        actual_rate = len(samples) / actual_duration

        print(f"\nStatistics ({len(samples)} samples, {actual_rate:.1f} Hz):")
        print(f"  X: min={min(x_vals):.3f}, max={max(x_vals):.3f}, mean={np.mean(x_vals):.3f}")
        print(f"  Y: min={min(y_vals):.3f}, max={max(y_vals):.3f}, mean={np.mean(y_vals):.3f}")
        print(f"  Z: min={min(z_vals):.3f}, max={max(z_vals):.3f}, mean={np.mean(z_vals):.3f}")

        # Magnitude (should be ~9.81 m/s² when stationary)
        magnitudes = [np.sqrt(s['x']**2 + s['y']**2 + s['z']**2) for s in samples]
        print(f"  Magnitude: mean={np.mean(magnitudes):.3f} m/s² (gravity ~9.81)")

    return samples


def demo_gyroscope(robot, duration: float = 5.0, frequency: int = 100):
    """Demonstrate gyroscope data streaming."""
    print("\n=== Gyroscope Demo ===")
    print("Try rotating the robot to see angular velocity changes!")

    robot.set_imu_frequency(frequency)

    samples = []
    start_time = None

    def on_gyro(data):
        nonlocal start_time
        if start_time is None:
            start_time = time.time()

        vec = data.get('vector', {})
        samples.append({
            'time': time.time() - start_time,
            'x': vec.get('x', 0),
            'y': vec.get('y', 0),
            'z': vec.get('z', 0),
        })

        # Print every 25th sample
        if len(samples) % 25 == 0:
            print(f"  Sample {len(samples)}: "
                  f"x={vec.get('x', 0):7.4f}, "
                  f"y={vec.get('y', 0):7.4f}, "
                  f"z={vec.get('z', 0):7.4f} rad/s")

    print(f"Reading gyroscope for {duration}s...")
    sub = robot.subscribe_imu(on_gyro, data_type="gyroscope")
    time.sleep(duration)
    sub.unsubscribe()

    # Statistics
    if samples:
        x_vals = [s['x'] for s in samples]
        y_vals = [s['y'] for s in samples]
        z_vals = [s['z'] for s in samples]

        print(f"\nStatistics ({len(samples)} samples):")
        print(f"  X (roll):  min={min(x_vals):.4f}, max={max(x_vals):.4f}, mean={np.mean(x_vals):.4f}")
        print(f"  Y (pitch): min={min(y_vals):.4f}, max={max(y_vals):.4f}, mean={np.mean(y_vals):.4f}")
        print(f"  Z (yaw):   min={min(z_vals):.4f}, max={max(z_vals):.4f}, mean={np.mean(z_vals):.4f}")

    return samples


def demo_full_imu(robot, duration: float = 5.0, frequency: int = 100):
    """Demonstrate full IMU data streaming."""
    print("\n=== Full IMU Demo ===")

    robot.set_imu_frequency(frequency)

    samples = []
    start_time = None

    def on_imu(data):
        nonlocal start_time
        if start_time is None:
            start_time = time.time()

        accel = data.get('linear_acceleration', {})
        gyro = data.get('angular_velocity', {})

        samples.append({
            'time': time.time() - start_time,
            'accel_x': accel.get('x', 0),
            'accel_y': accel.get('y', 0),
            'accel_z': accel.get('z', 0),
            'gyro_x': gyro.get('x', 0),
            'gyro_y': gyro.get('y', 0),
            'gyro_z': gyro.get('z', 0),
        })

        # Print every 50th sample
        if len(samples) % 50 == 0:
            print(f"  Sample {len(samples)}:")
            print(f"    Accel: ({accel.get('x', 0):6.2f}, {accel.get('y', 0):6.2f}, {accel.get('z', 0):6.2f}) m/s²")
            print(f"    Gyro:  ({gyro.get('x', 0):6.4f}, {gyro.get('y', 0):6.4f}, {gyro.get('z', 0):6.4f}) rad/s")

    print(f"Reading full IMU for {duration}s...")
    sub = robot.subscribe_imu(on_imu, data_type="full")
    time.sleep(duration)
    sub.unsubscribe()

    print(f"\nCollected {len(samples)} samples")
    return samples


def demo_realtime_plot(robot, duration: float = 30.0, frequency: int = 50):
    """Real-time plotting of IMU data."""
    if not HAS_MATPLOTLIB:
        print("Error: matplotlib required for plotting")
        print("Install with: pip install matplotlib")
        return

    print("\n=== Real-time IMU Plot ===")
    print(f"Plotting for {duration}s (close window to stop early)")

    robot.set_imu_frequency(frequency)

    # Data buffers (keep last 5 seconds)
    buffer_size = frequency * 5
    times = deque(maxlen=buffer_size)
    accel_x = deque(maxlen=buffer_size)
    accel_y = deque(maxlen=buffer_size)
    accel_z = deque(maxlen=buffer_size)
    gyro_x = deque(maxlen=buffer_size)
    gyro_y = deque(maxlen=buffer_size)
    gyro_z = deque(maxlen=buffer_size)

    start_time = [None]
    data_lock = threading.Lock()

    def on_imu(data):
        if start_time[0] is None:
            start_time[0] = time.time()

        with data_lock:
            times.append(time.time() - start_time[0])

            accel = data.get('linear_acceleration', {})
            accel_x.append(accel.get('x', 0))
            accel_y.append(accel.get('y', 0))
            accel_z.append(accel.get('z', 0))

            gyro = data.get('angular_velocity', {})
            gyro_x.append(gyro.get('x', 0))
            gyro_y.append(gyro.get('y', 0))
            gyro_z.append(gyro.get('z', 0))

    # Set up plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Real-time IMU Data')

    # Initialize empty lines
    line_ax, = ax1.plot([], [], 'r-', label='X', alpha=0.8)
    line_ay, = ax1.plot([], [], 'g-', label='Y', alpha=0.8)
    line_az, = ax1.plot([], [], 'b-', label='Z', alpha=0.8)
    ax1.set_ylabel('Acceleration (m/s²)')
    ax1.set_title('Accelerometer')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(0, 5)
    ax1.set_ylim(-15, 15)

    line_gx, = ax2.plot([], [], 'r-', label='X (roll)', alpha=0.8)
    line_gy, = ax2.plot([], [], 'g-', label='Y (pitch)', alpha=0.8)
    line_gz, = ax2.plot([], [], 'b-', label='Z (yaw)', alpha=0.8)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title('Gyroscope')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(0, 5)
    ax2.set_ylim(-2, 2)

    def update(frame):
        with data_lock:
            if len(times) > 0:
                t = list(times)
                t_max = max(t) if t else 5

                # Update accelerometer
                line_ax.set_data(t, list(accel_x))
                line_ay.set_data(t, list(accel_y))
                line_az.set_data(t, list(accel_z))
                ax1.set_xlim(max(0, t_max - 5), t_max + 0.5)

                # Update gyroscope
                line_gx.set_data(t, list(gyro_x))
                line_gy.set_data(t, list(gyro_y))
                line_gz.set_data(t, list(gyro_z))
                ax2.set_xlim(max(0, t_max - 5), t_max + 0.5)

        return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

    # Start IMU subscription
    sub = robot.subscribe_imu(on_imu, data_type="full")

    try:
        ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
        plt.tight_layout()
        plt.show(block=True)
    finally:
        sub.unsubscribe()

    print("Plot closed")


def demo_static_plot(samples, title: str = "IMU Data"):
    """Create a static plot from collected samples."""
    if not HAS_MATPLOTLIB:
        print("Error: matplotlib required for plotting")
        return

    if not samples:
        print("No data to plot")
        return

    # Check what type of data we have
    has_accel = 'accel_x' in samples[0] or 'x' in samples[0]
    has_gyro = 'gyro_x' in samples[0]

    times = [s['time'] for s in samples]

    if has_accel and has_gyro:
        # Full IMU data
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

        ax1.plot(times, [s['accel_x'] for s in samples], 'r-', label='X', alpha=0.8)
        ax1.plot(times, [s['accel_y'] for s in samples], 'g-', label='Y', alpha=0.8)
        ax1.plot(times, [s['accel_z'] for s in samples], 'b-', label='Z', alpha=0.8)
        ax1.set_ylabel('Acceleration (m/s²)')
        ax1.set_title('Accelerometer')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        ax2.plot(times, [s['gyro_x'] for s in samples], 'r-', label='X (roll)', alpha=0.8)
        ax2.plot(times, [s['gyro_y'] for s in samples], 'g-', label='Y (pitch)', alpha=0.8)
        ax2.plot(times, [s['gyro_z'] for s in samples], 'b-', label='Z (yaw)', alpha=0.8)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angular Velocity (rad/s)')
        ax2.set_title('Gyroscope')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

    else:
        # Single sensor data
        fig, ax = plt.subplots(figsize=(12, 5))

        ax.plot(times, [s['x'] for s in samples], 'r-', label='X', alpha=0.8)
        ax.plot(times, [s['y'] for s in samples], 'g-', label='Y', alpha=0.8)
        ax.plot(times, [s['z'] for s in samples], 'b-', label='Z', alpha=0.8)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Value')
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="IMU Data Visualization Example",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python imu_visualization.py --host 172.26.34.149
  python imu_visualization.py --demo accel --duration 10
  python imu_visualization.py --demo gyro
  python imu_visualization.py --demo plot  # Real-time visualization

Note: This example requires the physical robot with OAK-D Lite camera.
      IMU features are not available in Webots simulation.
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
        choices=["accel", "gyro", "full", "plot", "all"],
        default="all",
        help="Which demo to run (default: all)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Duration for data collection in seconds (default: 5)"
    )
    parser.add_argument(
        "--frequency",
        type=int,
        default=100,
        help="IMU sampling frequency in Hz (default: 100)"
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

            if args.demo in ("accel", "all"):
                samples = demo_accelerometer(robot, args.duration, args.frequency)
                if args.demo == "accel" and HAS_MATPLOTLIB:
                    demo_static_plot(samples, "Accelerometer Data")

            if args.demo in ("gyro", "all"):
                samples = demo_gyroscope(robot, args.duration, args.frequency)
                if args.demo == "gyro" and HAS_MATPLOTLIB:
                    demo_static_plot(samples, "Gyroscope Data")

            if args.demo in ("full", "all"):
                samples = demo_full_imu(robot, args.duration, args.frequency)
                if args.demo == "full" and HAS_MATPLOTLIB:
                    demo_static_plot(samples, "Full IMU Data")

            if args.demo == "plot":
                demo_realtime_plot(robot, args.duration, args.frequency)

            print("\n=== Demo complete ===")

    except ConnectionError as e:
        print(f"Connection failed: {e}")
        print("\nTroubleshooting:")
        print("1. Check robot is powered on and connected to network")
        print("2. Verify rosbridge_server is running on the robot")
        print(f"3. Confirm IP address is correct: {args.host}")
        print("\nNote: This example requires the physical robot with OAK-D Lite camera.")
    except KeyboardInterrupt:
        print("\nInterrupted by user")


if __name__ == "__main__":
    main()
