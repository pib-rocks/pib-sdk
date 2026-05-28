#!/usr/bin/env python3
"""
Example: Camera, AI Detection, and IMU usage with pib3.

This script demonstrates:
1. Camera streaming
2. AI object detection with model switching (on-demand)
3. IMU sensor data access
4. Vision-based head tracking

Requirements:
    pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"
    pip install opencv-python numpy

Usage:
    python camera_ai_imu_example.py --host 172.26.34.149

    # Run specific demos
    python camera_ai_imu_example.py --demo camera
    python camera_ai_imu_example.py --demo ai
    python camera_ai_imu_example.py --demo imu
    python camera_ai_imu_example.py --demo tracking

Note:
    This example requires the physical robot with OAK-D Lite camera.
    Camera/AI/IMU features are not available in Webots simulation yet.
"""

import argparse
import time
from typing import Optional

import numpy as np

import pib3

# Optional: OpenCV for display
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    print("Note: OpenCV not installed. Display features disabled.")
    print("Install with: pip install opencv-python")

# Import pib3 - will fail gracefully if not installed
try:
    from pib3 import Robot, Joint
    HAS_PIB3 = True
except ImportError:
    HAS_PIB3 = False


def demo_camera_streaming(robot, duration: float = 10.0):
    """Demonstrate camera streaming."""
    print("\n=== Camera Streaming Demo ===")
    print("Streaming camera for", duration, "seconds...")

    frame_count = 0
    start_time = time.time()

    def on_frame(jpeg_bytes):
        nonlocal frame_count
        frame_count += 1

        if HAS_CV2:
            # Decode JPEG to numpy array
            img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            if frame is not None:
                # Add frame counter overlay
                cv2.putText(
                    frame,
                    f"Frame: {frame_count}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow("pib3 Camera", frame)
                cv2.waitKey(1)
        else:
            if frame_count % 30 == 0:
                print(f"  Received frame {frame_count}, size: {len(jpeg_bytes)} bytes")

    # Subscribe to camera (streaming starts automatically)
    sub = robot.subscribe_camera_image(on_frame)

    try:
        time.sleep(duration)
    finally:
        # Unsubscribe (streaming stops automatically)
        sub.unsubscribe()
        if HAS_CV2:
            cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    fps = frame_count / elapsed
    print(f"Received {frame_count} frames in {elapsed:.1f}s ({fps:.1f} FPS)")


def demo_ai_detection(robot, duration: float = 15.0):
    """Demonstrate AI object detection."""
    print("\n=== AI Detection Demo ===")
    print("Running AI detection for", duration, "seconds...")

    # First, check available models
    print("\nQuerying available models...")
    models = robot.get_available_ai_models(timeout=5.0)
    if models:
        print("Available models:")
        for name, info in models.items():
            print(f"  - {name}: {info.get('type', 'unknown')}")
    else:
        print("  (No model info received, using default)")

    detection_count = 0

    def on_detection(data):
        nonlocal detection_count
        detection_count += 1

        model = data.get('model', 'unknown')
        det_type = data.get('type', 'unknown')
        latency = data.get('latency_ms', 0)

        if det_type == 'detection':
            result = data.get('result', {})
            detections = result.get('detections', [])
            if detections:
                print(f"\n[Frame {data.get('frame_id', '?')}] "
                      f"Model: {model}, Latency: {latency:.1f}ms")
                for det in detections:
                    label = det.get('label', '?')
                    conf = det.get('confidence', 0)
                    bbox = det.get('bbox', {})
                    print(f"  → Class {label} ({conf:.2f}): "
                          f"x=[{bbox.get('xmin', 0):.2f}-{bbox.get('xmax', 0):.2f}], "
                          f"y=[{bbox.get('ymin', 0):.2f}-{bbox.get('ymax', 0):.2f}]")

        elif det_type == 'classification':
            result = data.get('result', {})
            classifications = result.get('classifications', [])
            if classifications and detection_count % 10 == 0:
                top = classifications[0]
                print(f"[Frame {data.get('frame_id', '?')}] "
                      f"Top class: {top.get('class_id')} ({top.get('confidence', 0):.2f})")

    # Subscribe to AI detections (inference starts automatically)
    sub = robot.subscribe_ai_detections(on_detection)

    try:
        time.sleep(duration)
    finally:
        # Unsubscribe (inference stops automatically)
        sub.unsubscribe()

    print(f"\nProcessed {detection_count} inference results")


def demo_imu_data(robot, duration: float = 5.0):
    """Demonstrate IMU sensor data access."""
    print("\n=== IMU Data Demo ===")
    print("Reading IMU data for", duration, "seconds...")

    # Set a reasonable frequency
    robot.set_imu_frequency(50)  # 50 Hz

    sample_count = 0

    def on_imu(data):
        nonlocal sample_count
        sample_count += 1

        # Full IMU data includes both accelerometer and gyroscope
        accel = data.get('linear_acceleration', {})
        gyro = data.get('angular_velocity', {})

        if sample_count % 25 == 0:  # Print every 0.5 seconds at 50Hz
            print(f"Sample {sample_count}:")
            print(f"  Accel: x={accel.get('x', 0):7.3f}, "
                  f"y={accel.get('y', 0):7.3f}, "
                  f"z={accel.get('z', 0):7.3f} m/s²")
            print(f"  Gyro:  x={gyro.get('x', 0):7.4f}, "
                  f"y={gyro.get('y', 0):7.4f}, "
                  f"z={gyro.get('z', 0):7.4f} rad/s")

    # Subscribe to full IMU data
    sub = robot.subscribe_imu(on_imu, data_type="full")

    try:
        time.sleep(duration)
    finally:
        sub.unsubscribe()

    print(f"\nReceived {sample_count} IMU samples")


def demo_person_tracking(robot, duration: float = 30.0):
    """Demonstrate vision-based person tracking with head movement."""
    print("\n=== Person Tracking Demo ===")
    print("Tracking persons for", duration, "seconds...")
    print("The robot will turn its head to follow detected persons.")

    # Use MobileNet-SSD for fast detection (class 15 = person in COCO)
    print("Switching to mobilenet-ssd model...")
    if robot.set_ai_model("mobilenet-ssd", timeout=5.0):
        print("Model ready!")
    else:
        print("Model switch timed out, using current model")

    # Set confidence threshold
    robot.set_ai_config(confidence=0.5)

    last_update = 0
    update_interval = 0.1  # Update head position every 100ms

    def on_detection(data):
        nonlocal last_update

        if data.get('type') != 'detection':
            return

        now = time.time()
        if now - last_update < update_interval:
            return

        result = data.get('result', {})
        detections = result.get('detections', [])

        # Find highest confidence person (class 15 in COCO = person)
        best_person = None
        best_conf = 0
        for det in detections:
            if det.get('label') == 15 and det.get('confidence', 0) > best_conf:
                best_person = det
                best_conf = det.get('confidence', 0)

        if best_person and best_conf > 0.5:
            bbox = best_person.get('bbox', {})
            center_x = (bbox.get('xmin', 0) + bbox.get('xmax', 1)) / 2

            # Map center_x (0-1) to head position (0-100%)
            # Invert: person on left → head turns left (higher %)
            head_pos = (1.0 - center_x) * 40 + 30  # Range: 30-70%

            try:
                robot.set_joint(Joint.TURN_HEAD, head_pos)
                print(f"Person at x={center_x:.2f} → head at {head_pos:.0f}%")
            except Exception as e:
                print(f"Failed to move head: {e}")

            last_update = now

    # Subscribe to AI detections
    sub = robot.subscribe_ai_detections(on_detection)

    try:
        # Center head initially
        robot.set_joint(Joint.TURN_HEAD, 50)
        time.sleep(duration)
    finally:
        sub.unsubscribe()
        # Return head to center
        robot.set_joint(Joint.TURN_HEAD, 50)

    print("Tracking complete.")


def main():
    parser = argparse.ArgumentParser(
        description="pib3 Camera, AI, and IMU Example",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python camera_ai_imu_example.py --host 172.26.34.149
  python camera_ai_imu_example.py --demo camera --duration 20
  python camera_ai_imu_example.py --demo ai
  python camera_ai_imu_example.py --demo tracking
        """
    )

    print(f'piB3 version: {pib3.__version__}')

    host_michael = '172.26.34.222', 'Michael'
    host_wolfgang = '172.26.46.47', 'Wolfgang'
    host_richard = '172.26.30.35', 'Richard'
    host_martin = '172.26.34.149', 'Martin'  # robot with arms

    host_ip, host_name = host_wolfgang

    print(f'I choose you "{host_name}"!')

    parser.add_argument(
        "--host",
        default=host_ip,
        help=f"Robot IP address (default: {host_ip})"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9090,
        help="Rosbridge port (default: 9090)"
    )
    parser.add_argument(
        "--demo",
        choices=["camera", "ai", "imu", "tracking", "all"],
        default="all",
        help="Which demo to run (default: all)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Duration for each demo in seconds (default: 10)"
    )
    args = parser.parse_args()

    # Check pib3 is installed
    if not HAS_PIB3:
        print("Error: pib3 not installed.")
        print("Install with: pip install 'pib3 @ git+https://github.com/mamrehn/pib3.git'")
        return

    print(f"Connecting to robot at {args.host}:{args.port}...")

    try:
        with Robot(host=args.host, port=args.port) as robot:
            print(f"Connected: {robot.is_connected}")

            if args.demo in ("camera", "all"):
                demo_camera_streaming(robot, args.duration)

            if args.demo in ("ai", "all"):
                demo_ai_detection(robot, args.duration)

            if args.demo in ("imu", "all"):
                demo_imu_data(robot, min(args.duration, 5.0))

            if args.demo in ("tracking", "all"):
                demo_person_tracking(robot, args.duration)

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
