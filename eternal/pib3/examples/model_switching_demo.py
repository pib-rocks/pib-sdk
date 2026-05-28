#!/usr/bin/env python3
"""
AI Model Switching Demo

This script demonstrates switching between different AI models on the OAK-D Lite:
1. Listing available models
2. Switching between detection models (YOLO, MobileNet-SSD)
3. Switching to pose estimation
4. Switching to segmentation
5. Comparing inference results

The set_ai_model() method is synchronous - it waits for the robot to confirm
the model has loaded before returning.

Requirements:
    pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"

Usage:
    python model_switching_demo.py --host 172.26.34.149
    python model_switching_demo.py --demo list
    python model_switching_demo.py --demo detection
    python model_switching_demo.py --demo pose
    python model_switching_demo.py --demo segmentation

Note:
    This example requires the physical robot with OAK-D Lite camera.
    Camera/AI features are not available in Webots simulation.
"""

import argparse
import time

try:
    from pib3 import Robot
    HAS_PIB3 = True
except ImportError:
    HAS_PIB3 = False


def demo_list_models(robot):
    """List all available AI models on the robot."""
    print("\n=== Available AI Models ===")

    models = robot.get_available_ai_models(timeout=5.0)

    if not models:
        print("No model information received (timeout)")
        return

    # Group by type
    by_type = {}
    for name, info in models.items():
        model_type = info.get('type', 'unknown')
        if model_type not in by_type:
            by_type[model_type] = []
        by_type[model_type].append((name, info))

    for model_type, model_list in sorted(by_type.items()):
        print(f"\n{model_type.upper()} models:")
        for name, info in model_list:
            desc = info.get('description', '')
            print(f"  - {name}: {desc}")

    print(f"\nTotal: {len(models)} models available")


def demo_detection_models(robot, duration: float = 5.0):
    """Compare different detection models."""

    # First, check available AI models
    print("🤖 Querying available AI models...")
    models = robot.get_available_ai_models(timeout=5.0)

    if models:
        print("Available AI models:")
        for name, info in models.items():
            print(f"  • {name}: {info.get('type', 'unknown')} - {info.get('description', '')}")
    else:
        print("  (No model info received - using default model)")

    print("\n=== Detection Model Comparison ===")

    detection_models = ["mobilenet-ssd", "yolov8n", "yolov6n"]

    for model_name in detection_models:
        print(f"\n--- Testing {model_name} ---")

        # Switch model (synchronous)
        print(f"Switching to {model_name}...")
        if robot.set_ai_model(model_name, timeout=10.0):
            print(f"Model ready!")
        else:
            print(f"Model switch timed out, skipping...")
            continue

        # Collect detection stats
        results = []
        latencies = []

        def on_detection(data):
            if data.get('type') == 'detection':
                results.append(data)
                latencies.append(data.get('latency_ms', 0))

                # Print first detection
                if len(results) == 1:
                    detections = data.get('result', {}).get('detections', [])
                    print(f"  First frame: {len(detections)} objects detected")
                    for det in detections[:3]:
                        label = det.get('label', '?')
                        conf = det.get('confidence', 0)
                        print(f"    - Class {label} (confidence: {conf:.2f})")

        # Subscribe and collect
        print(f"Running inference for {duration}s...")
        sub = robot.subscribe_ai_detections(on_detection)
        time.sleep(duration)
        sub.unsubscribe()

        # Print stats
        if results:
            avg_latency = sum(latencies) / len(latencies)
            fps = len(results) / duration
            total_objects = sum(
                len(r.get('result', {}).get('detections', []))
                for r in results
            )
            print(f"  Frames: {len(results)}, FPS: {fps:.1f}")
            print(f"  Avg latency: {avg_latency:.1f}ms")
            print(f"  Total objects: {total_objects}")
        else:
            print("  No results received")


def demo_pose_estimation(robot, duration: float = 5.0):
    """Demonstrate pose estimation models."""
    print("\n=== Pose Estimation Demo ===")

    pose_models = ["human-pose-estimation", "yolov8n-pose"]

    for model_name in pose_models:
        print(f"\n--- Testing {model_name} ---")

        # Switch model
        print(f"Switching to {model_name}...")
        if robot.set_ai_model(model_name, timeout=5.0):
            print(f"Model ready!")
        else:
            print(f"Model switch timed out, skipping...")
            continue

        # Collect pose stats
        results = []
        keypoint_counts = []

        def on_pose(data):
            results.append(data)

            result = data.get('result', {})
            keypoints = result.get('keypoints', [])
            detected = result.get('detected_count', len(keypoints))
            keypoint_counts.append(detected)

            # Print first few results
            if len(results) <= 2:
                latency = data.get('latency_ms', 0)
                print(f"  Frame {len(results)}: {detected} keypoints, latency: {latency:.1f}ms")

                # Show some keypoints
                for kp in keypoints[:5]:
                    kp_id = kp.get('id', '?')
                    x = kp.get('x', 0)
                    y = kp.get('y', 0)
                    conf = kp.get('confidence', 0)
                    print(f"    Keypoint {kp_id}: ({x:.2f}, {y:.2f}) conf={conf:.2f}")

        # Subscribe and collect
        print(f"Running pose estimation for {duration}s...")
        sub = robot.subscribe_ai_detections(on_pose)
        time.sleep(duration)
        sub.unsubscribe()

        # Print stats
        if results:
            fps = len(results) / duration
            avg_keypoints = sum(keypoint_counts) / len(keypoint_counts) if keypoint_counts else 0
            print(f"  Frames: {len(results)}, FPS: {fps:.1f}")
            print(f"  Avg keypoints detected: {avg_keypoints:.1f}")
        else:
            print("  No results received")


def demo_segmentation(robot, duration: float = 5.0):
    """Demonstrate segmentation models."""
    print("\n=== Segmentation Demo ===")

    # Switch to segmentation model
    model_name = "deeplabv3"
    print(f"Switching to {model_name}...")

    if robot.set_ai_model(model_name, timeout=5.0):
        print("Model ready!")
    else:
        print("Model switch timed out")
        return

    # Test bbox mode (lightweight)
    print("\n--- BBox Mode (lightweight) ---")
    robot.set_ai_config(segmentation_mode="bbox")

    results = []

    def on_seg_bbox(data):
        if data.get('type') == 'segmentation':
            results.append(data)
            if len(results) <= 2:
                result = data.get('result', {})
                classes = result.get('classes_detected', [])
                bboxes = result.get('bboxes', [])
                latency = data.get('latency_ms', 0)
                print(f"  Frame {len(results)}: {len(classes)} classes, "
                      f"{len(bboxes)} bboxes, latency: {latency:.1f}ms")

    sub = robot.subscribe_ai_detections(on_seg_bbox)
    time.sleep(duration)
    sub.unsubscribe()

    if results:
        print(f"  Total frames: {len(results)}")

    # Test mask mode (detailed) - target person class
    print("\n--- Mask Mode (detailed, person class) ---")
    robot.set_ai_config(
        segmentation_mode="mask",
        segmentation_target_class=15  # Person class in COCO
    )

    results = []

    def on_seg_mask(data):
        if data.get('type') == 'segmentation':
            results.append(data)
            if len(results) <= 2:
                result = data.get('result', {})
                mode = result.get('mode', 'unknown')
                latency = data.get('latency_ms', 0)

                if mode == 'mask' and 'mask_rle' in result:
                    rle = result['mask_rle']
                    size = rle.get('size', [0, 0])
                    print(f"  Frame {len(results)}: mask {size[0]}x{size[1]}, "
                          f"latency: {latency:.1f}ms")
                else:
                    print(f"  Frame {len(results)}: mode={mode}, latency: {latency:.1f}ms")

    sub = robot.subscribe_ai_detections(on_seg_mask)
    time.sleep(duration)
    sub.unsubscribe()

    if results:
        print(f"  Total frames: {len(results)}")


def demo_quick_switch(robot):
    """Demonstrate rapid model switching."""
    print("\n=== Quick Switch Demo ===")
    print("Switching between models rapidly...")

    models = ["yolov8n", "mobilenet-ssd", "human-pose-estimation", "yolov8n"]

    for model_name in models:
        start = time.time()

        if robot.set_ai_model(model_name, timeout=5.0):
            elapsed = (time.time() - start) * 1000
            print(f"  {model_name}: switched in {elapsed:.0f}ms")
        else:
            print(f"  {model_name}: timeout")

    print("\nModel switching complete!")


def main():
    parser = argparse.ArgumentParser(
        description="AI Model Switching Demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python model_switching_demo.py --host 172.26.34.149
  python model_switching_demo.py --demo list
  python model_switching_demo.py --demo detection --duration 10
  python model_switching_demo.py --demo pose
  python model_switching_demo.py --demo segmentation

Note: This example requires the physical robot with OAK-D Lite camera.
      Camera/AI features are not available in Webots simulation.
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
        choices=["list", "detection", "pose", "segmentation", "switch", "all"],
        default="all",
        help="Which demo to run (default: all)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Duration for each model test in seconds (default: 5)"
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

            if args.demo in ("list", "all"):
                demo_list_models(robot)

            if args.demo in ("detection", "all"):
                demo_detection_models(robot, args.duration)

            if args.demo in ("pose", "all"):
                demo_pose_estimation(robot, args.duration)

            if args.demo in ("segmentation", "all"):
                demo_segmentation(robot, args.duration)

            if args.demo in ("switch", "all"):
                demo_quick_switch(robot)

            print("\n=== All demos complete ===")

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
