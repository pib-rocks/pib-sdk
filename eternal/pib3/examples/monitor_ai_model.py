#!/usr/bin/env python3
"""
Example: Monitor Current AI Model

This script demonstrates how to subscribe to the 'current AI model' topic
to receive real-time updates when the active AI model changes on the OAK-D Lite camera.

Usage:
    python monitor_ai_model.py --host <robot_ip>
"""

import argparse
import time
import json
from pib3 import Robot

def main():
    parser = argparse.ArgumentParser(description="Monitor Current AI Model")
    parser.add_argument("--host", default="172.26.34.149", help="Robot IP address")
    args = parser.parse_args()

    print(f"Connecting to robot at {args.host}...")
    
    with Robot(host=args.host) as robot:
        print("Connected.")

        def on_model_change(info):
            print("\n[EVENT] AI Model Changed!")
            print(f"  Name: {info.get('name', 'unknown')}")
            print(f"  Type: {info.get('type', 'unknown')}")
            print(f"  Input Size: {info.get('input_size', '?')}")
            print(f"  Full Info: {json.dumps(info, indent=2)}")

        print("\nSubscribing to model updates...")
        sub = robot.subscribe_current_ai_model(on_model_change)

        print("\nMonitoring for 30 seconds. Try changing the model in another terminal/script to see updates!")
        print("Press Ctrl+C to exit early.")

        try:
            # We just wait here. The callback will execute in a background thread whenever an update arrives.
            # You should see an initial update shortly after subscribing if the camera is active.
            for i in range(30):
                time.sleep(1)
                if i % 10 == 0:
                    print(f"  (Monitoring... {30-i}s remaining)")
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            sub.unsubscribe()
            print("Unsubscribed.")

if __name__ == "__main__":
    main()
