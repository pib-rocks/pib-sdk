#!/usr/bin/env python3
"""
Scan and Draw Proof of Concept

This script demonstrates a full "Sense-Plan-Act" loop:
1.  **Sense**: Scan the environment by moving the head and processing AI detections.
2.  **Plan**: Select a random detected object and generate a drawing trajectory.
3.  **Act**: Draw the object using the "clenched fist" (pencil grip) mode.

Requirements:
    pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"
    pip install Pillow numpy

Note:
    This example requires the physical robot with OAK-D Lite camera.
    Camera/AI features are not available in Webots simulation yet.
"""

import argparse
import time
import random
import threading
from pathlib import Path
from typing import Dict, List, Tuple
import PIL.Image
import PIL.ImageDraw
import PIL.ImageFont

import pib3
from pib3 import Robot, Joint, generate_trajectory, TrajectoryConfig, PaperConfig


def ensure_icons_exist(output_dir: Path):
    """Generate simple placeholder icons for common detections."""
    output_dir.mkdir(parents=True, exist_ok=True)
    
    icons = {
        "person": "웃",
        "cup": "U",
        "bottle": "[]",
        "tv": "[_]",
        "laptop": "/_/",
        "keyboard": "...",
        "mouse": "o",
        "cell phone": "[]",
        "default": "?"
    }
    
    font_size = 80
    image_size = (100, 100)
    
    for name, symbol in icons.items():
        filename = output_dir / f"{name.replace(' ', '_')}.png"
        if not filename.exists():
            img = PIL.Image.new('RGB', image_size, color=(255, 255, 255))
            draw = PIL.ImageDraw.Draw(img)
            
            # Simple text-based icon
            try:
                # Try to use a default font
                font = PIL.ImageFont.load_default()
                # Scale up isn't easy with default font, but we'll stroke the shape manually if needed
                # For PoC, we validly assume simple shapes or large text if font available
                # Let's just draw the text centralized
                draw.text((25, 25), symbol, fill=(0, 0, 0))
            except Exception:
                pass
                
            # Draw a box border
            draw.rectangle([(0,0), (99,99)], outline=(0,0,0), width=2)
            
            # Draw symbol (simulated with lines for robustness)
            if name == "person":
                # Head
                draw.ellipse([(40, 20), (60, 40)], outline=(0,0,0), width=2)
                # Body
                draw.line([(50, 40), (50, 70)], fill=(0,0,0), width=2)
                # Arms
                draw.line([(30, 50), (70, 50)], fill=(0,0,0), width=2)
                # Legs
                draw.line([(50, 70), (30, 90)], fill=(0,0,0), width=2)
                draw.line([(50, 70), (70, 90)], fill=(0,0,0), width=2)
            elif name == "cup":
                 draw.arc([(30, 30), (70, 80)], 0, 180, fill=(0,0,0), width=2)
                 draw.line([(30, 30), (30, 55)], fill=(0,0,0), width=2)
                 draw.line([(70, 30), (70, 55)], fill=(0,0,0), width=2)
            elif name == "bottle":
                draw.rectangle([(40, 20), (60, 90)], outline=(0,0,0), width=2)
                draw.line([(45, 20), (45, 10)], fill=(0,0,0), width=2)
                draw.line([(55, 20), (55, 10)], fill=(0,0,0), width=2)
            else:
                 # Default circle
                 draw.ellipse([(20, 20), (80, 80)], outline=(0,0,0), width=2)
                 draw.text((40, 45), "?", fill=(0,0,0))

            img.save(filename)
            print(f"Generated icon: {filename}")


class SceneScanner:
    def __init__(self, robot):
        self.robot = robot
        # Label -> info dict
        self.detected_objects: Dict[str, dict] = {} 
        self.lock = threading.Lock()
        self.is_scanning = False
        
    def start(self):
        self.is_scanning = True
        self.sub = self.robot.subscribe_ai_detections(self._on_detection)
        
    def stop(self):
        self.is_scanning = False
        if hasattr(self, 'sub'):
            self.sub.unsubscribe()
            
    def _on_detection(self, data):
        if not self.is_scanning:
            return
            
        detections = data.get('result', {}).get('detections', [])
        for det in detections:
            label = det.get('label')
            conf = det.get('confidence', 0.0)
            
            # Filter low confidence
            if conf < 0.5:
                continue
                
            # Translate numeric labels if needed (though pib3 often returns strings now if configured)
            # Assuming string labels or COCO mapping. 
            # For this PoC, we'll store whatever we get.
            
            with self.lock:
                # Store if new or better confidence
                if label not in self.detected_objects or conf > self.detected_objects[label]['confidence']:
                    # Get current head position in percent (0-100)
                    try:
                        head_pos = self.robot.get_joint(Joint.TURN_HEAD, unit="percent")
                    except Exception:
                        head_pos = None
                        
                    self.detected_objects[label] = {
                        'confidence': conf,
                        'head_position': head_pos,
                        'timestamp': time.time()
                    }


def main():

    print(f'piB3 version: {pib3.__version__}')

    host_michael = '172.26.34.222', 'Michael'
    host_wolfgang = '172.26.46.47', 'Wolfgang'
    host_richard = '172.26.30.35', 'Richard'
    host_martin = '172.26.34.149', 'Martin'  # robot with arms

    host_ip, host_name = host_wolfgang

    print(f'I choose you "{host_name}"!')

    parser = argparse.ArgumentParser(description="Scan and Draw PoC")
    parser.add_argument("--host", default=host_ip, help="Robot IP")
    args = parser.parse_args()

    # 1. Setup Resources
    icon_dir = Path(__file__).parent / "icons"
    ensure_icons_exist(icon_dir)

    print(f"Connecting to robot at {args.host}...")
    try:
        with Robot(host=args.host) as robot:
            print("Connected.")
            
            # 2. Configure AI Model
            # Using 'yolov8n' for general object detection (80 COCO classes)
            model_name = "yolov8n"
            print(f"Setting AI model to {model_name}...")
            if robot.set_ai_model(model_name, timeout=5.0):
                print(f"Model switched to {model_name}")
            else:
                print(f"Warning: Model switch timed out, continuing with default model")
            
            # 3. Start Scanning Routine
            scanner = SceneScanner(robot)
            scanner.start()
            
            print("Starting scan sequence...")
            
            # Look Left
            print("Turning head LEFT (100%)...")
            robot.set_joint(Joint.TURN_HEAD, 100)
            time.sleep(2.0)
            
            # Look Right
            print("Turning head RIGHT (0%)...")
            robot.set_joint(Joint.TURN_HEAD, 0)
            time.sleep(2.0)
            
            # Look Center
            print("Centering head (50%)...")
            robot.set_joint(Joint.TURN_HEAD, 50)
            time.sleep(1.0)
            
            scanner.stop()
            
            # 4. Select Object
            detections = scanner.detected_objects
            print("\nScan complete. Detected objects:")
            for label, info in detections.items():
                head_str = f"{info['head_position']:.0f}%" if info['head_position'] is not None else "unknown"
                print(f"  - {label}: {info['confidence']:.2f} (at head={head_str})")
            
            if not detections:
                print("No objects detected. Drawing default icon.")
                selected_label = "default"
            else:
                selected_label = random.choice(list(detections.keys()))
                # Map COCO class ID/Name to file if necessary. 
                # Assuming the label comes as a string name for now.
                # If it's an int, we might need a map. 
                # pib3's latest backend usually sends strings if using SDK 2.x, 
                # but standard ROS messages might be IDs. 
                # For safety, let's coerce to string.
                selected_label = str(selected_label)
                
            print(f"Selected object to draw: {selected_label}")
            
            # Find matching icon
            icon_path = icon_dir / f"{selected_label}.png"
            if not icon_path.exists():
                print(f"Icon for {selected_label} not found, using default.")
                icon_path = icon_dir / "default.png"
                
            # 5. Generate Drawing Trajectory
            print(f"Generating trajectory from {icon_path}...")
            
            # Configure for Pencil Grip (Clenched Fist)
            config = TrajectoryConfig()
            config.ik.grip_style = "pencil_grip"
            
            # Adjust paper for pencil grip (as per GRIP_STYLES.md)
            config.paper.start_x = 0.35
            config.paper.center_y = -0.24
            config.paper.height_z = 0.83
            config.paper.size = 0.08 # Small drawing
            
            try:
                trajectory = generate_trajectory(str(icon_path), config=config)
                print(f"Trajectory generated with {len(trajectory)} waypoints.")
                
                # 6. Execute Drawing
                print("Executing drawing...")
                robot.run_trajectory(trajectory)
                print("Drawing complete!")
                
            except Exception as e:
                print(f"Failed to generate or execute trajectory: {e}")
                
    except ConnectionError:
        print("Could not connect to robot. Check IP and network.")
    except KeyboardInterrupt:
        print("\nStopped by user.")

if __name__ == "__main__":
    main()
