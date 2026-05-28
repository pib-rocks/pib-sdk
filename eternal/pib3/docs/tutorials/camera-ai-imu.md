# Camera, AI Detection, and IMU

Access the OAK-D Lite camera, AI inference, and IMU sensors through the pib3 API.

## Objectives

By the end of this tutorial, you will:

- Stream camera images from the OAK-D Lite
- Use on-demand AI object detection, pose estimation, and segmentation
- Switch between AI models at runtime
- Access IMU accelerometer and gyroscope data
- Understand the on-demand activation pattern

## Prerequisites

- pib3 installed with robot support: `pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"`
- A PIB robot with OAK-D Lite camera connected
- Rosbridge running on the robot (port 9090)

---

## Key Concept: On-Demand Activation

All camera, AI, and IMU features follow an **on-demand pattern**:

- **Streaming/inference only runs while you're subscribed**
- When you call `subscribe_*()`, the robot starts the pipeline
- When you call `.unsubscribe()`, the robot stops the pipeline
- This saves computational resources and power

```python
# Subscribe → Pipeline starts automatically
sub = robot.subscribe_camera_image(callback)

# ... do your processing ...

# Unsubscribe → Pipeline stops automatically
sub.unsubscribe()
```

---

## Quick Start: Subsystem API (Recommended)

For most use cases, use the simplified subsystem APIs instead of raw subscriptions. The subsystems automatically manage subscriptions and provide typed results.

### AI Subsystem (`robot.ai`)

```python
from pib3 import Robot, AIModel

with Robot(host="192.168.178.71") as robot:
    # Set AI model (waits for confirmation)
    robot.ai.set_model(AIModel.YOLOV8N)
    
    # Get detections (waits automatically for results)
    for det in robot.ai.get_detections():
        print(f"{det.label}: {det.confidence:.0%} at {det.bbox.center}")
    
    # Check performance
    print(f"FPS: {robot.ai.fps:.1f}")
```

### Hand Tracking with Servo Control

```python
from pib3 import Robot, AIModel

with Robot(host="192.168.178.71") as robot:
    robot.ai.set_model(AIModel.HAND)
    
    for hand in robot.ai.get_hand_landmarks():
        print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
        
        # Convert finger angles to robot servo percentages
        servos = hand.finger_angles.to_servo_values()
        
        # Mirror to robot hand
        if hand.handedness.value == "left":
            robot.set_joints({
                "index_left_stretch": servos["index"],
                "middle_left_stretch": servos["middle"],
            })
```

### Camera Subsystem (`robot.camera`)

```python
from pib3 import Robot

with Robot(host="192.168.178.71") as robot:
    # Get latest frame
    frame = robot.camera.get_frame()
    if frame:
        print(f"Frame {frame.frame_id}: {len(frame.jpeg_bytes)} bytes")
        
        # Decode to numpy (requires OpenCV)
        img = frame.to_numpy()
        
        # Configure camera
        robot.camera.configure(fps=30, quality=80)
```

!!! tip "API Reference"
    For complete details on subsystem methods and types, see the 
    [AI & Camera Subsystems](../api/ai-camera-subsystems.md) API reference.

---

## Low-Level API: Raw Subscriptions

The following sections cover the raw subscription API for advanced use cases 
(custom buffering, multiple callbacks, etc.).

---

## Camera Streaming

### Basic Camera Streaming

The camera publishes hardware-encoded MJPEG frames. The callback receives raw JPEG bytes:

```python
from pib3 import Robot
import cv2
import numpy as np

def on_frame(jpeg_bytes):
    """Callback receives raw JPEG bytes."""
    # Decode JPEG to numpy array
    img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

    # Process frame...
    print(f"Frame shape: {frame.shape}")

    # Display (optional)
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)

with Robot(host="192.168.178.71") as robot:
    # Subscribe to camera (streaming starts)
    sub = robot.subscribe_camera_image(on_frame)

    # Keep running for 10 seconds
    import time
    time.sleep(10)

    # Unsubscribe (streaming stops)
    sub.unsubscribe()
    cv2.destroyAllWindows()
```

### Using PIL for Image Processing

```python
from PIL import Image
import io

def on_frame(jpeg_bytes):
    """Process frames with PIL."""
    img = Image.open(io.BytesIO(jpeg_bytes))
    print(f"Image size: {img.size[0]}x{img.size[1]} pixels")

    # Save frame
    img.save("captured_frame.jpg")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_camera_image(on_frame)
    time.sleep(5)
    sub.unsubscribe()
```

### Configuring Camera Settings

Adjust FPS, quality, and resolution:

```python
with Robot(host="192.168.178.71") as robot:
    # Set individual parameters
    robot.set_camera_config(fps=30)
    robot.set_camera_config(quality=80)
    robot.set_camera_config(resolution=(1280, 720))

    # Or set multiple at once
    robot.set_camera_config(
        fps=30,
        quality=80,
        resolution=(1280, 720)
    )
```

!!! warning "Brief Interruption"
    Changing camera settings causes a brief stream interruption (~100-200ms)
    as the pipeline rebuilds.

---

## AI Object Detection

### Available AI Models

The OAK-D Lite supports multiple AI model types:

| Model Type | Example Models | Output |
|------------|----------------|--------|
| **Detection** | `mobilenet-ssd`, `yolov8n`, `yolov6n`, `face-detection-retail-0004` | Bounding boxes with class labels |
| **Classification** | `resnet50` | Top-K class predictions |
| **Segmentation** | `deeplabv3`, `deeplabv3-person`, `selfie-segmentation` | Pixel-wise masks |
| **Instance Segmentation** | `yolov8n-seg` | Per-object masks |
| **Pose Estimation** | `human-pose-estimation`, `openpose`, `yolov8n-pose` | Body keypoints |
| **Age/Gender** | `age-gender` | Age and gender estimation |
| **Emotion** | `emotion-recognition` | Facial emotion detection |

Query available models at runtime:

```python
with Robot(host="192.168.178.71") as robot:
    models = robot.get_available_ai_models()

    for name, info in models.items():
        print(f"{name}:")
        print(f"  Type: {info['type']}")
        print(f"  Input size: {info.get('input_size', 'N/A')}")
        print(f"  Description: {info.get('description', '')}")
```

### Subscribing to AI Detections

AI inference only runs while you're subscribed:

```python
from pib3 import Robot

def on_detection(data):
    """Callback receives detection results."""
    model = data['model']
    det_type = data['type']
    latency = data['latency_ms']

    print(f"Model: {model}, Type: {det_type}, Latency: {latency:.1f}ms")

    if det_type == 'detection':
        for det in data['result']['detections']:
            label = det['label']
            conf = det['confidence']
            bbox = det['bbox']
            print(f"  Found class {label} ({conf:.2f}) at {bbox}")

with Robot(host="192.168.178.71") as robot:
    # Subscribe to AI detections (inference starts)
    sub = robot.subscribe_ai_detections(on_detection)

    import time
    time.sleep(10)

    # Unsubscribe (inference stops)
    sub.unsubscribe()
```

### Detection Result Format

All detection messages include common metadata:

```python
{
    "model": "mobilenet-ssd",
    "type": "detection",  # or "classification", "segmentation", "pose"
    "frame_id": 42,
    "timestamp_ns": 1234567890123456789,
    "latency_ms": 12.5,
    "result": { ... }  # Model-specific results
}
```

#### Detection Models (YOLO, MobileNet-SSD, etc.)

```python
{
    "result": {
        "detections": [
            {
                "label": 15,        # Class ID (15 = person in COCO)
                "confidence": 0.92,
                "bbox": {
                    "xmin": 0.1, "ymin": 0.05,
                    "xmax": 0.3, "ymax": 0.4
                }
            }
        ],
        "count": 1
    }
}
```

#### Pose Estimation Models

```python
{
    "result": {
        "keypoints": [
            {"id": 0, "x": 0.45, "y": 0.12, "confidence": 0.95},  # nose
            {"id": 1, "x": 0.47, "y": 0.10, "confidence": 0.92},  # left_eye
            # ... 18 keypoints for human-pose-estimation
        ],
        "detected_count": 17
    }
}
```

#### Classification Models

```python
{
    "result": {
        "classifications": [
            {"class_id": 281, "confidence": 0.85},
            {"class_id": 282, "confidence": 0.10}
        ]
    }
}
```

### Switching AI Models (Synchronous)

The `set_ai_model()` method is **synchronous** - it waits for confirmation that the model has loaded before returning:

```python
with Robot(host="192.168.178.71") as robot:
    # List available models
    models = robot.get_available_ai_models()
    print(f"Available: {list(models.keys())}")

    # Switch to YOLO (waits for confirmation)
    if robot.set_ai_model("yolov8n"):
        print("Model switched to yolov8n")
    else:
        print("Model switch timed out")

    # Subscribe to get detections
    sub = robot.subscribe_ai_detections(on_detection)
    time.sleep(5)
    sub.unsubscribe()

    # Switch to pose estimation
    if robot.set_ai_model("human-pose-estimation", timeout=5.0):
        print("Model switched to pose estimation")

    sub = robot.subscribe_ai_detections(on_pose)
    time.sleep(5)
    sub.unsubscribe()
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_name` | `str` | *required* | Name of the model to switch to |
| `timeout` | `float` | `5.0` | Maximum time to wait for confirmation |

**Returns:** `bool` - `True` if model switch confirmed, `False` if timeout.

!!! note "Model Switch Delay"
    Switching models causes a brief interruption (~200-500ms)
    as the neural network pipeline rebuilds on the OAK-D Lite.

### Advanced AI Configuration

For more control, use `set_ai_config()`:

```python
with Robot(host="192.168.178.71") as robot:
    robot.set_ai_config(
        model="yolov8n",
        confidence=0.5,  # Detection threshold (0.0-1.0)
    )
```

!!! warning "Not Synchronous"
    Unlike `set_ai_model()`, the `set_ai_config()` method does not wait
    for confirmation. Use `set_ai_model()` when you need to ensure the
    model is loaded before proceeding.

### Track Current Model

Subscribe to model change notifications:

```python
def on_model_change(info):
    print(f"Now using: {info['name']} ({info['type']})")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_current_ai_model(on_model_change)

    # Switch models - callback will fire when switch completes
    robot.set_ai_model("yolov8n")
    robot.set_ai_model("human-pose-estimation")

    sub.unsubscribe()
```

---

## Segmentation Models

Segmentation models support two output modes:

### BBox Mode (Default, Lightweight)

Returns bounding boxes around detected segments:

```python
robot.set_ai_config(
    model="deeplabv3",
    segmentation_mode="bbox"
)
```

Result:

```python
{
    "result": {
        "mode": "bbox",
        "image_size": [256, 256],
        "classes_detected": [0, 15],
        "bboxes": [
            {"class_id": 15, "bbox": {"xmin": 0.2, ...}}
        ]
    }
}
```

### Mask Mode (Detailed)

Returns full segmentation mask as RLE (Run-Length Encoded):

```python
from pib3.backends import rle_decode

robot.set_ai_config(
    model="deeplabv3",
    segmentation_mode="mask",
    segmentation_target_class=15  # Person class
)

def on_segmentation(data):
    if data['type'] == 'segmentation':
        result = data['result']
        if result.get('mode') == 'mask':
            # Decode RLE to numpy mask
            mask = rle_decode(result['mask_rle'])
            print(f"Mask shape: {mask.shape}")
            # mask is binary numpy array (0 or 1)

sub = robot.subscribe_ai_detections(on_segmentation)
```

---

## IMU Sensor Data

Access accelerometer and gyroscope from the OAK-D Lite's BMI270 IMU.

!!! info "Data Source"
    All IMU data types (`full`, `accelerometer`, `gyroscope`) subscribe to the
    same `/imu/data` ROS topic. The `accelerometer` and `gyroscope` options
    filter the data client-side for convenience.

### Full IMU Data

Get both accelerometer and gyroscope:

```python
def on_imu(data):
    """Callback receives full IMU data."""
    # Accelerometer (m/s²)
    accel = data['linear_acceleration']
    print(f"Accel: x={accel['x']:.2f}, y={accel['y']:.2f}, z={accel['z']:.2f}")

    # Gyroscope (rad/s)
    gyro = data['angular_velocity']
    print(f"Gyro: x={gyro['x']:.4f}, y={gyro['y']:.4f}, z={gyro['z']:.4f}")

with Robot(host="192.168.178.71") as robot:
    # Set frequency first (optional)
    robot.set_imu_frequency(100)  # 100 Hz

    sub = robot.subscribe_imu(on_imu, data_type="full")

    import time
    time.sleep(5)

    sub.unsubscribe()
```

### Accelerometer Only

Get just acceleration data in a simplified format:

```python
def on_accel(data):
    """Callback receives accelerometer data."""
    # Data is in Vector3Stamped-like format
    vec = data['vector']
    header = data['header']
    print(f"Accel: {vec['x']:.2f}, {vec['y']:.2f}, {vec['z']:.2f} m/s²")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_imu(on_accel, data_type="accelerometer")
    time.sleep(3)
    sub.unsubscribe()
```

### Gyroscope Only

```python
def on_gyro(data):
    """Callback receives gyroscope data."""
    vec = data['vector']
    print(f"Gyro: {vec['x']:.4f}, {vec['y']:.4f}, {vec['z']:.4f} rad/s")

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_imu(on_gyro, data_type="gyroscope")
    time.sleep(3)
    sub.unsubscribe()
```

### Setting IMU Frequency

The BMI270 supports specific frequencies:

```python
with Robot(host="192.168.178.71") as robot:
    # Valid frequencies: 25, 50, 100, 200, 250 Hz
    robot.set_imu_frequency(100)

    # Note: BMI270 rounds DOWN to nearest valid frequency
    # Request 99Hz → Get 50Hz
    # Request 150Hz → Get 100Hz
```

### IMU Data Format

**Full IMU (`data_type="full"`):**

```python
{
    "header": {
        "stamp": {"sec": 1234567890, "nanosec": 123456789},
        "frame_id": "imu_frame"
    },
    "linear_acceleration": {"x": 0.05, "y": -0.02, "z": 9.81},
    "angular_velocity": {"x": 0.001, "y": -0.002, "z": 0.0005},
    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
    "linear_acceleration_covariance": [...],
    "angular_velocity_covariance": [...],
    "orientation_covariance": [...]
}
```

**Accelerometer only (`data_type="accelerometer"`):**

```python
{
    "header": {...},
    "vector": {"x": 0.05, "y": -0.02, "z": 9.81}
}
```

**Gyroscope only (`data_type="gyroscope"`):**

```python
{
    "header": {...},
    "vector": {"x": 0.001, "y": -0.002, "z": 0.0005}
}
```

---

## Complete Example: Multi-Model AI Demo

Demonstrates switching between detection and pose estimation:

```python
from pib3 import Robot
import time

def test_ai_models(robot):
    """Test multiple AI models in sequence."""

    # ===== Object Detection =====
    print("=" * 50)
    print("Testing OBJECT DETECTION (yolov8n)")
    print("=" * 50)

    detection_count = 0

    def on_detection(data):
        nonlocal detection_count
        detection_count += 1
        if detection_count <= 3:
            detections = data.get('result', {}).get('detections', [])
            print(f"  Frame {detection_count}: {len(detections)} objects detected")
            for det in detections[:2]:
                print(f"    - Class {det['label']} (conf: {det['confidence']:.2f})")

    # Switch model (synchronous - waits for confirmation)
    if robot.set_ai_model("yolov8n", timeout=3.0):
        print("Model switched to yolov8n")
    else:
        print("Model switch timeout")
        return

    # Run detection
    sub = robot.subscribe_ai_detections(on_detection)
    time.sleep(5)
    sub.unsubscribe()
    print(f"Total frames: {detection_count}")

    # ===== Pose Estimation =====
    print("\n" + "=" * 50)
    print("Testing POSE ESTIMATION (human-pose-estimation)")
    print("=" * 50)

    pose_count = 0

    def on_pose(data):
        nonlocal pose_count
        pose_count += 1
        if pose_count <= 3:
            keypoints = data.get('result', {}).get('keypoints', [])
            detected = data.get('result', {}).get('detected_count', len(keypoints))
            print(f"  Frame {pose_count}: {detected} keypoints detected")

    # Switch to pose model
    if robot.set_ai_model("human-pose-estimation", timeout=3.0):
        print("Model switched to pose estimation")
    else:
        print("Model switch timeout")
        return

    # Run pose estimation
    sub = robot.subscribe_ai_detections(on_pose)
    time.sleep(5)
    sub.unsubscribe()
    print(f"Total frames: {pose_count}")

# Run the demo
with Robot(host="192.168.178.71") as robot:
    test_ai_models(robot)
```

---

## Complete Example: Vision-Based Control

Combine camera, AI, and robot control:

```python
from pib3 import Robot
import time

class VisionController:
    def __init__(self, robot):
        self.robot = robot
        self.person_detected = False
        self.last_bbox = None

    def on_detection(self, data):
        """React to AI detections."""
        if data['type'] != 'detection':
            return

        for det in data['result']['detections']:
            if det['label'] == 0 and det['confidence'] > 0.7:  # Person (COCO class 0)
                self.person_detected = True
                self.last_bbox = det['bbox']
                print(f"Person at: {det['bbox']}")

    def run(self, duration=30):
        """Run vision-based control loop."""
        # Switch to YOLO model for person detection
        if not self.robot.set_ai_model("yolov8n"):
            print("Failed to switch model")
            return

        # Subscribe to detections
        sub = self.robot.subscribe_ai_detections(self.on_detection)

        try:
            start = time.time()
            while time.time() - start < duration:
                if self.person_detected and self.last_bbox:
                    # Track person by moving head
                    center_x = (self.last_bbox['xmin'] + self.last_bbox['xmax']) / 2

                    if center_x < 0.4:
                        # Person on left, turn head left
                        self.robot.set_joint(Joint.TURN_HEAD, 60)
                    elif center_x > 0.6:
                        # Person on right, turn head right
                        self.robot.set_joint(Joint.TURN_HEAD, 40)
                    else:
                        # Person centered
                        self.robot.set_joint(Joint.TURN_HEAD, 50)

                    self.person_detected = False

                time.sleep(0.1)
        finally:
            sub.unsubscribe()

# Usage
from pib3 import Robot, Joint
with Robot(host="192.168.178.71") as robot:
    controller = VisionController(robot)
    controller.run(duration=30)
```

---

## API Reference Summary

### Camera Methods

| Method | Description |
|--------|-------------|
| `subscribe_camera_image(callback)` | Stream camera images (JPEG bytes) |
| `set_camera_config(fps, quality, resolution)` | Configure camera settings |

### AI Detection Methods

| Method | Description |
|--------|-------------|
| `subscribe_ai_detections(callback)` | Subscribe to AI inference results |
| `get_available_ai_models(timeout)` | List available models on the robot |
| `set_ai_model(model_name, timeout)` | Switch AI model (synchronous, returns `bool`) |
| `set_ai_config(model, confidence, ...)` | Configure AI settings (not synchronous) |
| `subscribe_current_ai_model(callback)` | Track model changes |

### IMU Methods

| Method | Description |
|--------|-------------|
| `subscribe_imu(callback, data_type)` | Subscribe to IMU data (`full`, `accelerometer`, `gyroscope`) |
| `set_imu_frequency(frequency)` | Set sampling rate (25, 50, 100, 200, 250 Hz) |

### Helper Functions

| Function | Description |
|----------|-------------|
| `rle_decode(rle)` | Decode RLE segmentation mask to numpy array |

---

## Troubleshooting

### No Camera Data

1. Check OAK-D Lite is connected to the robot
2. Verify the ROS camera node is running: `ros2 topic list | grep camera`
3. Check network connectivity to the robot
4. Verify rosbridge is running on port 9090

### AI Inference Slow

1. Some models are more demanding - try `mobilenet-ssd` for speed
2. Check robot CPU/NPU usage
3. Reduce camera resolution
4. YOLOv8 models are fast; ResNet/DeepLab are slower

### Model Switch Timeout

1. Increase timeout: `robot.set_ai_model("model", timeout=10.0)`
2. Check that the model exists: `robot.get_available_ai_models()`
3. Verify `/ai/current_model` topic is publishing: `ros2 topic echo /ai/current_model`

### IMU Data Issues

1. IMU needs calibration at startup - keep robot stationary for first few seconds
2. Use `data_type="full"` to verify data is coming through
3. Check `/imu/data` topic: `ros2 topic echo /imu/data`
4. Valid frequencies are 25, 50, 100, 200, 250 Hz only

### Connection Issues

1. Verify robot IP: `ping 192.168.178.71`
2. Check rosbridge port: `nc -zv 192.168.178.71 9090`
3. Increase connection timeout: `Robot(host="...", timeout=10.0)`

---

## Next Steps

- [Controlling the Robot](controlling-robot.md) - Joint control basics
- [Image to Trajectory](image-to-trajectory.md) - Drawing with IK
- [Custom Configurations](custom-configurations.md) - Fine-tune parameters
