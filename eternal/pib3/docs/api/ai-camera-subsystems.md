# AI and Camera Subsystems

Simplified, high-level APIs for accessing AI inference and camera streaming on the PIB robot.

## Overview

The subsystem APIs (`robot.ai` and `robot.camera`) provide a simpler alternative to manual subscription management. They automatically handle subscription lifecycle and provide typed results.

| Approach | When to Use |
|----------|-------------|
| **Subsystem API** (`robot.ai`, `robot.camera`) | Most use cases—simple, auto-managed |
| **Raw subscriptions** (`subscribe_ai_detections()`) | Custom buffering, multiple callbacks, advanced scenarios |

```python
from pib3 import Robot, AIModel

with Robot(host="172.26.34.149") as robot:
    # Subsystem API - simple and direct
    robot.ai.set_model(AIModel.HAND)
    for hand in robot.ai.get_hand_landmarks():
        print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
```

---

## AISubsystem (`robot.ai`)

Access AI inference results from the OAK-D Lite camera with automatic subscription management.

### Quick Start

```python
from pib3 import Robot, AIModel

with Robot(host="172.26.34.149") as robot:
    # Set model (waits for confirmation)
    robot.ai.set_model(AIModel.YOLOV8N)
    
    # Get detections (waits automatically for results)
    for det in robot.ai.get_detections():
        print(f"{det.label}: {det.confidence:.0%} at {det.bbox}")
    
    # Check performance
    print(f"FPS: {robot.ai.fps:.1f}, Latency: {robot.ai.avg_latency_ms:.1f}ms")
```

### Methods

#### `set_model()`

Switch AI model on the OAK-D Lite camera.

```python
def set_model(
    model: Union[AIModel, str],
    timeout: float = 5.0
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | `AIModel` or `str` | *required* | Model to load (prefer enum for IDE support) |
| `timeout` | `float` | `5.0` | Max wait time for confirmation |

**Returns:** `True` if switch confirmed, `False` if timeout.

```python
from pib3 import AIModel

# Using enum (recommended - IDE autocomplete)
robot.ai.set_model(AIModel.HAND)
robot.ai.set_model(AIModel.YOLOV8N)
robot.ai.set_model(AIModel.POSE)

# String also works
robot.ai.set_model("mobilenet-ssd")
```

---

#### `get_detections()`

Get object detections from detection models (YOLO, MobileNet-SSD, etc.).

```python
def get_detections(timeout: float = 5.0) -> List[Detection]
```

Waits automatically for results if buffer is empty.

```python
robot.ai.set_model(AIModel.YOLOV8N)
for det in robot.ai.get_detections():
    print(f"Found {det.label} ({det.confidence:.0%})")
    print(f"  BBox: {det.bbox.center}")
```

---

#### `get_hand_landmarks()`

Get hand tracking results with finger angles.

```python
def get_hand_landmarks(timeout: float = 5.0) -> List[HandLandmarks]
```

```python
robot.ai.set_model(AIModel.HAND)
for hand in robot.ai.get_hand_landmarks():
    print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
    
    # Convert to servo values for robot hand control
    servos = hand.finger_angles.to_servo_values()
    robot.set_joints({
        "index_left_stretch": servos["index"],
        "middle_left_stretch": servos["middle"],
    })
```

---

#### `get_poses()`

Get body pose estimation results.

```python
def get_poses(timeout: float = 5.0) -> List[PoseKeypoints]
```

```python
robot.ai.set_model(AIModel.POSE)
for pose in robot.ai.get_poses():
    if pose.nose:
        print(f"Nose at: ({pose.nose.x:.2f}, {pose.nose.y:.2f})")
```

---

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `model` | `Optional[str]` | Currently active model name |
| `fps` | `float` | Current inference frames per second |
| `avg_latency_ms` | `float` | Average inference latency in milliseconds |

### Other Methods

| Method | Description |
|--------|-------------|
| `clear()` | Clear buffered results |
| `stop()` | Stop AI inference (unsubscribe) |

---

## CameraSubsystem (`robot.camera`)

Access raw camera frames from the OAK-D Lite with automatic subscription management.

### Quick Start

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Get latest frame
    frame = robot.camera.get_frame()
    if frame:
        print(f"Frame {frame.frame_id}, {len(frame.jpeg_bytes)} bytes")
        
        # Decode to numpy (requires OpenCV)
        img = frame.to_numpy()
```

### Methods

#### `get_frame()`

Get the latest camera frame.

```python
def get_frame(timeout: float = 5.0) -> Optional[CameraFrame]
```

```python
frame = robot.camera.get_frame()
if frame:
    # Save JPEG directly
    with open("capture.jpg", "wb") as f:
        f.write(frame.jpeg_bytes)
    
    # Or decode to numpy for processing
    img = frame.to_numpy()  # Requires OpenCV
```

---

#### `get_frames()`

Get all buffered frames and clear the buffer.

```python
def get_frames() -> List[CameraFrame]
```

---

#### `configure()`

Configure camera settings.

```python
def configure(
    fps: Optional[int] = None,
    quality: Optional[int] = None,
    resolution: Optional[tuple] = None,
) -> None
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `fps` | `int` | Frames per second (e.g., 30) |
| `quality` | `int` | JPEG quality 1-100 (e.g., 80) |
| `resolution` | `tuple` | (width, height) e.g., (1280, 720) |

```python
robot.camera.configure(fps=30, quality=80, resolution=(1280, 720))
```

!!! warning "Brief Interruption"
    Changing settings causes ~100-200ms stream interruption.

---

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `frame_count` | `int` | Total number of frames received |

### Other Methods

| Method | Description |
|--------|-------------|
| `stop()` | Stop camera streaming (unsubscribe) |

---

## AIModel Enum

Type-safe enum for available AI models on the OAK-D Lite.

```python
from pib3 import AIModel
```

### Detection Models

| Enum Value | String | Description |
|------------|--------|-------------|
| `AIModel.MOBILENET_SSD` | `"mobilenet-ssd"` | Fast general detection |
| `AIModel.YOLOV6N` | `"yolov6n"` | YOLOv6 nano |
| `AIModel.YOLOV8N` | `"yolov8n"` | YOLOv8 nano |
| `AIModel.YOLO11N` | `"yolo11n"` | YOLO11 nano |
| `AIModel.YOLO11S` | `"yolo11s"` | YOLO11 small |

### Hand Tracking

| Enum Value | String | Description |
|------------|--------|-------------|
| `AIModel.HAND` | `"hand"` | Hand landmark detection with finger angles |

### Pose Estimation

| Enum Value | String | Description |
|------------|--------|-------------|
| `AIModel.POSE` | `"pose"` | Body pose estimation |
| `AIModel.POSE_YOLO` | `"pose_yolo"` | YOLO-based pose |

### Segmentation

| Enum Value | String | Description |
|------------|--------|-------------|
| `AIModel.DEEPLABV3` | `"deeplabv3"` | Semantic segmentation |
| `AIModel.YOLOV8N_SEG` | `"yolov8n-seg"` | Instance segmentation |
| `AIModel.FASTSAM` | `"fastsam"` | Fast segment anything |

### Other Models

| Enum Value | String | Description |
|------------|--------|-------------|
| `AIModel.GAZE` | `"gaze"` | Gaze estimation |
| `AIModel.LINES` | `"lines"` | Line detection |

---

## Type Reference

### Detection

Object detection result from detection models.

```python
@dataclass
class Detection:
    label_id: int           # Numeric class ID (e.g., 0 for person in COCO)
    confidence: float       # Detection confidence (0.0 to 1.0)
    bbox: BoundingBox       # Bounding box in normalized coordinates
    label: str              # Human-readable class name (auto-resolved from COCO)
    mask_rle: Optional[Dict] # Optional RLE-encoded segmentation mask
```

**Properties:**

```python
det.label          # "person"
det.confidence     # 0.92
det.bbox.center    # (0.5, 0.3) - center point
det.bbox.width     # 0.2
det.bbox.height    # 0.4
det.bbox.to_pixels(640, 480)  # (x1, y1, x2, y2) in pixels
```

---

### HandLandmarks

Hand tracking result with 21 landmarks and calculated finger angles.

```python
@dataclass
class HandLandmarks:
    landmarks: np.ndarray          # Shape (21, 2) normalized coordinates
    keypoints: List[Keypoint]      # 21 Keypoint objects with confidence
    handedness: Handedness         # LEFT, RIGHT, or UNKNOWN
    confidence: float              # Overall detection confidence
    finger_angles: FingerAngles    # Calculated finger bend angles
```

**Properties:**

```python
hand.handedness           # Handedness.LEFT
hand.wrist_position       # (x, y) tuple
hand.finger_angles.index  # Index finger bend angle in degrees
```

---

### FingerAngles

Finger bend angles calculated from hand landmarks.

```python
@dataclass
class FingerAngles:
    thumb_bend: float       # Thumb flexion angle
    thumb_rotation: float   # Thumb rotation relative to palm
    index: float            # Index finger bend angle (0° = straight, 90° = bent)
    middle: float           # Middle finger bend angle
    ring: float             # Ring finger bend angle
    pinky: float            # Pinky finger bend angle
```

**Methods:**

```python
# Convert to list
angles = finger_angles.to_list()  # [thumb_bend, thumb_rot, index, middle, ring, pinky]

# Convert to servo percentages (for robot hand control)
servos = finger_angles.to_servo_values()  # {"index": 75.0, "middle": 80.0, ...}

# Use directly with robot
robot.set_joints({
    "index_left_stretch": servos["index"],
    "middle_left_stretch": servos["middle"],
})
```

---

### PoseKeypoints

Body pose estimation with 17 COCO keypoints.

```python
@dataclass
class PoseKeypoints:
    keypoints: List[Keypoint]       # 17 keypoints
    confidence: float               # Overall pose confidence
    bbox: Optional[BoundingBox]     # Bounding box around person
```

**Convenience properties:**

```python
pose.nose             # Keypoint or None
pose.left_shoulder    # Keypoint or None
pose.right_shoulder   # Keypoint or None
pose.get_keypoint(5)  # Get by COCO index (0-16)
```

---

### CameraFrame

Single camera frame with metadata.

```python
@dataclass
class CameraFrame:
    jpeg_bytes: bytes       # Raw JPEG image data
    frame_id: int           # Sequential frame number
    timestamp_ns: int       # Timestamp in nanoseconds
```

**Properties and Methods:**

```python
frame.timestamp        # Timestamp as float seconds
frame.to_numpy()       # Decode to BGR numpy array (requires OpenCV)
```

---

## Complete Example

Hand tracking with robot hand mirroring:

```python
from pib3 import Robot, AIModel
import time

with Robot(host="172.26.34.149") as robot:
    # Enable hand tracking
    if not robot.ai.set_model(AIModel.HAND):
        print("Failed to set model")
        exit(1)
    
    print("Tracking hands... (Ctrl+C to stop)")
    
    try:
        while True:
            # Get hand landmarks (waits automatically)
            hands = robot.ai.get_hand_landmarks(timeout=1.0)
            
            for hand in hands:
                print(f"\n{hand.handedness}:")
                print(f"  Index: {hand.finger_angles.index:.0f}°")
                print(f"  Middle: {hand.finger_angles.middle:.0f}°")
                
                # Mirror to robot hand
                servos = hand.finger_angles.to_servo_values()
                if hand.handedness.value == "left":
                    robot.set_joints({
                        "index_left_stretch": servos["index"],
                        "middle_left_stretch": servos["middle"],
                        "ring_left_stretch": servos["ring"],
                        "pinky_left_stretch": servos["pinky"],
                    })
            
            # Show stats
            print(f"FPS: {robot.ai.fps:.1f}, Latency: {robot.ai.avg_latency_ms:.1f}ms")
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        robot.ai.stop()
```

---

## See Also

- [Camera, AI, and IMU Tutorial](../tutorials/camera-ai-imu.md) - Step-by-step guide
- [RealRobotBackend](backends/robot.md) - Low-level subscription methods
- [Types Reference](types.md) - All pib3 types
