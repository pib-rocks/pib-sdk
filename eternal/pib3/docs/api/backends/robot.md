# Real Robot Backend

Control the physical PIB robot via rosbridge websocket connection.

## Overview

`RealRobotBackend` connects to a PIB robot's ROS system via rosbridge and provides joint control and trajectory execution.

## RealRobotBackend Class

::: pib3.backends.robot.RealRobotBackend
    options:
      show_root_heading: true
      show_source: false
      members: false

---

## Quick Start

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)  # IDE tab completion
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    robot.run_trajectory("trajectory.json")
```

---

## Constructor

```python
Robot(
    host: str = "172.26.34.149",
    port: int = 9090,
    timeout: float = 5.0,
    motor_mode: str = "direct",
)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | `str` | `"172.26.34.149"` | IP address of the robot. |
| `port` | `int` | `9090` | Rosbridge websocket port. |
| `timeout` | `float` | `5.0` | Connection timeout in seconds. |
| `motor_mode` | `str` | `"direct"` | Motor control mode: `"direct"` (Tinkerforge, auto-discovered) or `"ros"` (via rosbridge). |

**Example:**

```python
from pib3 import Robot

# Default connection (direct Tinkerforge control, auto-discovered)
robot = Robot()

# Custom host
robot = Robot(host="192.168.1.100")

# All parameters
robot = Robot(
    host="192.168.1.100",
    port=9090,
    timeout=10.0,
)

# Use ROS for motor control instead of Tinkerforge
robot = Robot(host="192.168.1.100", motor_mode="ros")
```

---

## Subsystem Properties

The robot provides high-level subsystem APIs for simplified access to AI and camera features.

### `robot.ai`

Access the [AI subsystem](../ai-camera-subsystems.md#aisubsystem-robotai) for AI inference with auto-managed subscriptions.

```python
with Robot(host="172.26.34.149") as robot:
    robot.ai.set_model(AIModel.HAND)
    for hand in robot.ai.get_hand_landmarks():
        print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
```

### `robot.camera`

Access the [camera subsystem](../ai-camera-subsystems.md#camerasubsystem-robotcamera) for camera streaming with auto-managed subscriptions.

```python
with Robot(host="172.26.34.149") as robot:
    frame = robot.camera.get_frame()
    if frame:
        img = frame.to_numpy()  # Requires OpenCV
```

!!! tip "Subsystems vs Raw Subscriptions"
    For most use cases, use `robot.ai` and `robot.camera` instead of the low-level 
    `subscribe_ai_detections()` and `subscribe_camera_image()` methods. The subsystems 
    automatically manage subscription lifecycle and provide typed return values.

---

## Connection

### connect()

Establish websocket connection to the robot's rosbridge server.

```python
def connect(self) -> None
```

**Raises:**

- `ConnectionError`: If unable to connect within timeout

**Example:**

```python
from pib3 import Robot

robot = Robot(host="172.26.34.149")

try:
    robot.connect()
    print(f"Connected: {robot.is_connected}")
    # ... use robot ...
finally:
    robot.disconnect()
```

### Using Context Manager

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)
# Automatically disconnected
```

---

## Joint Control

The robot backend inherits all methods from [`RobotBackend`](base.md). Key methods:

### set_joint()

Set a single joint position.

```python
def set_joint(
    self,
    motor_name: Union[str, Joint],
    position: float,
    unit: Literal["percent", "rad", "deg"] = "percent",
    async_: bool = False,
    timeout: float = 2.0,
    tolerance: Optional[float] = None,
    speed: Optional[float] = None,
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum (e.g., `Joint.ELBOW_LEFT`). |
| `position` | `float` | *required* | Target position in specified unit. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `False` | If `False` (default), poll until joint reaches target. |
| `timeout` | `float` | `2.0` | Max wait time when `async_=False`. |
| `tolerance` | `float` | `None` | Acceptable error (default: 2%, 3°, or 0.05 rad). |
| `speed` | `float` or `None` | `None` | Movement speed in deg/s. `None` uses the servo channel's current motion config (default ≈ 150°/s on the direct path — see [`DEFAULT_MOTION_VELOCITY`](#default-motion-config)). |

**Example:**

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joint(Joint.TURN_HEAD, 50.0)               # Percentage
    robot.set_joint(Joint.ELBOW_LEFT, 1.25, unit="rad")  # Radians
    robot.set_joint(Joint.ELBOW_LEFT, -30.0, unit="deg") # Degrees

    # Fire and forget (default waits for completion)
    robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=True)
```

### set_joints()

Set multiple joint positions simultaneously.

```python
def set_joints(
    self,
    positions: Dict[Union[str, Joint], float],
    unit: Literal["percent", "rad", "deg"] = "percent",
    async_: bool = False,
    timeout: float = 2.0,
    tolerance: Optional[float] = None,
    speed: Optional[float] = None,
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `positions` | `Dict[str\|Joint, float]` | *required* | Target positions. A `HandPose` or sequence raises `TypeError` — use `set_joints_pose()` / `set_joints_sequence()` instead. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `False` | If `False` (default), poll until joints reach targets. |
| `timeout` | `float` | `2.0` | Max wait time when `async_=False`. |
| `tolerance` | `float` | `None` | Acceptable error. |
| `speed` | `float` or `None` | `None` | Movement speed in deg/s. On the direct path this rewrites the servo's shared motion config — see [base class docs](base.md#set_joints). |

**Example:**

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 30.0,
        Joint.ELBOW_LEFT: 60.0,
    })
```

### get_joint()

Read a single joint position.

```python
def get_joint(
    self,
    motor_name: Union[str, Joint],
    unit: Literal["percent", "rad", "deg"] = "percent",
    timeout: Optional[float] = None,
) -> Optional[float]
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Return unit. |
| `timeout` | `float` | `5.0` | Max wait time for ROS messages (seconds). |

**Returns:** `float` or `None` - Current position.

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    pos_rad = robot.get_joint(Joint.ELBOW_LEFT, unit="rad")
```

### get_joints()

Read multiple joint positions.

```python
def get_joints(
    self,
    motor_names: Optional[List[Union[str, Joint]]] = None,
    unit: Literal["percent", "rad", "deg"] = "percent",
    timeout: Optional[float] = None,
) -> Dict[str, float]
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_names` | `List[str\|Joint]` or `None` | `None` | Motors to query. `None` returns all. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Return unit. |
| `timeout` | `float` | `5.0` | Max wait time for ROS messages (seconds). |

**Returns:** `Dict[str, float]` - Motor names mapped to positions.

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    arm = robot.get_joints([Joint.ELBOW_LEFT, Joint.WRIST_LEFT])
    all_joints = robot.get_joints()
```

---

## Trajectory Execution

### run_trajectory()

```python
def run_trajectory(
    self,
    trajectory: Union[str, Path, Trajectory],
    rate_hz: float = 20.0,
    progress_callback: Optional[Callable[[int, int], None]] = None,
) -> bool
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trajectory` | `str`, `Path`, `Trajectory` | *required* | Trajectory file or object. |
| `rate_hz` | `float` | `20.0` | Waypoints per second. |
| `progress_callback` | `Callable[[int, int], None]` | `None` | Progress callback `(current, total)`. |

```python
from pib3 import Robot, Trajectory

with Robot(host="172.26.34.149") as robot:
    robot.run_trajectory("trajectory.json")

    # With progress
    robot.run_trajectory(
        trajectory,
        rate_hz=20.0,
        progress_callback=lambda c, t: print(f"\r{c}/{t}", end=""),
    )
```

---

## Save and Restore Poses

```python
import json
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    pose = robot.get_joints()                    # Save
    with open("pose.json", "w") as f:
        json.dump(pose, f)

    with open("pose.json") as f:
        robot.set_joints(json.load(f))           # Restore
```

---

## Camera Streaming

### subscribe_camera_image()

Subscribe to camera images (raw JPEG bytes).

```python
def subscribe_camera_image(callback: Callable[[bytes], None]) -> roslibpy.Topic
```

Call `.unsubscribe()` on the returned topic to stop streaming.

```python
import cv2, numpy as np

def on_frame(jpeg_bytes):
    frame = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)

with Robot(host="192.168.178.71") as robot:
    sub = robot.subscribe_camera_image(on_frame)
    time.sleep(10)
    sub.unsubscribe()
```

### set_camera_config()

```python
def set_camera_config(
    fps: Optional[int] = None,
    quality: Optional[int] = None,      # JPEG quality 1-100
    resolution: Optional[tuple] = None,  # (width, height)
) -> None
```

!!! note
    Changing settings causes ~100-200ms stream interruption.

---

## AI Detection

AI inference runs only while subscribers are active. Unsubscribe to stop and save resources.

### subscribe_ai_detections()

```python
def subscribe_ai_detections(callback: Callable[[dict], None]) -> roslibpy.Topic
```

Callback receives:
```python
{"model": "mobilenet-ssd", "type": "detection", "frame_id": 42,
 "result": {"detections": [{"label": 15, "confidence": 0.92, "bbox": {...}}]}}
```

```python
def on_detection(data):
    for det in data['result']['detections']:
        print(f"Class {det['label']} @ {det['confidence']:.0%}")

sub = robot.subscribe_ai_detections(on_detection)
time.sleep(10)
sub.unsubscribe()
```

### set_ai_model()

Switch AI model (synchronous, waits for confirmation).

```python
def set_ai_model(model_name: str, timeout: float = 5.0) -> bool
```

```python
if robot.set_ai_model("yolov8n"):
    print("Model ready!")
```

!!! note
    Model switching causes ~200-500ms interruption.

### set_ai_config()

```python
def set_ai_config(
    model: Optional[str] = None,
    confidence: Optional[float] = None,           # 0.0-1.0
    segmentation_mode: Optional[str] = None,      # "bbox" or "mask"
    segmentation_target_class: Optional[int] = None,
) -> None
```

### get_available_ai_models()

```python
models = robot.get_available_ai_models()  # Returns dict of model info
```

---

## IMU Sensor

Access BMI270 IMU data from OAK-D Lite.

### subscribe_imu()

```python
def subscribe_imu(
    callback: Callable[[dict], None],
    data_type: str = "full",  # "full", "accelerometer", or "gyroscope"
) -> roslibpy.Topic
```

| `data_type` | Callback receives |
|-------------|-------------------|
| `"full"` | `{"linear_acceleration": {x,y,z}, "angular_velocity": {x,y,z}}` |
| `"accelerometer"` | `{"vector": {x,y,z}}` |
| `"gyroscope"` | `{"vector": {x,y,z}}` |

```python
def on_imu(data):
    accel = data['linear_acceleration']
    print(f"Accel: {accel['x']:.2f}, {accel['y']:.2f}, {accel['z']:.2f}")

sub = robot.subscribe_imu(on_imu)
time.sleep(5)
sub.unsubscribe()
```

### set_imu_frequency()

```python
def set_imu_frequency(frequency: int) -> None  # 25, 50, 100, 200, or 250 Hz
```

---

## Helper Functions

### rle_decode()

Decode RLE-encoded segmentation masks.

```python
from pib3.backends import rle_decode

mask = rle_decode(result['mask_rle'])  # Returns np.ndarray (height, width)
```

---

## Text-to-Speech

`RealRobotBackend.speak()` overrides the base implementation and prefers the robot's on-board TTS service `/play_audio_from_speech` — speech is synthesized directly on the robot, avoiding Piper inference on the client and streaming audio over rosbridge. Falls back to base-class Piper synthesis for `AudioOutput.LOCAL` or when the robot service is unavailable.

```python
def speak(
    text: str,
    output: Optional[AudioOutput] = None,
    voice: Optional[str] = None,          # Piper voice (local fallback only)
    block: bool = True,
    use_robot_tts: Optional[bool] = None, # None = auto (prefer robot when output includes ROBOT)
    language: str = "de",                 # Robot TTS language code
) -> bool
```

See [`play_audio_from_speech()`](../audio.md) for the underlying ROS service (supports `wait=False` for fire-and-forget).

---

## Low-Latency Mode

Bypass ROS for direct Tinkerforge motor control with ~5-20ms latency (vs ~100-200ms via ROS). When enabled, both `get_joint()`/`get_joints()` and `set_joint()`/`set_joints()` use direct Tinkerforge communication.

### Quick Setup

Direct Tinkerforge control is the default mode. Servo bricklets are auto-discovered on connect:

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    robot.set_joint("elbow_left", 0.5, unit="rad")  # Direct write
    pos = robot.get_joint("elbow_left", unit="rad")  # Direct read

# To use ROS for motor control instead:
with Robot(host="172.26.34.149", motor_mode="ros") as robot:
    robot.set_joint("elbow_left", 0.5, unit="rad")  # Via ROS
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `low_latency_available` | `bool` | True if connected and configured |
| `low_latency_enabled` | `bool` | Get/set enabled state. The setter **raises `RuntimeError`** when the backend is not connected or when Tinkerforge discovery fails (the flag is rolled back). |
| `low_latency_sync_to_ros` | `bool` | Get/set cache sync setting |
| `discovered_servo_uids` | `List[str]` | UIDs from last discovery |

### Default motion config

The direct Tinkerforge path uses these class-level defaults whenever `speed` is not provided. Override per call via `speed=...` or per channel via `configure_servo_channel()`.

| Constant | Default | Unit | Notes |
|----------|---------|------|-------|
| `RealRobotBackend.DEFAULT_MOTION_VELOCITY` | `15000` | 0.01 °/s | ≈ 150°/s |
| `RealRobotBackend.DEFAULT_MOTION_ACCELERATION` | `15000` | 0.01 °/s² | |
| `RealRobotBackend.DEFAULT_MOTION_DECELERATION` | `15000` | 0.01 °/s² | |

### Methods

#### discover_servo_bricklets()

```python
def discover_servo_bricklets(timeout: float = 1.0) -> List[str]
```

Returns list of Servo Bricklet V2 UIDs.

#### configure_motor_mapping()

```python
def configure_motor_mapping(
    mapping: Dict[str, Tuple[str, int]],
    reinitialize: bool = True,
) -> None
```

Configure motor-to-bricklet mapping at runtime. Auto-configures servo channels.

#### configure_servo_channel()

```python
def configure_servo_channel(
    motor_name: str,
    pulse_width_min: int = 700,
    pulse_width_max: int = 2500,
    velocity: int = 9000,
    acceleration: int = 9000,
    deceleration: int = 9000,
) -> bool
```

Configure individual servo channel settings.

### Helper Functions

```python
from pib3 import build_motor_mapping, PIB_SERVO_CHANNELS

# Build complete mapping from 3 UIDs
mapping = build_motor_mapping("UID1", "UID2", "UID3")

# Reference standard channel assignments: (bricklet_number, channel)
print(PIB_SERVO_CHANNELS["elbow_left"])  # (3, 8)
```

See the [Low-Latency Tutorial](../../tutorials/low-latency-mode.md) for complete examples.

---

## ROS Integration

| Topic/Service | Purpose |
|---------------|---------|
| `/motor_current` | Joint position feedback |
| `/apply_joint_trajectory` | Motor commands |

Commands use ROS2 JointTrajectory format with positions in centidegrees.

---

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Connection refused | Rosbridge not running | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| Connection timeout | Wrong IP or network issue | `ping <ip>`, increase `timeout` parameter |
| Joint not moving | Limits not calibrated | [Calibrate](../../getting-started/calibration.md) or use `unit="rad"` |
| `get_joint` returns None | ROS message timeout | Increase `timeout`, verify `/motor_current` topic |
