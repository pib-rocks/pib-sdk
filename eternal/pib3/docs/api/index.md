# API Reference

Complete API documentation for pib3.

## Module Overview

```
pib3
├── Core Functions
│   ├── generate_trajectory()    # One-shot image to trajectory
│   ├── image_to_sketch()        # Image to 2D strokes
│   └── sketch_to_trajectory()   # 2D strokes to 3D trajectory
│
├── Core Types
│   ├── Joint                    # Enum for joint names (IDE autocomplete)
│   ├── AIModel                  # Enum for AI models (YOLOV8N, HAND, POSE, etc.)
│   ├── Stroke                   # Single continuous line
│   ├── Sketch                   # Collection of strokes
│   └── Trajectory               # Robot joint positions
│
├── AI/Camera Types
│   ├── Detection                # Object detection result
│   ├── HandLandmarks            # Hand tracking with finger angles
│   ├── FingerAngles             # Finger bend angles → servo values
│   ├── PoseKeypoints            # Body pose estimation
│   └── CameraFrame              # Single camera frame
│
├── Configuration
│   ├── TrajectoryConfig         # Main configuration
│   ├── PaperConfig              # Drawing surface settings
│   ├── IKConfig                 # IK solver settings
│   └── ImageConfig              # Image processing settings
│
├── Backends
│   ├── Robot / RealRobotBackend # Real robot via rosbridge
│   │   ├── robot.ai             # AI subsystem (simplified API)
│   │   └── robot.camera         # Camera subsystem (simplified API)
│   └── Webots / WebotsBackend   # Webots simulator
│
├── Kinematics
│   ├── PibLeft / PibRight       # DH models for arm kinematics
│   └── camera_to_base()         # Coordinate transformations
│
└── Hand Poses
    ├── HandPose                 # Preset hand poses (enum)
    ├── LEFT_HAND_JOINTS         # Left hand joint list
    └── RIGHT_HAND_JOINTS        # Right hand joint list
```

## Quick Reference

### Main Functions

| Function | Description |
|----------|-------------|
| [`generate_trajectory()`](trajectory.md#pib3.generate_trajectory) | Convert image directly to trajectory |
| [`image_to_sketch()`](image.md#pib3.image.image_to_sketch) | Convert image to 2D strokes |
| [`sketch_to_trajectory()`](trajectory.md#pib3.trajectory.sketch_to_trajectory) | Convert strokes to 3D trajectory |
| [`play_audio()`](audio.md#play_audio) | Play raw audio data |
| [`play_file()`](audio.md#play_file) | Play audio from WAV file |
| [`speak()`](audio.md#speak) | Text-to-speech synthesis |

### Core Types

| Type | Description |
|------|-------------|
| [`Stroke`](types.md#pib3.types.Stroke) | Single continuous drawing line |
| [`Sketch`](types.md#pib3.types.Sketch) | Collection of strokes |
| [`Trajectory`](trajectory.md#pib3.trajectory.Trajectory) | Robot joint positions over time |
| [`Joint`](types.md) | Enum for joint names with IDE autocomplete |
| [`AIModel`](ai-camera-subsystems.md#aimodel-enum) | Enum for AI models with IDE autocomplete |

### Backends

| Backend | Description |
|---------|-------------|
| [`RealRobotBackend`](backends/robot.md) | Control real PIB robot |
| [`WebotsBackend`](backends/webots.md) | Webots simulator |

### AI/Camera Subsystems

| Subsystem | Description |
|-----------|-------------|
| [`robot.ai`](ai-camera-subsystems.md#aisubsystem-robotai) | AI inference with auto-managed subscriptions |
| [`robot.camera`](ai-camera-subsystems.md#camerasubsystem-robotcamera) | Camera streaming with auto-managed subscriptions |

## Import Examples

```python
import pib3
from pib3 import Robot, Joint, Trajectory

# Generate trajectory
trajectory = pib3.generate_trajectory("image.png")

# Control robot with IDE tab completion
with Robot(host="172.26.34.149") as robot:
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)
    robot.run_trajectory(trajectory)
```

## Documentation Sections

<div class="grid cards" markdown>

-   :material-vector-polyline: **[Core Types](types.md)**

    ---

    Stroke, Sketch - Data structures for 2D drawings

-   :material-cog: **[Configuration](config.md)**

    ---

    All configuration dataclasses and options

-   :material-image: **[Image Processing](image.md)**

    ---

    Image to sketch conversion functions

-   :material-robot-industrial: **[Trajectory Generation](trajectory.md)**

    ---

    Trajectory class and IK functions

-   :material-chip: **[Backends](backends/base.md)**

    ---

    Robot and Webots backends

-   :material-tools: **[Tools](tools.md)**

    ---

    Calibration and utility tools

-   :material-hand-wave: **[Hand Poses](hand-poses.md)**

    ---

    Hand position presets and functions

-   :material-volume-high: **[Audio System](audio.md)**

    ---

    Audio playback, TTS, and recording

-   :material-robot: **[AI & Camera Subsystems](ai-camera-subsystems.md)**

    ---

    Simplified AI inference and camera APIs

-   :material-axis-arrow: **[Kinematics](kinematics.md)**

    ---

    DH models and coordinate transformations

</div>
