# Tutorials

Learn how to use pib3 effectively through step-by-step tutorials.

## Getting Started Tutorials

### [Image to Trajectory](image-to-trajectory.md)

Learn how to convert images to robot drawing trajectories. Covers image processing, contour extraction, and IK solving.

### [Controlling the Robot](controlling-robot.md)

Master the joint control API. Learn percentage-based control, pose saving/restoring, and verification.

### [Camera, AI Detection, and IMU](camera-ai-imu.md)

Access OAK-D Lite camera streaming, AI object detection, and IMU sensor data. Includes on-demand activation patterns.

### [Audio System](../api/audio.md)

Play audio files, record from microphones, and use text-to-speech on the robot. Supports local and robot audio devices.

Use browser-based 3D visualization. Run trajectories, use interactive mode, and add visual elements.

## Advanced Tutorials

### [Low-Latency Motor Control](low-latency-mode.md)

Bypass ROS for direct Tinkerforge motor control with ~5-20ms latency. Essential for real-time applications like hand tracking and teleoperation.

### [Working with Sketches](working-with-sketches.md)

Manipulate sketches programmatically. Create, modify, and optimize strokes for better drawings.

### [Custom Configurations](custom-configurations.md)

Fine-tune all parameters. Configure paper size, IK solver, image processing, and more.

---

## Tutorial Structure

Each tutorial includes:

- **Objectives**: What you'll learn
- **Prerequisites**: What you need before starting
- **Step-by-step instructions**: With code examples
- **Complete examples**: Ready-to-run code
- **Troubleshooting**: Common issues and solutions

## Prerequisites

Before starting the tutorials, ensure you have:

1. [Installed pib3](../getting-started/installation.md) with the features you need
2. A basic understanding of Python
3. (For robot tutorials) Access to a PIB robot or Webots simulation

## Suggested Learning Path

```mermaid
graph LR
    A[Installation] --> B[Quick Start]
    B --> C[Image to Trajectory]
    C --> D[Controlling Robot]
    D --> E[Camera, AI & IMU]
    E --> F[Audio System]
    F --> G[Calibration]
    G --> H[Custom Configurations]
```

1. **Start here**: [Image to Trajectory](image-to-trajectory.md) - Core functionality
2. **Robot Control**: [Controlling the Robot](controlling-robot.md) - When you have access
3. **Sensors**: [Camera, AI & IMU](camera-ai-imu.md) - Vision and sensors
4. **Audio**: [Audio System](../api/audio.md) - Playback, recording, TTS
5. **Real-time**: [Low-Latency Mode](low-latency-mode.md) - Direct motor control
6. **Advanced**: [Custom Configurations](custom-configurations.md) - Fine-tune behavior
