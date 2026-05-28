# Controlling the Robot

Master the joint control API for precise robot manipulation.

## Objectives

By the end of this tutorial, you will:

- Connect to a real PIB robot
- Control joints using percentage and radian units
- Save and restore robot poses
- Use verification to ensure movements complete
- Control hand positions

## Prerequisites

- pib3 installed with robot support: `pip install "pib3[robot] @ git+https://github.com/mamrehn/pib3.git"`
- A PIB robot connected to your network
- Rosbridge running on the robot
- [Calibrated joint limits](../getting-started/calibration.md) (recommended)

---

## Connecting to the Robot

### Basic Connection

```python
from pib3 import Robot

# Connect using context manager (recommended)
with Robot(host="172.26.34.149") as robot:
    # Robot is connected here
    print(f"Connected: {robot.is_connected}")
    # ... control the robot ...
# Automatically disconnected when exiting the block
```

### Manual Connection

```python
from pib3 import Robot

robot = Robot(host="172.26.34.149", port=9090)
robot.connect()

try:
    # Control the robot
    print(f"Connected: {robot.is_connected}")
finally:
    robot.disconnect()
```

### Connection Options

```python
from pib3 import Robot

robot = Robot(
    host="172.26.34.149",  # Robot IP address
    port=9090,             # Rosbridge port (default: 9090)
    timeout=5.0,           # Connection timeout in seconds
)
```

---

## Percentage-Based Control

The percentage system maps joint ranges to 0-100%:

| Value | Position |
|-------|----------|
| 0% | Joint minimum |
| 50% | Middle of range |
| 100% | Joint maximum |

### Setting Joint Positions

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Use Joint enum for IDE tab completion
    robot.set_joint(Joint.TURN_HEAD, 50.0)           # Center head
    robot.set_joint(Joint.ELBOW_LEFT, 75.0)          # 75% of range
    robot.set_joint(Joint.SHOULDER_VERTICAL_LEFT, 0.0)  # Minimum
```

!!! tip "IDE Tab Completion"
    Use `Joint.` to see all available joints with autocomplete in your IDE.

### Reading Joint Positions

Joint readings wait up to 5s by default for ROS messages.

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Single joint
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    print(f"Elbow: {pos:.1f}%")

    # Multiple joints
    arm = robot.get_joints([Joint.ELBOW_LEFT, Joint.WRIST_LEFT])

    # All joints
    all_joints = robot.get_joints()
```

---

## Using Radians or Degrees

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Set/get in radians
    robot.set_joint(Joint.ELBOW_LEFT, 1.25, unit="rad")
    pos = robot.get_joint(Joint.ELBOW_LEFT, unit="rad")

    # Set/get in degrees
    robot.set_joint(Joint.ELBOW_LEFT, -30.0, unit="deg")
    pos = robot.get_joint(Joint.ELBOW_LEFT, unit="deg")
```

---

## Setting Multiple Joints

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Multiple joints at once
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 30.0,
        Joint.ELBOW_LEFT: 60.0,
        Joint.WRIST_LEFT: 50.0,
    })

    # Modify current pose
    current = robot.get_joints()
    current["elbow_left"] = 75.0
    robot.set_joints(current)
```

---

## Saving and Restoring Poses

```python
import json
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Save current pose
    pose = robot.get_joints()

    # ... move robot around ...

    # Restore
    robot.set_joints(pose)

    # Persist to file
    with open("pose.json", "w") as f:
        json.dump(pose, f)

# Load from file
with open("pose.json") as f:
    robot.set_joints(json.load(f))
```

---

## Position Verification

`set_joint` / `set_joints` default to `async_=False` with a 2 s timeout, so
calls block until the joint reaches its target (or times out). Pass
`async_=True` for fire-and-forget commands.

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Default is blocking: waits for completion
    success = robot.set_joint(
        Joint.ELBOW_LEFT, 50.0,
        timeout=2.0,     # Max wait (seconds), default
        tolerance=2.0,   # Acceptable error (%)
        speed=30.0,      # Optional: %/s motion speed
    )

    # Explicit fire-and-forget
    robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=True)

    # Wait for multiple joints
    success = robot.set_joints(
        {Joint.ELBOW_LEFT: 60.0, Joint.WRIST_LEFT: 50.0},
        timeout=3.0,
    )
```

!!! note "`speed` on the real robot"
    On `RealRobotBackend`'s direct-Tinkerforge path, `speed` mutates the
    servo's shared motion-velocity register for the duration of the call.
    Concurrent movers on the same servo will see the updated value until
    the call returns — serialize `speed`-overriding moves per joint.

---

## Controlling the Hands

`HandPose` presets are applied via `set_joints_pose()`. `set_joints()` is
strictly dict-only and raises `TypeError` if you pass a `HandPose` directly.

```python
from pib3 import Robot, HandPose, LEFT_HAND_JOINTS

with Robot(host="172.26.34.149") as robot:
    # Preset poses via set_joints_pose
    robot.set_joints_pose(HandPose.LEFT_OPEN)     # Open hand
    robot.set_joints_pose(HandPose.LEFT_CLOSED)   # Close hand

    # Custom grip via set_joints (plain dict)
    robot.set_joints({j: 50.0 for j in LEFT_HAND_JOINTS})  # 50% closed
```

---

## Running Trajectories

Execute pre-generated trajectories:

```python
from pib3 import Robot, Trajectory

with Robot(host="172.26.34.149") as robot:
    # Load trajectory from file
    trajectory = Trajectory.from_json("my_trajectory.json")

    # Execute with progress callback
    def on_progress(current, total):
        print(f"\rProgress: {current}/{total}", end="")

    robot.run_trajectory(
        trajectory,
        rate_hz=20.0,  # Playback rate
        progress_callback=on_progress,
    )
    print("\nDone!")
```

---

## Complete Example: Drawing Session

```python
"""
Complete robot drawing session example.
"""
import pib3
from pib3 import Robot, Trajectory

def main():
    # Generate trajectory from image
    print("Generating trajectory...")
    trajectory = pib3.generate_trajectory("drawing.png")
    trajectory.to_json("session_trajectory.json")
    print(f"Generated {len(trajectory)} waypoints")

    # Connect and execute
    with Robot(host="172.26.34.149") as robot:
        # Save initial pose
        initial_pose = robot.get_joints()
        print("Saved initial pose")

        # Move to starting position (blocking by default)
        print("Moving to start position...")
        robot.set_joints({
            "shoulder_vertical_left": 50.0,
            "elbow_left": 50.0,
        })

        # Execute drawing
        print("Drawing...")
        def progress(current, total):
            if current % 100 == 0:
                print(f"  {current}/{total} waypoints")

        robot.run_trajectory(
            trajectory,
            rate_hz=20.0,
            progress_callback=progress,
        )

        print("Drawing complete!")

        # Ask user before restoring
        input("Press Enter to restore initial pose...")
        robot.set_joints(initial_pose)
        print("Restored initial pose")

if __name__ == "__main__":
    main()
```

---

## Available Joints Reference

Use `Joint.` enum values for IDE tab completion. See [Types Reference](../api/types.md) for full list.

| Joint Enum | String | Description |
|------------|--------|-------------|
| `Joint.TURN_HEAD` | `turn_head_motor` | Head left/right |
| `Joint.TILT_HEAD` | `tilt_forward_motor` | Head up/down |
| `Joint.SHOULDER_VERTICAL_LEFT` | `shoulder_vertical_left` | Shoulder up/down |
| `Joint.ELBOW_LEFT` | `elbow_left` | Elbow bend |
| `Joint.WRIST_LEFT` | `wrist_left` | Wrist bend |
| ... | ... | Right side: replace `LEFT` with `RIGHT` |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Connection refused | Start rosbridge: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| Joints not moving | [Calibrate joint limits](../getting-started/calibration.md) or use `unit="rad"` |
| Timeout waiting | Increase `timeout` and `tolerance` values |
| Slow movement | Increase trajectory rate: `rate_hz=30.0` |

---

## Next Steps

- [Webots Simulation](../api/backends/webots.md) - Test in simulation
- [Calibration Guide](../getting-started/calibration.md) - Calibrate joint limits
- [API Reference: RealRobotBackend](../api/backends/robot.md) - Full API documentation
