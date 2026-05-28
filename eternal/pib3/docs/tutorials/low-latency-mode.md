# Direct Motor Control (Low-Latency Mode)

Bypass ROS for direct Tinkerforge motor control with significantly reduced latency.

## Overview

The standard motor control path goes through:

```
Python -> roslibpy -> rosbridge -> ROS node -> Tinkerforge
```

This adds ~100-200ms latency per command. Direct mode bypasses ROS:

```
Python -> Tinkerforge (direct)
```

Reducing latency to ~5-20ms for both **reading** and **writing** motor positions.

**Direct Tinkerforge control is the default mode.** Servo bricklets are
auto-discovered on connect. ROS is still connected for audio, camera,
and AI subsystems.

## When to Use Each Mode

| Mode | Latency | Best For |
|------|---------|----------|
| Direct (default) | 5-20ms | Real-time control, high-frequency updates, interactive demos |
| ROS | 100-200ms | Trajectory playback, non-time-critical operations |

## Prerequisites

Install the Tinkerforge Python bindings:

```bash
pip install tinkerforge
```

Ensure the robot's Tinkerforge Brick Daemon is accessible (port 4223).

---

## Quick Start

Direct mode is the default -- no extra configuration needed:

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Motor commands go directly to Tinkerforge (auto-discovered)
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)
    angle = robot.get_joint(Joint.ELBOW_LEFT)
```

To use ROS for motor control instead:

```python
from pib3 import Robot

with Robot(host="172.26.34.149", motor_mode="ros") as robot:
    robot.set_joint("elbow_left", 50.0)  # Via ROS/rosbridge
```

---

## Runtime Control

### Enable/Disable at Runtime

```python
with Robot(host="172.26.34.149") as robot:
    # Check status
    print(f"Low-latency available: {robot.low_latency_available}")
    print(f"Low-latency enabled: {robot.low_latency_enabled}")

    # Disable for specific operations
    robot.low_latency_enabled = False
    robot.run_trajectory("trajectory.json")  # Uses ROS

    # Re-enable
    robot.low_latency_enabled = True
```

!!! warning "Setter raises on unavailable path"
    Assigning `True` to `low_latency_enabled` raises `RuntimeError` if
    Tinkerforge isn't installed, the robot isn't connected, or servo
    discovery failed. This surfaces configuration problems immediately
    instead of silently falling back to ROS the next time you set a
    joint. Guard enablement with `low_latency_available` if you want
    the tolerant behavior.

---

## Reading Positions

When direct mode is enabled, `get_joint()` and `get_joints()` also read directly from the Tinkerforge servo bricklets instead of waiting for ROS messages.

### Automatic Direct Reading

```python
with Robot(host="172.26.34.149") as robot:
    # Both reading and writing use direct Tinkerforge control
    robot.set_joint("elbow_left", 0.5, unit="rad")  # Direct write
    pos = robot.get_joint("elbow_left", unit="rad")  # Direct read (~5ms)
```

### Reading Multiple Joints

```python
# Read all mapped motors directly
positions = robot.get_joints(unit="rad")

# Read specific motors
arm_positions = robot.get_joints(
    ["elbow_left", "wrist_left", "shoulder_vertical_left"],
    unit="rad",
)
```

### Mixed Mode Reading

Motors not in the Tinkerforge mapping fall back to ROS:

```python
positions = robot.get_joints([
    "elbow_left",      # Direct (in mapping)
    "turn_head_motor", # Falls back to ROS (not in mapping)
], unit="rad")
```

!!! note "Actual vs. Commanded Position"
    Direct reads return the **actual measured position** from the servo's potentiometer feedback, not the last commanded position. This is useful for:

    - Detecting mechanical issues (motor slippage, loose gears)
    - Sensing external forces on the arm (collision detection)
    - Verifying the arm reached its target position
    - Implementing compliant control where the arm responds to physical interaction

---

## Advanced Configuration

### LowLatencyConfig

For advanced use cases, you can customize the Tinkerforge connection
settings via `RobotConfig`:

```python
from pib3 import LowLatencyConfig, RobotConfig
from pib3.backends import RealRobotBackend

config = RobotConfig(
    host="172.26.34.149",
    low_latency=LowLatencyConfig(
        enabled=True,                    # Default: True
        tinkerforge_host=None,           # Defaults to robot host IP
        tinkerforge_port=4223,           # Standard Tinkerforge port
        motor_mapping=None,              # None = auto-discover (recommended)
        sync_to_ros=True,                # Update local cache for get_joint()
        command_timeout=0.5,             # Command timeout in seconds
    ),
)

robot = RealRobotBackend.from_config(config)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabled` | `bool` | `True` | Enable direct Tinkerforge control |
| `tinkerforge_host` | `str` | `None` | Tinkerforge daemon host (defaults to robot IP) |
| `tinkerforge_port` | `int` | `4223` | Tinkerforge daemon port |
| `motor_mapping` | `dict` | `None` | Maps motor names to `(bricklet_uid, channel)`. `None` = auto-discover. |
| `sync_to_ros` | `bool` | `True` | Update local position cache after commands |
| `command_timeout` | `float` | `0.5` | Timeout for direct commands |

### sync_to_ros Explained

When `sync_to_ros=True` (default), the local position cache is updated after each direct command. This ensures `get_joint()` returns the correct value.

!!! warning "No ROS Topic Updates"
    Direct commands do **not** publish to ROS topics. The position will not be visible in ROS tools like `rostopic echo`. This is intentional - publishing to `/joint_trajectory` would trigger a second motor command.

### Manual Motor Mapping

If auto-discovery doesn't work for your setup, you can provide
an explicit motor mapping:

```python
from pib3 import Robot, build_motor_mapping

with Robot(host="172.26.34.149") as robot:
    # Build mapping from known servo bricklet UIDs
    mapping = build_motor_mapping(
        servo1_uid="ABC1",  # Right arm bricklet
        servo2_uid="ABC2",  # Shoulder verticals bricklet
        servo3_uid="ABC3",  # Left arm + hand bricklet
    )
    robot.configure_motor_mapping(mapping)
```

---

## Custom Servo Configuration

Servo channels are automatically configured with default settings. To customize:

### Configure Individual Motor

```python
robot.configure_servo_channel(
    "elbow_left",
    pulse_width_min=700,    # Minimum PWM pulse (microseconds)
    pulse_width_max=2500,   # Maximum PWM pulse (microseconds)
    velocity=9000,          # Max velocity (0.01 deg/s) = 90 deg/s
    acceleration=9000,      # Acceleration (0.01 deg/s^2)
    deceleration=9000,      # Deceleration (0.01 deg/s^2)
)
```

### Configure All Motors

```python
robot.configure_all_servo_channels(
    pulse_width_min=700,
    pulse_width_max=2500,
    velocity=12000,         # Faster: 120 deg/s
    acceleration=15000,
    deceleration=15000,
)
```

---

## Helper Functions

### build_motor_mapping()

Creates a complete motor mapping from three servo bricklet UIDs:

```python
from pib3 import build_motor_mapping

mapping = build_motor_mapping(
    servo1_uid="ABC1",  # Controls right arm motors
    servo2_uid="ABC2",  # Controls shoulder vertical motors (both arms)
    servo3_uid="ABC3",  # Controls left arm + left hand motors
)
```

This uses the standard PIB wiring where:

- **Servo 1**: Right arm (shoulder horizontal, elbow, rotation, hand)
- **Servo 2**: Both shoulder verticals
- **Servo 3**: Left arm + left hand

### PIB_SERVO_CHANNELS

Reference for standard PIB channel assignments:

```python
from pib3 import PIB_SERVO_CHANNELS

# See which bricklet and channel each motor uses: (bricklet_number, channel)
print(PIB_SERVO_CHANNELS["elbow_left"])  # (3, 8)
print(PIB_SERVO_CHANNELS["wrist_left"])  # (3, 6)
```

---

## Fallback Behavior

Motors not in the Tinkerforge mapping automatically fall back to ROS control:

```python
# If only left arm is mapped, right arm uses ROS
robot.set_joints({
    "elbow_left": 0.5,   # Direct (in mapping)
    "elbow_right": 0.5,  # Falls back to ROS (not in mapping)
}, unit="rad")
```

If Tinkerforge is unavailable (not installed, connection failed, or fewer
than 3 servo bricklets found), all motor commands fall back to ROS
automatically.

---

## Complete Example

```python
from pib3 import Robot
import time
import math

def main():
    with Robot(host="172.26.34.149") as robot:
        print(f"Low-latency available: {robot.low_latency_available}")

        # Smooth sinusoidal motion at 20 Hz
        start_time = time.time()
        duration = 5.0  # seconds
        frequency = 20  # Hz

        while time.time() - start_time < duration:
            t = time.time() - start_time
            # Sinusoidal motion between 0.2 and 0.8 radians
            position = 0.5 + 0.3 * math.sin(2 * math.pi * 0.5 * t)

            robot.set_joint("elbow_left", position, unit="rad")
            time.sleep(1.0 / frequency)

        print("Done!")

if __name__ == "__main__":
    main()
```

---

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| `low_latency_available` is `False` | Tinkerforge not installed | `pip install tinkerforge` |
| | Connection failed | Check robot IP and port 4223 is accessible |
| | Fewer than 3 servo bricklets | Check wiring; or provide manual `motor_mapping` |
| Motors don't move | Servo power relay off | Check relay is enabled |
| | Wrong bricklet UID | Use `discover_servo_bricklets()` to verify |
| `get_joint()` returns old value | `sync_to_ros=False` | Set `sync_to_ros=True` in config |
| Jerky motion | Velocity too low | Increase velocity in `configure_servo_channel()` |

!!! tip "When to Use Each Mode"
    Use **ROS mode** (`motor_mode="ros"`) for trajectory execution (`run_trajectory()`) where timing is managed by waypoints. Use **direct mode** (default) for interactive applications where you need immediate response to input.
