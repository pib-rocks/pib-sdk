# Webots Simulator Backend

Run trajectories in the Webots robotics simulator.

## Overview

`WebotsBackend` integrates with the Webots simulator, allowing you to test trajectories in a physics-based simulation environment.

## WebotsBackend Class

::: pib3.backends.webots.WebotsBackend
    options:
      show_root_heading: true
      show_source: false
      members: false

---

## Quick Start

```python
# In your Webots controller file:
from pib3.backends import WebotsBackend

with WebotsBackend() as backend:
    backend.run_trajectory("trajectory.json")
```

!!! warning "Webots Environment Required"
    This backend must be instantiated from within a Webots controller script. It cannot be used standalone.

---

## Constructor

```python
WebotsBackend(step_ms: int = 50)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `step_ms` | `int` | `50` | Simulation time step per waypoint in milliseconds. Smaller values give smoother motion but slower playback. |

**Example:**

```python
from pib3.backends import WebotsBackend

# Default: 50ms per step
backend = WebotsBackend()

# Custom step timing (slower, smoother)
backend = WebotsBackend(step_ms=100)

# Faster playback
backend = WebotsBackend(step_ms=20)
```

---

## Connection

### connect()

Initialize Webots robot and motor devices.

```python
def connect(self) -> None
```

**Raises:**

- `ImportError`: If not running from within a Webots controller

**Example:**

```python
from pib3.backends import WebotsBackend

backend = WebotsBackend()
backend.connect()  # Initializes robot and motors
# ... use backend ...
backend.disconnect()
```

### Using Context Manager

The recommended way to use the Webots backend:

```python
from pib3 import Joint
from pib3.backends import WebotsBackend

with WebotsBackend() as backend:
    # Robot automatically initialized
    backend.set_joint(Joint.ELBOW_LEFT, 50.0)
# Cleanup handled automatically
```

---

## Joint Control

The Webots backend inherits all methods from [`RobotBackend`](base.md). Key methods:

### set_joint()

Set a single joint position. Inherits from [`RobotBackend`](base.md#set_joint) — see the base class for the full parameter list (including `speed` and the `"deg"` unit).

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

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum (e.g., `Joint.ELBOW_LEFT`). |
| `position` | `float` | *required* | Target position (0-100 for percent, radians for rad, degrees for deg). |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `False` | If `True`, return immediately. If `False` (default), step simulation until motor reaches target. |
| `timeout` | `float` | `2.0` | Max wait time (only used when `async_=False`). |
| `tolerance` | `float` or `None` | `None` | Acceptable error (2.0%, 3.0°, or 0.05 rad default). |
| `speed` | `float` or `None` | `None` | Movement speed in deg/s. |

**Returns:** `bool` - `True` if successful.

**Example:**

```python
from pib3 import Joint

with WebotsBackend() as robot:
    # Set individual joints
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)  # 50%

    # Using radians
    robot.set_joint(Joint.ELBOW_LEFT, 1.25, unit="rad")
```

### set_joints()

Set multiple joint positions simultaneously. Takes a plain dict — for hand-pose presets use [`set_joints_pose()`](base.md#set_joints_pose); for a sequence of waypoints use [`set_joints_sequence()`](base.md#set_joints_sequence).

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

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `positions` | `Dict[str\|Joint, float]` | *required* | Target positions. Passing a `HandPose` or a plain sequence raises `TypeError`. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Position unit. |
| `async_` | `bool` | `False` | If `True`, return immediately. If `False` (default), step simulation until motors reach targets. |
| `timeout` | `float` | `2.0` | Max wait time (only used when `async_=False`). |
| `tolerance` | `float` or `None` | `None` | Acceptable error. |
| `speed` | `float` or `None` | `None` | Movement speed in deg/s. |

**Example:**

```python
from pib3 import Joint

with WebotsBackend() as robot:
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 30.0,
        Joint.SHOULDER_HORIZONTAL_LEFT: 40.0,
        Joint.ELBOW_LEFT: 60.0,
    })
```

### get_joint()

Read a single joint position. Waits for motor readings to stabilize (same value twice) before returning.

```python
def get_joint(
    self,
    motor_name: Union[str, Joint],
    unit: Literal["percent", "rad", "deg"] = "percent",
    timeout: Optional[float] = None,
) -> Optional[float]
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_name` | `str` or `Joint` | *required* | Motor name or `Joint` enum to query. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Return unit. |
| `timeout` | `float` or `None` | `5.0` | Max time to wait for motor reading to stabilize (seconds). |

**Returns:** `float` or `None` - Current position, or `None` if unavailable or motor still moving.

**Example:**

```python
from pib3 import Joint

with WebotsBackend() as robot:
    # Uses default 5s timeout for stabilization
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    print(f"Elbow at {pos:.1f}%")

    # Shorter timeout
    pos = robot.get_joint(Joint.ELBOW_LEFT, timeout=1.0)
```

### get_joints()

Read multiple joint positions. Waits for each motor reading to stabilize before returning.

```python
def get_joints(
    self,
    motor_names: Optional[List[Union[str, Joint]]] = None,
    unit: Literal["percent", "rad", "deg"] = "percent",
    timeout: Optional[float] = None,
) -> Dict[str, float]
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_names` | `List[str\|Joint]` or `None` | `None` | Motors to query. `None` returns all. |
| `unit` | `"percent"`, `"rad"`, `"deg"` | `"percent"` | Return unit. |
| `timeout` | `float` or `None` | `5.0` | Max time to wait for each motor reading to stabilize (seconds). |

**Returns:** `Dict[str, float]` - Motor names mapped to positions.

---

## Trajectory Execution

### run_trajectory()

Execute a trajectory in simulation.

```python
def run_trajectory(
    self,
    trajectory: Union[str, Path, Trajectory],
    rate_hz: float = 20.0,
    progress_callback: Optional[Callable[[int, int], None]] = None,
) -> bool
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trajectory` | `str`, `Path`, or `Trajectory` | *required* | Trajectory file path or object. |
| `rate_hz` | `float` | `20.0` | Playback rate (waypoints per second). |
| `progress_callback` | `Callable[[int, int], None]` or `None` | `None` | Progress callback `(current, total)`. |

**Returns:** `bool` - `True` if completed successfully.

**Example:**

```python
from pib3.backends import WebotsBackend
from pib3 import Trajectory

trajectory = Trajectory.from_json("trajectory.json")

def on_progress(current, total):
    print(f"\rProgress: {current}/{total}", end="")

with WebotsBackend() as robot:
    robot.run_trajectory(
        trajectory,
        rate_hz=20.0,
        progress_callback=on_progress,
    )
    print("\nDone!")
```

---

## Motor Mapping

The backend maps trajectory joint names to Webots motor device names:

| Trajectory Name | Webots Motor |
|-----------------|--------------|
| `turn_head_motor` | `head_horizontal` |
| `tilt_forward_motor` | `head_vertical` |
| `shoulder_vertical_left` | `shoulder_vertical_left` |
| `shoulder_horizontal_left` | `shoulder_horizontal_left` |
| `upper_arm_left_rotation` | `upper_arm_left` |
| `elbow_left` | `elbow_left` |
| `lower_arm_left_rotation` | `forearm_left` |
| `wrist_left` | `wrist_left` |
| `thumb_left_opposition` | `thumb_left_opposition` |
| `thumb_left_stretch` | `thumb_left_distal` |

Similar mappings exist for right arm and remaining fingers.

---

## Coordinate System

The canonical format uses **Webots motor radians** directly. No offset is needed:

```
Webots_position = Canonical_radians  (no offset)
```

Webots motors use sensible radian ranges (e.g., -π/2 to +π/2 for the head motor).

---

## Webots Project Setup

### Directory Structure

```
my_webots_project/
├── worlds/
│   └── pib_world.wbt
├── controllers/
│   └── pib_controller/
│       └── pib_controller.py
└── protos/
    └── pib.proto
```

### Controller Script Example

Create a controller file `pib_controller.py`:

```python
"""PIB robot controller for Webots."""
from pib3.backends import WebotsBackend

def main():
    with WebotsBackend() as robot:
        # Run a pre-generated trajectory
        robot.run_trajectory("trajectory.json")

if __name__ == "__main__":
    main()
```

### World File Configuration

In your `.wbt` world file, configure the robot to use your controller:

```
Robot {
  controller "pib_controller"
  ...
}
```

---

## Examples

### Wave Animation

```python
from pib3 import Joint
from pib3.backends import WebotsBackend

with WebotsBackend() as robot:
    # Raise arm (default async_=False waits for motor to reach position)
    robot.set_joint(Joint.SHOULDER_VERTICAL_LEFT, 30.0)

    # Wave back and forth
    for _ in range(5):
        robot.set_joint(Joint.WRIST_LEFT, 20.0)
        robot.set_joint(Joint.WRIST_LEFT, 80.0)

    # Return to neutral
    robot.set_joint(Joint.WRIST_LEFT, 50.0)
    robot.set_joint(Joint.SHOULDER_VERTICAL_LEFT, 50.0)
```

!!! tip "The default `async_=False` replaces `time.sleep()`"
    In Webots, simulation time only advances when `robot.step()` is called internally.
    Using `time.sleep()` wastes wall-clock time without advancing the simulation.
    The default `async_=False` automatically steps the simulation until motors reach their targets.
    Pass `async_=True` to override and fire-and-forget.

### Complete Drawing Session

```python
from pib3.backends import WebotsBackend
import pib3

# Generate trajectory from image (outside Webots)
trajectory = pib3.generate_trajectory("drawing.png")
trajectory.to_json("drawing_trajectory.json")

# Execute in Webots (in controller)
with WebotsBackend() as robot:
    robot.run_trajectory("drawing_trajectory.json", rate_hz=30.0)
```

---

## Troubleshooting

!!! warning "ImportError: No module named 'controller'"
    **Cause:** Not running from within Webots.

    **Solution:** This backend must be used in a Webots controller script. It cannot be run standalone from the command line.

!!! warning "Motor Not Found"
    **Cause:** Motor name mismatch between trajectory and Webots model.

    **Solution:** Check that the motor names in your Webots proto file match the expected names in `JOINT_TO_WEBOTS_MOTOR`.

!!! warning "Simulation Runs Too Fast/Slow"
    **Cause:** Step timing mismatch.

    **Solution:** Adjust the `step_ms` parameter or `rate_hz`:

    ```python
    # Slower, more accurate simulation
    backend = WebotsBackend(step_ms=100)

    # Faster simulation
    backend = WebotsBackend(step_ms=20)

    # Match rate_hz to timestep (for 50ms, use 20 Hz)
    robot.run_trajectory(trajectory, rate_hz=20.0)
    ```

!!! warning "Robot Doesn't Move Smoothly"
    **Cause:** Step timing too large.

    **Solution:** Use smaller `step_ms` values for smoother motion:

    ```python
    backend = WebotsBackend(step_ms=20)  # Smoother
    ```

!!! warning "Motors Don't Reach Target Position"
    **Cause:** Explicitly passing `async_=True` only steps the simulation once.

    **Solution:** Drop the `async_=True` override — the default `async_=False` waits for motors to reach their targets:

    ```python
    # Wrong - returns immediately, motor barely moves
    robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=True)

    # Correct - default behavior waits for motor to reach position
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)
    ```

    This also ensures code compatibility with the real robot, which behaves the same way.
