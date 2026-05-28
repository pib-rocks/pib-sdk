# Trajectory Generation

Functions and classes for generating robot trajectories.

## Overview

Trajectory generation converts 2D sketches to 3D robot joint positions using inverse kinematics.

```mermaid
graph LR
    A[Sketch] --> B[Map to 3D]
    B --> C[Solve IK]
    C --> D[Interpolate]
    D --> E[Trajectory]
```

## Main Function

### generate_trajectory

::: pib3.generate_trajectory
    options:
      show_root_heading: true
      show_source: true

### Usage

```python
import pib3

# Basic usage
trajectory = pib3.generate_trajectory("drawing.png")
trajectory.to_json("output.json")

# With configuration
from pib3 import TrajectoryConfig, PaperConfig
config = TrajectoryConfig(
    paper=PaperConfig(size=0.15, drawing_scale=0.9)
)
trajectory = pib3.generate_trajectory("drawing.png", config=config)

# With visualization during IK solving
trajectory = pib3.generate_trajectory(
    "drawing.png",
    visualize=True  # Ignored (Swift removed)
)
```

---

## sketch_to_trajectory

::: pib3.trajectory.sketch_to_trajectory
    options:
      show_root_heading: true
      show_source: true

### Usage

```python
import pib3

# Step-by-step approach
sketch = pib3.image_to_sketch("drawing.png")
trajectory = pib3.sketch_to_trajectory(sketch)

# With progress callback
def on_progress(current, total, success):
    print(f"Point {current}/{total}: {'OK' if success else 'FAIL'}")

trajectory = pib3.sketch_to_trajectory(
    sketch,
    progress_callback=on_progress
)

# With custom config
from pib3 import TrajectoryConfig
config = TrajectoryConfig(...)
trajectory = pib3.sketch_to_trajectory(sketch, config)
```

---

## Trajectory Class

::: pib3.trajectory.Trajectory
    options:
      show_root_heading: true
      show_source: true
      members:
        - __init__
        - __len__
        - to_json
        - from_json
        - to_webots_format
        - to_robot_format

### Creating Trajectories

```python
import numpy as np
from pib3 import Trajectory

# From arrays
joint_names = ["joint_0", "joint_1", "joint_2"]
waypoints = np.array([
    [0.0, 0.0, 0.0],
    [0.1, 0.2, 0.3],
    [0.2, 0.4, 0.6],
])

trajectory = Trajectory(
    joint_names=joint_names,
    waypoints=waypoints,
    metadata={"source": "custom"}
)
```

### Saving and Loading

```python
from pib3 import Trajectory

# Save to JSON
trajectory.to_json("my_trajectory.json")

# Load from JSON
loaded = Trajectory.from_json("my_trajectory.json")

# Access data
print(f"Waypoints: {len(loaded)}")
print(f"Joints: {loaded.joint_names}")
print(f"Metadata: {loaded.metadata}")
```

### Format Conversion

```python
# Get waypoints in Webots format (no offset, canonical format)
webots_waypoints = trajectory.to_webots_format()



# Get waypoints in robot format (centidegrees)
robot_waypoints = trajectory.to_robot_format()
```

### JSON Format

```json
{
  "format_version": "1.0",
  "unit": "radians",
  "coordinate_frame": "webots",
  "joint_names": ["turn_head_motor", "tilt_forward_motor", ...],
  "waypoints": [
    [0.1, 0.2, 0.3, ...],
    [0.15, 0.25, 0.35, ...]
  ],
  "metadata": {
    "source": "pib3",
    "robot_model": "pib",
    "success_rate": 0.95,
    "created_at": "2024-01-01T12:00:00Z"
  }
}
```

### Schema Validation

`Trajectory.from_json()` validates the file and raises a clear error instead of silently accepting malformed data:

| Check | Failure mode |
|-------|--------------|
| Top level is a `dict` | `ValueError` |
| `format_version` matches supported | warning logged |
| `unit` equals the expected value | `ValueError` on mismatch |
| `joint_names` is a `list[str]` | `ValueError` |
| `waypoints` is a 2D numeric array with columns matching `len(joint_names)` | `ValueError` |
| `metadata` is a `dict` | `ValueError` |

This keeps downstream execution from failing deep inside IK or motor code with opaque `IndexError`/`KeyError` tracebacks.

---

## IK Solver Details

The inverse kinematics solver uses:

- **Algorithm**: Damped Least Squares (DLS) gradient descent
- **Convergence**: Position error below tolerance
- **Limits**: Joint limits enforced during solving
- **Fallback**: Linear interpolation for failed points

### Solver Parameters

| Parameter | Effect |
|-----------|--------|
| `max_iterations` | More iterations = better accuracy, slower |
| `tolerance` | Smaller = more precise, harder to converge |
| `step_size` | Larger = faster convergence, risk of oscillation |
| `damping` | Higher = more stable near singularities |
