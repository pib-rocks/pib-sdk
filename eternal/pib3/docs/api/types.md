# Core Types

Data structures for drawings and robot joints.

---

## Joint Enum

Enum for robot joint names with IDE tab completion support.

```python
from pib3 import Joint, Robot

with Robot(host="172.26.34.149") as robot:
    # Use enum for tab completion
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)
    pos = robot.get_joint(Joint.SHOULDER_VERTICAL_RIGHT)

    # Multiple joints
    robot.set_joints({
        Joint.SHOULDER_VERTICAL_LEFT: 30.0,
        Joint.ELBOW_LEFT: 60.0,
    })
```

### Available Joints

| Enum | String Value |
|------|--------------|
| **Head** | |
| `Joint.TURN_HEAD` | `turn_head_motor` |
| `Joint.TILT_HEAD` | `tilt_forward_motor` |
| **Left Arm** | |
| `Joint.SHOULDER_VERTICAL_LEFT` | `shoulder_vertical_left` |
| `Joint.SHOULDER_HORIZONTAL_LEFT` | `shoulder_horizontal_left` |
| `Joint.UPPER_ARM_LEFT_ROTATION` | `upper_arm_left_rotation` |
| `Joint.ELBOW_LEFT` | `elbow_left` |
| `Joint.LOWER_ARM_LEFT_ROTATION` | `lower_arm_left_rotation` |
| `Joint.WRIST_LEFT` | `wrist_left` |
| **Left Hand** | |
| `Joint.THUMB_LEFT_OPPOSITION` | `thumb_left_opposition` |
| `Joint.THUMB_LEFT_STRETCH` | `thumb_left_stretch` |
| `Joint.INDEX_LEFT` | `index_left_stretch` |
| `Joint.MIDDLE_LEFT` | `middle_left_stretch` |
| `Joint.RING_LEFT` | `ring_left_stretch` |
| `Joint.PINKY_LEFT` | `pinky_left_stretch` |
| **Right Arm** | |
| `Joint.SHOULDER_VERTICAL_RIGHT` | `shoulder_vertical_right` |
| `Joint.SHOULDER_HORIZONTAL_RIGHT` | `shoulder_horizontal_right` |
| `Joint.UPPER_ARM_RIGHT_ROTATION` | `upper_arm_right_rotation` |
| `Joint.ELBOW_RIGHT` | `elbow_right` |
| `Joint.LOWER_ARM_RIGHT_ROTATION` | `lower_arm_right_rotation` |
| `Joint.WRIST_RIGHT` | `wrist_right` |
| **Right Hand** | |
| `Joint.THUMB_RIGHT_OPPOSITION` | `thumb_right_opposition` |
| `Joint.THUMB_RIGHT_STRETCH` | `thumb_right_stretch` |
| `Joint.INDEX_RIGHT` | `index_right_stretch` |
| `Joint.MIDDLE_RIGHT` | `middle_right_stretch` |
| `Joint.RING_RIGHT` | `ring_right_stretch` |
| `Joint.PINKY_RIGHT` | `pinky_right_stretch` |

!!! tip "Backward Compatibility"
    String joint names still work: `robot.set_joint("elbow_left", 50.0)`

---

## Stroke

A stroke represents a single continuous drawing motion - a sequence of points connected without lifting the pen.

::: pib3.types.Stroke
    options:
      show_root_heading: true
      show_source: true
      members:
        - __init__
        - __len__
        - length
        - reverse
        - start
        - end

### Creating Strokes

```python
import numpy as np
from pib3 import Stroke

# From a list of points
stroke = Stroke(
    points=[[0.1, 0.1], [0.5, 0.2], [0.9, 0.1]],
    closed=False
)

# From numpy array
points = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
closed_stroke = Stroke(points=points, closed=True)

# Access properties
print(f"Length: {stroke.length()}")
print(f"Start: {stroke.start()}")
print(f"End: {stroke.end()}")
```

### Coordinate System

Stroke coordinates are **normalized** to the range [0, 1]:

- `(0, 0)` = top-left corner
- `(1, 1)` = bottom-right corner
- `(0.5, 0.5)` = center

---

## Sketch

A sketch is a collection of strokes representing a complete drawing.

::: pib3.types.Sketch
    options:
      show_root_heading: true
      show_source: true
      members:
        - __init__
        - __len__
        - __iter__
        - __getitem__
        - total_points
        - total_length
        - bounds
        - add_stroke
        - to_dict
        - from_dict

### Creating Sketches

```python
from pib3 import Sketch, Stroke

# Empty sketch
sketch = Sketch()

# Add strokes
sketch.add_stroke(Stroke(points=[[0.1, 0.1], [0.9, 0.1]]))
sketch.add_stroke(Stroke(points=[[0.1, 0.9], [0.9, 0.9]]))

# With strokes and source info
sketch = Sketch(
    strokes=[stroke1, stroke2],
    source_size=(800, 600)  # Original image dimensions
)
```

### Iterating and Accessing

```python
# Iterate over strokes
for stroke in sketch:
    print(f"Stroke with {len(stroke)} points")

# Access by index
first = sketch[0]
last = sketch[-1]

# Get statistics
print(f"Strokes: {len(sketch)}")
print(f"Total points: {sketch.total_points()}")
print(f"Total length: {sketch.total_length()}")
print(f"Bounds: {sketch.bounds()}")
```

!!! warning "`Sketch.bounds()` raises on empty sketches"
    Calling `bounds()` on a sketch with no strokes (or strokes containing
    no points) raises `ValueError` instead of silently returning a
    degenerate `(0, 0, 0, 0)` box. Check `len(sketch) > 0` before
    relying on the returned bounding box.

### Serialization

```python
import json
from pib3 import Sketch

# To dictionary (JSON-serializable)
data = sketch.to_dict()

# Save to file
with open("sketch.json", "w") as f:
    json.dump(data, f)

# Load from file
with open("sketch.json") as f:
    data = json.load(f)
sketch = Sketch.from_dict(data)
```
