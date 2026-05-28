# Coordinate Systems

Understanding the coordinate frames used in pib3.

## Robot Frame

The robot uses a right-handed coordinate system:

```
        Z+ (up)
        │
        │
        │
        └───────── X+ (forward)
       ╱
      ╱
     Y+ (left)
```

| Axis | Direction | Example |
|------|-----------|---------|
| X+ | Forward | Away from robot, toward paper |
| Y+ | Left | Robot's left side |
| Z+ | Up | Toward ceiling |

## Coordinate Spaces

### 1. Image Space (pixels)

Raw image coordinates from the input file.

- Origin: Top-left corner
- X: Right (0 to width)
- Y: Down (0 to height)

```python
# Image coordinates (pixels)
img = Image.open("drawing.png")
pixel_x = 100  # pixels from left
pixel_y = 50   # pixels from top
```

### 2. Sketch Space (normalized)

Normalized 2D coordinates after image processing.

- Origin: Top-left (like image)
- X: Right (0.0 to 1.0)
- Y: Down (0.0 to 1.0)

```python
# Sketch coordinates (normalized)
from pib3 import Stroke, Point

stroke = Stroke([
    Point(0.0, 0.0),   # Top-left
    Point(1.0, 0.0),   # Top-right
    Point(1.0, 1.0),   # Bottom-right
    Point(0.0, 1.0),   # Bottom-left
])
```

### 3. Paper Space (meters)

Physical coordinates on the drawing paper, relative to paper center.

- Origin: Paper center
- X: Right on paper (+paper_size/2 to -paper_size/2)
- Y: Up on paper (+paper_size/2 to -paper_size/2)

```python
# Paper coordinates (meters from paper center)
from pib3 import PaperConfig

config = PaperConfig(
    center_x=0.15,   # Paper center X in robot frame
    center_y=0.15,   # Paper center Y in robot frame
    height_z=0.74,   # Paper surface Z in robot frame
    size=0.12,       # Paper is 12cm x 12cm
)
```

### 4. Robot Space (meters)

3D coordinates in the robot's base frame.

- Origin: Robot base (center of torso)
- Axes: As defined in Robot Frame section

```python
# Robot frame coordinates for paper corners
paper_center = (0.15, 0.15, 0.74)  # meters

# A point on the paper
point_x = 0.15 + 0.03   # 3cm right of paper center
point_y = 0.15 - 0.02   # 2cm below paper center
point_z = 0.74          # On paper surface
```

### 5. Joint Space (radians/percent)

Robot joint angles.

- **Radians**: Raw angular position (-π to +π typical)
- **Percentage**: 0% to 100% of calibrated range

```python
# Joint space
from pib3 import Robot, Joint

with Robot() as robot:
    # Percentage (requires calibration)
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)

    # Radians (direct control)
    robot.set_joint(Joint.ELBOW_LEFT, 1.25, unit="rad")
```

## Coordinate Transformations

### Sketch → Paper

```python
def sketch_to_paper(sketch_point, paper_config):
    """Convert normalized sketch point to paper-relative position."""
    # Flip Y (sketch Y is down, paper Y is up)
    paper_x = (sketch_point.x - 0.5) * paper_config.size * paper_config.drawing_scale
    paper_y = (0.5 - sketch_point.y) * paper_config.size * paper_config.drawing_scale
    return (paper_x, paper_y)
```

### Paper → Robot

```python
def paper_to_robot(paper_point, paper_config):
    """Convert paper-relative position to robot frame."""
    robot_x = paper_config.center_x + paper_point[0]
    robot_y = paper_config.center_y + paper_point[1]
    robot_z = paper_config.height_z
    return (robot_x, robot_y, robot_z)
```

### Robot → Joint (IK)

```python
def robot_to_joint(target_pos, current_joints, robot_model):
    """Inverse kinematics: Cartesian position to joint angles."""
    # Computed by IK solver using:
    # 1. Forward kinematics to get current end-effector position
    # 2. Jacobian to relate joint velocities to Cartesian velocities
    # 3. Iterative solving with damped least squares
    return new_joint_angles
```

## Visual Reference

```
Image/Sketch Space          Paper Space              Robot Space
(normalized)                (meters)                 (meters)

(0,0)───────(1,0)          (-0.06,+0.06)───(+0.06,+0.06)
  │           │                 │               │
  │  Drawing  │                 │    Paper     │        Z+
  │           │                 │    12cm      │        │
(0,1)───────(1,1)          (-0.06,-0.06)───(+0.06,-0.06) │
                                                        └───X+
                                                       ╱
                           Paper center at            Y+
                           (0.15, 0.15, 0.74)
                           in robot frame
```

## Common Pitfalls

### Y-Axis Inversion

Image Y goes **down**, but robot Y goes **left**. The library handles this:

```python
# Internal conversion in sketch_to_trajectory
robot_y = paper_center_y + (0.5 - sketch_y) * paper_size
#                          ^^^^^^^^^^^^^^^^
#                          Inverts Y direction
```

### Paper Orientation

The paper is positioned to the robot's **left and forward**:

```python
# Default paper position
center_x = 0.15  # 15cm forward of robot
center_y = 0.15  # 15cm to robot's left
```

This places the paper within reach of the **left arm**.

### Units

Always check what units you're working with:

| Component | Unit |
|-----------|------|
| Image processing | pixels |
| Sketch | normalized (0-1) |
| Paper config | meters |
| IK solver | radians |
| Robot percentage API | percent (0-100) |
| Trajectory JSON | radians |
