# Kinematics

DH (Denavit-Hartenberg) kinematics models and coordinate transformations for the PIB robot arms.

## Overview

The `pib3.dh_model` module provides calibrated DH models for the PIB humanoid robot arms. These are used internally for inverse kinematics but can also be used directly for advanced robotics applications.

!!! note "Advanced Usage"
    This module is primarily for advanced users who need direct access to forward kinematics or coordinate transformations. For most use cases, use the high-level trajectory functions instead.

## DH Robot Models

### PibLeft

DH model for the left arm with expert-calibrated parameters.

```python
from pib3.dh_model import PibLeft
from spatialmath import SE3

# Create left arm model
left_arm = PibLeft()

# Forward kinematics - get end-effector pose for joint angles
q = [0, 0, 0, 0, 0, 0]  # 6 joint angles in radians
T = left_arm.fkine(q)  # Returns SE3 transform

# Inverse kinematics
target = SE3.Trans(100, 0, 200)  # Target position in mm
solution = left_arm.ikine_LM(target)
```

**Predefined Configurations:**

```python
left_arm.q_observe  # Observation pose (arm raised, looking forward)
left_arm.q_rest     # Rest pose (arm relaxed at side)
left_arm.q_draw     # Drawing pose (arm extended for drawing)
```

### PibRight

DH model for the right arm (mirrored parameters).

```python
from pib3.dh_model import PibRight

right_arm = PibRight()
```

---

## Coordinate Transformations

Transform points between camera frame and robot base frame (torso origin).

### camera_to_base()

Transform a point from camera frame to robot base frame.

```python
from pib3.dh_model import camera_to_base
import numpy as np

# Point detected by camera (in mm)
point_camera = np.array([100, 50, 500])

# Transform to robot base frame
point_base = camera_to_base(point_camera)
print(f"In base frame: {point_base}")
```

### base_to_camera()

Transform a point from robot base frame to camera frame.

```python
from pib3.dh_model import base_to_camera
import numpy as np

# Point in robot base frame (in mm)
point_base = np.array([0, 0, 300])

# Transform to camera frame
point_camera = base_to_camera(point_base)
```

---

## Constants

### CAMERA_TRANSFORM

SE3 transform from camera link to robot base frame (torso origin).

```python
from pib3.dh_model import CAMERA_TRANSFORM

# Use for custom transformations
T = CAMERA_TRANSFORM
```

### DEFAULT_MOTOR_SETTINGS

Default Tinkerforge servo settings for all motors.

```python
from pib3.dh_model import DEFAULT_MOTOR_SETTINGS

print(DEFAULT_MOTOR_SETTINGS)
# {
#     "pulse_width_min": 700,
#     "pulse_width_max": 2500,
#     "period": 19500,
#     "rotation_range_min": -9000,  # centidegrees (-90°)
#     "rotation_range_max": 9000,   # centidegrees (+90°)
# }
```

### MOTOR_GROUPS

Motor groups organized by body part.

```python
from pib3.dh_model import MOTOR_GROUPS

print(MOTOR_GROUPS["right_arm"])
# ['shoulder_vertical_right', 'shoulder_horizontal_right', 
#  'upper_arm_right_rotation', 'elbow_right', 
#  'lower_arm_right_rotation', 'wrist_right']

print(MOTOR_GROUPS["left_hand"])
# ['index_left_stretch', 'middle_left_stretch', 'ring_left_stretch',
#  'pinky_left_stretch', 'thumb_left_stretch', 'thumb_left_opposition']

print(MOTOR_GROUPS["head"])
# ['turn_head_motor', 'tilt_forward_motor']
```

---

## Utility Functions

### get_dh_robot()

Get or create a cached DH robot for the given arm and tool offset.

```python
from pib3.dh_model import get_dh_robot
from spatialmath import SE3

# Get left arm with pen tool offset
tool = SE3.Tz(50)  # 50mm tool in Z direction
robot = get_dh_robot("left", tool)
```

### get_shoulder_position()

Get the shoulder position in world frame from the URDF model.

```python
from pib3.dh_model import get_shoulder_position

# Requires URDF robot model (from roboticstoolbox)
shoulder_pos = get_shoulder_position(urdf_robot, "left")
print(f"Left shoulder at: {shoulder_pos}")  # [x, y, z] in meters
```

### clear_caches()

Clear all module-level caches (useful for testing).

```python
from pib3.dh_model import clear_caches

clear_caches()
```

---

## Example: Vision-Based Reaching

Transform a detected object position to a reachable target:

```python
from pib3 import Robot, AIModel
from pib3.dh_model import camera_to_base, PibLeft
import numpy as np

with Robot(host="172.26.34.149") as robot:
    # Detect objects
    robot.ai.set_model(AIModel.YOLOV8N)
    detections = robot.ai.get_detections()
    
    if detections:
        det = detections[0]
        
        # Assume depth of 500mm for detected object
        # (In practice, use stereo depth from OAK-D)
        depth_mm = 500
        
        # Convert normalized bbox center to camera coordinates
        cx, cy = det.bbox.center
        # Camera intrinsics would be needed for accurate conversion
        # This is simplified
        x_camera = (cx - 0.5) * depth_mm
        y_camera = (cy - 0.5) * depth_mm
        z_camera = depth_mm
        
        # Transform to robot base frame
        point_camera = np.array([x_camera, y_camera, z_camera])
        point_base = camera_to_base(point_camera)
        
        print(f"Object at (base frame): {point_base} mm")
        
        # Use for IK solving (advanced)
        # left_arm = PibLeft()
        # target = SE3.Trans(*point_base)
        # solution = left_arm.ikine_LM(target)
```

---

## See Also

- [Trajectory Generation](trajectory.md) - High-level trajectory functions
- [RealRobotBackend](backends/robot.md) - Robot control
- [Low-Latency Mode](../tutorials/low-latency-mode.md) - Direct motor control
