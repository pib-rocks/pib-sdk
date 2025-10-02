# pib-sdk
SDK for **pib** including forward, inverse kinematics and trajectory generation

## Features
* **One‑liner FK / IK** for the right & left arm using Robotics Toolbox under the hood  
* Ready‑made Denavit‑Hartenberg (DH) parameters for **pib**  
* Multi point trajectory generation
* Numeric **Jacobian** and analytical **pose error** utilities  
* Zero ROS / Gazebo dependencies – pure Python ≥ 3.9
* Writing joint values to ROS topic without ROS enbironment requirement  

## Installation
```
pip install pib-sdk
```

## Usage
```
from pib_sdk.kinematics import fk, ik
'''
fk for Forward kinematics, returns pose of end effector from given angles
ik for inverse kinematics, returns joint angles from given end effector position [xyz] and orientation [rpy] (optional)
Specify right or left to calculate for designated pib arm
'''
print('FK pose:', fk('right', [0,45,0,0,90,0]))
print('IK angles:', ik('right', xyz=[150,0,350]))
# To write or read values from joints
write('shoulder_vertical_right', position=20, velocity=5000)
read('shoulder_horizontal_left')
```


## Getting started

```python
from pib_sdk.control import *
from pib_sdk.kinematics import ik, fk

# Control client
w = Write(debug=True)  # connects to rosbridge (localhost:9090 by default)

# Move a couple of joints with the same angle
w.move("shoulder_vertical_right", "elbow_right", -30.0)

# Broadcast to all
w.move(All, -45.0)
w.move(All, zero_position)        # every motor to 0°
w.move(All, resting_position)     # special: elbows=5000 internal, fingers=-9000, others=0
```

### Hand presets
```python
w.move(open_hand_left)   # all left fingers (endswith *_stretch) to -90°
w.move(close_hand_right) # all right fingers to +90°
```

### Arm groups
```python
# All non-finger, non-thumb-opposition joints whose names end with _right / _left
w.move(right_arm, -20.0)
w.move(left_arm,  15.0)
```

> **Naming rule recap**
>
> - **Fingers**: name ends with `_stretch`  
> - **Right/Left**: motor name ends with `_right` / `_left`  

### Per-motor angles (one call)
```python
w.move("shoulder_vertical_right", "shoulder_horizontal_right", "elbow_right",
       -30.0, 10.0, 5.0)
```

---

## Settings API

Apply settings to one or more motors:

```python
w.set("shoulder_vertical_right",
      turned_on=True, velocity=6000, acceleration=10000,
      deceleration=5000, period=19500,
      pulse_width_min=700, pulse_width_max=2500,
      rotation_range_min=-9000, rotation_range_max=9000,
      visible=True, invert=False)
```

Multiple motors, same settings:
```python
w.set("shoulder_vertical_right", "wrist_right", velocity=6000, acceleration=10000)
```

All motors, default preset (you can override any field inline):
```python
w.set(All, default)                   # or w.set(All, default=True)
w.set(All, default=True, velocity=12_000)  # override velocity only
```

**Default preset** (applied by `default`):
```
turned_on=True
velocity=16000
acceleration=10000
deceleration=5000
period=19500
pulse_width_min=700
pulse_width_max=2500
rotation_range_min=-9000
rotation_range_max=9000
visible=True
invert=False
```

---

## IK / FK

`kinematics.py` provides one-liners around your `pib_DH` models.

```python
from pib_sdk.kinematics import ik, fk

# Inverse kinematics: position-only (mm)
q_deg = ik("right", xyz=[150, 0, 350])         # returns np.ndarray of degrees (len = DOF)

# Move the whole right arm using that IK (broadcasting one angle per joint)
from pib_sdk.control import Write, right_arm
w = Write()
w.move(right_arm, *q_deg)

# Forward kinematics
pose = fk("right", q_deg=[0, 45, 0, 0, 90, 0]) # returns SE3
print(pose)
```

**Notes**
- If your arm has more joints than the IK vector, slice: `w.move(right_arm, *q_deg[:N])`.
- IK defaults to **position-only** (RPY ignored); pass `rpy_deg=[roll, pitch, yaw]` to constrain orientation.
- `ik(..., q0_deg=[...])` sets an initial guess (deg). Convergence settings: `tol`, `max_steps`, `custom_mask`.

---

## Motor discovery & database

`Write` builds the motor list from two sources:

1. **SQLite DB** (optional): looks for table `motor(name)` in  
   `/home/pib/app/pib-backend/pib_api/flask/pibdata.db`  
   Override via:
   - env var: `PIB_MOTOR_DB=/path/to/pibdata.db`, or
   - constructor: `Write(db_path="/path/to/pibdata.db")`
2. **Telemetry**: any name seen on `/motor_settings` is cached.

You can inspect what’s known:

```python
print(w._get_all_motors())
```

---

## CLI (optional)

A tiny CLI is included:

```bash
python -m pib_sdk.control send --motor elbow_right --position-deg -30
python -m pib_sdk.control echo --motor elbow_right
```

Common flags:
- `--turn-on`, `--set-defaults`
- `--velocity`, `--acceleration`, `--deceleration`, `--period`
- `--verify-echo` (waits for `/motor_settings` echo), `--echo-timeout`

---

## Troubleshooting

- **“Only the first joint moved”**  
  Some servers ignore multi-joint trajectories. The SDK sends **one service call per joint** for multi-motor moves to guarantee motion.

- **“No motors known yet”**  
  Provide the DB path/env var *or* let one `/motor_settings` message flow to seed names.

- **ValueError: degrees range**  
  Angles must be within **−90 … +90**.

- **Arm groups empty / missing joints**  
  Check your naming: arm members must end with `_right` / `_left`. Fingers end with `_stretch` (excluded). Thumb opposition joints (contain both `thumb` and `opposition`) are excluded.

---

## Examples cheat-sheet

```python
# All to -90
w.move(All, -90)

# All to 0
w.move(All, zero_position)

# Resting
w.move(All, resting_position)

# Open/close hands
w.move(open_hand_left)
w.move(close_hand_right)

# Right/left arm broadcast
w.move(right_arm, -15)
w.move(left_arm,  20)

# Per-joint list
w.move("shoulder_vertical_right", "shoulder_horizontal_right", "elbow_right",
       -30, 5, 10)

# Apply defaults to all
w.set(All, default)

# Apply custom velocity to two joints
w.set("shoulder_vertical_right", "wrist_right", velocity=8000)
```

