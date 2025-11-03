# pib-sdk
SDK for **pib** including forward, inverse kinematics and trajectory generation

## Features
* **One‑liner FK / IK** for the right & left arm using Robotics Toolbox under the hood  
* Ready‑made Denavit‑Hartenberg (DH) parameters for **pib**  
* Multi point trajectory generation
* Numeric **Jacobian** and analytical **pose error** utilities  
* Zero ROS – pure Python ≥ 3.9
* Writing joint values to ROS topic without ROS environment requirement
* Sending speech packets to voice assistant without ROS environment
* Variety of demos utilizing all pib features  

## Installation
requires-python>=3.9,<3.12
python 3.14 is automatically installed in new raspberry pi OS, you can either run the sdk in virtual environment or install a compatible python version like 3.9 alongside the default one and install the sdk with ``` pip3.9 ``` . Instructions are at the end of the ReadMe.
```
pip install pib-sdk
```

## Usage
```python
from pib_sdk.kinematics import fk, ik
'''
fk for Forward kinematics, returns pose of end effector from given angles
ik for inverse kinematics, returns joint angles from given end effector position [xyz] and orientation [rpy] (optional)
Specify right or left to calculate for designated pib arm
'''
print('FK pose:', fk('right', [0,45,0,0,90,0]))
print('IK angles:', ik('right', xyz=[150,0,350]))
# To write or read values from joints
```


## Getting started
```python
from pib_sdk.control import *
from pib_sdk.kinematics import ik, fk

# Control client
w = Write()  # connects to rosbridge (localhost:9090 by default)

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

### Per-motor angles (one call)
```python
w.move("shoulder_vertical_right", -30.0, "shoulder_horizontal_right", 10.0, "elbow_right", 5.0)
```
Clients are initiated with local host as default if between parenthisis is empty. To control a pib in the same network from your computer clients should be initiated like this
```python
w = Write(host="<ip_address>")  # Replace <ip_address> with the ip address of your pib
sp = Speak(host="<ip_address>") 
```

To Enable prinitng on terminal to monitor success and failure initiate clients with debug=True
```python
w = (debug=True)  # Default is false
```
---

## Speech 
pib-sdk is able to send speech commands to voice assistant ROS topic easily similar to movement
```python
from pib_sdk.speech import speak

# Connect to pib's ROSBridge
sp = speak()

# Speak text with default voice (Emma – Female, English)
sp.say("Hello, I am pib!")

# Speak with a specific voice preset
sp.say("Guten Tag!", voice="Hannah")

# Or explicitly define gender/language
sp.say("Hi there", gender="Male", language="English")
```

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


## Troubleshooting
- **ValueError: degrees range**  
  Angles must be within **−90 … +90**.  

- **ERROR: Could not find a version that satisfies the requirement mediapipe (from pib-sdk) (from versions: none)**  
  Python version must be equal to or between **3.9 … 3.11.9**.  

- **roslibpy.core.RosTimeoutError: Failed to connect to ROS**  
  pib software installation on host is not correct.  

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
w.move("shoulder_vertical_right", -30, "shoulder_horizontal_right", 5, "elbow_right", 10)

# Apply defaults to all
w.set(All, default)

# Apply custom velocity to two joints
w.set("shoulder_vertical_right", "wrist_right", velocity=8000)
```

## Installing a compatible python version alongside your current one in pi OS
```
sudo apt update
sudo apt install -y build-essential zlib1g-dev libncurses5-dev libgdbm-dev \
libnss3-dev libssl-dev libreadline-dev libffi-dev libsqlite3-dev wget libbz2-dev

cd /usr/src
sudo wget https://www.python.org/ftp/python/3.9.18/Python-3.9.18.tgz
sudo tar -xf Python-3.9.18.tgz
cd Python-3.9.18

sudo ./configure --enable-optimizations
sudo make -j$(nproc)
sudo make altinstall
```

