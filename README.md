# pib-sdk
SDK for **pib** including forward, inverse kinematics and trajectory generation

## Features
* **One‑liner FK / IK** for the right & left arm using Robotics Toolbox under the hood  
* Ready‑made Denavit‑Hartenberg (DH) parameters for **pib**  
* Multi point trajectory generation
* Numeric **Jacobian** and analytical **pose error** utilities  
* Zero ROS / Gazebo dependencies – pure Python ≥ 3.9  

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
```
