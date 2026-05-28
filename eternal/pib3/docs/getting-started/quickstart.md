# Quick Start

## Prerequisites

```bash
pip install "pib3 @ git+https://github.com/mamrehn/pib3.git"
```

---

## Image to Trajectory

### Generate Trajectory

```python
import pib3

# Convert image to robot trajectory
trajectory = pib3.generate_trajectory("my_drawing.png")
trajectory.to_json("my_trajectory.json")
print(f"Generated {len(trajectory)} waypoints")
```

### Visualize in Simulation (Do this before running on real robot!)

```python
from pib3 import Webots, Trajectory

trajectory = Trajectory.from_json("my_trajectory.json")
with Webots() as sim:
    sim.run_trajectory(trajectory)
```

### Execute on Robot

```python
from pib3 import Robot, Trajectory

trajectory = Trajectory.from_json("my_trajectory.json")
with Robot(host="172.26.34.149") as robot:
    robot.run_trajectory(trajectory)
```

---

## Joint Control

pib3 uses percentage-based control: **0%** = min, **100%** = max, **50%** = middle.

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Move single joint
    robot.set_joint(Joint.TURN_HEAD, 50.0)

    # Read position
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    print(f"Elbow at {pos:.1f}%")

    # Save and restore pose
    saved_pose = robot.get_joints()
    robot.set_joint(Joint.ELBOW_LEFT, 25.0)
    robot.set_joints(saved_pose)  # Restore

    # Using radians
    robot.set_joint(Joint.ELBOW_LEFT, 1.25, unit="rad")
```

---

## Webots Visualization

### Run Trajectory

```python
from pib3 import Webots, Trajectory

trajectory = Trajectory.from_json("my_trajectory.json")
with Webots() as viz:
    viz.run_trajectory(trajectory)
```

### Interactive Mode

```python
from pib3 import Webots

viz = Webots()
viz.connect()
viz.launch_interactive()  # Sliders at http://localhost:8001
# 3D view at http://localhost:52000
# Press Ctrl+C to exit
```

---

## Step-by-Step Processing

For more control over the pipeline:

```python
import pib3

# Step 1: Image → Sketch (2D strokes)
sketch = pib3.image_to_sketch("drawing.png")
print(f"Extracted {len(sketch)} strokes")

# Step 2: Sketch → Trajectory (3D robot motion)
trajectory = pib3.sketch_to_trajectory(sketch)
print(f"Generated {len(trajectory)} waypoints")

# Step 3: Save or execute
trajectory.to_json("output.json")
```

---

## Custom Configuration

```python
import pib3
from pib3 import TrajectoryConfig, PaperConfig, ImageConfig

config = TrajectoryConfig(
    paper=PaperConfig(
        size=0.15,           # 15cm paper
        height_z=0.74,       # Table height
        drawing_scale=0.9,   # Use 90% of paper
    ),
    image=ImageConfig(
        threshold=100,       # Darker threshold
        simplify_tolerance=3.0,
    ),
)

trajectory = pib3.generate_trajectory("drawing.png", config=config)
```

---

## Audio System

Play audio, record from microphones, and use text-to-speech:

```python
from pib3 import Robot, AudioOutput, AudioInput

with Robot(host="172.26.34.149") as robot:
    # Text-to-speech (German by default)
    robot.speak("Hallo, ich bin pib!")

    # Play on local speakers
    robot.speak("Hello!", output=AudioOutput.LOCAL)

    # Play a WAV file on the robot
    robot.play_file("sound.wav", output=AudioOutput.ROBOT)

    # Record 5 seconds from local microphone
    audio_data = robot.record_audio(duration=5.0, input_source=AudioInput.LOCAL)

    # Play recorded audio back on robot
    robot.play_audio(audio_data, output=AudioOutput.ROBOT)
```

See the [Audio System API](../api/audio.md) for complete documentation.

---

## Joint Names

| Group | Joints |
|-------|--------|
| **Head** | `turn_head_motor`, `tilt_forward_motor` |
| **Left Arm** | `shoulder_vertical_left`, `shoulder_horizontal_left`, `upper_arm_left_rotation`, `elbow_left`, `lower_arm_left_rotation`, `wrist_left` |
| **Left Hand** | `thumb_left_opposition`, `thumb_left_stretch`, `index_left_stretch`, `middle_left_stretch`, `ring_left_stretch`, `pinky_left_stretch` |
| **Right Arm/Hand** | Same as left, replace `_left` with `_right` |

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No IK solutions | Move paper closer, use smaller scale |
| Robot not responding | Check power, rosbridge, IP: `ping 172.26.34.149` |
| Joints not reaching targets | [Calibrate joint limits](calibration.md) |

---

**Next:** [Calibration](calibration.md) | [Image to Trajectory](../tutorials/image-to-trajectory.md) | [Controlling the Robot](../tutorials/controlling-robot.md) | [Audio System](../api/audio.md)
