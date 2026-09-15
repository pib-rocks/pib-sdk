# pib-sdk tutorials

These guides build on the [README](../README.md). Exact signatures and failure
behavior are in the [API reference](REFERENCE.md); complete scripts are indexed
in [Examples](EXAMPLES.md). Replace `pib.local` if your robot has another host.
Move one joint slowly on first use, keep clear of the mechanism, and retain a
way to remove motor power.

## Contents

- [First steps](#first-steps)
- [Reach a point with inverse kinematics](#reach-a-point-with-inverse-kinematics)
- [Save and smoothly replay poses](#save-and-smoothly-replay-poses)
- [Read color and depth](#read-color-and-depth)
- [Draw an image](#draw-an-image)
- [Use the voice assistant](#use-the-voice-assistant)
- [Run a Blockly program and bind a button](#run-a-blockly-program-and-bind-a-button)
- [Use the display and relay](#use-the-display-and-relay)

## First steps

Needs a live robot.

1. Create an environment and install:

   ```bash
   python3 -m venv .venv
   . .venv/bin/activate
   python -m pip install pib-sdk
   ```

2. Confirm that `pib.local:9090` (rosbridge) and `pib.local:5000`
   (pib-backend) are reachable from the same trusted network.
3. Read telemetry before moving. Position is the last commanded target, not
   encoder feedback; current is real milliamps and can time out when that
   motor has no connected current bricklet.
4. Enable defaults and move one motor through a small angle:

   ```python
   from pib_sdk.robot import Robot

   with Robot(host="pib.local") as robot:
       name = "elbow_right"
       print("commanded degrees:", robot.telemetry.get_position_deg(name))
       if not robot.write.set(name, default=True, velocity=3000):
           raise RuntimeError("motor settings were rejected")
       if not robot.write.move(name, 10.0):
           raise RuntimeError("move was rejected")
   ```

`Robot` closes its three rosbridge clients on exit. Run the complete
[`first_steps.py`](../examples/first_steps.py) script to include current
telemetry and a return to zero.

## Reach a point with inverse kinematics

Kinematics works offline; sending the result needs a live robot.

1. Construct one `ArmKinematics` and inspect its motor order and limits.
2. Choose XYZ in millimetres in pib's base frame.
3. Solve position-only IK. Add `rpy_deg=[roll, pitch, yaw]` only when tool
   orientation matters; six-axis constraints are harder to satisfy.
4. Verify with FK before moving.
5. Send exactly the same vector order to `Write.move`.

```python
from pib_sdk import ArmKinematics, Write, right_arm

arm = ArmKinematics("right")
target_mm = [-400.0, 100.0, 900.0]
q_deg = arm.inverse(target_mm)
reached = arm.forward(q_deg).translation
print(dict(zip(arm.motor_names, q_deg)), reached)

with Write(host="pib.local") as writer:
    if not writer.move(right_arm, *q_deg):
        raise RuntimeError("move rejected")
```

An unreachable target raises `ValueError`; it does not return a best-effort
unsafe command. Try the offline [`kinematics.py`](../examples/kinematics.py)
first.

## Save and smoothly replay poses

Needs a live robot and pib-backend.

1. Choose the motors to save. `save_current_pose` reads last-commanded
   targets, not hand-moved physical positions.
2. Save a named pose to the same store Cerebra uses.
3. For explicit stops, use `play_pose_sequence` with a hold after each pose.
4. For continuous motion, use `play_pose_sequence_timed`. Each number is the
   duration of that leg; the SDK sums these into absolute trajectory times.

```python
from pib_sdk import Write
from pib_sdk.backend import BackendClient
from pib_sdk.features.poses import play_pose_sequence_timed, save_current_pose
from pib_sdk.robot_model import get_arm_model
from pib_sdk.telemetry import Telemetry

host = "pib.local"
backend = BackendClient(host)
with Telemetry(host) as telemetry:
    save_current_pose(
        telemetry, backend, "sdk_start", get_arm_model("right").motor_names
    )

with Write(host) as writer:
    play_pose_sequence_timed(
        writer, backend, [("sdk_start", 1.0), ("rest", 2.0)]
    )
```

Timed playback sends one multi-point trajectory. The backend blends through
the via-points with monotone cubic Hermite interpolation. Poses with different
motor sets are normalized by carrying values forward and backfilling from the
first known value. This snippet assumes a Cerebra pose named `rest` already
exists; it is unrelated to the robot model's in-memory named configuration.
See [`poses_timed.py`](../examples/poses_timed.py).

## Read color and depth

Needs a live robot with camera/depth services.

```python
from pib_sdk.features.camera import Camera

with Camera("pib.local") as camera:
    camera.save_snapshot("frame.jpg")
    depth = camera.get_depth_frame()
    if depth is None:
        raise RuntimeError("no depth frame")
    valid = depth[depth != 0]
    print(depth.shape, "nearest mm:", int(valid.min()) if valid.size else None)
    print("center mm:", camera.get_distance_at_px(depth.shape[1] // 2, depth.shape[0] // 2))
    camera.save_depth("depth.npy")
```

Depth is a `numpy.ndarray` of shape `(height, width)`, dtype `uint16`, in
millimetres. Zero is invalid. `get_distance_at_px` also returns `0.0` for no
cache, invalid pixels, or out-of-bounds coordinates. The `.npy` file preserves
values exactly. See [`vision_depth.py`](../examples/vision_depth.py).

## Draw an image

Trajectory generation works offline; playback needs a live robot. Install the
drawing extra:

```bash
pip install "pib-sdk[drawing]"
```

1. Use clean dark line art on a light background.
2. Define the real paper surface in the robot base frame. Its local X/Y axes
   map image X/Y; local +Z is the pen-lift direction.
3. Generate the trajectory offline and inspect its size.
4. Only then attach the pen and play at a conservative rate.

```python
from pib_sdk import Write, pose_from_xyz_rpy
from pib_sdk.features.drawing import DrawingSurface, image_to_sketch, sketch_to_trajectory

sketch = image_to_sketch("line-art.png")
surface = DrawingSurface(
    pose_from_xyz_rpy([-350, 100, 850]), width_mm=120, height_mm=120, lift_mm=20
)
trajectory = sketch_to_trajectory(sketch, surface, "right", points_per_stroke=20)
print(len(trajectory), "waypoints")
with Write("pib.local") as writer:
    trajectory.play(writer, rate_hz=6)
```

Calibrate with the arm unpowered or without a pen first. The runnable
[`drawing_image.py`](../examples/drawing_image.py) takes the host and image
path as arguments.

## Use the voice assistant

Needs a live robot, pib-backend, and a configured assistant.

1. List typed `Personality` records.
2. Create a `Chat` with one personality.
3. Turn on `AssistantState` for that chat.
4. Send a `ChatMessage`. Acceptance is synchronous; the reply is not.
5. Poll `list_chat_messages` until an assistant message appears.

```python
import time
from pib_sdk.backend import BackendClient
from pib_sdk.features.assistant import (
    Assistant, create_chat, list_chat_messages, list_personalities
)

backend = BackendClient("pib.local")
personality = list_personalities(backend)[0]
chat = create_chat(backend, topic="SDK tutorial", personality_id=personality.personality_id)
with Assistant("pib.local") as assistant:
    assistant.set_state(turned_on=True, chat_id=chat.chat_id)
    if not assistant.send_message(chat.chat_id, "Say hello in one sentence."):
        raise RuntimeError("message rejected")
    while True:
        replies = [m for m in list_chat_messages(backend, chat.chat_id) if not m.is_user]
        if replies:
            print(replies[-1].content)
            break
        time.sleep(1)
```

Production code should add a polling deadline. See
[`voice_assistant.py`](../examples/voice_assistant.py).

## Run a Blockly program and bind a button

Needs a live robot and a Blockly program already saved in Cerebra.

```python
from pib_sdk.backend import BackendClient
from pib_sdk.features.buttons import set_button_program
from pib_sdk.features.programs import Programs, list_programs

host = "pib.local"
backend = BackendClient(host)
program = list_programs(backend)[0]
with Programs(host) as programs:
    result = programs.run(
        program.program_number,
        timeout=60,
        on_output=lambda line, stderr: print("ERR" if stderr else "OUT", line),
    )
print(result.status, result.exit_code, result.timed_out)
set_button_program(backend, bricklet_number=5, program_number=program.program_number)
```

Only goals started by a given `Programs` instance are visible to its
`stop_all`. Button color is backend-controlled status, not an SDK setting.
See [`program_and_button.py`](../examples/program_and_button.py).

## Use the display and relay

Needs a live robot. The display publish is fire-and-forget. The relay reports
state on a topic and may time out if hardware is absent.

```python
from pathlib import Path
from pib_sdk.features.display import Display, ImageFormat
from pib_sdk.features.relay import Relay

with Display("pib.local") as display:
    display.show_custom(Path("logo.png").read_bytes(), format=ImageFormat.PNG)

with Relay("pib.local") as relay:
    if not relay.set(True):
        raise RuntimeError("relay command rejected")
    print(relay.get_state())
    relay.set(False)
```

Confirm what the relay physically switches before use. The complete
[`display_and_relay.py`](../examples/display_and_relay.py) defaults to the
built-in eyes and requires `--enable-relay` before switching the relay.
