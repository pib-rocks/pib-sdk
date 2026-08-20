# pib-sdk reference

Full detail on everything beyond the [README](../README.md)'s quick-start:
telemetry, the pib-backend REST client, the `Robot` facade, and every module
under `pib_sdk.features`. For *how these pieces fit together* — diagrams of
the trickier control flows (the voice assistant, Blockly programs, the
kinematics/Pinocchio layering) — see **[docs/ARCHITECTURE.md](ARCHITECTURE.md)**.

Two things are true of every module here:

* **No authentication.** pib-backend's REST API and rosbridge are both
  unauthenticated in the current backend — this SDK sends none. It's meant
  for a script running on the same trusted local network as the robot.
* **Nothing here is invented.** Every field name, unit, and caveat below was
  confirmed by reading pib-backend's own source, not guessed from
  Cerebra's UI. Where the backend genuinely doesn't expose something (e.g.
  RGB-button colors, true position sensor feedback), that's stated
  explicitly rather than papered over.

## Contents

- [`pib_sdk.telemetry`](#pib_sdktelemetry) — motor position/current readback
- [`pib_sdk.backend`](#pib_sdkbackend) — low-level REST client
- [`pib_sdk.robot`](#pib_sdkrobot) — bundles the four core clients
- [`pib_sdk.features.drawing`](#pib_sdkfeaturesdrawing) — image-to-trajectory drawing
- [`pib_sdk.features.poses`](#pib_sdkfeaturesposes) — poses from Cerebra
- [`pib_sdk.features.programs`](#pib_sdkfeaturesprograms) — run Blockly programs
- [`pib_sdk.features.buttons`](#pib_sdkfeaturesbuttons) — RGB-button bindings
- [`pib_sdk.features.camera`](#pib_sdkfeaturescamera) — camera snapshots
- [`pib_sdk.features.assistant`](#pib_sdkfeaturesassistant) — voice assistant
- [`pib_sdk.features.display`](#pib_sdkfeaturesdisplay) — pib's screen
- [`pib_sdk.features.relay`](#pib_sdkfeaturesrelay) — solid-state relay

---

## `pib_sdk.telemetry`

`Telemetry` mirrors `Write`'s connection pattern, but for reading rather than
commanding. It wraps the only two motor values pib-backend actually publishes:

```python
from pib_sdk.telemetry import Telemetry

telemetry = Telemetry(host="localhost")
print(telemetry.get_position_deg("elbow_right"))          # last commanded angle
print(telemetry.get_positions_deg(["elbow_right", "wrist_right"]))  # batch
print(telemetry.get_current_ma("elbow_right"))             # latest current draw (mA)
telemetry.subscribe_current(lambda name, ma: print(name, ma))
telemetry.close()
```

> **`get_position_deg` is not live sensor feedback.** It reports the last
> position *commanded* — by you or anyone else — via the `get_joint_position`
> service. pib-backend never publishes true physical position feedback over
> rosbridge or REST (its internal `get_current_position()` reading, backed by
> the servo's encoder, is only used by the robot's own startup-pose logic).
> This won't detect a stall or a hand-moved joint.
>
> **`get_current_ma` is genuine live telemetry** — electrical current in
> milliamps (Tinkerforge Servo Bricklet units), pushed ~4×/second per
> connected motor. A motor with no bricklet wired up simply never appears on
> it; that's steady state, not a timeout bug. Use `timeout=` to control how
> long `get_current_ma` waits, or `subscribe_current`/`unsubscribe_current`
> for a continuous stream instead of one-shot reads.
>
> These two values arrive over fundamentally different channels (position is
> pulled on demand via a service call, current is pushed continuously over a
> topic) — see the diagram at
> [docs/ARCHITECTURE.md § telemetry](ARCHITECTURE.md#deep-dive-telemetry--pull-vs-push).

---

## `pib_sdk.backend`

`BackendClient` is a thin REST client for pib-backend's Flask API (default
port `5000`) — the durable data store behind poses, programs, motor/camera
settings, and the voice assistant. It returns the API's own JSON shape
(camelCase keys) rather than converting to snake_case, mirroring
pib-backend's own internal `pib_api_client` package that every ROS node on
the robot uses to talk to this same API.

```python
from pib_sdk.backend import BackendClient

backend = BackendClient(host="localhost")   # port defaults to 5000
poses = backend.list_poses()                # [{poseId, name, deletable}, ...]
```

Higher-level, typed wrappers exist for poses/programs/buttons/assistant (see
their sections below) — reach for `BackendClient` directly when you want the
raw REST shape, or for a route this SDK doesn't wrap yet.

| Resource | Methods |
|---|---|
| Poses | `list_poses`, `get_pose_by_name`, `get_motor_positions`, `create_pose`, `rename_pose`, `update_motor_positions`, `delete_pose` |
| Programs | `list_programs`, `get_program`, `create_program`, `rename_program`, `delete_program`, `get_program_code`, `set_program_code` |
| Button bindings | `list_button_programs`, `set_button_programs` |
| Motors | `list_motors`, `get_motor_settings`, `update_motor_settings` |
| Camera settings | `get_camera_settings`, `update_camera_settings` |
| Voice-assistant personalities | `list_personalities`, `get_personality`, `create_personality`, `update_personality`, `delete_personality` |
| Voice-assistant chats | `list_chats`, `get_chat`, `get_chat_messages`, `create_chat`, `delete_chat` |

Errors (unreachable host, non-2xx response) raise `pib_sdk.backend.BackendError`
with the HTTP status and response body, or the underlying connection error.

Note: motor **settings** here are the same fields `Write.set()` sends
(`turnedOn`, `velocity`, `invert`, ...) — this is REST-based configuration
read/write, not live telemetry; see `pib_sdk.telemetry` for that.

---

## `pib_sdk.robot`

`Robot` bundles `Write`, `Speak`, `Telemetry`, and `BackendClient` behind one
`host`, since on a real robot they all point at the same machine and you'd
otherwise construct all four separately, repeating the host every time.

```python
from pib_sdk import right_arm
from pib_sdk.robot import Robot

with Robot(host="pib.local") as robot:
    robot.write.move(right_arm, -30.0)
    robot.speak.say("moving my arm")
    print(robot.telemetry.get_position_deg("shoulder_vertical_right"))
    print(robot.backend.list_poses())
```

`Robot(host=..., rosbridge_port=9090, backend_port=5000)` — override either
port if non-default. Nothing here is new behavior: use the four classes
directly instead if you only need one or two, or want different hosts/ports
for each (e.g. an SDK running somewhere other than the robot's own network).

---

## `pib_sdk.features.drawing`

Traces a raster image into a robot arm motion over a virtual drawing surface,
via inverse kinematics — built entirely on `pib_sdk.kinematics`/`control`, no
extra ROS/REST surface of its own.

```python
from pib_sdk.control import Write
from pib_sdk.kinematics import pose_from_xyz_rpy
from pib_sdk.features.drawing import DrawingSurface, image_to_sketch, sketch_to_trajectory

sketch = image_to_sketch("logo.png")   # needs Pillow: pip install pib-sdk[drawing]
surface = DrawingSurface(
    pose=pose_from_xyz_rpy([-350, 100, 850], rpy_deg=[0, 0, 0]),
    width_mm=200,
    height_mm=200,
)
trajectory = sketch_to_trajectory(sketch, surface, side="right")

with Write(host="localhost") as writer:
    trajectory.play(writer)
```

`image_to_sketch` is a simple flood-fill + nearest-neighbour tracer meant for
clean line art (sketches, logos), not a full vision pipeline — for
photographs or dense artwork, preprocess elsewhere and build a `Sketch`
directly from your own point lists. See the module's own docstring
(`src/pib_sdk/features/drawing.py`) for the full API (`Stroke`, `Sketch`,
`DrawingSurface`, `Trajectory`).

---

## `pib_sdk.features.poses`

Drives to, and saves, named poses a user creates in Cerebra by hand-posing
the robot. There's no single "move to pose" call on the robot — Cerebra
itself assembles one by fetching a pose's stored joint values and issuing one
batched joint-trajectory command — so this module does the same. Sequence
diagrams for both directions:
[docs/ARCHITECTURE.md § poses](ARCHITECTURE.md#deep-dive-poses--apply-vs-save).

```python
from pib_sdk.backend import BackendClient
from pib_sdk.control import Write
from pib_sdk.features.poses import set_pose, play_pose_sequence, save_current_pose
from pib_sdk.telemetry import Telemetry

backend = BackendClient(host="localhost")
writer = Write(host="localhost")

set_pose(writer, backend, name="wave_hello")

# A sequence of saved poses, held for the given number of seconds each --
# there's no native "pose sequence" concept in pib-backend (Cerebra only
# offers this via chained "move to pose" + "wait" Blockly blocks), so this
# just fetches and applies each pose in order.
play_pose_sequence(writer, backend, [("wave_hello", 1.5), ("rest", 0.0)])

# The other direction: read the robot's current commanded angles and save
# them to Cerebra as a new pose.
telemetry = Telemetry(host="localhost")
save_current_pose(telemetry, backend, "my_new_pose", motor_names=["elbow_right", "wrist_right"])
```

`get_pose`/`set_pose` accept either `name=` or `pose_id=`. Motor angles come
back in **degrees**, converted from pib-backend's internal hundredths-of-a-
degree units (confirmed against pib-backend's own `startup_pose_executor.py`,
which forwards a pose's stored value straight into a joint-trajectory command
with no scaling — same units `Write` already uses).

---

## `pib_sdk.features.programs`

Starts, stops, and streams output from Blockly programs saved in Cerebra.
Programs are stored via REST (`pib_sdk.backend`) but *executed* over
rosbridge through a service/topic "proxy" pib-backend built specifically
because rosbridge couldn't speak ROS2 actions when it was written — ordinary
rosbridge traffic, reachable like everything else in this SDK. Full proxy
sequence diagram and the `GoalStatus` state machine:
[docs/ARCHITECTURE.md § running a Blockly program](ARCHITECTURE.md#deep-dive-running-a-blockly-program).

```python
from pib_sdk.features.programs import Programs, list_programs, emergency_stop
from pib_sdk.backend import BackendClient

backend = BackendClient(host="localhost")
for program in list_programs(backend):
    print(program.program_number, program.name)

programs = Programs(host="localhost")
result = programs.run(program_number, on_output=print, timeout=30)
print(result.exit_code, result.status, result.timed_out)
```

`run()` blocks until the program finishes or `timeout` elapses (cancelling it
on timeout); `start()`/`stop()` are available if you'd rather not block.
`stop_all()` cancels every program started through that same `Programs`
instance that hasn't finished yet — it can't see (or stop) a program started
elsewhere, e.g. from Cerebra itself, since there's no "list running programs"
call on the backend.

`emergency_stop(writer, programs=None)` composes `stop_all()` (if a
`Programs` instance is given) with `writer.set(All, turned_on=False)` —
turning off every motor works regardless of what started the movement, even
without a `Programs` instance.

---

## `pib_sdk.features.buttons`

pib's RGB buttons are hardware-fixed status lights, not something you can set
the color of:

> **No color/brightness API exists.** pib-backend's `rgb_button_control` node
> hardcodes blue while idle, green while a program runs, red on error, and
> off when unassigned. There is nothing to "set" about a button's
> appearance — this isn't a missing feature of this SDK, the backend simply
> doesn't expose one.

What *is* controllable is which saved program each button starts when
pressed:

```python
from pib_sdk.backend import BackendClient
from pib_sdk.features.buttons import list_button_bindings, set_button_program

backend = BackendClient(host="localhost")
for binding in list_button_bindings(backend):
    print(binding.bricklet_number, "->", binding.program_number)

set_button_program(backend, bricklet_number=5, program_number=program_number)
set_button_program(backend, bricklet_number=5, program_number=None)  # unassign
```

---

## `pib_sdk.features.camera`

A single-frame JPEG snapshot — there's no live video streaming or on-board
vision/AI in pib-backend's camera interface, just this:

```python
from pib_sdk.features.camera import Camera

camera = Camera(host="localhost")
camera.save_snapshot("frame.jpg")
raw_bytes = camera.get_snapshot_bytes()
camera.close()
```

Camera *settings* (resolution, refresh rate, quality) are plain REST
configuration on `BackendClient.get_camera_settings()` /
`update_camera_settings()`, not part of this module.

---

## `pib_sdk.features.assistant`

pib's voice assistant: personalities and chats are pib-backend REST
resources (CRUD via `BackendClient`, wrapped here as typed dataclasses);
turning the assistant on/off, checking whether it's listening, and sending it
a message are live rosbridge calls via `Assistant`.

> **This is the same assistant Cerebra uses, not a second one.** Every call
> below hits a rosbridge service or REST route pib-backend already runs for
> Cerebra's own chat UI — `get_voice_assistant_state`, `set_voice_assistant_state`,
> `get_chat_is_listening`, `send_chat_message`, and the `/voice-assistant/...`
> REST routes. There is no dialogue/NLU/LLM logic in this SDK; sending a
> message here is indistinguishable, from pib-backend's side, from typing it
> into Cerebra. Full call sequence:
> [docs/ARCHITECTURE.md § voice assistant](ARCHITECTURE.md#deep-dive-the-voice-assistant-is-not-reimplemented-here).

```python
from pib_sdk.backend import BackendClient
from pib_sdk.features.assistant import Assistant, create_chat, list_personalities

backend = BackendClient(host="localhost")
personality = list_personalities(backend)[0]
chat = create_chat(backend, topic="testing", personality_id=personality.personality_id)

assistant = Assistant(host="localhost")
assistant.set_state(turned_on=True, chat_id=chat.chat_id)
assistant.send_message(chat.chat_id, "hello!")
assistant.close()
```

> **`send_message` doesn't return the assistant's reply.** The backend
> answers asynchronously (it may also run a program, speak out loud, etc.)
> and appends its response to the chat's message history. Poll
> `list_chat_messages(backend, chat.chat_id)` after sending to read it.

---

## `pib_sdk.features.display`

Pushes images to pib's physical screen over a fire-and-forget rosbridge
topic — publish and it shows, no request/response.

```python
from pathlib import Path
from pib_sdk.features.display import Display, ImageFormat

display = Display(host="localhost")
display.show_animated_eyes()                                    # pib's one built-in image
display.show_custom(Path("logo.png").read_bytes(), format=ImageFormat.PNG)
display.clear()
display.close()
```

`ImageFormat` is `ANIMATED_GIF`, `PNG`, or `JPEG` — it must match the actual
encoding of the bytes you pass to `show_custom`.

---

## `pib_sdk.features.relay`

pib's single on/off solid-state relay:

```python
from pib_sdk.features.relay import Relay

relay = Relay(host="localhost")
relay.set(True)
print(relay.get_state())
relay.subscribe(lambda turned_on: print("relay is now", turned_on))
relay.close()
```

`get_state()` rarely blocks long — pib-backend republishes the relay's state
about once a second regardless of whether it changed. What this relay
physically switches isn't documented in pib-backend's code; confirm with the
team before relying on it for anything safety-critical.
