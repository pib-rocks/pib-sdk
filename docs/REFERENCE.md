# pib-sdk reference

Complete API reference for pib-sdk. For task-oriented walkthroughs see the
[tutorials](TUTORIALS.md), and for complete scripts see the
[examples index](EXAMPLES.md). For *how these pieces fit together* — diagrams of
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

- [`pib_sdk.control`](#pib_sdkcontrol) — motor settings, movement, and timed trajectories
- [`pib_sdk.kinematics`](#pib_sdkkinematics) — forward/inverse kinematics
- [`pib_sdk.robot_model`](#pib_sdkrobot_model) — URDF chain definitions and Pinocchio models
- [`pib_sdk.speech`](#pib_sdkspeech) — text-to-speech
- [`pib_sdk.models`](#pib_sdkmodels) — start and stop on-device neural networks
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
- [`pib_sdk.features.imu`](#pib_sdkfeaturesimu) — live IMU acceleration and gyro

---

## `pib_sdk.control`

`Write` controls motors through rosbridge services. Construction immediately
connects; `Write(host="localhost", port=9090, joint_trajectory_service="/apply_joint_trajectory",
motor_settings_service="/apply_motor_settings",
joint_trajectory_topic="/joint_trajectory",
motor_settings_topic="/motor_settings", debug=False)` raises
`ConnectionError` if rosbridge is not connected within five seconds. Use
`with Write(...) as writer:` or call `close()`. `close() -> None` suppresses
shutdown errors and is safe to call repeatedly.

### Selectors, actions, and settings

Selectors are either literal firmware motor-name strings or exported tokens.
They expand locally; no connection or backend lookup is involved.

| Token | Expansion or meaning |
|---|---|
| `All` | Sorted union of all known arm, hand, and head motor names |
| `right_arm`, `left_arm` | Six arm motors, base-to-tip IK order |
| `right_hand`, `left_hand` | Six finger motors |
| `head` | `turn_head_motor`, `tilt_forward_motor` |
| `open_left_hand`, `open_right_hand` | Move all six motors of that hand to −90° |
| `close_left_hand`, `close_right_hand` | Move all six motors of that hand to +90° |
| `default` | Ask `set` to merge `DEFAULT_SETTINGS` |
| `zero_position` | Integer `0`, for explicit zero-degree moves |

`DEFAULT_SETTINGS` enables and exposes a motor and supplies the current
velocity, acceleration, deceleration, pulse-width, period, and ±9000 internal
rotation-range defaults. Those setting values are passed through unchanged;
only `move` converts public degrees to hundredths of a degree.

#### `Write.set`

`set(*motor_specs: str | token, verify_echo: bool = False,
echo_timeout: float = 1.0, **settings) -> bool`

Applies settings to each expanded motor. With no selector it selects `All`.
Place `default` last or pass `default=True` to merge `DEFAULT_SETTINGS`
under explicit keywords. Supported backend fields include `turned_on`,
`visible`, `invert`, `velocity`, `acceleration`, `deceleration`,
`pulse_width_min`, `pulse_width_max`, `period`, `rotation_range_min`, and
`rotation_range_max`; `None` and `position` are omitted.

The return is `True` only when all motors report settings applied or
persisted. Service errors are logged and become `False`. If the service
reports failure and `verify_echo=True`, an observed matching settings echo
can make that motor successful. Unsupported selector types raise `TypeError`;
unknown private token objects can raise `ValueError`.

```python
from pib_sdk import All, Write, default, left_hand

with Write(host="pib.local") as writer:
    assert writer.set(All, default)
    writer.set(left_hand, velocity=6000, invert=False)
```

#### `Write.move`

`move(*args: str | token | int | float) -> bool`

Angles are degrees in the inclusive range `[-90, 90]`. Uniform mode
(`move(right_arm, -20)`) applies the trailing angle to every selector.
Vector mode (`move(right_arm, a, b, c, d, e, f)`) consumes one angle per
expanded motor; literal names may be interleaved with values. A lone hand
action token opens or closes that hand. The method sends one batched request
and, if rejected, retries each joint separately.

It returns the backend success flag; service errors are logged and return
`False`. Missing arguments or motors and out-of-range angles raise
`ValueError`; malformed or unsupported arguments raise `TypeError`.
`writer.verify_echo = True` enables best-effort trajectory echo checking,
which is non-fatal if no echo arrives.

#### `Write.send_timed_trajectory`

`send_timed_trajectory(joint_names: list[str],
waypoints: list[tuple[list[float], float]]) -> bool`

This is the low-level smooth-motion API. Each waypoint is
`(positions_internal_per_joint, time_from_start_seconds)`. Unlike `move`,
positions are already in pib's internal **hundredths of a degree**. Every
positions list must match `joint_names`; times are absolute from trajectory
start, not per-leg durations. Supply non-negative, increasing times. Each is
converted to ROS `Duration {sec, nanosec}`.

The current pib-backend software trajectory controller blends through
multi-point trajectories using monotone cubic Hermite interpolation, so
intermediate points are via-points rather than stop-and-hold commands.
The method returns the service success flag and returns `False` on a service
exception. Empty names/waypoints, mismatched position counts, or negative
times raise `ValueError`. The SDK does not itself reject equal or decreasing
non-negative times; callers should not send them.

```python
with Write(host="pib.local") as writer:
    writer.send_timed_trajectory(
        ["elbow_right", "wrist_right"],
        [([0.0, 0.0], 1.0), ([4500.0, -1000.0], 2.5)],
    )
```

### Command-line entry point

Installation provides `pib-control` (equivalent to
`python -m pib_sdk.control`):

```bash
pib-control --host pib.local move right_arm 0 20 0 45 0 0
pib-control --host pib.local set All --use-default
pib-control --host pib.local set wrist_right --velocity 6000
```

The `move` command follows the same token/name modes as `Write.move`.
`set` accepts boolean flags, motion settings, pulse widths, and
`--min-deg`/`--max-deg`; despite those two CLI option names, the current code
forwards their numeric values directly as `rotation_range_min/max` without
degree conversion.

---

## `pib_sdk.kinematics`

All angles are degrees, translations are millimetres in pib's base frame, and
poses are real `pinocchio.SE3` objects. RPY means `[roll, pitch, yaw]` in
degrees with `Rz(yaw) @ Ry(pitch) @ Rx(roll)`.

### Pose conversion

| API | Signature and behavior |
|---|---|
| `pose_from_xyz_rpy` | `(xyz: Sequence[float], rpy_deg: Sequence[float] | None = None) -> pin.SE3`; builds a pose. `xyz` must contain three values (`ValueError` otherwise). An RPY sequence must unpack to three values. |
| `pose_to_xyz_rpy` | `(pose: pin.SE3) -> tuple[np.ndarray, np.ndarray]`; returns copied XYZ millimetres and RPY degrees. Attribute/type errors from an invalid pose propagate. |

### `ChainKinematics`

`ChainKinematics(chain: ChainModel)` reuses one mutable Pinocchio data buffer
for a chain. Do not share one instance across concurrent solver calls.

| Member | Type / purpose |
|---|---|
| `degrees_of_freedom` | `int`; number of configuration values |
| `joint_names` | `list[str]`; copied URDF names in vector order |
| `motor_names` | `list[str]`; copied firmware names in the same order |
| `joint_limits_deg` | `(lower: np.ndarray, upper: np.ndarray)` in degrees |
| `named_configuration(name: str)` | `np.ndarray` degrees; raises `KeyError` with available names if unknown |
| `forward(joint_angles_deg: Iterable[float])` | `pin.SE3`; raises `ValueError` unless the vector length equals the DOF |

`inverse(xyz, rpy_deg=None, initial_guess_deg=None, tolerance=1e-4,
max_iterations=200, mask=None, restarts=50, respect_limits=True) ->
np.ndarray` solves IK and returns degrees. `xyz` is three millimetre
coordinates. Omitting RPY selects position-only IK; supplying it selects all
six pose-error components. `mask` overrides that choice with six truthy/falsy
entries ordered `[x, y, z, rx, ry, rz]`. `tolerance` is the norm threshold,
`max_iterations` applies to each attempt, and `restarts` is the number of
deterministic random in-limit seeds after the initial guess.

With `respect_limits=True`, only an in-limit solution is returned.
`ValueError` is raised for malformed XYZ/RPY/mask input, wrong vector sizes,
or non-convergence. Numeric conversion and linear-algebra errors can
propagate for invalid numeric values.

`ArmKinematics(side: ArmSide | str = ArmSide.RIGHT)` selects a cached arm
model. Invalid strings raise `ValueError`. `HeadKinematics()` selects the
two-joint camera chain; its
`camera_pose(pan_deg: float = 0, tilt_deg: float = 0) -> pin.SE3` is forward
kinematics for `turn_head_motor` then `tilt_forward_motor`.

### Convenience functions and compatibility classes

| API | Signature, return, and errors |
|---|---|
| `fk` | `(side: ArmSide | str, q_deg: Iterable[float]) -> pin.SE3`; one-call arm FK; same errors as `ArmKinematics.forward`. |
| `ik` | `(side, *, xyz, rpy_deg=None, **solver_options) -> np.ndarray`; forwards options and errors to `ChainKinematics.inverse`. |
| `camera_pose` | `(pan_deg=0.0, tilt_deg=0.0) -> pin.SE3`; one-call head FK. |
| `get_hand_position_xyz` | `(side, *, telemetry_host="localhost", telemetry_port=9090, telemetry=None) -> tuple[float, float, float]`; reads the six last-commanded motor angles and returns hand-tip XYZ in mm. It owns a temporary `Telemetry` only when none is supplied. Raises `KeyError` for missing readings plus connection/service/model errors. This is not encoder feedback. |
| `FK` | `(side=ArmSide.RIGHT)` compatibility wrapper. `pose(q_deg) -> pin.SE3` calls arm FK; `joint_names` is captured at construction. |
| `IK` | `(side=ArmSide.RIGHT)` compatibility wrapper. `solve(xyz, rpy_deg=None, q0_deg=None, tol=1e-4, max_steps=100, custom_mask=None) -> np.ndarray` maps old option names to `inverse`; errors propagate. |

```python
from pib_sdk import ArmKinematics

arm = ArmKinematics("right")
q_deg = arm.inverse([-400, 100, 900])
print(arm.motor_names, arm.forward(q_deg).translation)
```

---

## `pib_sdk.robot_model`

This module loads the bundled V3 URDF through Pinocchio, locks unrelated
joints, and scales fixed translations from metres to millimetres.

`ArmSide` is a string enum with `RIGHT = "right"` and `LEFT = "left"`.
`coerce_arm_side(side: ArmSide | str) -> ArmSide` accepts case-insensitive
strings and raises `ValueError` for anything else.

`ChainDefinition(name: str, urdf_joint_names: tuple[str, ...],
motor_names: tuple[str, ...], tip_frame: str,
named_configurations_deg: Mapping[str, tuple[float, ...]] = {})` is an
immutable dataclass. Construction raises `ValueError` if motor/joint lengths
or named-configuration lengths differ. Exported definitions are `RIGHT_ARM`,
`LEFT_ARM`, and `HEAD`.

`ChainModel(definition: ChainDefinition, model: pin.Model,
tip_frame_id: int)` is the immutable loaded model:

| Member | Return / meaning |
|---|---|
| `degrees_of_freedom` | `int` (`model.nq`) |
| `joint_names` | `tuple[str, ...]`, URDF order |
| `motor_names` | `tuple[str, ...]`, matching firmware order |
| `joint_lower_limits`, `joint_upper_limits` | copied `np.ndarray` values in radians |
| `joint_limits_deg` | pair of copied arrays converted to degrees |
| `named_configuration_deg(name)` | copied `np.ndarray` degrees; `KeyError` if absent |
| `create_data()` | fresh `pinocchio.Data`; each solver/caller should own one |

`build_chain_model(definition, urdf_path: str | Path | None = None) ->
ChainModel` builds an uncached reduced model. It raises `FileNotFoundError`
for a missing URDF and `ValueError` for missing joints/frame, unexpected
order, or non-single-axis chain joints; Pinocchio parse errors propagate.

`get_chain_model(definition) -> ChainModel` caches by definition name and
resolved URDF path. `get_arm_model(side) -> ChainModel` selects `RIGHT_ARM`
or `LEFT_ARM`; `get_head_model() -> ChainModel` selects `HEAD`.
`set_urdf_path(urdf_path: str | Path) -> None` writes `PIB_URDF_PATH` and
clears both caches. The environment variable is also honored directly.

```python
from pib_sdk.robot_model import RIGHT_ARM, build_chain_model

chain = build_chain_model(RIGHT_ARM, "/path/to/pib.urdf")
print(chain.joint_names, chain.joint_limits_deg)
```

---

## `pib_sdk.speech`

`Speak(host="localhost", port=9090, debug=False,
service_name="play_audio_from_speech",
service_type="datatypes/PlayAudioFromSpeech", connect_timeout=10.0)`
connects immediately and raises `RuntimeError` if rosbridge is unavailable.
Use a context manager or `close() -> None`; close suppresses shutdown errors.
The backwards-compatible alias `speak` names the same class.

`say(text: str, *, voice: str | None = None, gender: str | None = None,
language: str | None = None, join: bool = True, timeout: float = 30.0) ->
dict[str, Any]` blocks for the speech service response. Presets are Hannah
(Female/German), Daniel (Male/German), Emma (Female/English, default), and
Brian (Male/English). A preset wins over explicit gender/language. If both
explicit values are not supplied, it falls back to Emma. `join` is passed to
the service.

Empty text or an unknown preset raises `ValueError`; a closed connection or
service error raises `RuntimeError`; no callback before the timeout raises
`TimeoutError`. Otherwise the raw response dictionary is returned.

---

## `pib_sdk.models`

Starts and stops the camera node's on-device neural networks over rosbridge
(`list_models`, `start_model`, `stop_model`, `stop_all_models`). pib-backend
HTTP does **not** proxy these services; `BackendClient` has no equivalent routes.

```python
from pib_sdk import Models

with Models(host="localhost") as models:
    for entry in models.list_models():
        print(entry.model_id, entry.active)
    result = models.start_model("hand_tracking")
    print(result.success, result.message)
    models.stop_model("hand_tracking")
    models.stop_all_models()
```

`Models(host="localhost", port=9090, owner=None,
list_models_service="list_models", start_model_service="start_model",
stop_model_service="stop_model", connect_timeout=5.0)` connects immediately
and raises `ConnectionError` if rosbridge is not connected within
`connect_timeout`. Use a context manager or `close() -> None`; close
suppresses shutdown errors. The default owner is the fixed string `pib-sdk`,
deliberately *not* a per-process identifier. The node matches start and stop
calls by owner, so a changing owner makes a model started by one script
impossible to stop from another - measured on the robot, where the node answered
`was not requested by <owner>` and left the model running while still reporting
success. Pass `owner=` on the constructor or on `start_model` / `stop_model` to
override it when callers must be told apart; `stop_model` always sends both
`model_id` and `owner` because the node rejects an id-only stop.

`list_models(timeout=10.0) -> list[ModelInfo]` returns the node's
`ModelInfo` entries. `start_model(model_id: str, *, owner: str | None = None,
timeout=120.0) -> ModelResult` and `stop_model(model_id: str, *, owner: str |
None = None, timeout=120.0) -> ModelResult` return `success` plus the node's
`message`. `stop_all_models(timeout=120.0, owner: str | None = None) ->
list[tuple[str, ModelResult]]` lists current models, then calls `stop_model`
for each entry whose `active` is true (instance owner unless `owner` is
passed). It returns one `(model_id, ModelResult)` pair per active model and
does **not** raise on the first failure: a rejected or failed stop is
recorded with `success=False` so the caller can see which models stopped
and which did not. Nothing active returns `[]`. "All" is not completeness:
the node reference-counts consumers and only stops what that owner started.
A foreign-owner rejection (`was not requested by <owner>` while the node
still reports success) is an expected per-model outcome here, not an
exception that aborts the rest. Callers never pass a shave budget: `start_model` always sends
`shaves=0` (the StartModel.srv registry / manifest default). A mismatched
shave count would be rejected by the node. The default timeout is generous on
purpose: starting a model rebuilds the camera pipeline on the robot and was
measured to outlast 30s, so a shorter timeout raises while the model is still
coming up and makes a successful start look like a failure.

`success=false` raises `ModelError` with the node's message (not a silent
no-op), except in `stop_all_models`, which records that outcome and continues.
`stop_model` also raises `ModelError` when the node ignored the stop -
that is, when the reply names a different owner - because the node answers
`success=true` for every rejection it makes. Transport failures during a call
(an unanswered or dropped service call) raise `ModelError` as well, with the
original exception as `__cause__`, and a failed connection raises
`ConnectionError`; the transport's own exception types are never passed through.
Using an already closed client raises `RuntimeError`.

Two node behaviours worth knowing, both measured:

- Starting a model that is *already* running answers
  `'Model <id> already requested by <owner>'` with `success=true`. That is
  correct - the model is up - but a caller cannot tell "I started it" from
  "it was already running" through the result alone.
- The node reference-counts consumers. A `stop_model` it accepts can still leave
  the model running while a subscriber holds it, answering
  `'Model <id> remains in use'`. That is an accepted stop, not a failure, and the
  model stops once the last consumer goes away. A client that dies without
  unsubscribing can keep a model alive that no later call can stop; only
  restarting the camera container clears it.

`ModelInfo` fields: `model_id: str`, `task: str`, `licence: str`,
`shaves: int`, `size_bytes: int`, `available: bool`, `active: bool`.
`ModelResult` fields: `success: bool`, `message: str`.

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

`Telemetry(host="localhost", port=9090,
get_position_service="get_joint_position",
motor_current_topic="motor_current")` connects immediately and raises
`ConnectionError` after five seconds. It owns a background current
subscription. Use a context manager or `close() -> None`.

| Method | Parameters and return | Errors |
|---|---|---|
| `get_position_deg(motor_name, timeout=5.0)` | motor firmware name; returns last commanded `float` degrees | `ValueError` if rejected; service exceptions propagate |
| `get_positions_deg(motor_names, timeout=5.0)` | iterable of names; sequentially returns `dict[str, float]` | first per-motor error propagates |
| `get_current_ma(motor_name, timeout=2.0)` | waits for and returns cached `int` milliamps | `TimeoutError` if unseen |
| `subscribe_current(callback)` | registers `callback(name: str, current_ma: int) -> None` | callback exceptions occur on the topic callback thread |
| `unsubscribe_current(callback)` | removes the callback if present; returns `None` | no error if absent |

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
Malformed JSON raises `json.JSONDecodeError`; absent response keys raise
`KeyError`.

`BackendClient(host="localhost", port=5000, timeout=10.0)` stores a base URL;
it opens an HTTP request only when a method is called and has no `close`.
All identifiers and names are URL-quoted. Complete signatures and raw JSON
returns:

| Methods | Signatures and return values |
|---|---|
| Poses | `list_poses() -> list[dict]`; `get_pose_by_name(name: str) -> dict`; `get_motor_positions(pose_id: str) -> list[dict]`; `create_pose(name: str, motor_positions: list[dict]) -> dict`; `rename_pose(pose_id: str, name: str) -> dict`; `update_motor_positions(pose_id: str, motor_positions: list[dict]) -> dict`; `delete_pose(pose_id: str) -> None` |
| Programs | `list_programs() -> list[dict]`; `get_program(program_number: str) -> dict`; `create_program(name: str) -> dict`; `rename_program(program_number: str, name: str) -> dict`; `delete_program(program_number: str) -> None`; `get_program_code(program_number: str) -> str`; `set_program_code(program_number: str, code_visual: str) -> None` |
| Buttons | `list_button_programs() -> list[dict]`; `set_button_programs(updates: list[dict]) -> list[dict]`, where each update has `brickletNumber` and nullable `programNumber` |
| Motors | `list_motors() -> list[dict]`; `get_motor_settings(name: str) -> dict`; `update_motor_settings(name: str, settings: dict) -> dict` |
| Camera | `get_camera_settings() -> dict`; `update_camera_settings(settings: dict) -> dict` |
| Personalities | `list_personalities() -> list[dict]`; `get_personality(personality_id: str) -> dict`; `create_personality(personality: dict) -> dict`; `update_personality(personality_id: str, personality: dict) -> dict`; `delete_personality(personality_id: str) -> None` |
| Chats | `list_chats() -> list[dict]`; `get_chat(chat_id: str) -> dict`; `get_chat_messages(chat_id: str) -> list[dict]`; `create_chat(topic: str, personality_id: str) -> dict`; `delete_chat(chat_id: str) -> None` |

Note: `update_motor_settings` forwards the supplied dictionary unchanged.
Use the backend field spelling represented by the current SDK paths
(`turned_on`, `velocity`, `invert`, ...). This is REST-based configuration
read/write, not live telemetry; see `pib_sdk.telemetry` for that.

---

## `pib_sdk.robot`

`Robot` bundles `Write`, `Speak`, `Telemetry`, and `BackendClient` behind one
`host` and lifecycle. It constructs three separate rosbridge WebSockets, one
for each live client; it is not a connection multiplexer. `BackendClient`
opens stateless HTTP requests.

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
port if non-default. Construction can raise the connection errors of any
live child, and a later failure can leave earlier children open. Attributes
are `write: Write`, `speak: Speak`, `telemetry: Telemetry`, and
`backend: BackendClient`. `close() -> None` closes the three live clients.
Nothing here is new behavior: use the four classes
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
directly from your own point lists.

| API | Signature / result | Errors and notes |
|---|---|---|
| `Stroke` | `(points: tuple[tuple[float, float], ...])`; normalized path | `ValueError` below two points |
| `Stroke.resampled` | `(count: int) -> list[tuple[float, float]]`; arc-length-spaced points | `ValueError` below two; a zero-length stroke repeats its first point |
| `Sketch` | `(strokes: tuple[Stroke, ...])` | `ValueError` when empty |
| `Sketch.point_count` | `int`; sum of source points | read-only |
| `image_to_sketch` | `(image_path, *, threshold=128, max_size=200, min_stroke_points=4) -> Sketch` | `ImportError` without Pillow; image I/O errors propagate; `ValueError` if no ink survives |
| `DrawingSurface` | `(pose: pin.SE3, width_mm: float, height_mm: float, lift_mm=20.0)` | immutable geometry |
| `DrawingSurface.point_mm` | `(u: float, v: float, *, lift=False) -> np.ndarray`; maps normalized surface coordinates to base-frame mm | inputs are not range-clamped |
| `Trajectory` | `(motor_names, side, waypoints_deg)`; `len(trajectory)` is waypoint count | immutable |
| `Trajectory.play` | `(writer, *, rate_hz=8.0, stop=None) -> None`; sends each waypoint, sleeping `1/rate_hz`; `stop() -> bool` may cancel | zero/negative rates and writer errors propagate; ignores writer's boolean result |
| `sketch_to_trajectory` | `(sketch, surface, side=RIGHT, *, rpy_deg=None, points_per_stroke=30, initial_guess_deg=None, ik_options=None) -> Trajectory` | skips individual IK `ValueError`s; raises `ValueError` if no point is reachable; other model/numeric errors propagate |

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
from pib_sdk.features.poses import (
    play_pose_sequence,
    play_pose_sequence_timed,
    save_current_pose,
    set_pose,
)
from pib_sdk.telemetry import Telemetry

backend = BackendClient(host="localhost")
writer = Write(host="localhost")

set_pose(writer, backend, name="wave_hello")

# Stop-and-hold playback:
play_pose_sequence(writer, backend, [("wave_hello", 1.5), ("rest", 0.0)])

# Smooth playback: each number is the duration of that leg.
play_pose_sequence_timed(
    writer, backend, [("wave_hello", 1.5), ("rest", 0.5)]
)

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

`PoseSummary(pose_id: str, name: str, deletable: bool)` identifies a pose
without values. `Pose(pose_id: str, name: str, deletable: bool,
motor_angles_deg: dict[str, float])` contains angles keyed by firmware motor
name. Both are immutable dataclasses.

| Function | Signature / result | Errors and semantics |
|---|---|---|
| `list_poses` | `(backend) -> list[PoseSummary]` | backend/shape errors propagate |
| `get_pose` | `(backend, *, name=None, pose_id=None) -> Pose` | exactly one selector is required (`ValueError`); an unknown ID raises `ValueError`; REST errors propagate |
| `apply_pose` | `(writer, pose) -> bool` | sends one vector `Write.move`; its errors/result propagate |
| `set_pose` | `(writer, backend, *, name=None, pose_id=None) -> Pose` | fetches and applies; returns the pose even if `Write.move` returns `False` |
| `play_pose_sequence` | `(writer, backend, sequence: Sequence[tuple[str, float]], *, by="name") -> None` | each duration is a post-move hold in seconds; non-positive holds do not sleep; `by` must be `"name"` or `"pose_id"` |
| `play_pose_sequence_timed` | `(writer, backend, schedule: Sequence[tuple[str, float]], *, by="name") -> None` | each duration is time for that leg; empty input is a no-op; invalid `by`, empty pose motor sets, negative cumulative times, and fetch errors can raise |
| `save_current_pose` | `(telemetry, backend, name: str, motor_names: Iterable[str]) -> Pose` | reads last-commanded degrees, converts to internal units, creates and returns a pose; telemetry/REST errors propagate |

`play_pose_sequence_timed` fetches all poses, creates the first-seen union of
their motors, carries missing later motors forward, and backfills missing
earlier motors with their first known values. It sums per-leg durations into
absolute `time_from_start` and sends one `Write.send_timed_trajectory` call.
The backend's software controller uses monotone cubic Hermite interpolation
and blends through via-points. The function currently returns `None` and does
not surface a `False` result from the writer. Use positive leg durations; the
SDK validates only negative cumulative times at the final conversion layer.

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
saved_programs = list_programs(backend)
for program in saved_programs:
    print(program.program_number, program.name)

program_number = saved_programs[0].program_number
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

`ProgramSummary(program_number: str, name: str)` and
`ProgramResult(exit_code: int | None, status: int | None, timed_out: bool,
output: tuple[str, ...] = ())` are immutable dataclasses.
`list_programs(backend) -> list[ProgramSummary]` converts the REST listing;
REST and response-shape errors propagate.

`Programs(host="localhost", port=9090)` connects immediately and raises
`ConnectionError` after five seconds. Use it as a context manager or call
`close() -> None`.

| Method | Signature / result | Errors and semantics |
|---|---|---|
| `start` | `(program_number: str, timeout=5.0) -> str`; returns proxy goal ID | service and missing-key errors propagate |
| `stop` | `(proxy_goal_id: str, timeout=5.0) -> None` | service errors propagate; removes local tracking |
| `stop_all` | `(timeout=5.0) -> list[str]`; returns IDs it attempted to stop | first stop error propagates |
| `run` | `(program_number: str, *, timeout: float | None=60.0, on_output: Callable[[str, bool], None] | None=None) -> ProgramResult` | callback receives `(content, is_stderr)`; on timeout it cancels unless already terminal; service/callback errors may propagate |

`STATUS_UNKNOWN` through `STATUS_ABORTED` are integer ROS action status
constants `0..6`; `TERMINAL_STATUSES` contains succeeded, canceled, and
aborted. `emergency_stop(writer, programs=None) -> None` first calls
`programs.stop_all()` when provided, then `writer.set(All,
turned_on=False)`. It does not check the returned setting boolean.

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

`ButtonBinding(bricklet_number: int, bricklet_uid: str,
program_number: str | None)` is an immutable typed view.
`list_button_bindings(backend) -> list[ButtonBinding]` returns all bindings.
`set_button_program(backend, *, bricklet_number: int,
program_number: str | None) -> list[ButtonBinding]` updates one binding and
returns the backend's complete post-update list. Pass `None` to unassign.
REST and malformed-response errors propagate from both functions.

---

## `pib_sdk.features.camera`

Still JPEG snapshots and depth frames are available; there is no live video
streaming in this client. On-device neural networks are
[`pib_sdk.models`](#pib_sdkmodels).

```python
from pib_sdk.features.camera import Camera

camera = Camera(host="localhost")
camera.save_snapshot("frame.jpg")
raw_bytes = camera.get_snapshot_bytes()
depth_mm = camera.get_depth_frame()
distance_mm = camera.get_distance_at_px(320, 240)
camera.save_depth("depth.npy")
camera.close()
```

Camera *settings* (resolution, refresh rate, quality) are plain REST
configuration on `BackendClient.get_camera_settings()` /
`update_camera_settings()`, not part of this module.

`Camera(host="localhost", port=9090, get_image_service="get_camera_image",
get_depth_frame_service="get_depth_frame",
get_distance_at_px_service="get_distance_at_px")` connects immediately and
raises `ConnectionError` after five seconds. Use a context manager or
`close() -> None`.

| Method | Signature / return | Errors and missing data |
|---|---|---|
| `get_snapshot_bytes` | `(timeout=10.0) -> bytes`; decoded JPEG | service, base64, and missing-key errors propagate |
| `save_snapshot` | `(path: str | Path, timeout=10.0) -> None` | snapshot and filesystem errors propagate |
| `get_depth_frame` | `(timeout=10.0) -> np.ndarray | None`; copied `(height, width)` little-endian `uint16`, **millimetres**, encoding `16UC1`; pixel `0` is invalid | `ImportError` without numpy; service/missing/malformed payload returns `None` |
| `get_distance_at_px` | `(x: int, y: int, timeout=10.0) -> float`; millimetres at image pixel `(x, y)` | service/malformed/missing/out-of-range/no cache returns `0.0`, which means invalid |
| `save_depth` | `(path: str | Path, timeout=10.0) -> None`; writes numpy `.npy` preserving `uint16` millimetres | `ImportError` without numpy; `RuntimeError` if no frame; filesystem errors propagate; `np.save` may append `.npy` |

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

Typed immutable REST records:

| Dataclass | Fields |
|---|---|
| `Personality` | `personality_id: str`, `name: str`, `gender: str`, `description: str | None`, `pause_threshold: float`, `message_history: int`, `assistant_model_id: int` |
| `Chat` | `chat_id: str`, `topic: str`, `personality_id: str` |
| `ChatMessage` | `message_id: str`, `timestamp: str`, `is_user: bool`, `content: str` |
| `AssistantState` | `turned_on: bool`, `chat_id: str` |

The typed REST helpers are `list_personalities(backend) -> list[Personality]`,
`list_chats(backend) -> list[Chat]`, `get_chat(backend, chat_id: str) ->
Chat`, `list_chat_messages(backend, chat_id: str) -> list[ChatMessage]`,
`create_chat(backend, *, topic: str, personality_id: str) -> Chat`, and
`delete_chat(backend, chat_id: str) -> None`. REST and response conversion
errors propagate. Personality create/update/delete remain available on the
raw `BackendClient`.

`Assistant(host="localhost", port=9090)` connects immediately and raises
`ConnectionError` after five seconds. Use a context manager or
`close() -> None`.

| Method | Signature / return | Errors |
|---|---|---|
| `get_state` | `(timeout=5.0) -> AssistantState` | service/malformed-response errors propagate |
| `set_state` | `(*, turned_on: bool, chat_id: str, timeout=5.0) -> bool` | returns backend success; service errors propagate |
| `is_listening` | `(chat_id: str, timeout=5.0) -> bool` | missing `listening` becomes `False`; service errors propagate |
| `send_message` | `(chat_id: str, content: str, timeout=5.0) -> bool` | returns acceptance, never reply text; service errors propagate |

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

`ImageFormat` is an `IntEnum` with values `0`, `1`, and `2` respectively.
`Display(host="localhost", port=9090, display_image_topic="display_image")`
connects immediately and raises `ConnectionError` after five seconds. Use a
context manager or `close() -> None`.
`show_animated_eyes() -> None` publishes the built-in animation;
`show_custom(image_bytes: bytes, *, format: ImageFormat) -> None` base64
encodes and publishes bytes; `clear() -> None` publishes the no-image ID.
These are fire-and-forget and cannot report whether hardware displayed the
image. Invalid data/format failures arise locally or in `roslibpy`.

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

`Relay(host="localhost", port=9090,
set_state_service="set_solid_state_relay_state",
state_topic="solid_state_relay_state")` connects immediately and raises
`ConnectionError` after five seconds. Use a context manager or
`close() -> None`.

`set(turned_on: bool, timeout=5.0) -> bool` returns backend success and lets
service errors propagate. `get_state(timeout=2.0) -> bool` returns the latest
topic value or raises `TimeoutError`. `subscribe(callback: Callable[[bool],
None]) -> None` registers future state callbacks; `unsubscribe(callback) ->
None` removes one if present without error. Callback exceptions occur on the
topic callback thread.

---

## `pib_sdk.features.imu`

Caches the latest `sensor_msgs/Imu` message from `/imu`. There is no callback
API and no assumed publish period: `latest()` returns immediately, including
`None` before the first well-formed sample. Measured intervals on this
hardware are not uniform (min 0 ms, max >100 ms), so use `IMUData.age_s`
instead of a 100 ms rule.

```python
from pib_sdk.features.imu import IMU

with IMU(host="localhost") as imu:
    sample = imu.latest()
    if sample is None:
        print("no sample yet")
    else:
        print(sample.acceleration_m_s2, sample.angular_velocity_rad_s, sample.age_s)
        print(sample.orientation_available, sample.orientation)
```

> **Orientation is not available on this hardware.** The topic still carries
> a quaternion field, but `orientation_covariance[0] == -1.0` (the
> sensor_msgs convention for "do not use this orientation"). The SDK sets
> `orientation_available` to `False` and `orientation` to `None` rather than
> treating an identity quaternion as a measurement. Linear acceleration is
> metres per second squared; angular velocity is radians per second.

`Vector3(x, y, z)` and `Quaternion(x, y, z, w)` are frozen dataclasses in
those units. `IMUData` fields: `acceleration_m_s2: Vector3`,
`angular_velocity_rad_s: Vector3`, `orientation: Quaternion | None`,
`orientation_available: bool`, `orientation_covariance: list[float]`,
`timestamp_s: float` (host wall clock when the message was received, **not**
the sensor header stamp), `age_s: float` (`time.time() - timestamp_s` at the
`latest()` call).

`IMU(host="localhost", port=9090, imu_topic="/imu",
imu_message_type="sensor_msgs/Imu")` connects immediately and raises
`ConnectionError` after five seconds. It owns a background `/imu`
subscription used only to refresh the cache. Use a context manager or
`close() -> None`. Malformed messages (missing `linear_acceleration` or
`angular_velocity` `x`/`y`/`z`) are ignored and do not clear a previous
sample. Subscribe/callback exceptions from `roslibpy` stay on the topic
thread; `latest()` itself does not raise for missing data.

| Method | Signature / return | Errors and missing data |
|---|---|---|
| `latest` | `() -> IMUData | None` | returns `None` until a well-formed sample arrives; does not block or time out |
| `close` | `() -> None` | connection teardown errors are swallowed |
