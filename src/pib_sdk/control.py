"""Motor control for the pib robot over rosbridge.

:class:`Write` issues motor commands through the rosbridge services
``/apply_joint_trajectory`` and ``/apply_motor_settings``. It provides:

* named motor *groups* (``right_arm``, ``left_hand`` ...) that expand to the
  underlying motor names without any database lookup,
* batched multi-joint moves sent as a single joint-trajectory request,
* dictionary-based motor settings layered on a :data:`DEFAULT_SETTINGS`
  baseline, and
* a :meth:`Write.move` API accepting either one angle applied to every selected
  motor (*uniform mode*) or one angle per motor (*vector mode*).

Angles are given in degrees within ``[-90, 90]`` and converted to the firmware's
internal hundredths-of-a-degree units.
"""

from __future__ import annotations

import argparse
import logging
import numbers
import threading
import time
from typing import Any

import roslibpy

logger = logging.getLogger("pib.control")
if not logger.handlers:
    _handler = logging.StreamHandler()
    _handler.setFormatter(logging.Formatter("[%(levelname)s] %(asctime)s %(name)s: %(message)s"))
    logger.addHandler(_handler)
logger.setLevel(logging.INFO)


# --------------------------------------------------------------------------- #
# Group / action tokens                                                       #
# --------------------------------------------------------------------------- #
class _Token:
    """A named selector -- a motor group or an action -- usable without quotes."""

    __slots__ = ("name",)

    def __init__(self, name: str) -> None:
        self.name = name

    def __repr__(self) -> str:
        return self.name


All = _Token("All")
default = _Token("default")
open_left_hand = _Token("open_left_hand")
close_left_hand = _Token("close_left_hand")
open_right_hand = _Token("open_right_hand")
close_right_hand = _Token("close_right_hand")
right_arm = _Token("right_arm")
left_arm = _Token("left_arm")
right_hand = _Token("right_hand")
left_hand = _Token("left_hand")
head = _Token("head")

# Zero angle, handy as an explicit target: ``w.move(All, zero_position)``.
zero_position: int = 0

# Motor names grouped by body part; group tokens expand to these lists.
_MOTOR_GROUPS: dict[str, list[str]] = {
    "right_arm": [
        "shoulder_vertical_right",
        "shoulder_horizontal_right",
        "upper_arm_right_rotation",
        "elbow_right",
        "lower_arm_right_rotation",
        "wrist_right",
    ],
    "left_arm": [
        "shoulder_vertical_left",
        "shoulder_horizontal_left",
        "upper_arm_left_rotation",
        "elbow_left",
        "lower_arm_left_rotation",
        "wrist_left",
    ],
    "right_hand": [
        "index_right_stretch",
        "middle_right_stretch",
        "ring_right_stretch",
        "pinky_right_stretch",
        "thumb_right_stretch",
        "thumb_right_opposition",
    ],
    "left_hand": [
        "index_left_stretch",
        "middle_left_stretch",
        "ring_left_stretch",
        "pinky_left_stretch",
        "thumb_left_stretch",
        "thumb_left_opposition",
    ],
    "head": [
        "turn_head_motor",
        "tilt_forward_motor",
    ],
}

_HAND_ACTIONS = frozenset(
    {"open_left_hand", "close_left_hand", "open_right_hand", "close_right_hand"}
)
# Tokens that do not expand to a motor list (handled as actions or flags).
_NON_GROUP_TOKENS = _HAND_ACTIONS | {"default"}

# Token name -> shared token instance. The CLI resolves user strings through
# this map so that identity checks (e.g. against :data:`default`) keep working.
_TOKENS_BY_NAME: dict[str, _Token] = {
    token.name: token
    for token in (
        All,
        default,
        open_left_hand,
        close_left_hand,
        open_right_hand,
        close_right_hand,
        right_arm,
        left_arm,
        right_hand,
        left_hand,
        head,
    )
}

# Default motor settings, mirroring the fields of the MotorSettings message.
DEFAULT_SETTINGS: dict[str, Any] = {
    "turned_on": True,
    "visible": True,
    "invert": False,
    "velocity": 16000,
    "acceleration": 10000,
    "deceleration": 5000,
    "pulse_width_min": 700,
    "pulse_width_max": 2500,
    "period": 19500,
    "rotation_range_min": -9000,
    "rotation_range_max": 9000,
}


# --------------------------------------------------------------------------- #
# Pure helpers (no ROS connection required)                                   #
# --------------------------------------------------------------------------- #
def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


def _all_motor_names() -> list[str]:
    names: set = set()
    for group in _MOTOR_GROUPS.values():
        names.update(group)
    return sorted(names)


def _expand_token(token: _Token) -> list[str]:
    """Expand a group token to its motor names (action/flag tokens expand to none)."""
    if token.name == "All":
        return _all_motor_names()
    if token.name in _MOTOR_GROUPS:
        return list(_MOTOR_GROUPS[token.name])
    if token.name in _NON_GROUP_TOKENS:
        return []
    raise ValueError(f"Unknown token: {token.name}")


def _expand_motor_specs(specs: list[str | _Token]) -> list[str]:
    """Flatten a mix of motor-name strings and tokens into a list of motor names."""
    names: list[str] = []
    for spec in specs:
        if isinstance(spec, _Token):
            names.extend(_expand_token(spec))
        elif isinstance(spec, str):
            names.append(spec)
        else:
            raise TypeError(f"Unsupported motor spec: {type(spec)}")
    return names


def _degrees_to_internal_units(position_deg: float) -> float:
    """Convert degrees in ``[-90, 90]`` to internal hundredths of a degree."""
    if not -90.0 <= float(position_deg) <= 90.0:
        raise ValueError(f"position_deg must be between -90 and 90 (got {position_deg})")
    return float(round(float(position_deg) * 100.0))


def _parse_vector_move(
    args: list[str | _Token | float],
) -> tuple[list[str], list[float]] | None:
    """Parse *vector-mode* move arguments: interleaved selectors and angles.

    A motor-name string consumes exactly one following angle; a group token
    consumes exactly one angle per motor in the group. Returns the expanded
    ``(motor_names, degrees)`` when every argument is consumed this way, or
    ``None`` when the arguments are not a valid vector-mode sequence (the caller
    then falls back to uniform mode). A genuinely unsupported argument type in a
    selector position raises ``TypeError``.
    """
    motor_names: list[str] = []
    degrees: list[float] = []
    index = 0
    while index < len(args):
        selector = args[index]
        if isinstance(selector, str):
            angle = args[index + 1] if index + 1 < len(args) else None
            if not isinstance(angle, numbers.Real):
                return None
            motor_names.append(selector)
            degrees.append(float(angle))
            index += 2
        elif isinstance(selector, _Token):
            group = _expand_token(selector)
            angles = args[index + 1 : index + 1 + len(group)]
            if len(angles) < len(group) or not all(
                isinstance(angle, numbers.Real) for angle in angles
            ):
                return None
            motor_names.extend(group)
            degrees.extend(float(angle) for angle in angles)
            index += 1 + len(group)
        elif isinstance(selector, numbers.Real):
            return None
        else:
            raise TypeError(f"Unsupported arg in move(): {type(selector)}")

    if motor_names and degrees and index == len(args):
        return motor_names, degrees
    return None


def _parse_uniform_move(
    args: list[str | _Token | float],
) -> tuple[list[str], float]:
    """Parse *uniform-mode* move arguments: selectors followed by one angle."""
    *specs, last = args
    if not isinstance(last, numbers.Real):
        raise TypeError("Uniform mode requires a single trailing degree number.")
    motor_names = _expand_motor_specs(specs)
    if not motor_names:
        raise ValueError("No motors specified for move()")
    return motor_names, float(last)


# --------------------------------------------------------------------------- #
# Write: service-based motor control                                          #
# --------------------------------------------------------------------------- #
class Write:
    """Service-based motor control over rosbridge.

    Example
    -------
    >>> w = Write(host="localhost", port=9090, debug=False)
    >>> w.set(All, default)               # apply DEFAULT_SETTINGS to every motor
    >>> w.set(left_hand, velocity=6000)   # override settings for a group
    >>> w.move(right_arm, -30.0)          # uniform: one angle for the whole group
    >>> w.move(left_arm, a, b, c, d, e, f)    # vector: one angle per joint
    >>> w.move(All, zero_position)        # everything to zero
    """

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        joint_trajectory_service: str = "/apply_joint_trajectory",
        motor_settings_service: str = "/apply_motor_settings",
        joint_trajectory_topic: str = "/joint_trajectory",
        motor_settings_topic: str = "/motor_settings",
        debug: bool = False,
    ) -> None:
        if debug:
            logger.setLevel(logging.DEBUG)

        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros)

        self._joint_trajectory_service = roslibpy.Service(
            self.ros, joint_trajectory_service, "datatypes/ApplyJointTrajectory"
        )
        self._motor_settings_service = roslibpy.Service(
            self.ros, motor_settings_service, "datatypes/ApplyMotorSettings"
        )

        self._joint_trajectory_topic = roslibpy.Topic(
            self.ros, joint_trajectory_topic, "trajectory_msgs/JointTrajectory"
        )
        self._motor_settings_topic = roslibpy.Topic(
            self.ros, motor_settings_topic, "datatypes/MotorSettings"
        )

        # Optional, best-effort verification that commands are echoed back.
        self.verify_echo: bool = False
        self._echo_lock = threading.Lock()
        self._last_trajectory_echo: dict[str, Any] | None = None
        self._joint_trajectory_topic.subscribe(self._remember_trajectory_echo)

    def __enter__(self) -> Write:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        """Close the underlying rosbridge connection."""
        try:
            if self.ros and self.ros.is_connected:
                self.ros.terminate()
        except Exception:
            pass

    def _remember_trajectory_echo(self, message: dict[str, Any]) -> None:
        with self._echo_lock:
            self._last_trajectory_echo = message

    # -- settings ----------------------------------------------------------- #
    def set(
        self,
        *motor_specs: str | _Token,
        verify_echo: bool = False,
        echo_timeout: float = 1.0,
        **settings: Any,
    ) -> bool:
        """Apply MotorSettings to one or more motors.

        Selectors may be motor-name strings or group tokens. Supplying the
        ``default`` token positionally, or ``default=True``, merges
        :data:`DEFAULT_SETTINGS` underneath any explicit keyword settings.

        Example
        -------
        >>> w.set("wrist_right", velocity=6000)
        >>> w.set(All, default)
        >>> w.set(left_hand, velocity=6000, acceleration=8000)
        """
        specs = list(motor_specs)
        use_default = bool(settings.pop("default", False))
        if specs and specs[-1] is default:
            specs.pop()
            use_default = True
        if use_default:
            settings = {**DEFAULT_SETTINGS, **settings}

        motor_names = _expand_motor_specs(specs or [All])
        results: list[bool] = []
        for name in motor_names:
            motor_settings: dict[str, Any] = {"motor_name": name}
            for key, value in settings.items():
                if value is not None and key != "position":
                    motor_settings[key] = value

            request = roslibpy.ServiceRequest({"motor_settings": motor_settings})
            try:
                response = self._motor_settings_service.call(request, timeout=5.0)
            except Exception as error:
                logger.error("ApplyMotorSettings failed for %s: %s", name, error)
                results.append(False)
                continue

            applied = bool(response.get("settings_applied", False))
            persisted = bool(response.get("settings_persisted", False))
            ok = applied or persisted
            logger.debug("set(%s) -> %s | response=%s", name, ok, response)

            if not ok and verify_echo:
                ok = self._confirm_settings_echo(name, motor_settings, echo_timeout)
            results.append(ok)
        return all(results)

    def _confirm_settings_echo(
        self, name: str, motor_settings: dict[str, Any], timeout: float
    ) -> bool:
        """Best-effort wait for a MotorSettings echo confirming the command."""
        confirmed = threading.Event()

        def _on_echo(message: dict[str, Any]) -> None:
            if message.get("motor_name") != name:
                return
            for key, value in motor_settings.items():
                if key != "motor_name" and message.get(key) == value:
                    confirmed.set()
                    return

        self._motor_settings_topic.subscribe(_on_echo)
        try:
            confirmed.wait(timeout)
        finally:
            try:
                self._motor_settings_topic.unsubscribe(_on_echo)
            except Exception:
                pass
        if not confirmed.is_set():
            logger.warning(
                "set(%s): service returned False and no telemetry confirmation within %.2fs",
                name,
                timeout,
            )
        return confirmed.is_set()

    # -- movement ----------------------------------------------------------- #
    def move(self, *args: str | _Token | int | float) -> bool:
        """Move one or more motors, in uniform or vector mode.

        * Uniform: ``move(selectors..., angle)`` applies one angle to every
          selected motor.
        * Vector: ``move(name, a, token, b, c, ...)`` takes one angle per motor,
          consuming one angle per name and one angle per motor of a group token.

        A lone hand-action token (``open_left_hand`` ...) performs that action.
        Angles are in degrees within ``[-90, 90]``.
        """
        if not args:
            raise ValueError("move() requires arguments")

        if len(args) == 1 and isinstance(args[0], _Token) and args[0].name in _HAND_ACTIONS:
            return self._move_hand(args[0])

        parsed = _parse_vector_move(list(args))
        if parsed is not None:
            joint_names, degrees = parsed
        else:
            joint_names, uniform_deg = _parse_uniform_move(list(args))
            degrees = [uniform_deg] * len(joint_names)

        positions_internal = [_degrees_to_internal_units(angle) for angle in degrees]
        ok = self._send_joint_trajectory(joint_names, positions_internal)
        if ok and self.verify_echo:
            ok = self._await_trajectory_echo(joint_names, positions_internal)
        return ok

    def _move_hand(self, action: _Token) -> bool:
        """Open or close a hand (fingers to -90 to open, +90 to close)."""
        hand_targets = {
            "open_left_hand": (left_hand, -90.0),
            "close_left_hand": (left_hand, 90.0),
            "open_right_hand": (right_hand, -90.0),
            "close_right_hand": (right_hand, 90.0),
        }
        if action.name not in hand_targets:
            raise ValueError(f"Unknown hand action {action}")
        hand, angle = hand_targets[action.name]
        return self.move(hand, angle)

    @staticmethod
    def _trajectory_point(position_internal: float) -> dict[str, Any]:
        # Exactly one value per point; the node reads positions[0].
        return {
            "positions": [float(position_internal)],
            "velocities": [],
            "accelerations": [],
            "effort": [],
            "time_from_start": {"sec": 0, "nanosec": 1_000_000},  # 1 ms
        }

    def _send_joint_trajectory(
        self, joint_names: list[str], positions_internal: list[float]
    ) -> bool:
        """Send one ApplyJointTrajectory request for all joints, with per-joint fallback."""
        assert len(joint_names) == len(positions_internal), "names/positions length mismatch"

        request = roslibpy.ServiceRequest(
            {
                "joint_trajectory": {
                    "joint_names": list(joint_names),
                    "points": [self._trajectory_point(p) for p in positions_internal],
                }
            }
        )
        try:
            response = self._joint_trajectory_service.call(request, timeout=2.5)
            ok = bool(response.get("successful", False))
            logger.debug("move(%s -> %s) -> %s", joint_names, positions_internal, ok)
            if ok or len(joint_names) == 1:
                return ok

            # Batched request rejected: retry once, one joint at a time.
            logger.warning("Batched joint trajectory rejected; falling back to per-joint sends.")
            all_ok = True
            for name, position in zip(joint_names, positions_internal):
                single_request = roslibpy.ServiceRequest(
                    {
                        "joint_trajectory": {
                            "joint_names": [name],
                            "points": [self._trajectory_point(position)],
                        }
                    }
                )
                single_response = self._joint_trajectory_service.call(single_request, timeout=2.0)
                all_ok &= bool(single_response.get("successful", False))
            return all_ok
        except Exception as error:
            logger.error("Batched move call failed: %s", error)
            return False

    def _await_trajectory_echo(
        self, names: list[str], positions_internal: list[float], timeout: float = 0.15
    ) -> bool:
        """Best-effort wait for a joint-trajectory echo matching the command."""
        deadline = time.time() + timeout
        expected_positions = [float(p) for p in positions_internal]
        while time.time() < deadline:
            with self._echo_lock:
                message = self._last_trajectory_echo
            if message:
                trajectory = message.get("joint_trajectory", {})
                points = trajectory.get("points", [])
                if trajectory.get("joint_names", []) == names and points:
                    # Accept either N single-value points or one N-value point.
                    if len(points) == len(expected_positions):
                        values = [float(point.get("positions", [None])[0]) for point in points]
                        if values == expected_positions:
                            return True
                    elif len(points) == 1:
                        values = [float(v) for v in points[0].get("positions", [])]
                        if values == expected_positions:
                            return True
            time.sleep(0.005)
        logger.debug("joint-trajectory echo verification timed out")
        return True  # non-fatal


# --------------------------------------------------------------------------- #
# Command-line interface                                                       #
# --------------------------------------------------------------------------- #
_SET_TOKEN_NAMES = frozenset({"All", "default"}) | frozenset(_MOTOR_GROUPS)
_MOVE_TOKEN_NAMES = frozenset({"All"}) | frozenset(_MOTOR_GROUPS) | _HAND_ACTIONS


def _coerce_number_or_name(value: str) -> int | float | str:
    try:
        return float(value) if "." in value else int(value)
    except ValueError:
        return value


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser("pib control")
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--debug", action="store_true")

    subcommands = parser.add_subparsers(dest="cmd")

    set_parser = subcommands.add_parser("set", help="Apply MotorSettings")
    set_parser.add_argument(
        "names",
        nargs="+",
        help=(
            "motor names or tokens (All, right_arm, left_arm, right_hand, left_hand, head, default)"
        ),
    )
    set_parser.add_argument("--verify-echo", action="store_true")
    set_parser.add_argument("--echo-timeout", type=float, default=1.0)
    set_parser.add_argument("--turned-on", type=str, choices=["true", "false"])
    set_parser.add_argument("--visible", type=str, choices=["true", "false"])
    set_parser.add_argument("--invert", type=str, choices=["true", "false"])
    set_parser.add_argument("--velocity", type=int)
    set_parser.add_argument("--acceleration", type=int)
    set_parser.add_argument("--deceleration", type=int)
    set_parser.add_argument("--period", type=int)
    set_parser.add_argument("--pulse-width-min", type=int)
    set_parser.add_argument("--pulse-width-max", type=int)
    set_parser.add_argument("--min-deg", type=float)
    set_parser.add_argument("--max-deg", type=float)
    set_parser.add_argument("--use-default", action="store_true")

    move_parser = subcommands.add_parser("move", help="Move motors")
    move_parser.add_argument(
        "names", nargs="+", help="names/tokens and degrees (vector or uniform modes)"
    )
    move_parser.add_argument("--verify-echo", action="store_true")
    return parser


def _run_set(writer: Write, args: argparse.Namespace) -> bool:
    settings: dict[str, Any] = {}

    # Tri-state string flags ("true"/"false"/unset) become real booleans.
    tri_state_flags = {
        "turned_on": args.turned_on,
        "visible": args.visible,
        "invert": args.invert,
    }
    for key, raw_value in tri_state_flags.items():
        if raw_value is not None:
            settings[key] = raw_value == "true"

    # Numeric settings are forwarded when provided (--min-deg / --max-deg map
    # onto the rotation-range fields).
    numeric_settings = {
        "velocity": args.velocity,
        "acceleration": args.acceleration,
        "deceleration": args.deceleration,
        "period": args.period,
        "pulse_width_min": args.pulse_width_min,
        "pulse_width_max": args.pulse_width_max,
        "rotation_range_min": args.min_deg,
        "rotation_range_max": args.max_deg,
    }
    for key, value in numeric_settings.items():
        if value is not None:
            settings[key] = value

    if args.use_default:
        settings["default"] = True

    specs: list[str | _Token] = [
        _TOKENS_BY_NAME[name] if name in _SET_TOKEN_NAMES else name for name in args.names
    ]
    return writer.set(
        *specs, verify_echo=args.verify_echo, echo_timeout=args.echo_timeout, **settings
    )


def _run_move(writer: Write, args: argparse.Namespace) -> bool:
    if args.verify_echo:
        writer.verify_echo = True
    parsed: list[str | _Token | int | float] = [
        _TOKENS_BY_NAME[name] if name in _MOVE_TOKEN_NAMES else _coerce_number_or_name(name)
        for name in args.names
    ]
    return writer.move(*parsed)


def _cli() -> None:
    parser = _build_argument_parser()
    args = parser.parse_args()

    if args.cmd not in {"set", "move"}:
        parser.print_help()
        return

    writer = Write(host=args.host, port=args.port, debug=args.debug)
    ok = _run_set(writer, args) if args.cmd == "set" else _run_move(writer, args)
    print("OK" if ok else "FAILED")


if __name__ == "__main__":
    _cli()
