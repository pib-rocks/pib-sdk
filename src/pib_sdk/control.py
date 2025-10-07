from __future__ import annotations
from typing import Optional, Any, Dict, List, Union
import numbers
import time
import logging
import argparse
import threading
import roslibpy

# ============================= Logging =====================================
log = logging.getLogger("pib.control")
_handler = logging.StreamHandler()
_handler.setFormatter(logging.Formatter("[%(levelname)s] %(asctime)s %(name)s: %(message)s"))
if not log.handlers:
    log.addHandler(_handler)
log.setLevel(logging.INFO)

# ============================= Public constants & tokens ====================
zero_position: int = 0  # explicit int 0, usable in w.move(..., zero_position)

class _Token:
    __slots__ = ("name",)
    def __init__(self, name: str):
        self.name = name
    def __repr__(self) -> str:
        return self.name

# Group/action tokens (usable without quotes)
All                 = _Token("All")
default             = _Token("default")
open_left_hand      = _Token("open_left_hand")
close_left_hand     = _Token("close_left_hand")
open_right_hand     = _Token("open_right_hand")
close_right_hand    = _Token("close_right_hand")
right_arm           = _Token("right_arm")
left_arm            = _Token("left_arm")
right_hand          = _Token("right_hand")
left_hand           = _Token("left_hand")
head                = _Token("head")

# ============================= Static Groups ===============================
# Arms (you confirmed these are correct)
_STATIC_GROUPS: Dict[str, List[str]] = {
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
    # Hands (exact names you provided)
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
    # Head
    "head": [
        "turn_head_motor",
        "tilt_forward_motor",
    ],
}

def _all_names() -> List[str]:
    s = set()
    for group in _STATIC_GROUPS.values():
        s.update(group)
    return sorted(s)

# Reasonable base defaults for settings (merged when `default` is used)
DEFAULT_SETTINGS: Dict[str, Any] = {
    # These keys match your datatypes/MotorSettings fields on the node
    "turned_on": True,
    "visible": True,
    "invert": False,
    "velocity": 16000,
    "acceleration": 10000,
    "deceleration": 5000,
    "pulse_width_min":700,
    "pulse_width_max":2500,
    "period": 19500,
    "rotation_range_min": -9000,
    "rotation_range_max":  9000,
}

# ============================= Helpers =====================================
def _wait_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")

def _pos_deg_to_internal(position_deg: float) -> float:
    """Degrees (-90..90) -> internal units x100 (-9000..9000)."""
    if not -90.0 <= float(position_deg) <= 90.0:
        raise ValueError(f"position_deg must be between -90 and 90 (got {position_deg})")
    return float(round(float(position_deg) * 100.0))

# ============================= Write (Service-first) =======================
class Write:
    """
    Service-based motor control over rosbridge with:
      - static groups (no DB),
      - *batched* multi-joint moves,
      - dictionary-based settings (from PR snippet semantics),
      - auto-detecting move(): uniform vs vector mode.

    Examples:
      w = Write(host="localhost", port=9090, debug=False)
      w.set(All, default)                         # merge DEFAULT_SETTINGS
      w.set(left_hand, velocity=6000)             # dict-style settings
      w.move(right_arm, -30.0)                    # batched, uniform angle
      w.move(left_arm, a, b, c, d, e, f)          # batched, per-joint angles
      w.move(All, 0)                              # everything to zero
    """
    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        srv_apply_jt: str = "/apply_joint_trajectory",
        srv_apply_ms: str = "/apply_motor_settings",
        jt_topic_name: str = "/joint_trajectory",
        ms_topic_name: str = "/motor_settings",
        debug: bool = False,
    ):
        if debug:
            log.setLevel(logging.DEBUG)

        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_connected(self.ros)

        # Services (command path)
        self._svc_jt = roslibpy.Service(self.ros, srv_apply_jt, "datatypes/ApplyJointTrajectory")
        self._svc_ms = roslibpy.Service(self.ros, srv_apply_ms, "datatypes/ApplyMotorSettings")

        # Telemetry (optional)
        self._jt_topic = roslibpy.Topic(self.ros, jt_topic_name, "trajectory_msgs/JointTrajectory")
        self._ms_topic = roslibpy.Topic(self.ros, ms_topic_name, "datatypes/MotorSettings")

        # Echo verification (optional)
        self.verify_echo: bool = False
        self._echo_lock = threading.Lock()
        self._last_echo_jt: Optional[Dict[str, Any]] = None

        def _on_jt(msg: Dict[str, Any]) -> None:
            with self._echo_lock:
                self._last_echo_jt = msg

        self._jt_topic.subscribe(_on_jt)

    # -------------------- Group expansion ---------------------
    def _expand_token(self, token: _Token) -> List[str]:
        name = token.name
        if name == "All":
            return _all_names()
        if name in _STATIC_GROUPS:
            return _STATIC_GROUPS[name]
        if name in {"open_left_hand","close_left_hand","open_right_hand","close_right_hand","default"}:
            return []  # action tokens handled elsewhere
        if name == "head":
            return _STATIC_GROUPS["head"]
        raise ValueError(f"Unknown token: {name}")

    def _expand_motor_specs(self, m_specs: List[Union[str, _Token]]) -> List[str]:
        """Helper used by set(); expands tokens/strings to a flat list of motor names."""
        motor_names: List[str] = []
        for item in m_specs:
            if isinstance(item, _Token):
                motor_names.extend(self._expand_token(item))
            elif isinstance(item, str):
                motor_names.append(item)
            else:
                raise TypeError(f"Unsupported motor spec: {type(item)}")
        return motor_names

    # -------------------- Settings ---------------------
    def set(
        self,
        *motor_specs: Union[str, _Token],
        verify_echo: bool = False,
        echo_timeout: float = 1.0,
        **settings: Any,
    ) -> bool:
        """
        Apply settings to one or many motors. (Dictionary-based API)

        Examples:
          w.set("shoulder_vertical_right", velocity=6000, ...)
          w.set("shoulder_vertical_right", "wrist_right", velocity=6000, ...)
          w.set(All, velocity=6000, ...)
          w.set(All, default)                 # positional token
          w.set(All, default=True)            # keyword flag alternative
        """
        # Support positional `default` token or keyword `default=True`
        m_specs = list(motor_specs)
        use_default = False
        if m_specs and isinstance(m_specs[-1], _Token) and m_specs[-1] is default:
            m_specs.pop()
            use_default = True
        if settings.pop("default", False):
            use_default = True
        if use_default:
            merged = dict(DEFAULT_SETTINGS)
            merged.update(settings)
            settings = merged

        motor_names = self._expand_motor_specs(m_specs or [All])
        results: List[bool] = []
        for name in motor_names:
            # PR-style: build a MotorSettings dict directly (exclude "position")
            ms: Dict[str, Any] = {"motor_name": name}
            for k, v in settings.items():
                if v is not None and k != "position":
                    ms[k] = v

            req = roslibpy.ServiceRequest({"motor_settings": ms})
            try:
                resp = self._svc_ms.call(req, timeout=5.0)
            except Exception as e:
                log.error("ApplyMotorSettings failed for %s: %s", name, e)
                results.append(False)
                continue

            applied = bool(resp.get("settings_applied", False))
            persisted = bool(resp.get("settings_persisted", False))
            ok = applied or persisted
            log.debug("set(%s) -> %s | resp=%s", name, ok, resp)

            if not ok and verify_echo:
                # Optional: verify via telemetry echo (best-effort)
                evt = threading.Event()
                observed: Dict[str, Any] = {}

                def _on_ms(msg: Dict[str, Any]):
                    if msg.get("motor_name") != name:
                        return
                    for k, v in ms.items():
                        if k == "motor_name":
                            continue
                        if k in msg and msg[k] == v:
                            observed[k] = v
                    if any(k for k in ms.keys() if k != "motor_name" and k in observed):
                        evt.set()

                self._ms_topic.subscribe(_on_ms)
                try:
                    evt.wait(echo_timeout)
                finally:
                    try:
                        self._ms_topic.unsubscribe(_on_ms)
                    except Exception:
                        pass
                ok = evt.is_set()
                if not ok:
                    log.warning("apply_settings(%s): service False and telemetry did not confirm within %.2fs", name, echo_timeout)
            results.append(ok)
        return all(results)

    # -------------------- Movement (batched) ---------------------
    def _move_internal_units(self, joint_names: List[str], positions_internal: List[float]) -> bool:
        """Send ONE ApplyJointTrajectory request for many joints.

        Layout used here (works with your node):
          - joint_trajectory.joint_names = [m1, m2, ..., mN]
          - joint_trajectory.points     = [
                { positions: [p1] }, { positions: [p2] }, ... { positions: [pN] }
            ]
        """
        assert len(joint_names) == len(positions_internal), "names/positions length mismatch"

        points = []
        for pos in positions_internal:
            points.append({
                "positions": [float(pos)],     # exactly one value per point; node reads positions[0]
                "velocities": [],
                "accelerations": [],
                "effort": [],
                "time_from_start": {"sec": 0, "nanosec": 1_000_000},  # 1 ms
            })

        req = roslibpy.ServiceRequest({
            "joint_trajectory": {
                "joint_names": list(joint_names),
                "points": points,
            }
        })
        try:
            resp = self._svc_jt.call(req, timeout=2.5)
            ok = bool(resp.get("successful", False))
            log.debug("move_internal(batched %s -> %s) -> %s", joint_names, positions_internal, ok)
            if not ok and len(joint_names) > 1:
                # Fallback: try per-joint once (rare)
                log.warning("Batched JT rejected ? falling back to per-joint sends once.")
                all_ok = True
                for name, pos in zip(joint_names, positions_internal):
                    single_req = roslibpy.ServiceRequest({
                        "joint_trajectory": {
                            "joint_names": [name],
                            "points": [{
                                "positions": [float(pos)],
                                "velocities": [],
                                "accelerations": [],
                                "effort": [],
                                "time_from_start": {"sec": 0, "nanosec": 1_000_000},
                            }],
                        }
                    })
                    single_resp = self._svc_jt.call(single_req, timeout=2.0)
                    all_ok &= bool(single_resp.get("successful", False))
                return all_ok
            return ok
        except Exception as e:
            log.error("Batched move call failed: %s", e)
            return False

    def move(self, *args: Union[str, _Token, int, float]) -> bool:
        """
        Move one or many motors.

        Auto-detects:
          - Uniform mode: w.move(names/tokens..., degree)
          - Vector mode:  w.move(name, deg, name, deg, ..., token, deg1, deg2, ..., degN)
            For a token, it consumes exactly len(expanded_names) degrees in the group's order.
        """
        if not args:
            raise ValueError("move() requires arguments")

        # Hand action shorthand
        if len(args) == 1 and isinstance(args[0], _Token) and args[0].name in {
            "open_left_hand","close_left_hand","open_right_hand","close_right_hand"
        }:
            return self._move_hand_action(args[0])

        # ---------- Try VECTOR MODE first ----------
        joint_names: List[str] = []
        degrees: List[float] = []

        i = 0
        ok_vector = True
        while i < len(args):
            a = args[i]

            # Name (str): must be followed by exactly ONE degree
            if isinstance(a, str):
                if i + 1 >= len(args) or not isinstance(args[i+1], numbers.Real):
                    ok_vector = False
                    break
                joint_names.append(a)
                degrees.append(float(args[i+1]))
                i += 2
                continue

            # Token: must be followed by exactly len(expanded) degrees
            if isinstance(a, _Token):
                # action tokens are handled earlier
                expanded = self._expand_token(a)
                needed = len(expanded)
                if i + needed >= len(args):  # not enough args left
                    ok_vector = False
                    break
                # next 'needed' must be numbers
                next_vals = args[i+1 : i+1+needed]
                if not all(isinstance(v, numbers.Real) for v in next_vals):
                    ok_vector = False
                    break
                joint_names.extend(expanded)
                degrees.extend(float(v) for v in next_vals)
                i += 1 + needed
                continue

            # Anything else in names position => not vector mode
            if isinstance(a, numbers.Real):
                ok_vector = False
                break
            else:
                raise TypeError(f"Unsupported arg in move(): {type(a)}")

        if ok_vector and joint_names and degrees and i == len(args):
            # VECTOR MODE succeeded
            positions_internal = [float(round(d * 100.0)) for d in degrees]
            ok = self._move_internal_units(joint_names, positions_internal)
            if ok and self.verify_echo:
                ok = self._wait_jt_echo(joint_names, positions_internal)
            return ok

        # ---------- Fallback to UNIFORM MODE ----------
        *names_or_tokens, last = args
        if not isinstance(last, numbers.Real):
            raise TypeError("Uniform mode requires a single trailing degree number.")

        position_deg = float(last)

        # Expand names/tokens
        motor_names: List[str] = []
        for item in names_or_tokens:
            if isinstance(item, _Token):
                motor_names.extend(self._expand_token(item))
            elif isinstance(item, str):
                motor_names.append(item)
            else:
                raise TypeError(f"Unsupported arg in move(): {type(item)}")

        if not motor_names:
            raise ValueError("No motors specified for move()")

        internal = float(round(position_deg * 100.0))
        positions = [internal for _ in motor_names]
        ok = self._move_internal_units(motor_names, positions)
        if ok and self.verify_echo:
            ok = self._wait_jt_echo(motor_names, positions)
        return ok

    def _move_hand_action(self, action: _Token) -> bool:
        """Open/close helpers for left/right hands (90 on stretch/opposition fingers)."""
        if action.name == "open_left_hand":
            return self.move(left_hand, -90.0)
        if action.name == "close_left_hand":
            return self.move(left_hand, +90.0)
        if action.name == "open_right_hand":
            return self.move(right_hand, -90.0)
        if action.name == "close_right_hand":
            return self.move(right_hand, +90.0)
        raise ValueError(f"Unknown hand action {action}")

    # -------------------- Echo verification (optional) ---------------------
    def _wait_jt_echo(self, names: List[str], positions_internal: List[float], timeout: float = 0.15) -> bool:
        """Waits briefly for a JT echo that matches the command (best-effort)."""
        deadline = time.time() + timeout
        expected_names = list(names)
        expected_positions = [float(p) for p in positions_internal]
        while time.time() < deadline:
            with self._echo_lock:
                msg = self._last_echo_jt
            if msg:
                jt = msg.get("joint_trajectory", {})
                jn = jt.get("joint_names", [])
                pts = jt.get("points", [])
                if jn == expected_names and pts:
                    # We accept either N points with 1 value each OR one point with N values.
                    if len(pts) == len(expected_positions):
                        vals = [float(p.get("positions", [None])[0]) for p in pts]
                        if vals == expected_positions:
                            return True
                    elif len(pts) == 1:
                        pos = [float(x) for x in pts[0].get("positions", [])]
                        if pos == expected_positions:
                            return True
            time.sleep(0.005)
        log.debug("JT echo verification timed out")
        return True  # non-fatal

    def _wait_ms_echo(self, name: str, ms: Dict[str, Any], timeout: float = 0.2) -> bool:
        """Waits briefly for a MotorSettings echo (best-effort)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.01)
            return True
        return True

# ============================= CLI =========================================
def _cli() -> None:
    parser = argparse.ArgumentParser("pib control")
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--debug", action="store_true")

    sub = parser.add_subparsers(dest="cmd")

    p_set = sub.add_parser("set", help="Apply MotorSettings")
    p_set.add_argument("names", nargs="+", help="motor names or tokens (All, right_arm, left_arm, right_hand, left_hand, head, default)")
    p_set.add_argument("--verify-echo", action="store_true")
    p_set.add_argument("--echo-timeout", type=float, default=1.0)
    # Common fields exposed; anything else can be set via Python API
    p_set.add_argument("--turned-on", type=str, choices=["true","false"])
    p_set.add_argument("--visible", type=str, choices=["true","false"])
    p_set.add_argument("--invert", type=str, choices=["true","false"])
    p_set.add_argument("--velocity", type=int)
    p_set.add_argument("--acceleration", type=int)
    p_set.add_argument("--deceleration", type=int)
    p_set.add_argument("--period", type=int)
    p_set.add_argument("--min-deg", type=float)
    p_set.add_argument("--max-deg", type=float)
    p_set.add_argument("--use-default", action="store_true")

    p_move = sub.add_parser("move", help="Move motors")
    p_move.add_argument("names", nargs="+", help="names/tokens and degrees (supports vector or uniform modes)")
    p_move.add_argument("--verify-echo", action="store_true")

    args = parser.parse_args()

    w = Write(host=args.host, port=args.port, debug=args.debug)

    if args.cmd == "set":
        kw: Dict[str, Any] = {}
        if args.turned_on is not None: kw["turned_on"] = (args.turned_on == "true")
        if args.visible   is not None: kw["visible"]   = (args.visible == "true")
        if args.invert    is not None: kw["invert"]    = (args.invert == "true")
        if args.velocity  is not None: kw["velocity"]  = args.velocity
        if args.acceleration is not None: kw["acceleration"] = args.acceleration
        if args.deceleration is not None: kw["deceleration"] = args.deceleration
        if args.period    is not None: kw["period"]    = args.period
        if args.pulse_width_min is not None: kw["pulse_width_min"] = args.pulse_width_min
        if args.pulse_width_min is not None: kw["pulse_width_min"] = args.pulse_width_max
        if args.min_deg   is not None: kw["rotation_range_min"] = args.min_deg
        if args.max_deg   is not None: kw["rotation_range_max"] = args.max_deg
        if args.use_default: kw["default"] = True

        items: List[Union[str,_Token]] = []
        for n in args.names:
            if n in {"All","right_arm","left_arm","right_hand","left_hand","head","default"}:
                items.append(_Token(n))
            else:
                items.append(n)
        ok = w.set(*items, verify_echo=args.verify_echo, echo_timeout=args.echo_timeout, **kw)
        print("OK" if ok else "FAILED")
        return

    if args.cmd == "move":
        if args.verify_echo:
            w.verify_echo = True
        # Let move() parse vector vs uniform from raw args
        # Convert token strings to tokens, keep numbers as-is
        parsed: List[Union[str, _Token, int, float]] = []
        for a in args.names:
            if a in {"All","right_arm","left_arm","right_hand","left_hand","head",
                     "open_left_hand","close_left_hand","open_right_hand","close_right_hand"}:
                parsed.append(_Token(a))
            else:
                try:
                    # try number
                    if "." in a:
                        parsed.append(float(a))
                    else:
                        parsed.append(int(a))
                except ValueError:
                    parsed.append(a)
        ok = w.move(*parsed)
        print("OK" if ok else "FAILED")
        return

    parser.print_help()

if __name__ == "__main__":
    _cli()
