from __future__ import annotations
from typing import Optional, Any, Dict, Callable, List, Iterable, Union, Set
import time
import json
import logging
import argparse
import threading
import os
import sqlite3
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
open_hand_left      = _Token("open_hand_left")
close_hand_left     = _Token("close_hand_left")
open_hand_right     = _Token("open_hand_right")
close_hand_right    = _Token("close_hand_right")
resting_position    = _Token("resting_position")
# Accept common misspellings
resting_postion     = resting_position
resting_postiion    = resting_position

# Arm grouping tokens
right_arm           = _Token("right_arm")
arm_right           = right_arm  # alias
left_arm            = _Token("left_arm")
arm_left            = left_arm   # alias

# ============================= Helpers =====================================
def _wait_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")

def _deg_to_internal(position_deg: float) -> float:
    """Map degrees (-90..90) -> internal units x100 (-9000..9000)."""
    if not -90.0 <= float(position_deg) <= 90.0:
        raise ValueError(f"position_deg must be between -90 and 90 (got {position_deg})")
    return float(round(float(position_deg) * 100.0))

def _jt_point(positions_internal: List[float]) -> Dict[str, Any]:
    """Build a single JointTrajectoryPoint for N joints with 1 ms duration."""
    return {
        "positions": [float(p) for p in positions_internal],
        "velocities": [],
        "accelerations": [],
        "effort": [],
        "time_from_start": {"sec": 0, "nanosec": 1_000_000},  # 1 ms
    }

# ============================= Write (Service-first) =======================
class Write:
    """
    Service-based motor control over rosbridge, with multi-motor support and *dynamic* motor discovery.

    Discovery sources (in order):
      1) SQLite DB table `motor` (column `name`) if available.
         - Default path: /home/pib/app/pib-backend/pib_api/flask/pibdata.db
         - Override with env PIB_MOTOR_DB or constructor arg `db_path`.
      2) Live telemetry from /motor_settings messages (names seen at runtime).

    Services:
      - /apply_motor_settings  (datatypes/ApplyMotorSettings)
      - /apply_joint_trajectory (datatypes/ApplyJointTrajectory)

    Telemetry topics (optional):
      - /joint_trajectory  (trajectory_msgs/JointTrajectory)
      - /motor_settings    (datatypes/MotorSettings)
    """

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        *,
        srv_apply_jt: str = "/apply_joint_trajectory",
        srv_apply_ms: str = "/apply_motor_settings",
        jt_topic_name: str = "/joint_trajectory",
        ms_topic_name: str = "/motor_settings",
        db_path: Optional[str] = None,
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

        # Dynamic discovery caches
        self._motors_from_db: List[str] = []
        self._motors_seen: Set[str] = set()

        # Subscribe to /motor_settings to collect names as they appear
        def _collect_ms(msg: Dict[str, Any]):
            name = msg.get("motor_name")
            if name:
                self._motors_seen.add(str(name))
        self._ms_topic.subscribe(_collect_ms)
        self._collector_cb = _collect_ms

        # Pre-load from DB if present
        self._db_path = db_path or os.getenv("PIB_MOTOR_DB") or "/home/pib/app/pib-backend/pib_api/flask/pibdata.db"
        self._load_motors_from_db()

        log.debug("Connected to rosbridge and prepared services.")

    # -------------------- Discovery utilities --------------------
    def _load_motors_from_db(self) -> None:
        path = self._db_path
        self._motors_from_db = []
        try:
            if path and os.path.exists(path):
                with sqlite3.connect(path) as conn:
                    cur = conn.cursor()
                    # Try id-ordered if `id` exists; else just by name
                    cols = {r[1] for r in cur.execute("PRAGMA table_info(motor)").fetchall()}
                    if "id" in cols:
                        rows = cur.execute("SELECT name FROM motor ORDER BY id").fetchall()
                    else:
                        rows = cur.execute("SELECT name FROM motor").fetchall()
                    self._motors_from_db = [str(r[0]) for r in rows if r and r[0]]
                    log.debug("Loaded %d motors from DB %s", len(self._motors_from_db), path)
            else:
                log.debug("Motor DB not found at %s", path)
        except Exception as e:
            log.warning("Failed to load motors from DB %s: %s", path, e)
            self._motors_from_db = []

    def _get_all_motors(self) -> List[str]:
        # Merge DB names and seen names (preserve DB order first), then seen
        seen_only = [n for n in sorted(self._motors_seen) if n not in self._motors_from_db]
        combined = list(self._motors_from_db) + seen_only
        return combined

    @staticmethod
    def _is_finger(name: str) -> bool:
        # Any motor that has _stretch at the end is a finger
        return name.endswith("_stretch")

    @staticmethod
    def _is_left(name: str) -> bool:
        return "left" in name

    @staticmethod
    def _is_right(name: str) -> bool:
        return "right" in name

    def _expand_motor_specs(self, specs: Iterable[Union[str, _Token]]) -> List[str]:
        """Turn tokens/group names into concrete motor names using dynamic discovery."""
        all_motors = self._get_all_motors()
        names: List[str] = []
        for s in specs:
            if isinstance(s, _Token):
                if s is All:
                    if not all_motors:
                        raise ValueError("No motors known yet. Ensure DB is present or publish /motor_settings once.")
                    names.extend(all_motors)
                elif s in (open_hand_left, close_hand_left, open_hand_right, close_hand_right):
                    # Expand to the proper finger group
                    fingers = [m for m in all_motors if self._is_finger(m) and ((self._is_left(m) and s in (open_hand_left, close_hand_left)) or (self._is_right(m) and s in (open_hand_right, close_hand_right)))]
                    if not fingers:
                        log.warning("No finger motors matched for token %s", s)
                    names.extend(fingers)
                elif s in (right_arm, arm_right):
                    members = ["shoulder_vertical_right", "shoulder_horizontal_right", "upper_arm_right_rotation", "elbow_right", "lower_arm_right_rotation", "wrist_right"]
                    if not members:
                        log.warning("No right-arm motors matched.")
                    names.extend(members)
                elif s in (left_arm, arm_left):
                    members = ["shoulder_vertical_left", "shoulder_horizontal_left", "upper_arm_left_rotation", "elbow_left", "lower_arm_left_rotation", "wrist_left"]
                    if not members:
                        log.warning("No left-arm motors matched.")
                    names.extend(members)
                elif s is resting_position:
                    # Expand to All; special positions handled in move()
                    if not all_motors:
                        raise ValueError("No motors known yet for resting_position. Ensure DB or telemetry.")
                    names.extend(all_motors)
                elif s is default:
                    # Not a motor spec ? handled in set()
                    continue
                else:
                    raise ValueError(f"Unknown token: {s}")
            else:
                names.append(str(s))
        # De-duplicate but keep order
        seen = set()
        out: List[str] = []
        for n in names:
            if n not in seen:
                seen.add(n)
                out.append(n)
        return out

    # -------------------- Settings ---------------------
    def set(
        self,
        *motor_specs: Union[str, _Token],
        verify_echo: bool = False,
        echo_timeout: float = 1.0,
        **settings: Any,
    ) -> bool:
        """
        Apply settings to one or many motors.

        Examples:
          w.set("shoulder_vertical_right", velocity=6000, ...)
          w.set("shoulder_vertical_right", "wrist_right", velocity=6000, ...)
          w.set(All, velocity=6000, ...)
          w.set(All, default)
          w.set(All, default=True)    # keyword flag alternative
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
            ms: Dict[str, Any] = {"motor_name": name}
            for k, v in settings.items():
                if v is not None and k != "position":
                    ms[k] = v
            req = roslibpy.ServiceRequest({"motor_settings": ms})
            resp = self._svc_ms.call(req, timeout=5.0)
            applied = bool(resp.get("settings_applied", False))
            persisted = bool(resp.get("settings_persisted", False))
            ok = applied or persisted
            log.debug("set(%s) -> %s | resp=%s", name, ok, resp)

            if not ok and verify_echo:
                # Optional: verify via telemetry echo
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

    # -------------------- Movement ---------------------
    def _move_internal_units(self, joint_names: List[str], positions_internal: List[float]) -> bool:
        """Move helper.
        For a single joint we send one multi-field trajectory as before.
        For multiple joints, many servers only honor the first joint; to
        guarantee movement for all, we issue one service call per joint.
        """
        if len(joint_names) <= 1:
            req = roslibpy.ServiceRequest({
                "joint_trajectory": {
                    "joint_names": joint_names,
                    "points": [_jt_point(positions_internal)],
                }
            })
            resp = self._svc_jt.call(req, timeout=5.0)
            ok = bool(resp.get("successful", False))
            log.debug("move_internal(single %s -> %s) -> %s", joint_names, positions_internal, ok)
            return ok

        all_ok = True
        for name, pos in zip(joint_names, positions_internal):
            req = roslibpy.ServiceRequest({
                "joint_trajectory": {
                    "joint_names": [name],
                    "points": [_jt_point([pos])],
                }
            })
            resp = self._svc_jt.call(req, timeout=5.0)
            ok = bool(resp.get("successful", False))
            log.debug("move_internal(seq %s -> %s) -> %s", name, pos, ok)
            all_ok = all_ok and ok
        return all_ok

    def move(self, *args: Union[str, _Token, int, float]) -> bool:
        '''
        Move one or many motors.

        Patterns supported:
          - w.move("shoulder_vertical_right", -90.0)
          - w.move("shoulder_vertical_right", "shoulder_horizontal_right", "elbow_right", -90.0)
          - w.move(All, -90.0)
          - w.move(All, zero_position)
          - w.move(All, resting_position)
          - w.move(open_hand_left)        # -90 for left fingers
          - w.move(close_hand_left)       # +90 for left fingers
          - w.move(open_hand_right)
          - w.move(close_hand_right)
          - w.move(right_arm, -30.0)      # everything ending with _right, except *_stretch and *thumb*opposition*
          - w.move(left_arm, -30.0)
          - w.move("a", "b", "c", -45.0, 10.0, 5.0)  # per-motor angles
        '''
        if not args:
            raise TypeError("move() requires at least one argument")

        # Split positional args into (specs) then (numbers)
        specs: List[Union[str, _Token]] = []
        numbers: List[float] = []
        hit_number = False
        for a in args:
            if isinstance(a, (int, float)):
                hit_number = True
                numbers.append(float(a))
            else:
                if hit_number:
                    raise TypeError("All motor specs must come before numeric positions")
                specs.append(a)

        # Special single-token hand/open/close cases without explicit numbers
        if len(specs) == 1 and isinstance(specs[0], _Token) and not numbers:
            tok = specs[0]
            if tok in (open_hand_left, open_hand_right):
                specs = [tok]
                numbers = [-90.0]
            elif tok in (close_hand_left, close_hand_right):
                specs = [tok]
                numbers = [90.0]
            elif tok is resting_position:
                # Create internal map: all -> 0; *elbow* -> 5000; *finger (_stretch)* -> -9000
                motors = self._expand_motor_specs([All])
                pos_internal: Dict[str, float] = {m: 0.0 for m in motors}
                for m in motors:
                    if "elbow" in m:
                        pos_internal[m] = 5000.0
                    if self._is_finger(m):
                        pos_internal[m] = -9000.0
                names = list(pos_internal.keys())
                vals = [pos_internal[n] for n in names]
                return self._move_internal_units(names, vals)
            else:
                # allow bare right_arm/left_arm with default broadcast 0 degrees
                if tok in (right_arm, arm_right, left_arm, arm_left):
                    specs = [tok]
                    numbers = [0.0]
                else:
                    raise ValueError(f"Unsupported token for bare move(): {tok}")

        # Expand motor names from specs (tokens/groups resolved to concrete names)
        motor_names = self._expand_motor_specs(specs)
        if not motor_names:
            raise ValueError("No motors resolved from specifications")

        # If token includes open/close hand alongside names and no numbers, broadcast defaults
        if any(isinstance(s, _Token) and s in (open_hand_left, open_hand_right, close_hand_left, close_hand_right) for s in specs) and not numbers:
            if any(isinstance(s, _Token) and s in (open_hand_left, open_hand_right) for s in specs):
                numbers = [-90.0]
            else:
                numbers = [90.0]

        # Determine positions list (degrees) -> convert to internal units
        if not numbers:
            raise TypeError("No target position provided for move(); supply a single angle or one per motor")
        if len(numbers) == 1:
            positions_internal = [_deg_to_internal(numbers[0])] * len(motor_names)
        elif len(numbers) == len(motor_names):
            positions_internal = [_deg_to_internal(n) for n in numbers]
        else:
            raise ValueError(
                f"Provided {len(numbers)} positions for {len(motor_names)} motors; must be 1 or equal count"
            )

        return self._move_internal_units(motor_names, positions_internal)

    # -------------------- Convenience ------------------
    def send(
        self,
        motor_name: str,
        *,
        position: Optional[float] = None,
        verify_echo: bool = False,
        echo_timeout: float = 1.0,
        **settings: Any,
    ) -> None:
        """Convenience: apply settings then move if position is provided (single motor)."""
        if any(v is not None for v in settings.values()):
            self.set(motor_name, verify_echo=verify_echo, echo_timeout=echo_timeout, **settings)
        if position is not None:
            self.move(motor_name, float(position))

    def close(self) -> None:
        try:
            if hasattr(self, "_collector_cb"):
                self._ms_topic.unsubscribe(self._collector_cb)
        except Exception:
            pass
        for t in (self._jt_topic, self._ms_topic):
            try:
                t.unadvertise()
            except Exception:
                pass
        try:
            self.ros.close()
        except Exception:
            pass

# ============================= Defaults preset =============================
DEFAULT_SETTINGS: Dict[str, Any] = dict(
    turned_on=True,
    velocity=16000,
    acceleration=10000,
    deceleration=5000,
    period=19500,
    pulse_width_min=700,
    pulse_width_max=2500,
    rotation_range_min=-9000,
    rotation_range_max=9000,
    visible=True,
    invert=False,
)

# ============================= Read (Telemetry) ============================
class Read:
    """
    Listen to telemetry published by /motor_control:
      - /joint_trajectory (trajectory_msgs/JointTrajectory)
      - /motor_settings (datatypes/MotorSettings)

    Emits merged per-motor dicts to your callback.
    """

    def __init__(
        self,
        callback: Callable[[Dict[str, Any]], None],
        host: str = "localhost",
        port: int = 9090,
        jt_topic_name: str = "/joint_trajectory",
        ms_topic_name: str = "/motor_settings",
        debug: bool = False,
    ):
        if debug:
            log.setLevel(logging.DEBUG)

        self._cb = callback
        self._cache: Dict[str, Dict[str, Any]] = {}

        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_connected(self.ros)

        self._jt = roslibpy.Topic(self.ros, jt_topic_name, "trajectory_msgs/JointTrajectory")
        self._ms = roslibpy.Topic(self.ros, ms_topic_name, "datatypes/MotorSettings")

        self._jt.subscribe(self._on_jt)
        self._ms.subscribe(self._on_ms)

    def _emit(self, motor_name: str) -> None:
        data = dict(self._cache.get(motor_name, {}))
        # expose `position_deg` if we have internal units cached
        if "position_internal" in data:
            try:
                data["position_deg"] = float(data["position_internal"]) / 100.0
            except Exception:
                pass
        self._cb(data)

    def _on_jt(self, msg: Dict[str, Any]) -> None:
        names = msg.get("joint_names") or []
        points = msg.get("points") or []
        if not names or not points:
            return
        positions = points[0].get("positions") or []
        if not positions:
            return
        try:
            pos_list = [int(round(float(p))) for p in positions]
        except Exception:
            return
        for idx, motor_name in enumerate(names):
            if idx >= len(pos_list):
                break
            entry = self._cache.setdefault(motor_name, {"motor_name": motor_name})
            entry["position_internal"] = pos_list[idx]
            self._emit(motor_name)

    def _on_ms(self, msg: Dict[str, Any]) -> None:
        motor_name = msg.get("motor_name")
        if not motor_name:
            return
        entry = self._cache.setdefault(motor_name, {"motor_name": motor_name})
        for k, v in msg.items():
            if k in ("motor_name", "position"):
                continue
            entry[k] = v
        self._emit(motor_name)

    def close(self) -> None:
        for (topic, cb) in ((self._jt, self._on_jt), (self._ms, self._on_ms)):
            try:
                topic.unsubscribe(cb)
            except Exception:
                pass
        try:
            self.ros.close()
        except Exception:
            pass

# ============================= CLI ========================================

def _cli_send(args: argparse.Namespace) -> int:
    w = Write(host=args.host, port=args.port, debug=args.debug)
    try:
        # Settings path
        if args.turn_on or args.set_defaults or any(
            a is not None for a in (args.velocity, args.acceleration, args.deceleration, args.period)
        ):
            specs: List[Union[str, _Token]] = [args.motor]
            kwargs: Dict[str, Any] = dict(
                turned_on=True if args.turn_on or args.set_defaults else None,
                pulse_width_min=700 if args.set_defaults else None,
                pulse_width_max=2500 if args.set_defaults else None,
                rotation_range_min=-9000 if args.set_defaults else None,
                rotation_range_max=9000 if args.set_defaults else None,
                velocity=args.velocity,
                acceleration=args.acceleration,
                deceleration=args.deceleration,
                period=args.period,
            )
            w.set(*specs, verify_echo=args.verify_echo, echo_timeout=args.echo_timeout, **kwargs)
        # Move path
        if args.position_deg is not None:
            ok = w.move(args.motor, args.position_deg)
            print("Move:", "OK" if ok else "FAILED")
        return 0
    finally:
        w.close()


def _cli_echo(args: argparse.Namespace) -> int:
    def _cb(d: Dict[str, Any]):
        if (args.motor is None) or (d.get("motor_name") == args.motor):
            print(json.dumps(d, ensure_ascii=False))
    r = Read(_cb, host=args.host, port=args.port, debug=args.debug)
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        r.close()
    return 0


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="pib control over rosbridge (service-based)")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=9090)
    p.add_argument("--debug", action="store_true")

    sub = p.add_subparsers(dest="cmd", required=True)

    p_send = sub.add_parser("send", help="apply settings and/or send a move (services)")
    p_send.add_argument("--motor", required=True)
    p_send.add_argument("--position-deg", dest="position_deg", type=float, default=None)
    p_send.add_argument("--turn-on", action="store_true", help="turn on motor")
    p_send.add_argument("--set-defaults", action="store_true", help="apply common limits/ranges")
    p_send.add_argument("--velocity", type=int, default=None)
    p_send.add_argument("--acceleration", type=int, default=None)
    p_send.add_argument("--deceleration", type=int, default=None)
    p_send.add_argument("--period", type=int, default=None)
    p_send.add_argument("--verify-echo", action="store_true", help="confirm via /motor_settings telemetry")
    p_send.add_argument("--echo-timeout", type=float, default=1.0)
    p_send.set_defaults(func=_cli_send)

    p_echo = sub.add_parser("echo", help="print merged telemetry")
    p_echo.add_argument("--motor", required=False)
    p_echo.set_defaults(func=_cli_echo)
    return p


def main():
    parser = _build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
