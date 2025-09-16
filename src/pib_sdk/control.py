
from __future__ import annotations
from typing import Optional, Any, Dict, Callable, Iterable, Tuple, List
import threading
import time
import json
import logging
import argparse
import roslibpy

# ----------------------------- Logging setup -----------------------------

log = logging.getLogger("pib.control")
handler = logging.StreamHandler()
formatter = logging.Formatter("[%(levelname)s] %(asctime)s %(name)s: %(message)s")
handler.setFormatter(formatter)
if not log.handlers:
    log.addHandler(handler)
log.setLevel(logging.INFO)


# ----------------------------- Conversions -------------------------------

def _normalize_ros_type(t: str | None) -> str | None:
    """
    Normalize ROS type names between ROS1 ("pkg/Type") and ROS2 ("pkg/msg/Type").
    """
    if t is None:
        return None
    return t.replace("/msg/", "/")

def _as_joint_trajectory_dict(motor_name: str, position_int: int) -> Dict[str, Any]:
    """
    trajectory_msgs/JointTrajectory represented as a rosbridge JSON dict.
    """
    return {
        "joint_names": [motor_name],
        "points": [{"positions": [int(position_int)]}],
    }


def _deg_to_internal(position_deg: float) -> int:
    """
    Map degrees (−90..90) to internal units ×100 (−9000..9000).
    """
    if not -90.0 <= float(position_deg) <= 90.0:
        raise ValueError(f"position_deg must be between -90 and 90 (got {position_deg})")
    return int(round(float(position_deg) * 100))


def _internal_to_deg(position: Optional[float]) -> Optional[float]:
    if position is None:
        return None
    try:
        return float(position) / 100.0
    except Exception:
        return None


# ----------------------------- Writers -----------------------------------

class Write:
    """
    Publish motor updates through ROSBridge:
      - Positions -> /joint_trajectory (trajectory_msgs/JointTrajectory)
      - Other settings -> /motor_settings (datatypes/MotorSettings)

    Diagnostics:
      - debug logging
      - send_with_ack() waits until your message is observed on the bus
      - health_check() verifies rosbridge and required topics/types
    """

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        jt_topic_name: str = "/joint_trajectory",
        jt_message_type: str = "trajectory_msgs/JointTrajectory",
        ms_topic_name: str = "/motor_settings",
        ms_message_type: str = "datatypes/MotorSettings",
        debug: bool = False,
    ):
        if debug:
            log.setLevel(logging.DEBUG)

        self.host = host
        self.port = port
        self.jt_topic_name = jt_topic_name
        self.jt_message_type = jt_message_type
        self.ms_topic_name = ms_topic_name
        self.ms_message_type = ms_message_type

        # Connect to ROSBridge (websocket)
        self.ros = roslibpy.Ros(host=host, port=port)

        # Register event hooks for visibility
        try:
            self.ros.on_ready(lambda: log.info("ROSBridge connection established"))
            self.ros.on_close(lambda: log.warning("ROSBridge connection closed"))
            self.ros.on_error(lambda err: log.error(f"ROSBridge error: {err}"))
        except Exception:
            # Older roslibpy might not support the on_* helpers
            pass

        log.debug("Connecting to ROSBridge...")
        self.ros.run()
        log.debug("Connected: %s", self.ros.is_connected)

        # Publishers
        self._jt_topic = roslibpy.Topic(self.ros, jt_topic_name, jt_message_type)
        self._ms_topic = roslibpy.Topic(self.ros, ms_topic_name, ms_message_type)

        log.debug("Advertised publishers: %s, %s", jt_topic_name, ms_topic_name)

    # ------------------ Diagnostics / health ------------------

    def health_check(self, timeout: float = 3.0) -> Dict[str, Any]:
        """
        Confirms connectivity and that required topics exist with the expected types,
        using rosapi services (requires rosbridge_server + rosapi).
        """
        if not self.ros.is_connected:
            raise ConnectionError("Not connected to ROSBridge")

        try:
            svc = roslibpy.Service(self.ros, "/rosapi/topics", "rosapi/Topics")
            req = roslibpy.ServiceRequest()
            done = threading.Event()
            out: Dict[str, Any] = {}

            def _cb(resp: Dict[str, Any]):
                out.update(resp or {})
                done.set()

            svc.call(req, callback=_cb, errback=lambda e: done.set())
            ok = done.wait(timeout)
            if not ok:
                raise TimeoutError("Timed out calling /rosapi/topics")

            topics: List[str] = out.get("topics", [])
            types: List[str] = out.get("types", [])
            tm = dict(zip(topics, types))

            jt_found = tm.get(self.jt_topic_name)
            ms_found = tm.get(self.ms_topic_name)
            result = {
                "connected": self.ros.is_connected,
                "topics_present": {
                    self.jt_topic_name: jt_found,
                    self.ms_topic_name: ms_found,
                },
                "types_ok": (
                    _normalize_ros_type(jt_found) == _normalize_ros_type(self.jt_message_type)
                    and _normalize_ros_type(ms_found) == _normalize_ros_type(self.ms_message_type)
                ),
            }

            log.info("Health: %s", json.dumps(result, indent=2))
            return result
        except Exception as e:
            log.error("Health check failed: %s", e)
            raise

    # ------------------ Publish APIs ------------------

    def send(
        self,
        motor_name: str,
        *,
        position_deg: Optional[float] = None,
        velocity: Optional[int] = None,
        acceleration: Optional[int] = None,
        deceleration: Optional[int] = None,
        turned_on: Optional[bool] = None,
        pulse_width_min: Optional[int] = None,
        pulse_width_max: Optional[int] = None,
        rotation_range_min: Optional[int] = None,
        rotation_range_max: Optional[int] = None,
        period: Optional[int] = None,
        visible: Optional[bool] = None,
        invert: Optional[bool] = None,
        extra_ms_fields: Optional[Dict[str, Any]] = None,
        **kwargs: Any,
    ) -> None:
        """
        - If position is provided, publish it via /joint_trajectory (only).
        - All other non-None settings are sent via /motor_settings.
        """
        # 1) Position via JointTrajectory
        if position_deg is not None:
            pos_int = _deg_to_internal(position_deg)
            jt_msg = _as_joint_trajectory_dict(motor_name, pos_int)
            log.debug("Publishing JT: %s", jt_msg)
            self._jt_topic.publish(roslibpy.Message(jt_msg))
            log.info("JT published -> %s: position=%s (int=%s)", motor_name, position_deg, pos_int)

        # 2) Remaining settings via MotorSettings (exclude position entirely)
        ms_payload: Dict[str, Any] = {"motor_name": motor_name}
        maybe = dict(
            velocity=velocity,
            acceleration=acceleration,
            deceleration=deceleration,
            turned_on=turned_on,
            pulse_width_min=pulse_width_min,
            pulse_width_max=pulse_width_max,
            rotation_range_min=rotation_range_min,
            rotation_range_max=rotation_range_max,
            period=period,
            visible=visible,
            invert=invert,
        )
        for k, v in maybe.items():
            if v is not None:
                ms_payload[k] = v

        if extra_ms_fields:
            ms_payload.update({k: v for k, v in extra_ms_fields.items() if k != "position"})
        if kwargs:
            for k, v in kwargs.items():
                if k != "position" and v is not None:
                    ms_payload[k] = v

        if any(k for k in ms_payload.keys() if k != "motor_name"):
            log.debug("Publishing MS: %s", ms_payload)
            self._ms_topic.publish(roslibpy.Message(ms_payload))
            log.info("MS published -> %s: fields=%s", motor_name, [k for k in ms_payload if k != "motor_name"])

    def send_with_ack(
        self,
        motor_name: str,
        *,
        position_deg: Optional[float] = None,
        ack_timeout: float = 2.5,
        observe_ms: bool = False,
        **settings: Any,
    ) -> bool:
        """
        Publish (like send) then wait to *observe* the matching message on the bus.
        Returns True if observed within timeout, else False.
        - For position: listens on /joint_trajectory for the motor name + position.
        - If observe_ms=True: also waits for a /motor_settings update for that motor.
        """
        evt_jt = threading.Event()
        evt_ms = threading.Event() if observe_ms else None

        jt_expected_int = None
        if position_deg is not None:
            jt_expected_int = _deg_to_internal(position_deg)

        # Subscribe BEFORE publish to avoid missing fast round-trips
        def _on_jt(msg: Dict[str, Any]):
            try:
                names = msg.get("joint_names") or []
                pts = msg.get("points") or []
                if not names or not pts:
                    return
                name = names[0]
                positions = pts[0].get("positions") or []
                if not positions:
                    return
                pos = int(positions[0])
                if name == motor_name and (jt_expected_int is None or pos == jt_expected_int):
                    log.debug("Observed JT echo for %s: %s", motor_name, msg)
                    evt_jt.set()
            except Exception:
                pass

        self._jt_topic.subscribe(_on_jt)

        def _on_ms(msg: Dict[str, Any]):
            try:
                if msg.get("motor_name") == motor_name:
                    log.debug("Observed MS echo for %s: %s", motor_name, msg)
                    if evt_ms:
                        evt_ms.set()
            except Exception:
                pass

        if evt_ms is not None:
            self._ms_topic.subscribe(_on_ms)

        # Publish
        try:
            self.send(motor_name, position_deg=position_deg, **settings)
        except Exception as e:
            log.error("Publish failed: %s", e)
            # Unsubscribe before raising
            try:
                self._jt_topic.unsubscribe(_on_jt)
            except Exception:
                pass
            if evt_ms is not None:
                try:
                    self._ms_topic.unsubscribe(_on_ms)
                except Exception:
                    pass
            raise

        # Wait
        ok_jt = evt_jt.wait(ack_timeout) if jt_expected_int is not None else True
        ok_ms = evt_ms.wait(ack_timeout) if evt_ms is not None else True

        # Cleanup subscriptions
        try:
            self._jt_topic.unsubscribe(_on_jt)
        except Exception:
            pass
        if evt_ms is not None:
            try:
                self._ms_topic.unsubscribe(_on_ms)
            except Exception:
                pass

        if ok_jt and ok_ms:
            log.info("ACK OK for %s (JT%s%s)", motor_name, "" if jt_expected_int is None else f"={jt_expected_int}",
                     " + MS" if observe_ms else "")
            return True
        else:
            if not ok_jt:
                log.warning("ACK TIMEOUT: Did not observe JointTrajectory for %s within %.2fs", motor_name, ack_timeout)
            if evt_ms is not None and not ok_ms:
                log.warning("ACK TIMEOUT: Did not observe MotorSettings for %s within %.2fs", motor_name, ack_timeout)
            return False

    def close(self) -> None:
        # Cleanly close
        try:
            self._jt_topic.unadvertise()
        except Exception:
            pass
        try:
            self._ms_topic.unadvertise()
        except Exception:
            pass
        try:
            self.ros.close()
        except Exception:
            pass


def write(
    motor_name: str,
    *,
    position_deg: Optional[float] = None,
    host: str = "localhost",
    port: int = 9090,
    **settings: Any,
) -> None:
    """
    One-shot helper: sends position (if provided) to /joint_trajectory,
    and other settings to /motor_settings.
    """
    w = Write(host=host, port=port)
    try:
        w.send(motor_name=motor_name, position_deg=position_deg, **settings)
    finally:
        w.close()


# ----------------------------- Readers -----------------------------------

class Read:
    """
    Subscribe to both /joint_trajectory and /motor_settings.
    Your callback receives a merged dict per motor update:

        {
          'motor_name': 'joint1',
          'position': 1234,            # from JointTrajectory (if available)
          'position_deg': 12.34,       # convenience
          'velocity': 100, ...         # from MotorSettings (if provided)
        }

    Diagnostics: set debug=True to get verbose logs as messages arrive.
    """

    def __init__(
        self,
        callback: Callable[[Dict[str, Any]], None],
        host: str = "localhost",
        port: int = 9090,
        jt_topic_name: str = "/joint_trajectory",
        jt_message_type: str = "trajectory_msgs/JointTrajectory",
        ms_topic_name: str = "/motor_settings",
        ms_message_type: str = "datatypes/MotorSettings",
        debug: bool = False,
    ):
        if debug:
            log.setLevel(logging.DEBUG)

        self._cb = callback
        self._cache: Dict[str, Dict[str, Any]] = {}

        # Connect
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()

        # Topics
        self._jt = roslibpy.Topic(self.ros, jt_topic_name, jt_message_type)
        self._ms = roslibpy.Topic(self.ros, ms_topic_name, ms_message_type)

        # Subscriptions
        self._jt.subscribe(self._on_jt)
        self._ms.subscribe(self._on_ms)

        log.debug("Read subscribed to %s and %s", jt_topic_name, ms_topic_name)

    # ---- internal handlers ----

    def _emit(self, motor_name: str) -> None:
        data = dict(self._cache.get(motor_name, {}))
        if "position" in data:
            data["position_deg"] = _internal_to_deg(data.get("position"))
        log.debug("Emit merged update: %s", data)
        self._cb(data)

    def _on_jt(self, msg: Dict[str, Any]) -> None:
        names = msg.get("joint_names") or []
        points = msg.get("points") or []
        if not names or not points:
            return
        motor_name = names[0]
        positions = points[0].get("positions") or []
        if not positions:
            return
        pos = int(positions[0])

        entry = self._cache.setdefault(motor_name, {"motor_name": motor_name})
        entry["position"] = pos
        self._emit(motor_name)

    def _on_ms(self, msg: Dict[str, Any]) -> None:
        motor_name = msg.get("motor_name")
        if not motor_name:
            return
        entry = self._cache.setdefault(motor_name, {"motor_name": motor_name})
        for k, v in msg.items():
            if k in ("motor_name", "position"):
                continue  # ignore 'position' here; position comes from JointTrajectory
            entry[k] = v
        self._emit(motor_name)

    def close(self) -> None:
        try:
            self._jt.unsubscribe(self._on_jt)
        except Exception:
            pass
        try:
            self._ms.unsubscribe(self._on_ms)
        except Exception:
            pass
        try:
            self.ros.close()
        except Exception:
            pass


def read_stream(
    callback: Callable[[Dict[str, Any]], None],
    *,
    host: str = "localhost",
    port: int = 9090,
    debug: bool = False,
) -> Read:
    """
    Continuous merged stream from /joint_trajectory + /motor_settings.
    """
    return Read(callback, host=host, port=port, debug=debug)


def read(
    motor_name: str,
    *,
    host: str = "localhost",
    port: int = 9090,
    timeout: float = 3.0,
    debug: bool = False,
) -> Dict[str, Any]:
    """
    One-shot blocking read: waits for either a JointTrajectory or MotorSettings
    update for the given motor and returns the latest merged view (within timeout).
    """
    if debug:
        log.setLevel(logging.DEBUG)

    result: Dict[str, Any] = {}
    evt = threading.Event()

    def _cb(data: Dict[str, Any]) -> None:
        if data.get("motor_name") == motor_name:
            result.clear()
            result.update(data)
            evt.set()

    r = Read(_cb, host=host, port=port, debug=debug)
    ok = evt.wait(timeout)
    r.close()

    if not ok:
        raise TimeoutError(f"No update for '{motor_name}' within {timeout} seconds")
    return result


# ----------------------------- CLI / Demos --------------------------------

def _cli_check(args: argparse.Namespace) -> int:
    w = Write(host=args.host, port=args.port, debug=args.debug)
    try:
        w.health_check(timeout=args.timeout)
        return 0
    finally:
        w.close()


def _cli_send(args: argparse.Namespace) -> int:
    w = Write(host=args.host, port=args.port, debug=args.debug)
    try:
        if args.ack:
            ok = w.send_with_ack(
                args.motor,
                position_deg=args.position_deg,
                ack_timeout=args.timeout,
                observe_ms=args.observe_ms,
            )
            print("ACK:", "OK" if ok else "TIMEOUT")
            return 0 if ok else 2
        else:
            w.send(args.motor, position_deg=args.position_deg)
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
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        r.close()
    return 0


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="pib control SDK over rosbridge")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=9090)
    p.add_argument("--debug", action="store_true", help="enable verbose logging")

    sub = p.add_subparsers(dest="cmd", required=True)

    p_check = sub.add_parser("check", help="verify connection & topics via rosapi")
    p_check.add_argument("--timeout", type=float, default=3.0)
    p_check.set_defaults(func=_cli_check)

    p_send = sub.add_parser("send", help="send a single position (deg)")
    p_send.add_argument("--motor", required=True, help="motor/joint name")
    p_send.add_argument("--position-deg", type=float, required=False, default=None)
    p_send.add_argument("--ack", action="store_true", help="wait to observe echo on the bus")
    p_send.add_argument("--observe-ms", action="store_true", help="also wait for /motor_settings echo")
    p_send.add_argument("--timeout", type=float, default=2.5)
    p_send.set_defaults(func=_cli_send)

    p_echo = sub.add_parser("echo", help="print merged messages")
    p_echo.add_argument("--motor", required=False, help="filter by motor name")
    p_echo.set_defaults(func=_cli_echo)

    return p


def main():
    parser = _build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
