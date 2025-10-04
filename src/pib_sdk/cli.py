import argparse
import sys
import json
from typing import Dict, Any, List
from . import control
from .demos import registry

def _add_common_ros_args(p: argparse.ArgumentParser):
    p.add_argument("--host", default="localhost", help="rosbridge host")
    p.add_argument("--port", type=int, default=9090, help="rosbridge port")
    p.add_argument("--debug", action="store_true")

def _cmd_demo_list(args: argparse.Namespace) -> int:
    for name, meta in registry.items():
        print(f"{name}\t- {meta.get('summary','')}")
    return 0

def _cmd_demo_run(args: argparse.Namespace) -> int:
    demo = registry.get(args.name)
    if not demo:
        print(f"Unknown demo: {args.name}", file=sys.stderr)
        return 2
    return int(bool(demo["run"](vars(args))))

def _cmd_motors_move(args: argparse.Namespace) -> int:
    w = control.Write(host=args.host, port=args.port, debug=args.debug)
    try:
        specs: List[Any] = []
        for s in args.specs:
            tok = getattr(control, s, None)
            specs.append(tok if isinstance(tok, control._Token) else s)
        ok = w.move(*specs, args.angle)
        print("OK" if ok else "FAILED")
        return 0 if ok else 1
    finally:
        w.close()

def _cmd_echo(args: argparse.Namespace) -> int:
    def _cb(d: Dict[str, Any]):
        if (args.motor is None) or (d.get("motor_name") == args.motor):
            print(json.dumps(d, ensure_ascii=False))
    r = control.Read(_cb, host=args.host, port=args.port, debug=args.debug)
    try:
        while True:
            pass
    except KeyboardInterrupt:
        r.close()
    return 0

def _cmd_defaults(args: argparse.Namespace) -> int:
    w = control.Write(host=args.host, port=args.port, debug=args.debug)
    try:
        group = getattr(control, args.group, control.All)
        ok = w.set(group, control.default, velocity=args.velocity)
        print("Applied defaults:", "OK" if ok else "FAILED")
        return 0 if ok else 1
    finally:
        w.close()

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="pib", description="pib SDK CLI")
    sub = p.add_subparsers(dest="cmd", required=True)

    # demos
    p_list = sub.add_parser("demo", help="list/run demos")
    sub_demo = p_list.add_subparsers(dest="demo_cmd", required=True)

    p_demo_list = sub_demo.add_parser("list", help="list available demos")
    p_demo_list.set_defaults(func=_cmd_demo_list)

    p_demo_run = sub_demo.add_parser("run", help="run a demo by name")
    p_demo_run.add_argument("name")
    _add_common_ros_args(p_demo_run)
    # imitation demo options (already supported)
    p_demo_run.add_argument("--edge", action="store_true")
    p_demo_run.add_argument("--resolution", default="full")
    p_demo_run.add_argument("--internal-fps", type=int, default=None)
    p_demo_run.add_argument("--trace", type=int, default=0)
    # color_follow demo options
    p_demo_run.add_argument("--color", default="blue",
                            help="blue|pink|green|orange|red")
    p_demo_run.add_argument("--show", action="store_true",
                            help="show preview window")
    p_demo_run.add_argument("--max-deg", type=float, default=30.0,
                            help="max absolute deg for yaw/pitch")
    p_demo_run.add_argument("--min-area", type=int, default=600,
                            help="min contour area to track")
    p_demo_run.set_defaults(func=_cmd_demo_run)

    # motors
    p_move = sub.add_parser("motors", help="quick motor helpers")
    sub_m = p_move.add_subparsers(dest="motors_cmd", required=True)

    p_move_run = sub_m.add_parser("move", help="move motors")
    _add_common_ros_args(p_move_run)
    p_move_run.add_argument("specs", nargs="+", help="e.g. right_arm or shoulder_vertical_right")
    p_move_run.add_argument("angle", type=float, help="angle deg (-90..90)")
    p_move_run.set_defaults(func=_cmd_motors_move)

    # echo telemetry
    p_echo = sub.add_parser("echo", help="print merged telemetry")
    _add_common_ros_args(p_echo)
    p_echo.add_argument("--motor", default=None)
    p_echo.set_defaults(func=_cmd_echo)

    # defaults
    p_defaults = sub.add_parser("defaults", help="apply default settings to a group")
    _add_common_ros_args(p_defaults)
    p_defaults.add_argument("--group", default="All",
                            help="All,left_arm,right_arm,left_hand,right_hand")
    p_defaults.add_argument("--velocity", type=int, default=None)
    p_defaults.set_defaults(func=_cmd_defaults)

    return p

def main():
    args = build_parser().parse_args()
    return args.func(args)

if __name__ == "__main__":
    raise SystemExit(main())
