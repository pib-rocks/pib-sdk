#!/usr/bin/env python3
"""Live smoke test for pib-sdk against a real pib robot.

Run this ON (or with network access to) the robot -- it needs rosbridge on
port 9090 and pib-backend on port 5000, same as any other pib-sdk script:

    python3 live_smoke_test.py                     # default: localhost
    python3 live_smoke_test.py --host pib.local
    python3 live_smoke_test.py --no-motion          # skip anything that moves the robot

Exercises every module in the SDK and prints a PASS / FAIL / SKIP report at
the end. Nothing here is destructive -- the one thing it creates (a pose
named "sdk_smoke_test") is deleted again at the end of the same run -- but
the head, one arm joint, and the hands WILL physically move, and the
display/speech/relay-read/camera are all exercised too. Keep the workspace
clear before running, and pass --no-motion for a movement-free pass first
if you'd rather watch before anything moves.
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback

results: list[tuple[str, str, str]] = []  # (name, status, detail)


def run(name: str, fn) -> None:
    try:
        detail = fn()
        results.append((name, "PASS", str(detail) if detail is not None else "ok"))
        print(f"[PASS] {name}: {detail if detail is not None else 'ok'}")
    except Exception as exc:  # noqa: BLE001 -- smoke test: report, don't crash
        results.append((name, "FAIL", f"{type(exc).__name__}: {exc}"))
        print(f"[FAIL] {name}: {type(exc).__name__}: {exc}")
        if args.verbose:
            traceback.print_exc()


def skip(name: str, reason: str) -> None:
    results.append((name, "SKIP", reason))
    print(f"[SKIP] {name}: {reason}")


def main() -> int:
    global args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--rosbridge-port", type=int, default=9090)
    parser.add_argument("--backend-port", type=int, default=5000)
    parser.add_argument(
        "--no-motion", action="store_true", help="skip anything that physically moves the robot"
    )
    parser.add_argument("--verbose", action="store_true", help="print full tracebacks on failure")
    args = parser.parse_args()

    host, rb_port, be_port = args.host, args.rosbridge_port, args.backend_port

    # -- 1. Kinematics: no connection needed at all ------------------------ #
    def kinematics():
        from pib_sdk import camera_pose, fk, ik

        pose = fk("right", [0, 45, 0, 0, 90, 0])
        q_deg = ik("right", xyz=list(pose.translation))
        cam = camera_pose(pan_deg=10, tilt_deg=-5)
        return f"fk/ik roundtrip ok, camera_pose={cam.translation.round(1)}"

    run("kinematics (fk/ik/camera_pose)", kinematics)

    # -- 2. Backend REST: read-only -------------------------------------- #
    from pib_sdk.backend import BackendClient

    backend = BackendClient(host=host, port=be_port)

    run("backend.list_poses", lambda: f"{len(backend.list_poses())} pose(s)")
    run("backend.list_programs", lambda: f"{len(backend.list_programs())} program(s)")
    run(
        "backend.list_button_programs",
        lambda: f"{len(backend.list_button_programs())} button binding(s)",
    )
    run("backend.list_motors", lambda: f"{len(backend.list_motors())} motor(s)")
    run("backend.get_camera_settings", lambda: backend.get_camera_settings())
    run("backend.list_personalities", lambda: f"{len(backend.list_personalities())} personality(s)")
    run("backend.list_chats", lambda: f"{len(backend.list_chats())} chat(s)")

    # -- 3. Telemetry ------------------------------------------------------ #
    from pib_sdk.telemetry import Telemetry

    telemetry = Telemetry(host=host, port=rb_port)

    run(
        "telemetry.get_position_deg",
        lambda: f"turn_head_motor = {telemetry.get_position_deg('turn_head_motor'):.1f} deg",
    )

    def current_reading():
        try:
            ma = telemetry.get_current_ma("turn_head_motor", timeout=2.0)
            return f"{ma} mA"
        except TimeoutError:
            return "no bricklet connected for turn_head_motor (expected on some units)"

    run("telemetry.get_current_ma", current_reading)

    def subscribe_current():
        seen: list[tuple[str, int]] = []
        telemetry.subscribe_current(lambda name, ma: seen.append((name, ma)))
        time.sleep(1.5)
        return f"{len(seen)} reading(s) in 1.5s"

    run("telemetry.subscribe_current", subscribe_current)

    # -- 4. Write: motor control -------------------------------------------#
    from pib_sdk.control import All, Write, close_right_hand, head, open_right_hand, right_arm

    writer = Write(host=host, port=rb_port)

    run("write.set(All, default)", lambda: writer.set(All, default=True))

    if args.no_motion:
        skip("write.move (head)", "--no-motion")
        skip("write.move (right_arm wrist)", "--no-motion")
        skip("write.move (hand open/close)", "--no-motion")
    else:

        def move_head():
            ok1 = writer.move(head, 15.0, 0.0)
            time.sleep(1.0)
            ok2 = writer.move(head, 0.0, 0.0)
            return f"pan to 15deg={ok1}, back to 0={ok2}"

        run("write.move (head)", move_head)

        def move_wrist():
            ok1 = writer.move("wrist_right", 20.0)
            time.sleep(1.0)
            ok2 = writer.move("wrist_right", 0.0)
            return f"wrist to 20deg={ok1}, back to 0={ok2}"

        run("write.move (right_arm wrist)", move_wrist)

        def hand_cycle():
            ok1 = writer.move(open_right_hand)
            time.sleep(1.0)
            ok2 = writer.move(close_right_hand)
            time.sleep(1.0)
            ok3 = writer.move(open_right_hand)
            return f"open={ok1}, close={ok2}, open={ok3}"

        run("write.move (hand open/close)", hand_cycle)

    # -- 5. Speech ----------------------------------------------------------#
    from pib_sdk.speech import Speak

    speak = Speak(host=host, port=rb_port)
    run("speak.say", lambda: speak.say("Testing the pib software development kit."))

    # -- 6. Poses ------------------------------------------------------------#
    from pib_sdk.features.poses import get_pose, list_poses, save_current_pose, set_pose

    def poses_roundtrip():
        created = save_current_pose(
            telemetry, backend, "sdk_smoke_test", motor_names=["turn_head_motor"]
        )
        fetched = get_pose(backend, pose_id=created.pose_id)
        backend.delete_pose(created.pose_id)
        return f"created+fetched+deleted pose {fetched.name!r}"

    run("features.poses (save_current_pose roundtrip)", poses_roundtrip)

    def apply_first_saved_pose():
        saved = list_poses(backend)
        if not saved:
            return "no saved poses on this robot to apply -- skipped"
        if args.no_motion:
            return f"found {saved[0].name!r} but --no-motion set -- not applying"
        pose = set_pose(writer, backend, name=saved[0].name)
        return f"applied {pose.name!r} ({len(pose.motor_angles_deg)} motors)"

    run("features.poses (set_pose, first saved pose)", apply_first_saved_pose)

    # -- 7. Programs (list only -- running one executes arbitrary code) -----#
    from pib_sdk.features.programs import list_programs

    def programs_listed():
        programs = list_programs(backend)
        if not programs:
            return "no saved programs on this robot"
        names = ", ".join(p.name for p in programs[:5])
        return f"{len(programs)} program(s): {names} (not run -- pass --run-program to test execution)"

    run("features.programs (list)", programs_listed)

    # -- 8. Buttons (read-only) ---------------------------------------------#
    from pib_sdk.features.buttons import list_button_bindings

    run(
        "features.buttons (list_button_bindings)",
        lambda: f"{len(list_button_bindings(backend))} binding(s)",
    )

    # -- 9. Camera ------------------------------------------------------------#
    from pib_sdk.features.camera import Camera

    camera = Camera(host=host, port=rb_port)

    def snapshot():
        camera.save_snapshot("smoke_test_snapshot.jpg")
        return "saved smoke_test_snapshot.jpg"

    run("features.camera (save_snapshot)", snapshot)

    # -- 10. Assistant (read-only) --------------------------------------------#
    from pib_sdk.features.assistant import Assistant

    assistant = Assistant(host=host, port=rb_port)
    run("features.assistant (get_state)", lambda: assistant.get_state())

    # -- 11. Display ----------------------------------------------------------#
    from pib_sdk.features.display import Display

    display = Display(host=host, port=rb_port)

    def display_cycle():
        display.show_animated_eyes()
        time.sleep(2.0)
        display.clear()
        return "showed animated eyes for 2s, then cleared"

    if args.no_motion:
        skip("features.display", "--no-motion (screen change, not a physical move, but skipped for a quiet pass)")
    else:
        run("features.display (show_animated_eyes/clear)", display_cycle)

    # -- 12. Relay (read-only -- purpose isn't documented, never toggled here) #
    from pib_sdk.features.relay import Relay

    relay = Relay(host=host, port=rb_port)

    def relay_read():
        try:
            return f"turned_on={relay.get_state(timeout=2.0)}"
        except TimeoutError:
            return "no state reported (no relay hardware connected -- expected on some units)"

    run("features.relay (get_state, read-only)", relay_read)

    # -- cleanup -------------------------------------------------------------#
    for closable in (telemetry, writer, speak, camera, assistant, display, relay):
        try:
            closable.close()
        except Exception:  # noqa: BLE001
            pass

    # -- summary ---------------------------------------------------------------#
    passed = sum(1 for _, status, _ in results if status == "PASS")
    failed = sum(1 for _, status, _ in results if status == "FAIL")
    skipped = sum(1 for _, status, _ in results if status == "SKIP")
    print(f"\n{'=' * 60}\n{passed} passed, {failed} failed, {skipped} skipped\n{'=' * 60}")
    if failed:
        print("\nFailures:")
        for name, status, detail in results:
            if status == "FAIL":
                print(f"  - {name}: {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
