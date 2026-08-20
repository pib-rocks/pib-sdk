"""Run Blockly programs saved in Cerebra, from pib-sdk.

Cerebra's visual programs are stored server-side (see
:class:`pib_sdk.backend.BackendClient` for the program CRUD/listing REST API,
and :func:`list_programs` below for a typed view of it) and executed on the
robot by a ROS2 action. rosbridge couldn't speak ROS2 actions at the time
pib-backend was built, so it wraps that action in a plain service/topic
"proxy" -- ordinary rosbridge traffic, reachable exactly like the rest of
pib-sdk's rosbridge calls. This module talks to that proxy.

Example
-------
    from pib_sdk.features.programs import Programs

    programs = Programs(host="localhost")
    result = programs.run(program_number, on_output=print, timeout=30)
    print(result.exit_code, result.status)
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import roslibpy

from pib_sdk.backend import BackendClient
from pib_sdk.control import All, Write

# Standard ROS2 action_msgs/msg/GoalStatus values, published verbatim on
# proxy_run_program_status by pib-backend's proxy_program.py.
STATUS_UNKNOWN = 0
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

# The set of statuses proxy_program.py itself treats as terminal.
TERMINAL_STATUSES = frozenset({STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED})


@dataclass(frozen=True)
class ProgramSummary:
    """A saved program's identity, as listed by Cerebra."""

    program_number: str
    name: str


@dataclass(frozen=True)
class ProgramResult:
    """Outcome of a :meth:`Programs.run` call."""

    exit_code: int | None
    status: int | None
    timed_out: bool
    output: tuple[str, ...] = ()


def list_programs(backend: BackendClient) -> list[ProgramSummary]:
    """List every program saved in Cerebra."""
    return [
        ProgramSummary(program_number=item["programNumber"], name=item["name"])
        for item in backend.list_programs()
    ]


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


class Programs:
    """Starts, stops, and streams output from Cerebra programs over rosbridge."""

    def __init__(self, host: str = "localhost", port: int = 9090) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros)

        self._start_service = roslibpy.Service(
            self.ros, "proxy_run_program_start", "datatypes/ProxyRunProgramStart"
        )
        self._stop_service = roslibpy.Service(
            self.ros, "proxy_run_program_stop", "datatypes/ProxyRunProgramStop"
        )
        self._feedback_topic = roslibpy.Topic(
            self.ros, "proxy_run_program_feedback", "datatypes/ProxyRunProgramFeedback"
        )
        self._result_topic = roslibpy.Topic(
            self.ros, "proxy_run_program_result", "datatypes/ProxyRunProgramResult"
        )
        self._status_topic = roslibpy.Topic(
            self.ros, "proxy_run_program_status", "datatypes/ProxyRunProgramStatus"
        )

        # Goal ids started through *this* instance that haven't finished yet;
        # backs stop_all() / emergency_stop(). We can't see goals started
        # elsewhere (e.g. from Cerebra itself) -- there's no "list running
        # programs" call on the backend. Subscribed permanently (not just
        # during run()) so a goal started via bare start() still gets cleaned
        # up here once it finishes on its own.
        self._active_goal_ids: set[str] = set()
        self._result_topic.subscribe(self._on_any_result)

    def _on_any_result(self, message: dict[str, Any]) -> None:
        goal_id = message.get("proxy_goal_id")
        if goal_id is not None:
            self._active_goal_ids.discard(goal_id)

    def __enter__(self) -> Programs:
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

    def start(self, program_number: str, timeout: float = 5.0) -> str:
        """Start a saved program by id; returns a ``proxy_goal_id`` for :meth:`stop`/:meth:`run`."""
        request = roslibpy.ServiceRequest({"program_number": program_number})
        response = self._start_service.call(request, timeout=timeout)
        goal_id = response["proxy_goal_id"]
        self._active_goal_ids.add(goal_id)
        return goal_id

    def stop(self, proxy_goal_id: str, timeout: float = 5.0) -> None:
        """Cancel a program started with :meth:`start`."""
        request = roslibpy.ServiceRequest({"proxy_goal_id": proxy_goal_id})
        self._stop_service.call(request, timeout=timeout)
        self._active_goal_ids.discard(proxy_goal_id)

    def stop_all(self, timeout: float = 5.0) -> list[str]:
        """Stop every program started through this instance that hasn't finished yet.

        Returns the goal ids that were stopped. Can't see (or stop) a program
        started elsewhere, e.g. from Cerebra itself -- see :attr:`_active_goal_ids`.
        """
        goal_ids = list(self._active_goal_ids)
        for goal_id in goal_ids:
            self.stop(goal_id, timeout=timeout)
        return goal_ids

    def run(
        self,
        program_number: str,
        *,
        timeout: float | None = 60.0,
        on_output: Callable[[str, bool], None] | None = None,
    ) -> ProgramResult:
        """Start a program and block until it finishes (or ``timeout`` elapses).

        ``on_output(line, is_stderr)`` is called for each line of program
        output as it streams in, if given. If ``timeout`` elapses first, the
        program is cancelled via :meth:`stop` and the result has
        ``timed_out=True``.
        """
        done = threading.Event()
        state: dict[str, Any] = {"exit_code": None, "status": None, "output": []}
        goal_id: str | None = None

        def _on_feedback(message: dict[str, Any]) -> None:
            if message.get("proxy_goal_id") != goal_id:
                return
            for line in message.get("output_lines", []):
                state["output"].append(line["content"])
                if on_output is not None:
                    on_output(line["content"], line["is_stderr"])

        def _on_result(message: dict[str, Any]) -> None:
            if message.get("proxy_goal_id") != goal_id:
                return
            state["exit_code"] = message.get("exit_code")
            done.set()

        def _on_status(message: dict[str, Any]) -> None:
            if message.get("proxy_goal_id") != goal_id:
                return
            state["status"] = message.get("status")

        self._feedback_topic.subscribe(_on_feedback)
        self._result_topic.subscribe(_on_result)
        self._status_topic.subscribe(_on_status)
        try:
            goal_id = self.start(program_number)
            finished = done.wait(timeout=timeout)
            if not finished and state["status"] not in TERMINAL_STATUSES:
                self.stop(goal_id)
            return ProgramResult(
                exit_code=state["exit_code"],
                status=state["status"],
                timed_out=not finished,
                output=tuple(state["output"]),
            )
        finally:
            self._feedback_topic.unsubscribe(_on_feedback)
            self._result_topic.unsubscribe(_on_result)
            self._status_topic.unsubscribe(_on_status)


def emergency_stop(writer: Write, programs: Programs | None = None) -> None:
    """Stop everything: cancel every program tracked by ``programs`` (if given),
    then turn off every motor.

    ``programs`` only knows about goals started through that same
    :class:`Programs` instance -- pass the one your script has been using.
    Even without one, this still turns off every motor via
    ``writer.set(All, turned_on=False)``, which stops all robot movement
    regardless of what commanded it.
    """
    if programs is not None:
        programs.stop_all()
    writer.set(All, turned_on=False)
