"""Offline tests for pib_sdk.features.programs (roslibpy is faked; no rosbridge)."""

from __future__ import annotations

import threading
import time

import pytest

import pib_sdk.control as control_module
import pib_sdk.features.programs as programs_module
from conftest import patch_roslibpy
from pib_sdk.features.programs import Programs, emergency_stop, list_programs


class _FakeBackend:
    def list_programs(self):
        return [{"programNumber": "a1", "name": "wave"}, {"programNumber": "b2", "name": "dance"}]


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, programs_module)


@pytest.fixture
def fake_roslibpy_with_control(monkeypatch):
    return patch_roslibpy(monkeypatch, programs_module, control_module)


def _wait_for_subscriber(topic, timeout=1.0):
    deadline = time.time() + timeout
    while not topic.subscribers:
        assert time.time() < deadline, "expected a subscriber to appear"
        time.sleep(0.005)


def test_list_programs_returns_typed_summaries():
    summaries = list_programs(_FakeBackend())
    assert [(s.program_number, s.name) for s in summaries] == [("a1", "wave"), ("b2", "dance")]


def test_run_collects_streamed_output_and_exit_code(fake_roslibpy):
    topics, services = fake_roslibpy
    programs = Programs(host="localhost")
    services["proxy_run_program_start"].default_response = {"proxy_goal_id": "goal-1"}
    result_holder: dict = {}

    def _runner():
        result_holder["result"] = programs.run("prog-123", timeout=2.0)

    thread = threading.Thread(target=_runner)
    thread.start()
    try:
        # There are two subscribers on the result topic once run() starts
        # (the permanent stop_all() tracker plus run()'s own), so wait for
        # the feedback topic specifically to know run() has reached start().
        _wait_for_subscriber(topics["proxy_run_program_feedback"])

        topics["proxy_run_program_feedback"].publish_fake(
            {
                "proxy_goal_id": "goal-1",
                "output_lines": [{"content": "hello", "is_stderr": False}],
            }
        )
        # A message for a different goal must be ignored, not crash, and not complete early.
        topics["proxy_run_program_result"].publish_fake(
            {"proxy_goal_id": "some-other-goal", "exit_code": 99}
        )
        topics["proxy_run_program_result"].publish_fake({"proxy_goal_id": "goal-1", "exit_code": 0})
    finally:
        thread.join(timeout=2.0)

    assert not thread.is_alive()
    result = result_holder["result"]
    assert result.exit_code == 0
    assert result.output == ("hello",)
    assert result.timed_out is False


def test_run_calls_on_output_callback_per_line(fake_roslibpy):
    topics, services = fake_roslibpy
    programs = Programs(host="localhost")
    services["proxy_run_program_start"].default_response = {"proxy_goal_id": "goal-1"}
    received: list[tuple[str, bool]] = []
    result_holder: dict = {}

    def _on_output(line, is_stderr):
        received.append((line, is_stderr))

    def _runner():
        result_holder["result"] = programs.run("prog-123", timeout=2.0, on_output=_on_output)

    thread = threading.Thread(target=_runner)
    thread.start()
    try:
        _wait_for_subscriber(topics["proxy_run_program_feedback"])
        topics["proxy_run_program_feedback"].publish_fake(
            {"proxy_goal_id": "goal-1", "output_lines": [{"content": "oops", "is_stderr": True}]}
        )
        topics["proxy_run_program_result"].publish_fake({"proxy_goal_id": "goal-1", "exit_code": 1})
    finally:
        thread.join(timeout=2.0)

    assert received == [("oops", True)]
    assert result_holder["result"].exit_code == 1


def test_run_times_out_and_stops_the_program(fake_roslibpy):
    _topics, services = fake_roslibpy
    programs = Programs(host="localhost")
    services["proxy_run_program_start"].default_response = {"proxy_goal_id": "goal-1"}

    result = programs.run("prog-x", timeout=0.05)

    assert result.timed_out is True
    assert result.exit_code is None
    stop_calls = services["proxy_run_program_stop"].calls
    assert len(stop_calls) == 1
    assert stop_calls[0]["proxy_goal_id"] == "goal-1"


def test_run_unsubscribes_its_own_listeners_after_completion(fake_roslibpy):
    topics, services = fake_roslibpy
    programs = Programs(host="localhost")
    services["proxy_run_program_start"].default_response = {"proxy_goal_id": "goal-1"}
    result_holder: dict = {}

    def _runner():
        result_holder["result"] = programs.run("prog-123", timeout=2.0)

    thread = threading.Thread(target=_runner)
    thread.start()
    try:
        _wait_for_subscriber(topics["proxy_run_program_feedback"])
        topics["proxy_run_program_result"].publish_fake({"proxy_goal_id": "goal-1", "exit_code": 0})
    finally:
        thread.join(timeout=2.0)

    assert topics["proxy_run_program_feedback"].subscribers == []
    # The permanent stop_all()-tracking subscriber remains on the result topic.
    assert topics["proxy_run_program_result"].subscribers == [programs._on_any_result]
    assert topics["proxy_run_program_status"].subscribers == []


def test_stop_all_stops_every_goal_started_through_this_instance(fake_roslibpy):
    _topics, services = fake_roslibpy
    programs = Programs(host="localhost")
    services["proxy_run_program_start"].default_response = {"proxy_goal_id": "goal-1"}
    programs.start("prog-a")
    services["proxy_run_program_start"].default_response = {"proxy_goal_id": "goal-2"}
    programs.start("prog-b")

    stopped = programs.stop_all()

    assert sorted(stopped) == ["goal-1", "goal-2"]
    stop_calls = {call["proxy_goal_id"] for call in services["proxy_run_program_stop"].calls}
    assert stop_calls == {"goal-1", "goal-2"}


def test_stop_all_skips_a_goal_that_already_finished_on_its_own(fake_roslibpy):
    topics, services = fake_roslibpy
    programs = Programs(host="localhost")
    services["proxy_run_program_start"].default_response = {"proxy_goal_id": "goal-1"}
    programs.start("prog-a")

    # The program finished by itself -- the permanent result listener should
    # notice even though nobody called .run() or .stop() for it.
    topics["proxy_run_program_result"].publish_fake({"proxy_goal_id": "goal-1", "exit_code": 0})

    assert programs.stop_all() == []
    assert services["proxy_run_program_stop"].calls == []


def test_emergency_stop_stops_tracked_programs_and_turns_off_all_motors(
    fake_roslibpy_with_control,
):
    _topics, services = fake_roslibpy_with_control
    programs = Programs(host="localhost")
    services["proxy_run_program_start"].default_response = {"proxy_goal_id": "goal-1"}
    programs.start("prog-a")
    writer = control_module.Write(host="localhost")

    emergency_stop(writer, programs)

    assert services["proxy_run_program_stop"].calls[0]["proxy_goal_id"] == "goal-1"
    settings_calls = services["/apply_motor_settings"].calls
    assert settings_calls, "expected Write.set(All, turned_on=False) to call apply_motor_settings"
    assert all(call["motor_settings"]["turned_on"] is False for call in settings_calls)


def test_emergency_stop_without_programs_still_turns_off_all_motors(fake_roslibpy_with_control):
    _topics, services = fake_roslibpy_with_control
    writer = control_module.Write(host="localhost")

    emergency_stop(writer)

    settings_calls = services["/apply_motor_settings"].calls
    assert settings_calls
    assert all(call["motor_settings"]["turned_on"] is False for call in settings_calls)
