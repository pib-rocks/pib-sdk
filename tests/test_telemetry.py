"""Offline tests for pib_sdk.telemetry (roslibpy is faked; no rosbridge)."""

from __future__ import annotations

import dataclasses
import math
import threading

import pytest

import pib_sdk.telemetry as telemetry_module
from conftest import patch_roslibpy
from pib_sdk.telemetry import JointStateSample, StaleJointStateError, Telemetry


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, telemetry_module)


def _diagnostic_status(motor_name: str, current_ma: int) -> dict:
    return {
        "level": 0,
        "name": motor_name,
        "message": "",
        "values": [{"key": motor_name, "value": str(current_ma)}],
    }


def test_get_position_deg_converts_internal_units_to_degrees(fake_roslibpy):
    _topics, services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    services["get_joint_position"].responses["elbow_right"] = {
        "successful": True,
        "position": 4500,
        "message": "",
    }

    assert telemetry.get_position_deg("elbow_right") == 45.0


def test_get_position_deg_raises_on_unknown_joint(fake_roslibpy):
    _topics, services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    services["get_joint_position"].responses["bogus"] = {
        "successful": False,
        "message": "unknown joint name 'bogus'",
    }

    with pytest.raises(ValueError, match="unknown joint name"):
        telemetry.get_position_deg("bogus")


def test_get_positions_deg_batches_several_motors(fake_roslibpy):
    _topics, services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    services["get_joint_position"].responses["a"] = {"successful": True, "position": 100}
    services["get_joint_position"].responses["b"] = {"successful": True, "position": -200}

    result = telemetry.get_positions_deg(["a", "b"])

    assert result == {"a": 1.0, "b": -2.0}


def test_get_current_ma_reads_from_the_cache_after_a_publish(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")

    topics["motor_current"].publish_fake(_diagnostic_status("elbow_right", 1234))

    assert telemetry.get_current_ma("elbow_right") == 1234


def test_get_current_ma_times_out_for_a_motor_never_seen(fake_roslibpy):
    telemetry = Telemetry(host="localhost")

    with pytest.raises(TimeoutError, match="elbow_right"):
        telemetry.get_current_ma("elbow_right", timeout=0.05)


def test_subscribe_current_streams_readings_until_unsubscribed(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    received: list[tuple[str, int]] = []

    def _on_current(motor_name, current_ma):
        received.append((motor_name, current_ma))

    telemetry.subscribe_current(_on_current)
    topics["motor_current"].publish_fake(_diagnostic_status("wrist_left", 42))
    topics["motor_current"].publish_fake(_diagnostic_status("wrist_left", 43))

    telemetry.unsubscribe_current(_on_current)
    topics["motor_current"].publish_fake(_diagnostic_status("wrist_left", 44))

    assert received == [("wrist_left", 42), ("wrist_left", 43)]
    assert telemetry.get_current_ma("wrist_left") == 44  # cache still updates either way


def test_on_motor_current_ignores_malformed_messages(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")

    topics["motor_current"].publish_fake({"name": "x", "values": []})
    topics["motor_current"].publish_fake({"values": [{"key": "x", "value": "1"}]})

    with pytest.raises(TimeoutError):
        telemetry.get_current_ma("x", timeout=0.05)


# -- measured joint states (/joint_states) ----------------------------------- #
def _joint_state(positions_rad: dict[str, float], sec: int = 1700000000, nanosec: int = 0) -> dict:
    return {
        "header": {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": ""},
        "name": list(positions_rad),
        "position": list(positions_rad.values()),
        "velocity": [],
        "effort": [],
    }


def _age(telemetry: Telemetry, seconds: float) -> None:
    """Pretend every cached /joint_states sample arrived ``seconds`` earlier."""
    shifted: dict[int, JointStateSample] = {}

    def shift(sample: JointStateSample) -> JointStateSample:
        if id(sample) not in shifted:
            shifted[id(sample)] = dataclasses.replace(
                sample, received_monotonic_s=sample.received_monotonic_s - seconds
            )
        return shifted[id(sample)]

    with telemetry._joint_state_lock:
        by_name = telemetry._latest_joint_by_name
        for name, sample in by_name.items():
            by_name[name] = shift(sample)
        if telemetry._latest_joint_state is not None:
            telemetry._latest_joint_state = shift(telemetry._latest_joint_state)


def test_joint_states_subscribed_lazily_with_the_ros2_type(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    topic = topics["/joint_states"]

    assert topic.subscribers == []
    topic.publish_fake(_joint_state({"elbow_right": 0.0}))  # nobody listening yet
    with pytest.raises(TimeoutError):
        telemetry.get_measured_position_deg("elbow_right", timeout=0.02)
    assert len(topic.subscribers) == 1


def test_measured_position_converts_radians_to_degrees(fake_roslibpy):
    topics, services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)

    topics["/joint_states"].publish_fake(_joint_state({"elbow_right": math.pi / 4}))

    assert telemetry.get_measured_position_deg("elbow_right") == pytest.approx(45.0)
    assert services["get_joint_position"].calls == []  # never the commanded-target service


def test_measured_positions_waits_for_a_message_arriving_later(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")

    timer = threading.Timer(
        0.05,
        lambda: topics["/joint_states"].publish_fake(
            _joint_state({"elbow_right": math.radians(10), "wrist_right": math.radians(-20)})
        ),
    )
    timer.start()
    try:
        result = telemetry.get_measured_positions_deg(["elbow_right", "wrist_right"], timeout=2.0)
    finally:
        timer.join()

    assert result == pytest.approx({"elbow_right": 10.0, "wrist_right": -20.0})


def test_measured_position_keeps_per_joint_readings_across_partial_sweeps(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)
    topic = topics["/joint_states"]

    topic.publish_fake(_joint_state({"a": math.radians(1), "b": math.radians(2)}))
    topic.publish_fake(_joint_state({"a": math.radians(3)}))  # sweep missed "b"

    assert telemetry.get_measured_positions_deg(["a", "b"]) == pytest.approx({"a": 3.0, "b": 2.0})
    assert set(telemetry.get_joint_state().positions_rad) == {"a"}


def test_measured_position_times_out_for_an_unpublished_joint(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)
    topics["/joint_states"].publish_fake(_joint_state({"elbow_right": 0.0}))

    with pytest.raises(TimeoutError, match="turn_head_motor"):
        telemetry.get_measured_position_deg("turn_head_motor", timeout=0.05)


def test_stale_reading_raises_unless_max_age_is_none(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)
    topics["/joint_states"].publish_fake(_joint_state({"elbow_right": math.radians(30)}))
    _age(telemetry, 0.8)

    with pytest.raises(StaleJointStateError, match="elbow_right"):
        telemetry.get_measured_position_deg("elbow_right")
    with pytest.raises(StaleJointStateError):
        telemetry.get_joint_state()
    assert telemetry.get_measured_position_deg("elbow_right", max_age=1.0) == pytest.approx(30.0)
    assert telemetry.get_measured_position_deg("elbow_right", max_age=None) == pytest.approx(30.0)
    assert telemetry.get_joint_state(max_age=None).age_s >= 0.8


def test_joint_state_sample_carries_header_stamp_and_arrival_time(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)

    topics["/joint_states"].publish_fake(
        _joint_state({"elbow_right": 0.5}, sec=1700000123, nanosec=250_000_000)
    )
    sample = telemetry.get_joint_state()

    assert isinstance(sample, JointStateSample)
    assert sample.stamp_s == pytest.approx(1700000123.25)
    assert sample.received_s > 0
    assert sample.positions_rad == {"elbow_right": 0.5}
    assert sample.positions_deg["elbow_right"] == pytest.approx(math.degrees(0.5))


def test_ros1_style_and_missing_stamps(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)
    topic = topics["/joint_states"]

    topic.publish_fake(
        {"header": {"stamp": {"secs": 5, "nsecs": 500_000_000}}, "name": ["a"], "position": [0.0]}
    )
    assert telemetry.get_joint_state().stamp_s == pytest.approx(5.5)

    topic.publish_fake({"name": ["a"], "position": [0.0]})
    assert telemetry.get_joint_state().stamp_s is None


def test_subscribe_joint_states_streams_until_unsubscribed(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    received: list[JointStateSample] = []

    telemetry.subscribe_joint_states(received.append)
    topics["/joint_states"].publish_fake(_joint_state({"a": 0.1}, sec=1))
    topics["/joint_states"].publish_fake(_joint_state({"a": 0.2}, sec=2))
    telemetry.unsubscribe_joint_states(received.append)
    topics["/joint_states"].publish_fake(_joint_state({"a": 0.3}, sec=3))

    assert [sample.stamp_s for sample in received] == [1.0, 2.0]
    assert [sample.positions_rad["a"] for sample in received] == [0.1, 0.2]
    assert telemetry.get_measured_position_deg("a") == pytest.approx(math.degrees(0.3))


def test_record_joint_states_collects_messages_for_a_duration(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)

    def _publish_three():
        for sec in (1, 2, 3):
            topics["/joint_states"].publish_fake(_joint_state({"a": float(sec)}, sec=sec))

    timer = threading.Timer(0.02, _publish_three)
    timer.start()
    recorded = telemetry.record_joint_states(0.2)
    timer.join()
    topics["/joint_states"].publish_fake(_joint_state({"a": 9.0}, sec=9))

    assert [sample.stamp_s for sample in recorded] == [1.0, 2.0, 3.0]


def test_malformed_joint_states_are_ignored(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)
    topic = topics["/joint_states"]

    topic.publish_fake({"name": [], "position": []})
    topic.publish_fake({"name": "a", "position": 1.0})
    topic.publish_fake({"name": ["a", "b", ""], "position": [float("nan"), "x", 1.0]})
    with pytest.raises(TimeoutError):
        telemetry.get_joint_state(timeout=0.05)

    topic.publish_fake({"name": ["a", "b"], "position": [float("inf"), 0.25]})
    assert telemetry.get_joint_state().positions_rad == {"b": 0.25}


def test_close_unsubscribes_from_joint_states(fake_roslibpy):
    topics, _services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)
    assert len(topics["/joint_states"].subscribers) == 1

    telemetry.close()

    assert topics["/joint_states"].subscribers == []


def test_get_position_deg_stays_the_commanded_service(fake_roslibpy):
    topics, services = fake_roslibpy
    telemetry = Telemetry(host="localhost")
    telemetry.subscribe_joint_states(lambda sample: None)
    topics["/joint_states"].publish_fake(_joint_state({"elbow_right": math.radians(10)}))
    services["get_joint_position"].responses["elbow_right"] = {"successful": True, "position": 4500}

    assert telemetry.get_position_deg("elbow_right") == 45.0
    assert telemetry.get_measured_position_deg("elbow_right") == pytest.approx(10.0)
