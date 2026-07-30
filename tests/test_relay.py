"""Offline tests for pib_sdk.features.relay (roslibpy is faked; no rosbridge)."""

from __future__ import annotations

import pytest

import pib_sdk.features.relay as relay_module
from conftest import patch_roslibpy
from pib_sdk.features.relay import Relay


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, relay_module)


def test_set_calls_the_service_and_returns_success(fake_roslibpy):
    _topics, services = fake_roslibpy
    relay = Relay(host="localhost")
    services["set_solid_state_relay_state"].default_response = {"successful": True}

    assert relay.set(True) is True
    call = services["set_solid_state_relay_state"].calls[0]
    assert call["solid_state_relay_state"]["turned_on"] is True


def test_get_state_reads_from_the_cache_after_a_publish(fake_roslibpy):
    topics, _services = fake_roslibpy
    relay = Relay(host="localhost")

    topics["solid_state_relay_state"].publish_fake({"turned_on": True})

    assert relay.get_state() is True


def test_get_state_times_out_when_nothing_has_been_reported(fake_roslibpy):
    relay = Relay(host="localhost")

    with pytest.raises(TimeoutError):
        relay.get_state(timeout=0.05)


def test_subscribe_streams_state_reports_until_unsubscribed(fake_roslibpy):
    topics, _services = fake_roslibpy
    relay = Relay(host="localhost")
    received: list[bool] = []

    def _on_state(turned_on):
        received.append(turned_on)

    relay.subscribe(_on_state)
    topics["solid_state_relay_state"].publish_fake({"turned_on": True})
    topics["solid_state_relay_state"].publish_fake({"turned_on": False})

    relay.unsubscribe(_on_state)
    topics["solid_state_relay_state"].publish_fake({"turned_on": True})

    assert received == [True, False]
