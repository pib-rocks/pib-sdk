"""Offline tests for pib_sdk.models (roslibpy is faked; no rosbridge)."""

from __future__ import annotations

import os

import pytest

import pib_sdk.models as models_module
from conftest import patch_roslibpy
from pib_sdk.models import ModelError, ModelInfo, Models


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, models_module)


def test_models_supports_the_context_manager_protocol(fake_roslibpy):
    with Models(host="localhost") as models:
        assert isinstance(models, Models)


def test_list_models_returns_the_entries_the_node_reports(fake_roslibpy):
    _topics, services = fake_roslibpy
    models = Models(host="localhost")
    services["list_models"].default_response = {
        "models": [
            {
                "model_id": "hand_tracking",
                "task": "hands",
                "licence": "unknown - see source",
                "shaves": 6,
                "size_bytes": 1234,
                "available": True,
                "active": False,
            }
        ]
    }

    listed = models.list_models()

    assert listed == [
        ModelInfo(
            model_id="hand_tracking",
            task="hands",
            licence="unknown - see source",
            shaves=6,
            size_bytes=1234,
            available=True,
            active=False,
        )
    ]
    assert services["list_models"].calls == [{}]


def test_start_model_happy_path_does_not_require_shaves(fake_roslibpy):
    _topics, services = fake_roslibpy
    models = Models(host="localhost")
    services["start_model"].default_response = {
        "success": True,
        "message": "Model hand_tracking started",
    }

    result = models.start_model("hand_tracking")

    assert result.success is True
    assert result.message == "Model hand_tracking started"
    assert services["start_model"].calls == [
        {
            "model_id": "hand_tracking",
            "shaves": 0,
            "owner": f"pib-sdk-{os.getpid()}",
        }
    ]


def test_start_model_success_false_is_an_error(fake_roslibpy):
    _topics, services = fake_roslibpy
    models = Models(host="localhost")
    services["start_model"].default_response = {
        "success": False,
        "message": "Model hand_tracking is compiled for 6 shaves",
    }

    with pytest.raises(ModelError, match="compiled for 6 shaves"):
        models.start_model("hand_tracking")


def test_stop_model_sends_owner_with_model_id(fake_roslibpy):
    _topics, services = fake_roslibpy
    models = Models(host="localhost")
    services["stop_model"].default_response = {
        "success": True,
        "message": "Model hand_tracking stopped",
    }

    result = models.stop_model("hand_tracking")

    assert result.success is True
    assert result.message == "Model hand_tracking stopped"
    assert services["stop_model"].calls == [
        {"model_id": "hand_tracking", "owner": f"pib-sdk-{os.getpid()}"}
    ]
    assert "shaves" not in services["stop_model"].calls[0]


def test_stop_model_success_false_is_an_error(fake_roslibpy):
    _topics, services = fake_roslibpy
    models = Models(host="localhost")
    services["stop_model"].default_response = {
        "success": False,
        "message": "owner must not be empty",
    }

    with pytest.raises(ModelError, match="owner must not be empty"):
        models.stop_model("hand_tracking")


def test_owner_can_be_overridden_on_the_client_and_per_call(fake_roslibpy):
    _topics, services = fake_roslibpy
    models = Models(host="localhost", owner="script-owner")
    services["start_model"].default_response = {"success": True, "message": "ok"}
    services["stop_model"].default_response = {"success": True, "message": "ok"}

    models.start_model("face")
    models.stop_model("face", owner="other-owner")

    assert services["start_model"].calls[0]["owner"] == "script-owner"
    assert services["stop_model"].calls[0]["owner"] == "other-owner"
