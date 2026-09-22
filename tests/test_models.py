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
            "owner": "pib-sdk",
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
        {"model_id": "hand_tracking", "owner": "pib-sdk"}
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


def test_default_owner_is_stable_across_runs(fake_roslibpy):
    """Regression fuer den Live-Befund: ein PID-Owner kann fremde Starts nicht stoppen.

    Zwei Models-Instanzen stehen fuer zwei getrennte Laeufe (Skripte, Aufrufe).
    Was der erste startet, muss der zweite stoppen koennen - das verlangt eine
    Owner-Kennung, die sich zwischen Laeufen nicht aendert.

    Der Fake legt bei jedem Models()-Aufbau neue Service-Objekte in ``services``
    ab, die zweite Instanz wuerde die erste also ueberschreiben; deshalb werden
    die Service-Handles hier pro Lauf festgehalten.
    """
    _topics, services = fake_roslibpy

    erster_lauf = Models(host="localhost")
    start_service = services["start_model"]
    start_service.default_response = {"success": True, "message": "ok"}
    erster_lauf.start_model("hand_tracking")

    zweiter_lauf = Models(host="localhost")
    stop_service = services["stop_model"]
    stop_service.default_response = {"success": True, "message": "ok"}
    zweiter_lauf.stop_model("hand_tracking")

    owner_start = start_service.calls[0]["owner"]
    owner_stop = stop_service.calls[0]["owner"]

    assert owner_start == owner_stop, "Stop nutzt einen anderen Owner als der Start"
    assert str(os.getpid()) not in owner_start, "Owner enthaelt die PID und ist damit nicht stabil"
    assert owner_start == "pib-sdk", "Standard-Owner ist nicht der entschiedene stabile Wert"


def test_stop_rejected_by_the_node_raises_instead_of_reporting_success(fake_roslibpy):
    """Befund 2: der Node meldet success=true, obwohl er den Stop ignoriert hat."""
    _topics, services = fake_roslibpy
    models = Models(host="localhost")
    services["stop_model"].default_response = {
        "success": True,
        "message": "Model hand_tracking was not requested by pib-sdk",
    }

    with pytest.raises(ModelError, match="ignored the stop"):
        models.stop_model("hand_tracking")


def test_stop_accepted_but_still_in_use_is_not_an_error(fake_roslibpy):
    """Gegenprobe: 'remains in use' ist ein akzeptierter Stop, kein Fehler."""
    _topics, services = fake_roslibpy
    models = Models(host="localhost")
    services["stop_model"].default_response = {
        "success": True,
        "message": "Model hand_tracking remains in use",
    }

    result = models.stop_model("hand_tracking")
    assert result.success is True


def test_transport_failure_is_raised_as_model_error(fake_roslibpy):
    """Befund 4: ein Verbindungsfehler darf nicht als roslibpy-Ausnahme durchfliegen."""
    _topics, services = fake_roslibpy
    models = Models(host="localhost")

    def boom(_request, timeout=None):
        raise RuntimeError("Timeout exceeded while waiting for service response")

    services["start_model"].call = boom

    with pytest.raises(ModelError, match="Timeout exceeded"):
        models.start_model("hand_tracking")


def test_start_model_default_timeout_tolerates_a_slow_pipeline_rebuild(fake_roslibpy):
    """Befund 3: ein Modellstart ueberlebt 30s, der Default muss groesser sein."""
    import inspect

    default = inspect.signature(Models.start_model).parameters["timeout"].default
    assert default > 30.0, f"Default-Timeout {default}s ist fuer einen Modellstart zu knapp"


def test_connection_failure_raises_connection_error(fake_roslibpy, monkeypatch):
    """Live-Befund: der Konstruktor darf roslibpy-Typen nicht durchreichen."""
    import pib_sdk.models as models_module

    class BoomRos:
        def __init__(self, host=None, port=None):
            self.is_connected = False

        def run(self):
            raise RuntimeError("Failed to connect to ROS")

        def terminate(self):
            pass

    monkeypatch.setattr(models_module.roslibpy, "Ros", BoomRos)

    with pytest.raises(ConnectionError, match="could not connect to rosbridge"):
        Models(host="nowhere", port=9999, connect_timeout=0.1)
