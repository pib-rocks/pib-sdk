"""Start and stop on-device neural networks over rosbridge.

The camera node exposes three ROS 2 services for the compiled vision models
on the robot: ``list_models``, ``start_model``, and ``stop_model``. They are
not proxied by pib-backend's HTTP API (:class:`pib_sdk.backend.BackendClient`),
so this module talks rosbridge directly — the same transport as
:mod:`pib_sdk.control`.

``start_model`` accepts a ``shaves`` field, but a value other than ``0`` or the
blob's compiled count is rejected. Callers never pass shaves: this wrapper
sends ``0``, which the node treats as the registry / manifest default.
``stop_model`` requires both ``model_id`` and ``owner``; calls with only the id
are rejected. Ownership is kept internal (``pib-sdk-<pid>``, same shape as
Blockly's ``blockly-<pid>``) and can be overridden.

Example
-------
    from pib_sdk.models import Models

    with Models(host="localhost") as models:
        for entry in models.list_models():
            print(entry.model_id, entry.active, entry.shaves)
        result = models.start_model("hand_tracking")
        print(result.success, result.message)
        models.stop_model("hand_tracking")
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Any

import roslibpy


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


def _default_owner() -> str:
    return f"pib-sdk-{os.getpid()}"


class ModelError(RuntimeError):
    """Raised when ``start_model`` or ``stop_model`` returns ``success=false``."""


@dataclass(frozen=True)
class ModelInfo:
    """One entry from the camera node's ``list_models`` response (``ModelInfo``)."""

    model_id: str
    task: str
    licence: str
    shaves: int
    size_bytes: int
    available: bool
    active: bool


@dataclass(frozen=True)
class ModelResult:
    """``success`` and ``message`` from ``start_model`` / ``stop_model``."""

    success: bool
    message: str


def _entry_field(entry: Any, field: str, default: Any = "") -> Any:
    if isinstance(entry, dict):
        return entry.get(field, default)
    return getattr(entry, field, default)


def _model_info_from_entry(entry: Any) -> ModelInfo:
    return ModelInfo(
        model_id=str(_entry_field(entry, "model_id", "")),
        task=str(_entry_field(entry, "task", "")),
        licence=str(_entry_field(entry, "licence", "")),
        shaves=int(_entry_field(entry, "shaves", 0) or 0),
        size_bytes=int(_entry_field(entry, "size_bytes", 0) or 0),
        available=bool(_entry_field(entry, "available", False)),
        active=bool(_entry_field(entry, "active", False)),
    )


class Models:
    """List, start, and stop on-device neural networks over rosbridge."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        *,
        owner: str | None = None,
        list_models_service: str = "list_models",
        start_model_service: str = "start_model",
        stop_model_service: str = "stop_model",
        connect_timeout: float = 5.0,
    ) -> None:
        self._owner = owner if owner is not None else _default_owner()
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros, timeout=connect_timeout)

        self._list_models_service = roslibpy.Service(
            self.ros, list_models_service, "datatypes/ListModels"
        )
        self._start_model_service = roslibpy.Service(
            self.ros, start_model_service, "datatypes/StartModel"
        )
        self._stop_model_service = roslibpy.Service(
            self.ros, stop_model_service, "datatypes/StopModel"
        )

    def __enter__(self) -> Models:
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

    def list_models(self, timeout: float = 10.0) -> list[ModelInfo]:
        """Return every model the camera node reports."""
        if not self.ros.is_connected:
            raise RuntimeError("rosbridge connection is not active.")
        response = self._list_models_service.call(roslibpy.ServiceRequest({}), timeout=timeout)
        entries = (response or {}).get("models") or []
        return [_model_info_from_entry(entry) for entry in entries]

    def start_model(
        self,
        model_id: str,
        *,
        owner: str | None = None,
        timeout: float = 30.0,
    ) -> ModelResult:
        """Start inference for ``model_id`` using the manifest shave count.

        ``shaves`` is sent as ``0`` (registry default). Callers do not choose a
        shave budget; a mismatched explicit value would be rejected by the node.
        """
        return self._call_lifecycle(
            self._start_model_service,
            {
                "model_id": str(model_id),
                "shaves": 0,
                "owner": owner if owner is not None else self._owner,
            },
            model_id=str(model_id),
            timeout=timeout,
        )

    def stop_model(
        self,
        model_id: str,
        *,
        owner: str | None = None,
        timeout: float = 30.0,
    ) -> ModelResult:
        """Stop inference for ``model_id``. Always sends the owning string."""
        return self._call_lifecycle(
            self._stop_model_service,
            {
                "model_id": str(model_id),
                "owner": owner if owner is not None else self._owner,
            },
            model_id=str(model_id),
            timeout=timeout,
        )

    def _call_lifecycle(
        self,
        service: roslibpy.Service,
        payload: dict[str, Any],
        *,
        model_id: str,
        timeout: float,
    ) -> ModelResult:
        if not self.ros.is_connected:
            raise RuntimeError("rosbridge connection is not active.")
        response = service.call(roslibpy.ServiceRequest(payload), timeout=timeout) or {}
        success = bool(response.get("success", False))
        message = str(response.get("message", "") or "")
        if not success:
            raise ModelError(message or f"model call failed for {model_id!r}")
        return ModelResult(success=True, message=message)
