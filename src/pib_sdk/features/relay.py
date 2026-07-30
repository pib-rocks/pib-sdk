"""pib's solid-state relay, over rosbridge.

A single on/off relay, set via the ``set_solid_state_relay_state`` service
and reported on the ``solid_state_relay_state`` topic (published ~1×/second
regardless of change, so :meth:`Relay.get_state` never blocks for long).
What it physically switches isn't documented in pib-backend's code -- confirm
with the team before relying on it for anything safety-critical.

Example
-------
    from pib_sdk.features.relay import Relay

    relay = Relay(host="localhost")
    relay.set(True)
    print(relay.get_state())
    relay.close()
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

import roslibpy


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


class Relay:
    """Reads and sets pib's solid-state relay over rosbridge."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        set_state_service: str = "set_solid_state_relay_state",
        state_topic: str = "solid_state_relay_state",
    ) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros)

        self._set_state_service = roslibpy.Service(
            self.ros, set_state_service, "datatypes/SetSolidStateRelay"
        )
        self._state_topic = roslibpy.Topic(
            self.ros, state_topic, "datatypes/SolidStateRelayState"
        )

        self._state_lock = threading.Lock()
        self._latest_state: bool | None = None
        self._subscribers: list[Callable[[bool], None]] = []
        self._state_topic.subscribe(self._on_state)

    def __enter__(self) -> Relay:
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

    def set(self, turned_on: bool, timeout: float = 5.0) -> bool:
        """Turn the relay on or off; returns whether the backend reported success."""
        request = roslibpy.ServiceRequest({"solid_state_relay_state": {"turned_on": turned_on}})
        response = self._set_state_service.call(request, timeout=timeout)
        return bool(response.get("successful", False))

    def get_state(self, timeout: float = 2.0) -> bool:
        """Return the relay's most recently reported state.

        It's republished ~once/second whether or not it changed, so this
        rarely waits long; raises :class:`TimeoutError` if nothing has been
        seen within ``timeout`` (e.g. no relay hardware is connected).
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._state_lock:
                if self._latest_state is not None:
                    return self._latest_state
            time.sleep(0.02)
        raise TimeoutError(f"No relay state reported within {timeout:.1f}s")

    def subscribe(self, callback: Callable[[bool], None]) -> None:
        """Call ``callback(turned_on)`` for every future state report."""
        with self._state_lock:
            self._subscribers.append(callback)

    def unsubscribe(self, callback: Callable[[bool], None]) -> None:
        with self._state_lock:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

    def _on_state(self, message: dict) -> None:
        turned_on = message.get("turned_on")
        if turned_on is None:
            return
        with self._state_lock:
            self._latest_state = bool(turned_on)
            subscribers = list(self._subscribers)
        for subscriber in subscribers:
            subscriber(bool(turned_on))
