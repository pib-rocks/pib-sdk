"""Shared fake roslibpy primitives for pib_sdk's offline (no-network) tests.

Every rosbridge-backed class in pib_sdk (``Write``, ``Speak``, ``Telemetry``,
``Programs``, ``Camera``, ``Display``, ``Relay``, ``Assistant``) follows the
same shape: construct a ``roslibpy.Ros``, then some ``roslibpy.Service``/
``roslibpy.Topic`` instances off it. These fakes stand in for all three so
tests can drive request/response behavior and simulate incoming topic
messages without a real rosbridge server.
"""

from __future__ import annotations


class FakeRos:
    def __init__(self, host=None, port=None):
        self.host = host
        self.port = port
        self.is_connected = True

    def run(self):
        pass

    def terminate(self):
        pass


class FakeTopic:
    """Fake roslibpy.Topic: records subscribers and published messages.

    ``publish_fake(message)`` is test-only sugar for simulating an incoming
    message by invoking every subscriber directly.
    """

    def __init__(self, ros, name, message_type):
        self.name = name
        self.subscribers: list = []
        self.published: list = []

    def subscribe(self, callback):
        self.subscribers.append(callback)

    def unsubscribe(self, callback):
        if callback in self.subscribers:
            self.subscribers.remove(callback)

    def publish(self, message):
        self.published.append(dict(message))

    def publish_fake(self, message):
        for callback in list(self.subscribers):
            callback(message)


class FakeService:
    """Fake roslibpy.Service.

    Configure a fixed reply regardless of input via ``default_response``, or
    a per-key reply via ``responses`` (keyed by the single value of a
    single-field request, e.g. ``{"joint_name": "elbow_right"}`` looks up
    ``responses["elbow_right"]`` -- every service in pib_sdk is called with
    exactly one identifying field). Every call is recorded in ``.calls``.
    """

    def __init__(self, ros, name, service_type):
        self.name = name
        self.calls: list = []
        self.responses: dict = {}
        self.default_response: dict = {}

    def call(self, request, timeout=None):
        self.calls.append(request)
        if len(request) == 1:
            key = next(iter(request.values()))
            if isinstance(key, (str, int, float, bool)) and key in self.responses:
                return self.responses[key]
        return self.default_response


def patch_roslibpy(monkeypatch, *modules) -> tuple[dict[str, FakeTopic], dict[str, FakeService]]:
    """Patch ``roslibpy.Ros/Service/Topic/ServiceRequest/Message`` on every ``modules``.

    Returns ``(topics, services)`` dicts keyed by name, populated as each
    module constructs its Service/Topic instances.
    """
    topics: dict[str, FakeTopic] = {}
    services: dict[str, FakeService] = {}

    def make_topic(ros, name, message_type):
        topic = FakeTopic(ros, name, message_type)
        topics[name] = topic
        return topic

    def make_service(ros, name, service_type):
        service = FakeService(ros, name, service_type)
        services[name] = service
        return service

    for module in modules:
        monkeypatch.setattr(module.roslibpy, "Ros", FakeRos)
        monkeypatch.setattr(module.roslibpy, "Service", make_service)
        monkeypatch.setattr(module.roslibpy, "Topic", make_topic)
        monkeypatch.setattr(module.roslibpy, "ServiceRequest", lambda payload: payload)
        if hasattr(module.roslibpy, "Message"):
            monkeypatch.setattr(module.roslibpy, "Message", lambda payload: payload)

    return topics, services
