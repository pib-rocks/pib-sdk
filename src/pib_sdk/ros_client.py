"""One shared rosbridge client per ``(host, port)`` for the whole process.

roslibpy drives every :class:`roslibpy.Ros` from a single twisted reactor, and
that reactor can be started only once per process: once any client's
``terminate()`` has stopped it, the next ``run()`` fails with
``ReactorNotRestartable`` and surfaces as ``RosTimeoutError: Failed to connect
to ROS``. So pib-sdk's rosbridge classes (:class:`~pib_sdk.control.Write`,
:class:`~pib_sdk.speech.Speak`, :class:`~pib_sdk.telemetry.Telemetry`,
:class:`~pib_sdk.models.Models` and the ``features`` clients) never own a
connection; they borrow one from here and give it back on ``close()``.

* :func:`get_ros` returns the client for an endpoint, creating and connecting
  it on first use. The reactor is started once, by the first client; clients
  for further endpoints connect on the already-running reactor.
* :func:`release_ros` is what a class's ``close()`` calls. It never
  disconnects, so leaving one ``with`` block cannot break the next one.
* :func:`terminate_all` is the explicit teardown for application shutdown and
  tests. It stops the reactor, which roslibpy cannot restart in this process.
"""

from __future__ import annotations

import threading
import time

import roslibpy

_lock = threading.Lock()
_clients: dict[tuple[str, int], roslibpy.Ros] = {}
_refcounts: dict[tuple[str, int], int] = {}
_reactor_started = False


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


def get_ros(host: str = "localhost", port: int = 9090, timeout: float = 5.0) -> roslibpy.Ros:
    """Return the process-wide client for ``host:port``, connected.

    Raises :class:`ConnectionError` if the client is not connected within
    ``timeout`` seconds; roslibpy's own errors from starting the reactor
    (e.g. ``RosTimeoutError``) propagate unchanged. A client that failed to
    connect stays registered, so a retry waits on it instead of opening another.
    """
    global _reactor_started
    key = (host, port)
    with _lock:
        ros = _clients.get(key)
        if ros is None:
            ros = roslibpy.Ros(host=host, port=port)
            _clients[key] = ros
            if _reactor_started:
                ros.connect()
            else:
                _reactor_started = True
                ros.run()
    _wait_until_connected(ros, timeout=timeout)
    with _lock:
        _refcounts[key] = _refcounts.get(key, 0) + 1
    return ros


def release_ros(ros: roslibpy.Ros) -> None:
    """Return a client obtained from :func:`get_ros`. Never disconnects it."""
    with _lock:
        for key, client in _clients.items():
            if client is ros:
                _refcounts[key] = max(0, _refcounts.get(key, 0) - 1)
                return


def terminate_all() -> None:
    """Terminate every shared client and forget them.

    This stops roslibpy's reactor, which cannot be started again in the same
    process; call it only at application shutdown or between tests.
    """
    global _reactor_started
    with _lock:
        clients = list(_clients.values())
        _clients.clear()
        _refcounts.clear()
        _reactor_started = False
    # The first client owns the reactor thread; the others must disconnect while it still runs.
    for ros in reversed(clients):
        try:
            ros.terminate()
        except Exception:
            pass
