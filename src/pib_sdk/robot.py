"""A single connection bundle for the four core pib-sdk clients.

:class:`Write` (motor control), :class:`Speak` (text-to-speech),
:class:`Telemetry` (position/current readback), and :class:`BackendClient`
(pib-backend's REST API) all point at the same host on a real robot.
:class:`Robot` constructs all four once and holds them as attributes, so
scripts don't have to repeat the host (and rosbridge port, if non-default)
four times. The three rosbridge clients share the process-wide connection
from :mod:`pib_sdk.ros_client`.

Example
-------
    from pib_sdk import right_arm
    from pib_sdk.robot import Robot

    with Robot(host="pib.local") as robot:
        robot.write.move(right_arm, -30.0)
        robot.speak.say("moving my arm")
        print(robot.telemetry.get_position_deg("shoulder_vertical_right"))
        print(robot.backend.list_poses())

Nothing here is new behaviour -- it's the same four classes you could
construct yourself; use them directly instead if you only need one or two, or
want different hosts/ports for each.
"""

from __future__ import annotations

from pib_sdk.backend import DEFAULT_PORT as DEFAULT_BACKEND_PORT
from pib_sdk.backend import BackendClient
from pib_sdk.control import Write
from pib_sdk.speech import Speak
from pib_sdk.telemetry import Telemetry

DEFAULT_ROSBRIDGE_PORT = 9090


class Robot:
    """Bundles :class:`Write`, :class:`Speak`, :class:`Telemetry`, and
    :class:`BackendClient` behind one ``host``.
    """

    def __init__(
        self,
        host: str = "localhost",
        *,
        rosbridge_port: int = DEFAULT_ROSBRIDGE_PORT,
        backend_port: int = DEFAULT_BACKEND_PORT,
    ) -> None:
        self.write = Write(host=host, port=rosbridge_port)
        self.speak = Speak(host=host, port=rosbridge_port)
        self.telemetry = Telemetry(host=host, port=rosbridge_port)
        self.backend = BackendClient(host=host, port=backend_port)

    def __enter__(self) -> Robot:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        """Release the shared rosbridge connection of each client (``backend`` is plain HTTP)."""
        self.write.close()
        self.speak.close()
        self.telemetry.close()
