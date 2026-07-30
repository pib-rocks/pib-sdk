"""REST client for pib-backend, the Flask API behind Cerebra.

This is a separate HTTP server from rosbridge (see :mod:`pib_sdk.control` and
:mod:`pib_sdk.speech`, which talk rosbridge directly for real-time motor/audio
control) -- typically the same robot host, a different port. pib-backend owns
the durable data Cerebra's UI lets a user create: saved poses, saved Blockly
programs, and RGB-button-to-program bindings. This module is the thin,
low-level client for that data; see :mod:`pib_sdk.features.poses`,
:mod:`pib_sdk.features.programs`, and :mod:`pib_sdk.features.buttons` for the
friendlier, typed layer built on top of it.

The current backend has no authentication, so this client sends none -- it's
meant for a script running on the same trusted local network as the robot.

Example
-------
    from pib_sdk.backend import BackendClient

    backend = BackendClient(host="localhost")
    for pose in backend.list_poses():
        print(pose["name"], pose["poseId"])
"""

from __future__ import annotations

import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

DEFAULT_PORT = 5000


class BackendError(RuntimeError):
    """Raised when pib-backend returns an error response or can't be reached."""


class BackendClient:
    """Thin REST client for pib-backend's Flask API.

    Every method returns the API's own JSON shape (camelCase keys) rather than
    a converted Python structure -- this mirrors pib-backend's own internal
    ``pib_api_client`` package, which every ROS node on the robot uses to talk
    to this same API.
    """

    def __init__(
        self, host: str = "localhost", port: int = DEFAULT_PORT, timeout: float = 10.0
    ) -> None:
        self.base_url = f"http://{host}:{port}"
        self.timeout = timeout

    def _request(self, method: str, path: str, json_body: dict[str, Any] | None = None) -> Any:
        data = json.dumps(json_body).encode("utf-8") if json_body is not None else None
        request = Request(self.base_url + path, data=data, method=method)
        if data is not None:
            request.add_header("Content-Type", "application/json")
        try:
            with urlopen(request, timeout=self.timeout) as response:
                body = response.read()
        except HTTPError as error:
            detail = error.fp.read().decode("utf-8", errors="replace") if error.fp else ""
            raise BackendError(
                f"{method} {path} -> HTTP {error.code} {error.reason}: {detail}"
            ) from error
        except URLError as error:
            raise BackendError(
                f"{method} {path} -> could not reach {self.base_url}: {error.reason}"
            ) from error
        if not body:
            return None
        return json.loads(body.decode("utf-8"))

    # -- poses ---------------------------------------------------------- #
    def list_poses(self) -> list[dict[str, Any]]:
        """Return ``[{poseId, name, deletable}, ...]`` (no motor positions)."""
        return self._request("GET", "/pose")["poses"]

    def get_pose_by_name(self, name: str) -> dict[str, Any]:
        """Return ``{poseId, name, motorPositions, deletable}`` for the pose named ``name``."""
        return self._request("GET", f"/pose/by-name/{quote(name, safe='')}")

    def get_motor_positions(self, pose_id: str) -> list[dict[str, Any]]:
        """Return ``[{motorName, position}, ...]`` for the pose ``pose_id``."""
        return self._request("GET", f"/pose/{quote(pose_id, safe='')}/motor-positions")[
            "motorPositions"
        ]

    def create_pose(self, name: str, motor_positions: list[dict[str, Any]]) -> dict[str, Any]:
        """Create a pose named ``name`` from ``[{motorName, position}, ...]``."""
        return self._request("POST", "/pose", {"name": name, "motorPositions": motor_positions})

    def rename_pose(self, pose_id: str, name: str) -> dict[str, Any]:
        return self._request("PATCH", f"/pose/{quote(pose_id, safe='')}", {"name": name})

    def update_motor_positions(
        self, pose_id: str, motor_positions: list[dict[str, Any]]
    ) -> dict[str, Any]:
        return self._request(
            "PATCH",
            f"/pose/{quote(pose_id, safe='')}/motor-positions",
            {"motorPositions": motor_positions},
        )

    def delete_pose(self, pose_id: str) -> None:
        self._request("DELETE", f"/pose/{quote(pose_id, safe='')}")

    # -- programs --------------------------------------------------------- #
    def list_programs(self) -> list[dict[str, Any]]:
        """Return ``[{name, programNumber}, ...]``."""
        return self._request("GET", "/program")["programs"]

    def get_program(self, program_number: str) -> dict[str, Any]:
        return self._request("GET", f"/program/{quote(program_number, safe='')}")

    def create_program(self, name: str) -> dict[str, Any]:
        """Create an empty program named ``name``; the server assigns its ``programNumber``."""
        return self._request("POST", "/program", {"name": name})

    def rename_program(self, program_number: str, name: str) -> dict[str, Any]:
        return self._request("PUT", f"/program/{quote(program_number, safe='')}", {"name": name})

    def delete_program(self, program_number: str) -> None:
        self._request("DELETE", f"/program/{quote(program_number, safe='')}")

    def get_program_code(self, program_number: str) -> str:
        """Return the program's Blockly workspace, serialized as a JSON string."""
        return self._request("GET", f"/program/{quote(program_number, safe='')}/code")[
            "codeVisual"
        ]

    def set_program_code(self, program_number: str, code_visual: str) -> None:
        self._request(
            "PUT", f"/program/{quote(program_number, safe='')}/code", {"codeVisual": code_visual}
        )

    # -- RGB button -> program bindings ------------------------------------ #
    def list_button_programs(self) -> list[dict[str, Any]]:
        """Return ``[{brickletNumber, brickletUid, programNumber}, ...]``."""
        return self._request("GET", "/button-programs")["buttonPrograms"]

    def set_button_programs(self, updates: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """Apply ``[{brickletNumber, programNumber}, ...]``; ``programNumber=None`` unassigns."""
        return self._request("PUT", "/button-programs", {"buttonProgramUpdates": updates})[
            "buttonPrograms"
        ]

    # -- motors (configuration; see pib_sdk.telemetry for live values) ---- #
    def list_motors(self) -> list[dict[str, Any]]:
        """Return every motor's full configuration, including settings and pin wiring."""
        return self._request("GET", "/motor")["motors"]

    def get_motor_settings(self, name: str) -> dict[str, Any]:
        """Return one motor's settings (the same fields :class:`pib_sdk.control.Write` sets)."""
        return self._request("GET", f"/motor/{quote(name, safe='')}/settings")

    def update_motor_settings(self, name: str, settings: dict[str, Any]) -> dict[str, Any]:
        return self._request("PUT", f"/motor/{quote(name, safe='')}/settings", settings)

    # -- camera settings ---------------------------------------------------- #
    def get_camera_settings(self) -> dict[str, Any]:
        """Return ``{resolution, refreshRate, qualityFactor, resX, resY}``."""
        return self._request("GET", "/camera-settings")

    def update_camera_settings(self, settings: dict[str, Any]) -> dict[str, Any]:
        return self._request("PUT", "/camera-settings", settings)

    # -- voice-assistant personalities -------------------------------------- #
    def list_personalities(self) -> list[dict[str, Any]]:
        """Return ``[{name, personalityId, gender, description, pauseThreshold,
        messageHistory, assistantModelId}, ...]``."""
        return self._request("GET", "/voice-assistant/personality")["voiceAssistantPersonalities"]

    def get_personality(self, personality_id: str) -> dict[str, Any]:
        return self._request(
            "GET", f"/voice-assistant/personality/{quote(personality_id, safe='')}"
        )

    def create_personality(self, personality: dict[str, Any]) -> dict[str, Any]:
        """``personality`` needs ``name``, ``gender``, ``pauseThreshold``,
        ``messageHistory``, ``assistantModelId``; ``description`` is optional."""
        return self._request("POST", "/voice-assistant/personality", personality)

    def update_personality(
        self, personality_id: str, personality: dict[str, Any]
    ) -> dict[str, Any]:
        return self._request(
            "PUT", f"/voice-assistant/personality/{quote(personality_id, safe='')}", personality
        )

    def delete_personality(self, personality_id: str) -> None:
        self._request("DELETE", f"/voice-assistant/personality/{quote(personality_id, safe='')}")

    # -- voice-assistant chats ------------------------------------------------ #
    def list_chats(self) -> list[dict[str, Any]]:
        """Return ``[{chatId, topic, personalityId}, ...]`` (no messages)."""
        return self._request("GET", "/voice-assistant/chat")["voiceAssistantChats"]

    def get_chat(self, chat_id: str) -> dict[str, Any]:
        """Return ``{chatId, topic, personalityId}`` for one chat (no messages)."""
        return self._request("GET", f"/voice-assistant/chat/{quote(chat_id, safe='')}")

    def get_chat_messages(self, chat_id: str) -> list[dict[str, Any]]:
        """Return every message in ``chat_id``: ``[{messageId, timestamp, isUser, content}]``."""
        return self._request(
            "GET", f"/voice-assistant/chat/{quote(chat_id, safe='')}/messages"
        )["messages"]

    def create_chat(self, topic: str, personality_id: str) -> dict[str, Any]:
        return self._request(
            "POST",
            "/voice-assistant/chat",
            {"topic": topic, "personalityId": personality_id},
        )

    def delete_chat(self, chat_id: str) -> None:
        self._request("DELETE", f"/voice-assistant/chat/{quote(chat_id, safe='')}")
