"""pib's voice assistant: personalities, chats, and live state, over REST + rosbridge.

Personalities and chats are pib-backend REST resources -- CRUD via
:class:`pib_sdk.backend.BackendClient`, wrapped here as typed dataclasses.
Turning the assistant on/off, checking whether it's listening, and sending it
a message are live rosbridge calls, via :class:`Assistant`.

Sending a message doesn't return the assistant's reply directly -- the
backend answers asynchronously (it may also run a program, speak out loud,
etc.) and appends its response to the chat's message history. Poll
:func:`list_chat_messages` after sending to read it.

Example
-------
    from pib_sdk.backend import BackendClient
    from pib_sdk.features.assistant import Assistant, create_chat, list_personalities

    backend = BackendClient(host="localhost")
    personality = list_personalities(backend)[0]
    chat = create_chat(backend, topic="testing", personality_id=personality.personality_id)

    assistant = Assistant(host="localhost")
    assistant.set_state(turned_on=True, chat_id=chat.chat_id)
    assistant.send_message(chat.chat_id, "hello!")
    assistant.close()
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import roslibpy

from pib_sdk.backend import BackendClient


def _wait_until_connected(ros: roslibpy.Ros, timeout: float = 5.0) -> None:
    start = time.time()
    while not ros.is_connected and time.time() - start < timeout:
        time.sleep(0.01)
    if not ros.is_connected:
        raise ConnectionError(f"ROSBridge not connected after {timeout:.1f}s")


# --------------------------------------------------------------------------- #
# REST: personalities and chats                                               #
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Personality:
    personality_id: str
    name: str
    gender: str
    description: str | None
    pause_threshold: float
    message_history: int
    assistant_model_id: int


@dataclass(frozen=True)
class Chat:
    chat_id: str
    topic: str
    personality_id: str


@dataclass(frozen=True)
class ChatMessage:
    message_id: str
    timestamp: str
    is_user: bool
    content: str


def _to_personality(item: dict) -> Personality:
    return Personality(
        personality_id=item["personalityId"],
        name=item["name"],
        gender=item["gender"],
        description=item.get("description"),
        pause_threshold=item["pauseThreshold"],
        message_history=item["messageHistory"],
        assistant_model_id=item["assistantModelId"],
    )


def _to_chat(item: dict) -> Chat:
    return Chat(chat_id=item["chatId"], topic=item["topic"], personality_id=item["personalityId"])


def _to_chat_message(item: dict) -> ChatMessage:
    return ChatMessage(
        message_id=item["messageId"],
        timestamp=item["timestamp"],
        is_user=item["isUser"],
        content=item["content"],
    )


def list_personalities(backend: BackendClient) -> list[Personality]:
    return [_to_personality(item) for item in backend.list_personalities()]


def list_chats(backend: BackendClient) -> list[Chat]:
    return [_to_chat(item) for item in backend.list_chats()]


def get_chat(backend: BackendClient, chat_id: str) -> Chat:
    return _to_chat(backend.get_chat(chat_id))


def list_chat_messages(backend: BackendClient, chat_id: str) -> list[ChatMessage]:
    return [_to_chat_message(item) for item in backend.get_chat_messages(chat_id)]


def create_chat(backend: BackendClient, *, topic: str, personality_id: str) -> Chat:
    return _to_chat(backend.create_chat(topic, personality_id))


def delete_chat(backend: BackendClient, chat_id: str) -> None:
    backend.delete_chat(chat_id)


# --------------------------------------------------------------------------- #
# rosbridge: live assistant state                                             #
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class AssistantState:
    turned_on: bool
    chat_id: str


class Assistant:
    """Turns the voice assistant on/off and sends it messages, over rosbridge."""

    def __init__(self, host: str = "localhost", port: int = 9090) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        _wait_until_connected(self.ros)

        self._get_state_service = roslibpy.Service(
            self.ros, "get_voice_assistant_state", "datatypes/GetVoiceAssistantState"
        )
        self._set_state_service = roslibpy.Service(
            self.ros, "set_voice_assistant_state", "datatypes/SetVoiceAssistantState"
        )
        self._get_listening_service = roslibpy.Service(
            self.ros, "get_chat_is_listening", "datatypes/GetChatIsListening"
        )
        self._send_message_service = roslibpy.Service(
            self.ros, "send_chat_message", "datatypes/SendChatMessage"
        )

    def __enter__(self) -> Assistant:
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

    def get_state(self, timeout: float = 5.0) -> AssistantState:
        response = self._get_state_service.call(roslibpy.ServiceRequest({}), timeout=timeout)
        state = response["voice_assistant_state"]
        return AssistantState(turned_on=state["turned_on"], chat_id=state["chat_id"])

    def set_state(self, *, turned_on: bool, chat_id: str, timeout: float = 5.0) -> bool:
        """Turn the assistant on or off for ``chat_id``; returns whether it succeeded."""
        request = roslibpy.ServiceRequest(
            {"voice_assistant_state": {"turned_on": turned_on, "chat_id": chat_id}}
        )
        response = self._set_state_service.call(request, timeout=timeout)
        return bool(response.get("successful", False))

    def is_listening(self, chat_id: str, timeout: float = 5.0) -> bool:
        request = roslibpy.ServiceRequest({"chat_id": chat_id})
        response = self._get_listening_service.call(request, timeout=timeout)
        return bool(response.get("listening", False))

    def send_message(self, chat_id: str, content: str, timeout: float = 5.0) -> bool:
        """Send ``content`` as a user message in ``chat_id``.

        Returns whether the backend accepted the message -- not the
        assistant's reply. See the module docstring.
        """
        request = roslibpy.ServiceRequest({"chat_id": chat_id, "content": content})
        response = self._send_message_service.call(request, timeout=timeout)
        return bool(response.get("successful", False))
