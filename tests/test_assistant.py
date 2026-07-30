"""Offline tests for pib_sdk.features.assistant (fake backend + faked roslibpy)."""

from __future__ import annotations

import pytest

import pib_sdk.features.assistant as assistant_module
from conftest import patch_roslibpy
from pib_sdk.features.assistant import (
    Assistant,
    Chat,
    ChatMessage,
    Personality,
    create_chat,
    delete_chat,
    get_chat,
    list_chat_messages,
    list_chats,
    list_personalities,
)


class _FakeBackend:
    def __init__(self):
        self.deleted_chats: list[str] = []
        self.created_chats: list[tuple[str, str]] = []

    def list_personalities(self):
        return [
            {
                "personalityId": "p1",
                "name": "Default",
                "gender": "female",
                "description": None,
                "pauseThreshold": 1.5,
                "messageHistory": 10,
                "assistantModelId": 1,
            }
        ]

    def list_chats(self):
        return [{"chatId": "c1", "topic": "testing", "personalityId": "p1"}]

    def get_chat(self, chat_id):
        assert chat_id == "c1"
        return {"chatId": "c1", "topic": "testing", "personalityId": "p1"}

    def get_chat_messages(self, chat_id):
        assert chat_id == "c1"
        return [
            {"messageId": "m1", "timestamp": "t0", "isUser": True, "content": "hi"},
            {"messageId": "m2", "timestamp": "t1", "isUser": False, "content": "hello!"},
        ]

    def create_chat(self, topic, personality_id):
        self.created_chats.append((topic, personality_id))
        return {"chatId": "c2", "topic": topic, "personalityId": personality_id}

    def delete_chat(self, chat_id):
        self.deleted_chats.append(chat_id)


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, assistant_module)


# --------------------------------------------------------------------------- #
# REST: personalities / chats                                                 #
# --------------------------------------------------------------------------- #
def test_list_personalities_converts_to_dataclasses():
    result = list_personalities(_FakeBackend())
    assert result == [
        Personality(
            personality_id="p1",
            name="Default",
            gender="female",
            description=None,
            pause_threshold=1.5,
            message_history=10,
            assistant_model_id=1,
        )
    ]


def test_list_chats_and_get_chat():
    backend = _FakeBackend()
    assert list_chats(backend) == [Chat(chat_id="c1", topic="testing", personality_id="p1")]
    assert get_chat(backend, "c1") == Chat(chat_id="c1", topic="testing", personality_id="p1")


def test_list_chat_messages_converts_to_dataclasses():
    messages = list_chat_messages(_FakeBackend(), "c1")
    assert messages == [
        ChatMessage(message_id="m1", timestamp="t0", is_user=True, content="hi"),
        ChatMessage(message_id="m2", timestamp="t1", is_user=False, content="hello!"),
    ]


def test_create_chat_sends_topic_and_personality_id():
    backend = _FakeBackend()
    chat = create_chat(backend, topic="testing", personality_id="p1")
    assert backend.created_chats == [("testing", "p1")]
    assert chat == Chat(chat_id="c2", topic="testing", personality_id="p1")


def test_delete_chat_forwards_to_backend():
    backend = _FakeBackend()
    delete_chat(backend, "c1")
    assert backend.deleted_chats == ["c1"]


# --------------------------------------------------------------------------- #
# rosbridge: live assistant state                                             #
# --------------------------------------------------------------------------- #
def test_get_state_reads_turned_on_and_chat_id(fake_roslibpy):
    _topics, services = fake_roslibpy
    assistant = Assistant(host="localhost")
    services["get_voice_assistant_state"].default_response = {
        "voice_assistant_state": {"turned_on": True, "chat_id": "c1"}
    }

    state = assistant.get_state()

    assert state.turned_on is True
    assert state.chat_id == "c1"


def test_set_state_sends_turned_on_and_chat_id(fake_roslibpy):
    _topics, services = fake_roslibpy
    assistant = Assistant(host="localhost")
    services["set_voice_assistant_state"].default_response = {"successful": True}

    assert assistant.set_state(turned_on=True, chat_id="c1") is True
    call = services["set_voice_assistant_state"].calls[0]
    assert call["voice_assistant_state"] == {"turned_on": True, "chat_id": "c1"}


def test_is_listening_reads_per_chat_state(fake_roslibpy):
    _topics, services = fake_roslibpy
    assistant = Assistant(host="localhost")
    services["get_chat_is_listening"].responses["c1"] = {"listening": True}

    assert assistant.is_listening("c1") is True


def test_send_message_sends_chat_id_and_content(fake_roslibpy):
    _topics, services = fake_roslibpy
    assistant = Assistant(host="localhost")
    services["send_chat_message"].default_response = {"successful": True}

    assert assistant.send_message("c1", "hello!") is True
    call = services["send_chat_message"].calls[0]
    assert call == {"chat_id": "c1", "content": "hello!"}
