"""Offline tests for pib_sdk.backend (urlopen is mocked; no real HTTP)."""

from __future__ import annotations

import json
from unittest.mock import MagicMock, patch
from urllib.error import HTTPError, URLError

import pytest

from pib_sdk.backend import BackendClient, BackendError


def _mock_response(payload):
    body = json.dumps(payload).encode("utf-8") if payload is not None else b""
    response = MagicMock()
    response.read.return_value = body
    response.__enter__.return_value = response
    response.__exit__.return_value = False
    return response


def test_list_poses_hits_correct_path_and_unwraps():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response(
            {"poses": [{"poseId": "p1", "name": "rest", "deletable": True}]}
        )
        client = BackendClient(host="robot.local", port=5000)
        poses = client.list_poses()

    assert poses == [{"poseId": "p1", "name": "rest", "deletable": True}]
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://robot.local:5000/pose"
    assert request.get_method() == "GET"


def test_get_pose_by_name_quotes_the_name():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response(
            {"poseId": "p1", "name": "a/b c", "motorPositions": [], "deletable": True}
        )
        client = BackendClient()
        client.get_pose_by_name("a/b c")

    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://localhost:5000/pose/by-name/a%2Fb%20c"


def test_create_pose_sends_name_and_motor_positions_as_json():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response(
            {"poseId": "p1", "name": "wave", "motorPositions": [], "deletable": True}
        )
        client = BackendClient()
        client.create_pose("wave", [{"motorName": "elbow_right", "position": 500}])

    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://localhost:5000/pose"
    assert request.get_method() == "POST"
    assert json.loads(request.data) == {
        "name": "wave",
        "motorPositions": [{"motorName": "elbow_right", "position": 500}],
    }
    assert request.get_header("Content-type") == "application/json"


def test_set_button_programs_wraps_updates_key():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response({"buttonPrograms": []})
        client = BackendClient()
        client.set_button_programs([{"brickletNumber": 5, "programNumber": None}])

    request = mock_urlopen.call_args[0][0]
    assert request.get_method() == "PUT"
    assert json.loads(request.data) == {
        "buttonProgramUpdates": [{"brickletNumber": 5, "programNumber": None}]
    }


def test_delete_pose_handles_empty_response_body():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response(None)
        client = BackendClient()
        result = client.delete_pose("p1")

    assert result is None
    request = mock_urlopen.call_args[0][0]
    assert request.get_method() == "DELETE"
    assert request.full_url == "http://localhost:5000/pose/p1"


def test_request_wraps_http_error():
    error_body = MagicMock()
    error_body.read.return_value = b'{"message": "not found"}'
    http_error = HTTPError(
        url="http://localhost:5000/pose/x", code=404, msg="Not Found", hdrs=None, fp=error_body
    )
    with patch("pib_sdk.backend.urlopen", side_effect=http_error):
        client = BackendClient()
        with pytest.raises(BackendError, match="404"):
            client.delete_pose("x")


def test_request_wraps_connection_error():
    with patch("pib_sdk.backend.urlopen", side_effect=URLError("connection refused")):
        client = BackendClient()
        with pytest.raises(BackendError, match="could not reach"):
            client.list_poses()


# --------------------------------------------------------------------------- #
# motors                                                                       #
# --------------------------------------------------------------------------- #
def test_list_motors_hits_get_motor():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response({"motors": [{"name": "elbow_right"}]})
        client = BackendClient()
        assert client.list_motors() == [{"name": "elbow_right"}]
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://localhost:5000/motor"
    assert request.get_method() == "GET"


def test_update_motor_settings_puts_to_the_settings_path():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response({"name": "elbow_right", "velocity": 6000})
        client = BackendClient()
        client.update_motor_settings("elbow_right", {"velocity": 6000})
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://localhost:5000/motor/elbow_right/settings"
    assert request.get_method() == "PUT"
    assert json.loads(request.data) == {"velocity": 6000}


# --------------------------------------------------------------------------- #
# camera settings                                                              #
# --------------------------------------------------------------------------- #
def test_get_camera_settings_hits_camera_settings_path():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response({"resolution": "480", "resX": 640, "resY": 480})
        client = BackendClient()
        settings = client.get_camera_settings()
    assert settings["resX"] == 640
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://localhost:5000/camera-settings"
    assert request.get_method() == "GET"


# --------------------------------------------------------------------------- #
# voice-assistant personalities and chats                                      #
# --------------------------------------------------------------------------- #
def test_list_personalities_hits_the_personality_path():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response({"voiceAssistantPersonalities": [{"name": "x"}]})
        client = BackendClient()
        assert client.list_personalities() == [{"name": "x"}]
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://localhost:5000/voice-assistant/personality"


def test_create_chat_sends_topic_and_personality_id():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response(
            {"chatId": "c1", "topic": "testing", "personalityId": "p1"}
        )
        client = BackendClient()
        client.create_chat("testing", "p1")
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://localhost:5000/voice-assistant/chat"
    assert request.get_method() == "POST"
    assert json.loads(request.data) == {"topic": "testing", "personalityId": "p1"}


def test_get_chat_messages_unwraps_the_messages_key():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response(
            {"messages": [{"messageId": "m1", "content": "hi"}]}
        )
        client = BackendClient()
        messages = client.get_chat_messages("c1")
    assert messages == [{"messageId": "m1", "content": "hi"}]
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://localhost:5000/voice-assistant/chat/c1/messages"


def test_delete_chat_deletes_by_id():
    with patch("pib_sdk.backend.urlopen") as mock_urlopen:
        mock_urlopen.return_value = _mock_response(None)
        client = BackendClient()
        client.delete_chat("c1")
    request = mock_urlopen.call_args[0][0]
    assert request.full_url == "http://localhost:5000/voice-assistant/chat/c1"
    assert request.get_method() == "DELETE"
