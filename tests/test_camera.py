"""Offline tests for pib_sdk.features.camera (roslibpy is faked; no rosbridge)."""

from __future__ import annotations

import base64

import pytest

import pib_sdk.features.camera as camera_module
from conftest import patch_roslibpy
from pib_sdk.features.camera import Camera


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, camera_module)


def test_get_snapshot_bytes_decodes_base64(fake_roslibpy):
    _topics, services = fake_roslibpy
    camera = Camera(host="localhost")
    raw = b"\xff\xd8\xff\xe0fake-jpeg-bytes"
    services["get_camera_image"].default_response = {
        "image_base64": base64.b64encode(raw).decode("ascii")
    }

    assert camera.get_snapshot_bytes() == raw


def test_save_snapshot_writes_decoded_bytes_to_disk(fake_roslibpy, tmp_path):
    _topics, services = fake_roslibpy
    camera = Camera(host="localhost")
    raw = b"some-jpeg-bytes"
    services["get_camera_image"].default_response = {
        "image_base64": base64.b64encode(raw).decode("ascii")
    }

    destination = tmp_path / "frame.jpg"
    camera.save_snapshot(destination)

    assert destination.read_bytes() == raw
