"""Offline tests for pib_sdk.features.display (roslibpy is faked; no rosbridge)."""

from __future__ import annotations

import base64

import pytest

import pib_sdk.features.display as display_module
from conftest import patch_roslibpy
from pib_sdk.features.display import Display, ImageFormat


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, display_module)


def test_show_animated_eyes_publishes_the_built_in_image_id(fake_roslibpy):
    topics, _services = fake_roslibpy
    display = Display(host="localhost")

    display.show_animated_eyes()

    (published,) = topics["display_image"].published
    assert published["id"] == {"value": 2}  # PIB_EYES_ANIMATED
    assert published["format"] == {"value": int(ImageFormat.ANIMATED_GIF)}


def test_show_custom_base64_encodes_the_image_bytes(fake_roslibpy):
    topics, _services = fake_roslibpy
    display = Display(host="localhost")
    raw = b"\x89PNGnot-a-real-png"

    display.show_custom(raw, format=ImageFormat.PNG)

    (published,) = topics["display_image"].published
    assert published["id"] == {"value": 1}  # CUSTOM
    assert published["format"] == {"value": int(ImageFormat.PNG)}
    assert base64.b64decode(published["data"]) == raw


def test_clear_publishes_the_none_image_id(fake_roslibpy):
    topics, _services = fake_roslibpy
    display = Display(host="localhost")

    display.clear()

    (published,) = topics["display_image"].published
    assert published["id"] == {"value": 0}  # NONE
