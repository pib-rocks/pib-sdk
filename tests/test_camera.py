"""Offline tests for pib_sdk.features.camera (roslibpy is faked; no rosbridge)."""

from __future__ import annotations

import base64
import builtins

import numpy as np
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


def _depth_response(width: int, height: int, values: list[int]) -> dict:
    raw = np.asarray(values, dtype="<u2").tobytes()
    return {
        "width": width,
        "height": height,
        "encoding": "16UC1",
        "depth_base64": base64.b64encode(raw).decode("ascii"),
    }


def test_get_depth_frame_decodes_little_endian_uint16_mm(fake_roslibpy):
    _topics, services = fake_roslibpy
    camera = Camera(host="localhost")
    services["get_depth_frame"].default_response = _depth_response(
        width=2, height=2, values=[0, 250, 1000, 65535]
    )

    depth = camera.get_depth_frame()

    assert depth is not None
    assert depth.dtype == np.uint16
    assert depth.shape == (2, 2)
    assert depth.tolist() == [[0, 250], [1000, 65535]]


def test_get_depth_frame_returns_none_when_service_has_no_data(fake_roslibpy):
    _topics, services = fake_roslibpy
    camera = Camera(host="localhost")
    services["get_depth_frame"].default_response = {
        "width": 0,
        "height": 0,
        "encoding": "16UC1",
        "depth_base64": "",
    }

    assert camera.get_depth_frame() is None


def test_get_depth_frame_returns_none_when_payload_is_malformed(fake_roslibpy):
    _topics, services = fake_roslibpy
    camera = Camera(host="localhost")
    services["get_depth_frame"].default_response = {
        "width": 2,
        "height": 2,
        "encoding": "16UC1",
        "depth_base64": base64.b64encode(b"too-short").decode("ascii"),
    }

    assert camera.get_depth_frame() is None


def test_get_distance_at_px_returns_distance_mm_and_sends_xy(fake_roslibpy):
    _topics, services = fake_roslibpy
    camera = Camera(host="localhost")
    services["get_distance_at_px"].default_response = {"distance_mm": 1375.5}

    assert camera.get_distance_at_px(320, 240) == pytest.approx(1375.5)
    assert services["get_distance_at_px"].calls == [{"x": 320, "y": 240}]


def test_get_distance_at_px_returns_zero_when_missing(fake_roslibpy):
    _topics, services = fake_roslibpy
    camera = Camera(host="localhost")
    services["get_distance_at_px"].default_response = {"distance_mm": 0.0}

    assert camera.get_distance_at_px(1, 1) == 0.0


def test_save_depth_writes_npy(fake_roslibpy, tmp_path):
    _topics, services = fake_roslibpy
    camera = Camera(host="localhost")
    services["get_depth_frame"].default_response = _depth_response(
        width=2, height=1, values=[42, 99]
    )

    destination = tmp_path / "depth.npy"
    camera.save_depth(destination)

    loaded = np.load(destination)
    assert loaded.dtype == np.uint16
    assert loaded.shape == (1, 2)
    assert loaded.tolist() == [[42, 99]]


def test_get_depth_frame_requires_numpy(fake_roslibpy, monkeypatch):
    _topics, services = fake_roslibpy
    camera = Camera(host="localhost")
    services["get_depth_frame"].default_response = _depth_response(
        width=1, height=1, values=[1]
    )
    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "numpy":
            raise ImportError("simulated missing numpy")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    with pytest.raises(ImportError, match="numpy"):
        camera.get_depth_frame()
