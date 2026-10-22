"""Tests for the compact motion model artifact and prompt text."""

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

from pib_sdk.motion_model import (
    MOTION_MODEL_PATH,
    build_motion_model,
    load_motion_model,
    prompt_text,
    serialize_motion_model,
)
from pib_sdk.robot_model import HEAD, LEFT_ARM, RIGHT_ARM, get_chain_model

REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_SCRIPT = REPO_ROOT / "scripts" / "build_motion_model.py"
CHAIN_DEFINITIONS = [RIGHT_ARM, LEFT_ARM, HEAD]
MAX_BYTES = 16_384


def _all_chain_motors_and_joints():
    motors: list[str] = []
    joints: list[str] = []
    for definition in CHAIN_DEFINITIONS:
        motors.extend(definition.motor_names)
        joints.extend(definition.urdf_joint_names)
    return motors, joints


def test_checked_in_artifact_matches_regeneration():
    on_disk = load_motion_model()
    fresh = build_motion_model()
    assert on_disk == fresh
    assert MOTION_MODEL_PATH.read_bytes() == serialize_motion_model(fresh).encode("utf-8")


def test_every_motor_and_joint_appears_once_with_urdf_limits():
    model = load_motion_model()
    motors_seen: list[str] = []
    joints_seen: list[str] = []
    for chain_name, chain in model["chains"].items():
        for joint in chain["joints"]:
            joints_seen.append(joint["urdf_joint"])
            motors_seen.append(joint["motor"])
            chain_model = get_chain_model(
                next(d for d in CHAIN_DEFINITIONS if d.name == chain_name)
            )
            index = chain_model.joint_names.index(joint["urdf_joint"])
            lower_deg, upper_deg = chain_model.joint_limits_deg
            assert joint["lower_deg"] == pytest.approx(float(lower_deg[index]))
            assert joint["upper_deg"] == pytest.approx(float(upper_deg[index]))

    expected_motors, expected_joints = _all_chain_motors_and_joints()
    assert sorted(motors_seen) == sorted(expected_motors)
    assert sorted(joints_seen) == sorted(expected_joints)
    assert len(motors_seen) == len(expected_motors)
    assert len(joints_seen) == len(expected_joints)


def test_artifact_and_prompt_text_within_size_budget():
    artifact_bytes = MOTION_MODEL_PATH.read_bytes()
    prompt_bytes = prompt_text().encode("utf-8")
    assert len(artifact_bytes) <= MAX_BYTES, f"artifact is {len(artifact_bytes)} bytes"
    assert len(prompt_bytes) <= MAX_BYTES, f"prompt is {len(prompt_bytes)} bytes"


def test_prompt_text_lists_every_motor_with_limits():
    text = prompt_text()
    motors, _ = _all_chain_motors_and_joints()
    for motor in motors:
        assert motor in text
    for line in text.splitlines():
        if line.startswith("  ") and ":" in line and "named:" not in line:
            assert "[" in line and "," in line


def test_build_script_check_reports_drift(tmp_path):
    drifted = tmp_path / "motion_model.json"
    drifted.write_text('{"schema_version": 0}\n', encoding="utf-8")
    env = {**os.environ, "PYTHONPATH": str(REPO_ROOT / "src")}
    result = subprocess.run(
        [
            sys.executable,
            str(BUILD_SCRIPT),
            "--check",
            "--output",
            str(drifted),
        ],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert "drift" in (result.stdout + result.stderr).lower()


def test_serialize_uses_sorted_keys_and_sig_figs():
    model = load_motion_model()
    text = serialize_motion_model(model)
    parsed = json.loads(text)
    assert list(parsed.keys()) == sorted(parsed.keys())
    chain = parsed["chains"]["right_arm"]
    assert list(chain.keys()) == sorted(chain.keys())
    joint = chain["joints"][0]
    for value in joint["axis"]:
        assert math.isfinite(value)


def test_sdk_version_survives_a_missing_pyproject(tmp_path):
    """Python 3.9/3.10 has no tomllib and a wheel has no pyproject: never crash."""
    from pib_sdk.motion_model import _sdk_version

    value = _sdk_version(tmp_path / "pyproject.toml")
    assert value
    assert value != "None"
