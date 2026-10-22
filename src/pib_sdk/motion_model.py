"""Compact motion model for voice-assistant and planning prompts.

The full URDF is hundreds of kilobytes; :func:`load_motion_model` and
:func:`prompt_text` expose only the commandable chains, joint axes, limits,
and named postures derived from :mod:`pib_sdk.robot_model` and the bundled URDF.
"""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ElementTree
from pathlib import Path
from typing import Any

from pib_sdk.robot_model import DEFAULT_URDF_PATH, HEAD, LEFT_ARM, RIGHT_ARM, ChainDefinition

SCHEMA_VERSION = 1
MOTION_MODEL_PATH = Path(__file__).resolve().parent / "data" / "motion_model.json"
_CHAIN_DEFINITIONS: tuple[ChainDefinition, ...] = (RIGHT_ARM, LEFT_ARM, HEAD)
_SIG_FIGS = 6


def load_motion_model() -> dict[str, Any]:
    """Load the checked-in motion model JSON bundled with the package."""
    return json.loads(MOTION_MODEL_PATH.read_text(encoding="utf-8"))


def build_motion_model(urdf_path: Path | None = None) -> dict[str, Any]:
    """Regenerate the motion model dict from the URDF and chain definitions."""
    path = (urdf_path or DEFAULT_URDF_PATH).resolve()
    urdf_joints = _parse_urdf_joints(path)
    chains: dict[str, Any] = {}
    for definition in sorted(_CHAIN_DEFINITIONS, key=lambda item: item.name):
        joints_payload: list[dict[str, Any]] = []
        for urdf_name, motor_name in zip(
            definition.urdf_joint_names, definition.motor_names, strict=True
        ):
            try:
                joint = urdf_joints[urdf_name]
            except KeyError:
                raise ValueError(f"URDF joint {urdf_name!r} is missing or has no limits") from None
            lower_deg = _round_sig(math.degrees(joint["lower_rad"]))
            upper_deg = _round_sig(math.degrees(joint["upper_rad"]))
            joints_payload.append(
                {
                    "axis": [_round_sig(value) for value in joint["axis"]],
                    "lower_deg": lower_deg,
                    "motor": motor_name,
                    "type": joint["type"],
                    "upper_deg": upper_deg,
                    "urdf_joint": urdf_name,
                }
            )
        named: dict[str, list[float]] = {}
        for name in sorted(definition.named_configurations_deg):
            named[name] = [
                _round_sig(angle) for angle in definition.named_configurations_deg[name]
            ]
        chains[definition.name] = {
            "joints": joints_payload,
            "named_configurations": named,
            "tip_frame": definition.tip_frame,
        }
    return {
        "chains": chains,
        "schema_version": SCHEMA_VERSION,
        "source": {
            "sdk_version": _sdk_version(),
            "urdf_file": path.name,
            "urdf_sha256": _sha256_file(path),
        },
        "units": {"angle": "deg", "length": "mm"},
    }


def serialize_motion_model(model: dict[str, Any]) -> str:
    """Return deterministic JSON text (sorted keys, stable float formatting)."""
    normalized = _normalize_tree(model)
    return json.dumps(normalized, indent=2, sort_keys=True) + "\n"


def prompt_text() -> str:
    """Compact, human-readable summary of :func:`load_motion_model` for prompts."""
    model = load_motion_model()
    lines = [
        "pib motion model (angles deg, positions mm in kinematics API)",
        f"schema {model['schema_version']} from {model['source']['urdf_file']}",
    ]
    for chain_name in sorted(model["chains"]):
        chain = model["chains"][chain_name]
        lines.append(f"{chain_name} tip={chain['tip_frame']}")
        for joint in chain["joints"]:
            axis = joint["axis"]
            lines.append(
                f"  {joint['motor']}: "
                f"[{joint['lower_deg']}, {joint['upper_deg']}] "
                f"axis=({axis[0]}, {axis[1]}, {axis[2]}) "
                f"urdf={joint['urdf_joint']}"
            )
        if chain["named_configurations"]:
            parts = []
            for name in sorted(chain["named_configurations"]):
                values = chain["named_configurations"][name]
                formatted = ",".join(str(value) for value in values)
                parts.append(f"{name}=({formatted})")
            lines.append(f"  named: {'; '.join(parts)}")
    return "\n".join(lines) + "\n"


def _parse_urdf_joints(urdf_path: Path) -> dict[str, dict[str, Any]]:
    root = ElementTree.parse(urdf_path).getroot()
    joints: dict[str, dict[str, Any]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if not name:
            continue
        axis_element = joint.find("axis")
        if axis_element is not None and axis_element.get("xyz"):
            axis = [float(value) for value in axis_element.get("xyz").split()]
        else:
            axis = [0.0, 0.0, 1.0]
        limit_element = joint.find("limit")
        if limit_element is None:
            continue
        lower = limit_element.get("lower")
        upper = limit_element.get("upper")
        if lower is None or upper is None:
            continue
        joints[name] = {
            "axis": axis,
            "lower_rad": float(lower),
            "type": joint.get("type") or "unknown",
            "upper_rad": float(upper),
        }
    return joints


def _sdk_version(pyproject: Path | None = None) -> str:
    """Return the pib-sdk version, also on interpreters without ``tomllib`` (3.9/3.10)."""
    pyproject = pyproject or Path(__file__).resolve().parents[2] / "pyproject.toml"
    if pyproject.is_file():
        try:
            import tomllib
        except ModuleNotFoundError:  # pragma: no cover - Python < 3.11
            tomllib = None  # type: ignore[assignment]
        if tomllib is not None:
            data = tomllib.loads(pyproject.read_text(encoding="utf-8"))
            return str(data["project"]["version"])
    try:
        from importlib.metadata import PackageNotFoundError, version

        return version("pib-sdk")
    except PackageNotFoundError:  # pragma: no cover - dev checkout without install
        return "unknown"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _round_sig(value: float, sig: int = _SIG_FIGS) -> float:
    if value == 0.0 or not math.isfinite(value):
        return float(value)
    return float(f"{value:.{sig}g}")


def _normalize_tree(value: Any) -> Any:
    if isinstance(value, float):
        return _round_sig(value)
    if isinstance(value, list):
        return [_normalize_tree(item) for item in value]
    if isinstance(value, dict):
        return {key: _normalize_tree(value[key]) for key in sorted(value)}
    return value
