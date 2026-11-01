#!/usr/bin/env python3
"""Generate pib_sdk/data/pib_model.urdf from the CAD control model and joint mapping.

Reads pib-3d's reduced control URDF and mapping.json, writes a minimal SDK URDF with
the 14 actuated chain joints under SDK names, CAD geometry (axes negated when
mapping direction is -1), and tip frames urdf_palm_right / urdf_palm_left /
urdf_camera_link. Lengths stay in metres; no meshes.
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPO_ROOT / "src" / "pib_sdk" / "data" / "pib_model.urdf"
DEFAULT_CONTROL_URDF = Path("/home/pib/work/pib-3d/urdf/pib.control.urdf")
DEFAULT_MAPPING = Path("/home/pib/work/pib-3d/joints/mapping.json")

# SDK URDF joint name -> CAD control URDF joint name (PR-2158 / pib-3d PR #11).
SDK_TO_CAD_JOINT: dict[str, str] = {
    "shoulder_vertical_right": "rotation_inner_shoulder_1",
    "shoulder_horizontal_right": "rotation_shoulder_left",
    "upper_arm_right": "central_rotator_rotation",
    "elbow_right": "rotation_elbow",
    "forearm_right": "rotate_forearm",
    "wrist_right": "rotation_wrist_right",
    "shoulder_vertical_left": "rotation_inner_shoulder",
    "shoulder_horizontal_left": "rotation_shoulder_left_1",
    "upper_arm_left": "central_rotator_rotation_1",
    "elbow_left": "rotation_elbow_1",
    "forearm_left": "rotate_forearm_1",
    "wrist_left": "rotation_wrist_left",
    "head_horizontal": "gearing_rotation__new_body_",
    "head_vertical": "nodding_rotation",
}

RIGHT_ARM_JOINTS = (
    "shoulder_vertical_right",
    "shoulder_horizontal_right",
    "upper_arm_right",
    "elbow_right",
    "forearm_right",
    "wrist_right",
)
LEFT_ARM_JOINTS = (
    "shoulder_vertical_left",
    "shoulder_horizontal_left",
    "upper_arm_left",
    "elbow_left",
    "forearm_left",
    "wrist_left",
)
HEAD_JOINTS = ("head_horizontal", "head_vertical")

# Fixed camera frame relative to urdf_head (unchanged from the pre-CAD idealized model).
CAMERA_FIXED_ORIGIN = (
    "-0.04740000000000000463 0.0012679757240900979554 -0.04199999999999999567",
    "2.4502643717466608141e-31 1.570796326794896558 0",
)


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: str
    rpy: str
    axis: str
    limit_lower: str | None
    limit_upper: str | None


def _format_float(value: float) -> str:
    """Deterministic, compact decimal text for URDF attributes."""
    if value == 0.0:
        return "0"
    text = f"{value:.17g}"
    if "e" in text or "E" in text:
        return text
    if "." not in text:
        return text + ".0"
    return text.rstrip("0").rstrip(".") or "0"


def _format_vec3(values: np.ndarray) -> str:
    return " ".join(_format_float(float(v)) for v in values)


def _parse_vec3(text: str | None) -> np.ndarray:
    if not text:
        return np.zeros(3)
    return np.array([float(part) for part in text.split()], dtype=float)


def _load_directions(mapping_path: Path) -> dict[str, int]:
    payload = json.loads(mapping_path.read_text(encoding="utf-8"))
    by_cad: dict[str, int] = {}
    for entry in payload["joints"]:
        direction = entry.get("direction")
        if direction is not None:
            by_cad[entry["urdfJoint"]] = int(direction)
    directions: dict[str, int] = {}
    for sdk_name, cad_name in SDK_TO_CAD_JOINT.items():
        if cad_name not in by_cad:
            raise KeyError(f"mapping.json has no direction for CAD joint {cad_name!r}")
        directions[sdk_name] = by_cad[cad_name]
    return directions


def _parse_control_joints(control_path: Path) -> dict[str, ElementTree.Element]:
    root = ElementTree.parse(control_path).getroot()
    joints = root.findall("joint")
    by_name = {joint.get("name"): joint for joint in joints}
    missing = [
        cad
        for cad in SDK_TO_CAD_JOINT.values()
        if cad not in by_name
    ]
    if missing:
        raise KeyError(f"{control_path} missing joints: {missing}")
    return by_name


def _joint_spec(
    sdk_name: str,
    cad_joint: ElementTree.Element,
    parent_link: str,
    child_link: str,
    direction: int,
) -> JointSpec:
    origin = cad_joint.find("origin")
    axis_el = cad_joint.find("axis")
    axis = _parse_vec3(axis_el.get("xyz") if axis_el is not None else "0 0 1")
    if direction == -1:
        axis = -axis
    joint_type = cad_joint.get("type", "revolute")
    limit = cad_joint.find("limit")
    lower = limit.get("lower") if limit is not None and limit.get("lower") is not None else None
    upper = limit.get("upper") if limit is not None and limit.get("upper") is not None else None
    # Pinocchio models URDF continuous joints with two configuration coordinates; the SDK
    # chain loader expects one revolute DOF per joint. Kinematics match revolute on [-pi, pi].
    if joint_type == "continuous":
        joint_type = "revolute"
        if lower is None or upper is None:
            lower = _format_float(-np.pi)
            upper = _format_float(np.pi)
    return JointSpec(
        name=sdk_name,
        joint_type=joint_type,
        parent=parent_link,
        child=child_link,
        xyz=origin.get("xyz") or "0 0 0",
        rpy=origin.get("rpy") or "0 0 0",
        axis=_format_vec3(axis),
        limit_lower=lower,
        limit_upper=upper,
    )


def _emit_link(name: str) -> str:
    return f'<link name="{name}"></link>\n'


def _emit_joint(spec: JointSpec) -> str:
    lines = [
        f'<joint name="{spec.name}" type="{spec.joint_type}">',
        f'<origin xyz="{spec.xyz}" rpy="{spec.rpy}"/>',
        f"<parent link=\"{spec.parent}\"/>",
        f"<child link=\"{spec.child}\"/>",
        f'<axis xyz="{spec.axis}"/>',
    ]
    if spec.limit_lower is not None and spec.limit_upper is not None:
        lines.append(
            f'<limit effort="1" velocity="20" lower="{spec.limit_lower}" upper="{spec.limit_upper}"/>'
        )
    lines.append("</joint>\n")
    return "\n".join(lines)


def _emit_fixed_joint(
    name: str, parent: str, child: str, xyz: str, rpy: str
) -> str:
    return (
        f'<joint name="{name}" type="fixed">\n'
        f'<origin xyz="{xyz}" rpy="{rpy}"/>\n'
        f'<parent link="{parent}"/>\n'
        f'<child link="{child}"/>\n'
        "</joint>\n"
    )


def _arm_link_names(side: str) -> tuple[str, ...]:
    prefix = "right" if side == "right" else "left"
    tip = "urdf_palm_right" if side == "right" else "urdf_palm_left"
    return (
        f"urdf_{prefix}_shoulder_vertical",
        f"urdf_{prefix}_shoulder_horizontal",
        f"urdf_{prefix}_upper_arm",
        f"urdf_{prefix}_elbow",
        f"urdf_{prefix}_forearm",
        tip,
    )


def build_urdf_xml(control_path: Path, mapping_path: Path) -> str:
    cad_joints = _parse_control_joints(control_path)
    directions = _load_directions(mapping_path)

    parts: list[str] = ['<?xml version="1.0"?>\n', '<robot name="pib">\n']
    parts.append(_emit_link("base_link"))
    parts.append(_emit_link("urdf_body"))
    parts.append(
        _emit_fixed_joint("base_link_to_base", "base_link", "urdf_body", "0 0 0", "0 0 0")
    )

    parts.append(_emit_link("urdf_head_base"))
    parts.append(_emit_link("urdf_head"))
    parts.append(_emit_link("urdf_camera_link"))

    head_specs = [
        _joint_spec(
            "head_horizontal",
            cad_joints["gearing_rotation__new_body_"],
            "urdf_body",
            "urdf_head_base",
            directions["head_horizontal"],
        ),
        _joint_spec(
            "head_vertical",
            cad_joints["nodding_rotation"],
            "urdf_head_base",
            "urdf_head",
            directions["head_vertical"],
        ),
    ]
    for spec in head_specs:
        parts.append(_emit_joint(spec))
    cam_xyz, cam_rpy = CAMERA_FIXED_ORIGIN
    parts.append(
        _emit_fixed_joint("camera", "urdf_head", "urdf_camera_link", cam_xyz, cam_rpy)
    )

    joint_count = len(head_specs) + 1  # + camera fixed not counted as chain

    for side, sdk_joints in (("left", LEFT_ARM_JOINTS), ("right", RIGHT_ARM_JOINTS)):
        link_names = _arm_link_names(side)
        for link in link_names[:-1]:
            parts.append(_emit_link(link))
        parts.append(_emit_link(link_names[-1]))

        parent = "urdf_body"
        for sdk_name, child in zip(sdk_joints, link_names, strict=True):
            cad_name = SDK_TO_CAD_JOINT[sdk_name]
            spec = _joint_spec(
                sdk_name,
                cad_joints[cad_name],
                parent,
                child,
                directions[sdk_name],
            )
            parts.append(_emit_joint(spec))
            parent = child
            joint_count += 1

    parts.append("</robot>\n")
    return "".join(parts)


def generate(
    control_path: Path,
    mapping_path: Path,
    output_path: Path,
) -> tuple[int, int]:
    xml_text = build_urdf_xml(control_path, mapping_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(xml_text, encoding="utf-8", newline="\n")
    link_count = xml_text.count("<link ")
    joint_count = xml_text.count("<joint ")
    return link_count, joint_count


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--control-urdf",
        type=Path,
        default=DEFAULT_CONTROL_URDF,
        help=f"CAD control URDF (default: {DEFAULT_CONTROL_URDF})",
    )
    parser.add_argument(
        "--mapping",
        type=Path,
        default=DEFAULT_MAPPING,
        help=f"Joint mapping JSON (default: {DEFAULT_MAPPING})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Output SDK URDF (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Exit 1 if the output file differs from what would be generated.",
    )
    args = parser.parse_args(argv)

    if not args.control_urdf.is_file():
        print(f"Control URDF not found: {args.control_urdf}", file=sys.stderr)
        return 1
    if not args.mapping.is_file():
        print(f"Mapping not found: {args.mapping}", file=sys.stderr)
        return 1

    generated = build_urdf_xml(args.control_urdf, args.mapping)
    if args.check:
        if not args.output.is_file():
            print(f"Missing output file: {args.output}", file=sys.stderr)
            return 1
        committed = args.output.read_text(encoding="utf-8")
        if committed != generated:
            print(
                f"{args.output} is out of date; run tools/generate_control_urdf.py",
                file=sys.stderr,
            )
            return 1
        print(f"OK: {args.output} matches generated control model.")
        return 0

    links, joints = generate(args.control_urdf, args.mapping, args.output)
    print(
        f"Wrote {args.output} ({links} links, {joints} joints) "
        f"from {args.control_urdf.name} + {args.mapping.name}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
