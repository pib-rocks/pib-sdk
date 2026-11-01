"""Verify the bundled SDK URDF matches the CAD control model kinematics.

Forward kinematics from ``pib_model.urdf`` must agree with an independent FK walk of
``pib-3d/urdf/pib.control.urdf`` (plain numpy, no Pinocchio) when chain joint angles
are mapped through ``joints/mapping.json`` sign conventions.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import numpy as np
import pytest

from pib_sdk.kinematics import ArmKinematics, fk_deg
from pib_sdk.robot_model import LEFT_ARM, RIGHT_ARM

CAD_CONTROL_URDF = Path("/home/pib/work/pib-3d/urdf/pib.control.urdf")
CAD_MAPPING = Path("/home/pib/work/pib-3d/joints/mapping.json")

CAD_TIP_BY_SIDE = {
    "right": "d01_r_palm_right",
    "left": "d01_l_palm_left",
}

SDK_TO_CAD = {
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
}

POSES_DEG = {
    "zero": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "observe_right": RIGHT_ARM.named_configurations_deg["observe"],
    "rest_right": RIGHT_ARM.named_configurations_deg["rest"],
    "observe_left": LEFT_ARM.named_configurations_deg["observe"],
    "rest_left": LEFT_ARM.named_configurations_deg["rest"],
    "sample": (10.0, -20.0, 15.0, -25.0, 30.0, 5.0),
}

TOLERANCE_MM = 0.5


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    rotation_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rotation_z @ rotation_y @ rotation_x


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    skew = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def _homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


class CadControlReferenceKinematics:
    """Minimal FK over the CAD control URDF from ``root`` to a tip link."""

    def __init__(self, urdf_path: Path, mapping_path: Path) -> None:
        if not urdf_path.is_file():
            pytest.skip(f"CAD control URDF not available: {urdf_path}")
        if not mapping_path.is_file():
            pytest.skip(f"CAD mapping not available: {mapping_path}")

        payload = json.loads(mapping_path.read_text(encoding="utf-8"))
        self._direction_by_cad = {
            entry["urdfJoint"]: int(entry["direction"])
            for entry in payload["joints"]
            if entry.get("direction") is not None
        }

        root = ElementTree.parse(urdf_path).getroot()
        self._joint_by_child: dict[str, dict] = {}
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy") or "0 0 0").split()]
            axis_element = joint.find("axis")
            axis = (
                [float(v) for v in axis_element.get("xyz").split()]
                if axis_element is not None
                else [0.0, 0.0, 1.0]
            )
            self._joint_by_child[joint.find("child").get("link")] = {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "origin": _homogeneous(_rpy_to_matrix(*rpy), np.array(xyz)),
                "axis": np.array(axis),
            }

    def _chain_to(self, tip_link: str, root_link: str = "root") -> list[dict]:
        chain, link = [], tip_link
        while link != root_link:
            joint = self._joint_by_child[link]
            chain.append(joint)
            link = joint["parent"]
        return list(reversed(chain))

    def tip_translation_mm(
        self,
        tip_link: str,
        sdk_joint_names: tuple[str, ...],
        q_deg: tuple[float, ...],
    ) -> np.ndarray:
        angles_rad = dict(zip(sdk_joint_names, np.deg2rad(q_deg), strict=True))
        cad_angles: dict[str, float] = {}
        for sdk_name, angle in angles_rad.items():
            cad_name = SDK_TO_CAD[sdk_name]
            cad_angles[cad_name] = self._direction_by_cad[cad_name] * angle

        transform = np.eye(4)
        for joint in self._chain_to(tip_link):
            transform = transform @ joint["origin"]
            if joint["type"] in ("revolute", "continuous"):
                angle = cad_angles.get(joint["name"], 0.0)
                transform = transform @ _homogeneous(
                    _axis_angle_matrix(joint["axis"], angle), np.zeros(3)
                )
        return transform[:3, 3] * 1000.0


@pytest.fixture(scope="module")
def cad_reference() -> CadControlReferenceKinematics:
    return CadControlReferenceKinematics(CAD_CONTROL_URDF, CAD_MAPPING)


@pytest.mark.parametrize(
    ("side", "pose_name", "q_deg"),
    [
        ("right", "zero", POSES_DEG["zero"]),
        ("right", "observe", POSES_DEG["observe_right"]),
        ("right", "rest", POSES_DEG["rest_right"]),
        ("right", "sample", POSES_DEG["sample"]),
        ("left", "zero", POSES_DEG["zero"]),
        ("left", "observe", POSES_DEG["observe_left"]),
        ("left", "rest", POSES_DEG["rest_left"]),
        (
            "left",
            "sample",
            tuple(-value for value in POSES_DEG["sample"][:2])
            + POSES_DEG["sample"][2:],
        ),
    ],
)
def test_sdk_fk_matches_cad_control_model(
    side: str, pose_name: str, q_deg: tuple[float, ...], cad_reference
) -> None:
    definition = RIGHT_ARM if side == "right" else LEFT_ARM
    sdk_mm = fk_deg(side, list(q_deg)).translation
    cad_mm = cad_reference.tip_translation_mm(
        CAD_TIP_BY_SIDE[side], definition.urdf_joint_names, q_deg
    )
    delta = float(np.linalg.norm(sdk_mm - cad_mm))
    assert delta < TOLERANCE_MM, (
        f"{side} {pose_name}: SDK vs CAD FK differ by {delta:.3f} mm "
        f"(sdk={sdk_mm}, cad={cad_mm})"
    )
