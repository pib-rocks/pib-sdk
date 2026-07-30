"""Cross-check the package's forward kinematics against an independent implementation.

The reference below parses the bundled URDF with ``xml.etree`` and multiplies
homogeneous transforms with plain numpy -- no Pinocchio involved. Agreement to
machine precision on random configurations verifies both the URDF loading and
the metres-to-millimetres scaling of the packaged models.
"""

import xml.etree.ElementTree as ElementTree

import numpy as np
import pytest

from pib_sdk.kinematics import ChainKinematics
from pib_sdk.robot_model import (
    DEFAULT_URDF_PATH,
    HEAD,
    LEFT_ARM,
    MILLIMETRES_PER_METRE,
    RIGHT_ARM,
    get_chain_model,
)

CHAIN_DEFINITIONS = [RIGHT_ARM, LEFT_ARM, HEAD]
RANDOM_CONFIGURATIONS_PER_CHAIN = 25


# --------------------------------------------------------------------------- #
# Reference forward kinematics (URDF + numpy only)                             #
# --------------------------------------------------------------------------- #
def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy: fixed-axis XYZ rotations, i.e. ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    rotation_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rotation_z @ rotation_y @ rotation_x


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation about an arbitrary unit axis."""
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


class UrdfReferenceKinematics:
    """Minimal, independent FK over a URDF's joint tree."""

    def __init__(self, urdf_path):
        root = ElementTree.parse(urdf_path).getroot()
        self._joint_by_child_link = {}
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
            self._joint_by_child_link[joint.find("child").get("link")] = {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "origin": _homogeneous(_rpy_to_matrix(*rpy), np.array(xyz)),
                "axis": np.array(axis),
            }

    def _chain_to(self, tip_link: str, root_link: str = "base_link"):
        chain, link = [], tip_link
        while link != root_link:
            joint = self._joint_by_child_link[link]
            chain.append(joint)
            link = joint["parent"]
        return list(reversed(chain))

    def tip_pose(self, tip_link: str, angles_by_joint_name) -> np.ndarray:
        """Homogeneous base->tip transform (metres) for the given joint angles."""
        transform = np.eye(4)
        for joint in self._chain_to(tip_link):
            transform = transform @ joint["origin"]
            if joint["type"] == "revolute":
                angle = angles_by_joint_name.get(joint["name"], 0.0)
                transform = transform @ _homogeneous(
                    _axis_angle_matrix(joint["axis"], angle), np.zeros(3)
                )
        return transform


@pytest.fixture(scope="module")
def reference() -> UrdfReferenceKinematics:
    return UrdfReferenceKinematics(DEFAULT_URDF_PATH)


@pytest.mark.parametrize("definition", CHAIN_DEFINITIONS, ids=lambda d: d.name)
def test_forward_kinematics_matches_independent_reference(definition, reference):
    chain = get_chain_model(definition)
    kinematics = ChainKinematics(chain)
    lower, upper = chain.joint_lower_limits, chain.joint_upper_limits
    random_generator = np.random.default_rng(seed=7)

    for _ in range(RANDOM_CONFIGURATIONS_PER_CHAIN):
        q_rad = lower + (upper - lower) * random_generator.random(chain.degrees_of_freedom)
        pose = kinematics.forward(np.rad2deg(q_rad))

        expected = reference.tip_pose(
            definition.tip_frame, dict(zip(definition.urdf_joint_names, q_rad))
        )
        expected_translation_mm = expected[:3, 3] * MILLIMETRES_PER_METRE

        np.testing.assert_allclose(pose.translation, expected_translation_mm, atol=1e-9)
        np.testing.assert_allclose(pose.rotation, expected[:3, :3], atol=1e-12)
