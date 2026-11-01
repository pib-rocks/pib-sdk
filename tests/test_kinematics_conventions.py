"""Pin frame conventions and rad/deg wrapper consistency for pib_sdk.kinematics."""

import inspect
import json
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import numpy as np
import pytest

from pib_sdk.kinematics import (
    FK,
    IK,
    ArmKinematics,
    HeadKinematics,
    camera_pose,
    camera_pose_deg,
    fk,
    fk_deg,
    get_hand_position_xyz_deg,
    ik,
    ik_deg,
    pose_from_xyz_rpy,
    pose_from_xyz_rpy_deg,
    pose_to_xyz_rpy,
    pose_to_xyz_rpy_deg,
)
from pib_sdk.robot_model import ArmSide

# The bundled model is generated from these two files (tools/generate_control_urdf.py).
REPO_ROOT = Path(__file__).resolve().parents[1]
PIB3D_ROOT = REPO_ROOT.parent / "pib-3d"
BUNDLED_URDF = REPO_ROOT / "src" / "pib_sdk" / "data" / "pib_model.urdf"
CAD_MODEL_URDF = PIB3D_ROOT / "urdf" / "pib.control.urdf"
CAD_MAPPING = PIB3D_ROOT / "joints" / "mapping.json"

SDK_TO_CAD_JOINT = {
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

# Mapping motor name -> bundled SDK joint name, where they differ.
SDK_JOINT_BY_MOTOR = {
    "upper_arm_left_rotation": "upper_arm_left",
    "upper_arm_right_rotation": "upper_arm_right",
    "lower_arm_left_rotation": "forearm_left",
    "lower_arm_right_rotation": "forearm_right",
    "turn_head_motor": "head_horizontal",
    "tilt_forward_motor": "head_vertical",
}

ZERO_Q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Measured zero pose (mm) on the CAD-derived bundled URDF (PR-2158).
ZERO_PALM_MM = {
    ArmSide.LEFT: np.array([509.2, 105.8, 711.8]),
    ArmSide.RIGHT: np.array([-609.1, 106.2, 731.7]),
}
ZERO_CAMERA_MM = np.array([-162.9, 59.3, 615.0])

# Shoulder mount offsets in the CAD export are not perfectly symmetric (~100 mm in X).


def _assert_proper_rotation(rotation: np.ndarray) -> None:
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
    assert np.linalg.det(rotation) > 0.0
    np.testing.assert_allclose(np.linalg.det(rotation), 1.0, atol=1e-9)


def _wrapper_pairs():
    """(rad_fn, deg_fn, sample_rad_args, sample_deg_args) for one-line deg wrappers."""
    arm = ArmKinematics(ArmSide.RIGHT)
    head = HeadKinematics()
    fk_legacy = FK("right")
    ik_legacy = IK("right")

    q_deg = [10.0, -20.0, 5.0, 30.0, -10.0, 0.0]
    q_rad = np.deg2rad(q_deg)
    target = arm.forward(q_rad)
    xyz, rpy_rad = pose_to_xyz_rpy(target)
    rpy_deg = np.rad2deg(rpy_rad)

    return [
        (lambda: fk("right", q_rad), lambda: fk_deg("right", q_deg), None, None),
        (
            lambda: ik("right", xyz=list(target.translation)),
            lambda: ik_deg("right", xyz=list(target.translation)),
            None,
            None,
        ),
        (lambda: arm.forward(q_rad), lambda: arm.forward_deg(q_deg), None, None),
        (
            lambda: arm.inverse(xyz=list(target.translation)),
            lambda: arm.inverse_deg(xyz=list(target.translation)),
            None,
            None,
        ),
        (
            lambda: head.camera_pose(np.deg2rad(10.0), np.deg2rad(-5.0)),
            lambda: head.camera_pose_deg(10.0, -5.0),
            None,
            None,
        ),
        (
            lambda: camera_pose(np.deg2rad(20.0), 0.0),
            lambda: camera_pose_deg(20.0, 0.0),
            None,
            None,
        ),
        (lambda: fk_legacy.pose(q_rad), lambda: fk_legacy.pose_deg(q_deg), None, None),
        (
            lambda: ik_legacy.solve(xyz=list(target.translation)),
            lambda: ik_legacy.solve_deg(xyz=list(target.translation)),
            None,
            None,
        ),
        (
            lambda: pose_from_xyz_rpy(xyz, rpy_rad),
            lambda: pose_from_xyz_rpy_deg(xyz, rpy_deg),
            None,
            None,
        ),
        (
            lambda: pose_to_xyz_rpy(target),
            lambda: pose_to_xyz_rpy_deg(target),
            None,
            None,
        ),
    ]


@pytest.mark.parametrize("rad_call,deg_call", [(p[0], p[1]) for p in _wrapper_pairs()])
def test_deg_wrapper_matches_rad_conversion(rad_call, deg_call):
    rad_result = rad_call()
    deg_result = deg_call()

    if isinstance(rad_result, tuple) and isinstance(deg_result, tuple):
        rad_a, rad_b = rad_result
        deg_a, deg_b = deg_result
        np.testing.assert_allclose(rad_a, deg_a, atol=1e-9)
        np.testing.assert_allclose(rad_b, np.deg2rad(deg_b), atol=1e-9)
        np.testing.assert_allclose(np.rad2deg(rad_b), deg_b, atol=1e-9)
        return

    if hasattr(rad_result, "translation"):
        np.testing.assert_allclose(
            rad_result.translation, deg_result.translation, atol=1e-9
        )
        np.testing.assert_allclose(rad_result.rotation, deg_result.rotation, atol=1e-9)
        return

    np.testing.assert_allclose(rad_result, np.deg2rad(deg_result), atol=1e-9)
    np.testing.assert_allclose(np.rad2deg(rad_result), deg_result, atol=1e-9)


def test_deg_wrappers_are_one_line_delegates():
    import pib_sdk.kinematics as kin

    deg_names = (
        "fk_deg",
        "ik_deg",
        "camera_pose_deg",
        "pose_from_xyz_rpy_deg",
        "pose_to_xyz_rpy_deg",
    )
    for name in deg_names:
        fn = getattr(kin, name)
        source = inspect.getsource(fn)
        return_lines = [
            line.strip()
            for line in source.splitlines()
            if line.strip().startswith("return ")
        ]
        assert (
            len(return_lines) == 1
        ), f"{name} should delegate in one return, got: {return_lines}"


def test_get_hand_position_xyz_deg_matches_rad_path():
    from pib_sdk.kinematics import get_hand_position_xyz

    class FakeTelemetry:
        def get_positions_deg(self, names):
            return {n: 0.0 for n in names}

    fake = FakeTelemetry()
    assert get_hand_position_xyz_deg("right", telemetry=fake) == get_hand_position_xyz(
        "right", telemetry=fake
    )


@pytest.mark.parametrize("side", [ArmSide.LEFT, ArmSide.RIGHT])
def test_zero_pose_palm_positions(side):
    pose = fk(side, ZERO_Q)
    np.testing.assert_allclose(pose.translation, ZERO_PALM_MM[side], atol=0.5)


def test_zero_pose_camera():
    np.testing.assert_allclose(camera_pose().translation, ZERO_CAMERA_MM, atol=0.5)


@pytest.mark.parametrize("side", [ArmSide.LEFT, ArmSide.RIGHT])
def test_palm_rotation_is_proper_orthonormal(side):
    _assert_proper_rotation(fk(side, ZERO_Q).rotation)


def test_camera_rotation_is_proper_orthonormal():
    _assert_proper_rotation(camera_pose().rotation)


def test_zero_pose_hands_reach_outward_at_shoulder_height():
    """At motor zero the arms point out to the sides, just below the shoulder.

    Since the model carries the CAD geometry with the base at the foot point (PR-2158), the
    palm's z is the shoulder height (~0.73 m), not ~0 as it was when the base sat in the body.
    Lateral reach and height are therefore both large; the sign of x says which side.
    """
    for side in (ArmSide.LEFT, ArmSide.RIGHT):
        x, _y, z = fk(side, ZERO_Q).translation
        assert abs(x) > 400.0, f"{side}: palm only {abs(x):.0f} mm out to the side"
        assert 600.0 < z < 800.0, f"{side}: palm at {z:.0f} mm, expected shoulder height"


def test_left_palm_on_positive_x_right_on_negative_x():
    left_x = fk(ArmSide.LEFT, ZERO_Q).translation[0]
    right_x = fk(ArmSide.RIGHT, ZERO_Q).translation[0]
    assert left_x > 0.0
    assert right_x < 0.0


def test_camera_zero_position_regression():
    np.testing.assert_allclose(
        camera_pose().translation, ZERO_CAMERA_MM, atol=0.5
    )


def test_camera_optical_axis_points_away_from_the_body():
    # The Onshape export has the camera frame rotated by 180 degrees; the correction
    # in robot_model (CAMERA_FRAME_CORRECTION) fixes it until PR-2045 repairs the
    # export itself. Optical +Z must point away from the body on the -Y front.
    pose = camera_pose()
    optical_in_base = pose.rotation @ np.array([0.0, 0.0, 1.0])
    axis_xy = optical_in_base[:2] / np.linalg.norm(optical_in_base[:2])
    toward_body_xy = -pose.translation[:2] / np.linalg.norm(pose.translation[:2])
    assert np.dot(axis_xy, toward_body_xy) < 0.0


def test_bundled_axes_follow_the_mapping_directions():
    """Every driven joint must carry the CAD axis, negated where the mapping says -1.

    The arms are not mirror-symmetric in the CAD (the two shoulder mountings sit 148 mm apart
    in x), so "the same joint vector mirrors the palms" is not a property this model has - it
    held at one pose and by luck. What must hold is the sign convention itself: the generator
    bakes `direction` into the axis, and a wrong direction silently turns one arm against the
    other. This checks the baked result against joints/mapping.json.
    """
    if not CAD_MODEL_URDF.exists() or not CAD_MAPPING.exists():
        pytest.skip("pib-3d CAD model not available next to this checkout")
    cad_root = ElementTree.parse(CAD_MODEL_URDF).getroot()
    cad_axes = {}
    for joint in cad_root.findall("joint"):
        axis = joint.find("axis")
        if axis is not None:
            cad_axes[joint.get("name")] = np.array(
                [float(v) for v in axis.get("xyz").split()]
            )

    sdk_root = ElementTree.parse(BUNDLED_URDF).getroot()
    sdk_axes = {}
    for joint in sdk_root.findall("joint"):
        axis = joint.find("axis")
        if axis is not None:
            sdk_axes[joint.get("name")] = np.array([float(v) for v in axis.get("xyz").split()])

    mapping = json.loads(CAD_MAPPING.read_text())["joints"]
    checked = 0
    for entry in mapping:
        if entry["class"] != "driven" or entry["direction"] is None:
            continue
        sdk_joint = SDK_JOINT_BY_MOTOR.get(entry["motor"], entry["motor"])
        cad_joint = SDK_TO_CAD_JOINT.get(sdk_joint)
        if cad_joint is None:
            continue
        expected = cad_axes[cad_joint] * entry["direction"]
        actual = sdk_axes[sdk_joint]
        np.testing.assert_allclose(
            actual,
            expected,
            atol=1e-9,
            err_msg=f"{entry['motor']}: bundled axis {actual} != CAD {cad_axes[cad_joint]} "
            f"x direction {entry['direction']}",
        )
        checked += 1

    assert checked >= 12, f"only {checked} joints checked - the mapping or the name table changed"

