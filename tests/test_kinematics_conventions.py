"""Pin frame conventions and rad/deg wrapper consistency for pib_sdk.kinematics."""

import inspect

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

ZERO_Q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Measured zero pose (mm), pinocchio 4.1 on bundled URDF (2026-10-23).
ZERO_PALM_MM = {
    ArmSide.LEFT: np.array([626.2, 27.5, 994.5]),
    ArmSide.RIGHT: np.array([-616.4, 27.7, 1030.6]),
}
ZERO_CAMERA_MM = np.array([-12.7, -82.8, 998.8])

MIRROR_X_TOLERANCE_MM = 10.0


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
        np.testing.assert_allclose(rad_result.translation, deg_result.translation, atol=1e-9)
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
            line.strip() for line in source.splitlines() if line.strip().startswith("return ")
        ]
        assert len(return_lines) == 1, f"{name} should delegate in one return, got: {return_lines}"


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


def test_up_is_positive_z():
    for side in (ArmSide.LEFT, ArmSide.RIGHT):
        assert fk(side, ZERO_Q).translation[2] > 0.0
    assert camera_pose().translation[2] > 0.0


def test_left_palm_on_positive_x_right_on_negative_x():
    left_x = fk(ArmSide.LEFT, ZERO_Q).translation[0]
    right_x = fk(ArmSide.RIGHT, ZERO_Q).translation[0]
    assert left_x > 0.0
    assert right_x < 0.0


def test_camera_on_negative_y_front_is_negative_y():
    assert camera_pose().translation[1] < 0.0


def test_camera_optical_axis_known_defect_pr_2045():
    # PR-2045: the model maps the camera +Z (optical) axis to +Y in the base
    # frame; it should point toward -Y (outward from the body). Document the
    # current wrong value so the fix is obvious.
    rotation = camera_pose().rotation
    optical_in_base = rotation @ np.array([0.0, 0.0, 1.0])
    np.testing.assert_allclose(optical_in_base, np.array([0.0, 1.0, 0.0]), atol=1e-2)


def test_mirrored_joints_mirror_palm_x():
    """The same joint vector on left vs right arms mirrors palm X (within tolerance)."""
    left_arm = ArmKinematics(ArmSide.LEFT)
    right_arm = ArmKinematics(ArmSide.RIGHT)
    q = np.deg2rad([15.0, -10.0, 20.0, -5.0, 12.0, 0.0])
    left_x = left_arm.forward(q).translation[0]
    right_x = right_arm.forward(q).translation[0]
    np.testing.assert_allclose(left_x, -right_x, atol=MIRROR_X_TOLERANCE_MM)
