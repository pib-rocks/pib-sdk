"""Tests for pib_sdk.kinematics on the bundled pib V3 URDF."""

import numpy as np
import pytest

from pib_sdk.kinematics import (
    FK,
    IK,
    ArmKinematics,
    HeadKinematics,
    camera_pose,
    fk,
    ik,
    pose_from_xyz_rpy,
    pose_to_xyz_rpy,
)
from pib_sdk.robot_model import ArmSide

SIDES = [ArmSide.RIGHT, ArmSide.LEFT]

# Zero-configuration palm positions (mm), independently verified against a
# plain-numpy walk of the URDF (see test_urdf_reference.py). In the URDF's
# zero pose both arms point straight out to the robot's sides.
ZERO_POSE_PALM_MM = {
    ArmSide.RIGHT: np.array([-616.409, 27.666, 1030.579]),
    ArmSide.LEFT: np.array([626.205, 27.466, 994.516]),
}
ZERO_POSE_CAMERA_MM = np.array([-12.660, -82.836, 998.819])


def _interior_configurations(arm: ArmKinematics, count: int, rng) -> list:
    """Sample joint vectors (degrees) comfortably inside the arm's limits."""
    lower_deg, upper_deg = arm.joint_limits_deg
    span = upper_deg - lower_deg
    low, high = lower_deg + 0.15 * span, upper_deg - 0.15 * span
    return [low + (high - low) * rng.random(arm.degrees_of_freedom) for _ in range(count)]


# --------------------------------------------------------------------------- #
# Forward kinematics                                                           #
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("side", SIDES)
def test_forward_kinematics_zero_pose_regression(side):
    pose = fk(side, [0, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(pose.translation, ZERO_POSE_PALM_MM[side], atol=1e-2)


@pytest.mark.parametrize("side", SIDES)
def test_forward_kinematics_rejects_wrong_joint_count(side):
    with pytest.raises(ValueError, match="Expected 6 joint values"):
        fk(side, [0, 0, 0])


def test_forward_kinematics_moves_with_the_shoulder():
    arm = ArmKinematics(ArmSide.RIGHT)
    at_zero = arm.forward([0, 0, 0, 0, 0, 0]).translation
    lifted = arm.forward([45, 0, 0, 0, 0, 0]).translation
    assert np.linalg.norm(lifted - at_zero) > 100.0  # the palm travels decimetres


# --------------------------------------------------------------------------- #
# Inverse kinematics                                                           #
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("side", SIDES)
def test_ik_position_only_roundtrip(side):
    arm = ArmKinematics(side)
    rng = np.random.default_rng(seed=1)
    for q_deg in _interior_configurations(arm, count=6, rng=rng):
        target = arm.forward(q_deg)
        solution_deg = arm.inverse(xyz=target.translation)
        reached = arm.forward(solution_deg)
        assert np.linalg.norm(reached.translation - target.translation) < 1e-2  # mm


@pytest.mark.parametrize("side", SIDES)
def test_ik_full_pose_roundtrip(side):
    arm = ArmKinematics(side)
    rng = np.random.default_rng(seed=2)
    for q_deg in _interior_configurations(arm, count=4, rng=rng):
        target = arm.forward(q_deg)
        xyz, rpy_deg = pose_to_xyz_rpy(target)
        solution_deg = arm.inverse(xyz=xyz, rpy_deg=rpy_deg)
        reached = arm.forward(solution_deg)
        assert np.linalg.norm(reached.translation - target.translation) < 1e-2  # mm
        rotation_error = reached.rotation.T @ target.rotation
        assert np.arccos(np.clip((np.trace(rotation_error) - 1) / 2, -1, 1)) < 1e-4  # rad


def test_ik_solutions_respect_joint_limits():
    arm = ArmKinematics(ArmSide.RIGHT)
    lower_deg, upper_deg = arm.joint_limits_deg
    rng = np.random.default_rng(seed=3)
    for q_deg in _interior_configurations(arm, count=4, rng=rng):
        solution_deg = arm.inverse(xyz=arm.forward(q_deg).translation)
        assert np.all(solution_deg >= lower_deg - 1e-6)
        assert np.all(solution_deg <= upper_deg + 1e-6)


def test_ik_unreachable_target_raises():
    arm = ArmKinematics(ArmSide.RIGHT)
    with pytest.raises(ValueError, match="failed to converge"):
        arm.inverse(xyz=[5000.0, 0.0, 0.0], restarts=2, max_iterations=50)


def test_ik_mask_constrains_selected_components_only():
    arm = ArmKinematics(ArmSide.RIGHT)
    target_xy = [-450.0, 80.0]
    solution_deg = arm.inverse(xyz=[target_xy[0], target_xy[1], 0.0], mask=[1, 1, 0, 0, 0, 0])
    reached = arm.forward(solution_deg).translation
    assert abs(reached[0] - target_xy[0]) < 1e-2
    assert abs(reached[1] - target_xy[1]) < 1e-2
    assert abs(reached[2]) > 100.0  # z was left free and stays near the arm


def test_ik_rejects_malformed_mask():
    with pytest.raises(ValueError, match="6 elements"):
        ik(ArmSide.RIGHT, xyz=[-400, 100, 900], mask=[1, 1, 1])


def test_ik_uses_the_initial_guess():
    arm = ArmKinematics(ArmSide.RIGHT)
    q_deg = np.array([20.0, -30.0, 10.0, 25.0, -40.0, 5.0])
    target = arm.forward(q_deg)
    solution_deg = arm.inverse(
        xyz=target.translation,
        rpy_deg=pose_to_xyz_rpy(target)[1],
        initial_guess_deg=q_deg + 1.0,
        restarts=0,
    )
    np.testing.assert_allclose(solution_deg, q_deg, atol=0.5)


# --------------------------------------------------------------------------- #
# Pose helpers                                                                 #
# --------------------------------------------------------------------------- #
def test_pose_helpers_roundtrip():
    xyz = [-350.0, 120.0, 900.0]
    rpy_deg = [10.0, -20.0, 30.0]
    pose = pose_from_xyz_rpy(xyz, rpy_deg)
    xyz_back, rpy_back = pose_to_xyz_rpy(pose)
    np.testing.assert_allclose(xyz_back, xyz, atol=1e-9)
    np.testing.assert_allclose(rpy_back, rpy_deg, atol=1e-9)


def test_pose_from_xyz_rpy_validates_shape():
    with pytest.raises(ValueError, match="exactly 3"):
        pose_from_xyz_rpy([1.0, 2.0])


# --------------------------------------------------------------------------- #
# Head and camera                                                              #
# --------------------------------------------------------------------------- #
def test_camera_pose_zero_regression():
    np.testing.assert_allclose(camera_pose().translation, ZERO_POSE_CAMERA_MM, atol=1e-2)


def test_camera_pose_pan_moves_the_camera():
    at_zero = camera_pose().translation
    panned = camera_pose(pan_deg=45.0).translation
    assert np.linalg.norm(panned - at_zero) > 10.0


def test_head_kinematics_exposes_motor_names():
    head = HeadKinematics()
    assert head.motor_names == ["turn_head_motor", "tilt_forward_motor"]
    assert head.degrees_of_freedom == 2


# --------------------------------------------------------------------------- #
# Backwards-compatible aliases                                                 #
# --------------------------------------------------------------------------- #
def test_legacy_fk_class_matches_new_api():
    legacy = FK("right")
    assert len(legacy.joint_names) == 6
    np.testing.assert_allclose(
        legacy.pose([0, 45, 0, 0, 90, 0]).translation,
        fk("right", [0, 45, 0, 0, 90, 0]).translation,
    )


def test_legacy_ik_class_solves_with_old_signature():
    arm = ArmKinematics(ArmSide.RIGHT)
    target = arm.forward([10, -20, 5, 30, -10, 0])
    solution_deg = IK("right").solve(xyz=target.translation, tol=1e-4, max_steps=200)
    reached = arm.forward(solution_deg)
    assert np.linalg.norm(reached.translation - target.translation) < 1e-2
