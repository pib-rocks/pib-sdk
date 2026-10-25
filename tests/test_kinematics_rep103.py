"""REP-103 frame conversion and effective joint-axis conventions (PR-2044)."""

import numpy as np
import pinocchio as pin
import pytest

from pib_sdk.kinematics import from_rep103, to_rep103
from pib_sdk.robot_model import ArmSide, get_arm_model, get_head_model

MODEL_FORWARD = np.array([0.0, -1.0, 0.0])
MODEL_LEFT = np.array([1.0, 0.0, 0.0])
MODEL_UP = np.array([0.0, 0.0, 1.0])


def _effective_rotation_axes_at_zero(chain) -> dict[str, np.ndarray]:
    """Unit rotation axes in the model base frame via ``computeJointJacobians``."""
    model = chain.model
    data = model.createData()
    q = pin.neutral(model)
    pin.forwardKinematics(model, data, q)
    pin.computeJointJacobians(model, data, q)
    axes: dict[str, np.ndarray] = {}
    for joint_name in chain.joint_names:
        joint_id = model.getJointId(joint_name)
        jacobian = pin.getJointJacobian(model, data, joint_id, pin.LOCAL_WORLD_ALIGNED)
        velocity_index = model.idx_vs[joint_id]
        angular = jacobian[3:6, velocity_index]
        axes[joint_name] = angular / np.linalg.norm(angular)
    return axes


@pytest.mark.parametrize(
    "point",
    [
        np.array([626.2, 27.5, 994.5]),
        np.array([0.0, -82.8, 998.8]),
        np.array([-400.0, 100.0, 900.0]),
    ],
)
def test_to_from_rep103_point_roundtrip(point):
    np.testing.assert_allclose(from_rep103(to_rep103(point)), point, atol=1e-9)


def test_to_from_rep103_pose_roundtrip():
    rotation = pin.utils.rotate("y", 0.3) @ pin.utils.rotate("x", -0.2)
    pose = pin.SE3(rotation, np.array([100.0, -50.0, 800.0]))
    restored = from_rep103(to_rep103(pose))
    np.testing.assert_allclose(restored.translation, pose.translation, atol=1e-9)
    np.testing.assert_allclose(restored.rotation, pose.rotation, atol=1e-9)


def test_rep103_axes_match_convention():
    """REP-103 X forward, Y robot-left, Z up — even though the model frame does not."""
    rep_forward = to_rep103(MODEL_FORWARD)
    rep_left = to_rep103(MODEL_LEFT)
    rep_up = to_rep103(MODEL_UP)
    np.testing.assert_allclose(rep_forward, np.array([1.0, 0.0, 0.0]), atol=1e-9)
    np.testing.assert_allclose(rep_left, np.array([0.0, 1.0, 0.0]), atol=1e-9)
    np.testing.assert_allclose(rep_up, np.array([0.0, 0.0, 1.0]), atol=1e-9)


def test_effective_joint_axes_not_all_world_z():
    """URDF ``axis`` tags are local; composed transforms tilt several joints."""
    left_axes = _effective_rotation_axes_at_zero(get_arm_model(ArmSide.LEFT))
    np.testing.assert_allclose(
        left_axes["elbow_left"], np.array([0.0, -1.0, 0.0]), atol=1e-4
    )
    np.testing.assert_allclose(
        left_axes["forearm_left"],
        np.array([-np.sqrt(0.5), 0.0, -np.sqrt(0.5)]),
        atol=1e-4,
    )

    head_axes = _effective_rotation_axes_at_zero(get_head_model())
    np.testing.assert_allclose(
        head_axes["head_vertical"], np.array([-1.0, 0.0029, 0.0]), atol=1e-3
    )
