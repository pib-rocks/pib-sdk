"""Cross-check SDK forward kinematics against plain Pinocchio (no SDK chain builder)."""

import numpy as np
import pinocchio as pin
import pytest

from pib_sdk.kinematics import camera_pose, fk
from pib_sdk.robot_model import (
    DEFAULT_URDF_PATH,
    HEAD,
    LEFT_ARM,
    MILLIMETRES_PER_METRE,
    RIGHT_ARM,
    ArmSide,
    ChainDefinition,
    _apply_camera_frame_correction,
)

CHAIN_CASES = [
    (RIGHT_ARM, lambda q: fk(ArmSide.RIGHT, q)),
    (LEFT_ARM, lambda q: fk(ArmSide.LEFT, q)),
    (HEAD, lambda q: camera_pose(q[0], q[1])),
]

RANDOM_SAMPLES_PER_CHAIN = 20


def _scale_model_to_millimetres(model: pin.Model) -> None:
    for index in range(len(model.jointPlacements)):
        placement = model.jointPlacements[index]
        model.jointPlacements[index] = pin.SE3(
            placement.rotation, placement.translation * MILLIMETRES_PER_METRE
        )
    for frame in model.frames:
        frame.placement.translation *= MILLIMETRES_PER_METRE


def _plain_pinocchio_tip_pose(
    definition: ChainDefinition, q_rad: np.ndarray
) -> pin.SE3:
    """Reduced chain FK using only ``pin.buildModelFromUrdf`` and friends.

    The SDK's :func:`pib_sdk.robot_model.build_chain_model` adds validation,
    caching, and the same reduction rules; this path exists to show the tip
    pose at ``definition.tip_frame`` matches when those extras are applied
    identically by hand.
    """
    full_model = pin.buildModelFromUrdf(str(DEFAULT_URDF_PATH))
    _apply_camera_frame_correction(full_model)
    chain_joint_ids = {
        full_model.getJointId(name) for name in definition.urdf_joint_names
    }
    joints_to_lock = [
        joint_id
        for joint_id in range(1, full_model.njoints)
        if joint_id not in chain_joint_ids
    ]
    model = pin.buildReducedModel(full_model, joints_to_lock, pin.neutral(full_model))
    _scale_model_to_millimetres(model)
    data = model.createData()
    tip_frame_id = model.getFrameId(definition.tip_frame)
    pin.forwardKinematics(model, data, q_rad)
    pin.updateFramePlacement(model, data, tip_frame_id)
    return data.oMf[tip_frame_id].copy()


@pytest.mark.parametrize(
    "definition,sdk_fk",
    CHAIN_CASES,
    ids=[definition.name for definition, _ in CHAIN_CASES],
)
def test_sdk_forward_kinematics_matches_plain_pinocchio(definition, sdk_fk):
    from pib_sdk.robot_model import get_chain_model

    chain = get_chain_model(definition)
    lower, upper = chain.joint_lower_limits, chain.joint_upper_limits
    rng = np.random.default_rng(seed=11)

    for _ in range(RANDOM_SAMPLES_PER_CHAIN):
        q_rad = lower + (upper - lower) * rng.random(chain.degrees_of_freedom)
        sdk_pose = sdk_fk(q_rad)
        plain_pose = _plain_pinocchio_tip_pose(definition, q_rad)
        assert np.linalg.norm(sdk_pose.translation - plain_pose.translation) < 0.1
        np.testing.assert_allclose(sdk_pose.rotation, plain_pose.rotation, atol=1e-9)
