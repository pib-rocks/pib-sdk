"""Interim PR-2055 forearm roll axis correction (load-time URDF patch)."""

import numpy as np
import pinocchio as pin
import pytest

from pib_sdk.robot_model import (
    DEFAULT_URDF_PATH,
    LEFT_ARM,
    MILLIMETRES_PER_METRE,
    RIGHT_ARM,
    _apply_camera_frame_correction,
    get_chain_model,
)

ARM_DEFINITIONS = [LEFT_ARM, RIGHT_ARM]
FOREARM_JOINT_INDEX = 4
ROLL_TEST_DEG = 30.0


def _build_chain_without_forearm_axis_patch(definition):
    """Same reduction and scaling as the SDK, but raw URDF roll axes."""
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
    for index in range(len(model.jointPlacements)):
        placement = model.jointPlacements[index]
        model.jointPlacements[index] = pin.SE3(
            placement.rotation, placement.translation * MILLIMETRES_PER_METRE
        )
    for frame in model.frames:
        frame.placement.translation *= MILLIMETRES_PER_METRE
    return model


def _joint_origins_mm(model, data, q):
    return {
        name: data.oMi[model.getJointId(name)].translation.copy()
        for name in model.names[1:]
    }


def _tip_pose_mm(chain, q):
    data = chain.create_data()
    pin.forwardKinematics(chain.model, data, q)
    pin.updateFramePlacements(chain.model, data)
    return data.oMf[chain.tip_frame_id].copy()


def _raw_urdf_forearm_axis_misalignment_deg(definition):
    """Angle between roll axis and forearm segment on disk URDF (pre-PR-2055 SDK)."""
    model = _build_chain_without_forearm_axis_patch(definition)
    data = model.createData()
    q = pin.neutral(model)
    pin.forwardKinematics(model, data, q)
    pin.computeJointJacobians(model, data, q)
    forearm_name = definition.urdf_joint_names[FOREARM_JOINT_INDEX]
    wrist_name = definition.urdf_joint_names[FOREARM_JOINT_INDEX + 1]
    forearm_id = model.getJointId(forearm_name)
    wrist_id = model.getJointId(wrist_name)
    segment = data.oMi[wrist_id].translation - data.oMi[forearm_id].translation
    segment /= np.linalg.norm(segment)
    velocity_index = model.idx_vs[forearm_id]
    jacobian = pin.getJointJacobian(model, data, forearm_id, pin.LOCAL_WORLD_ALIGNED)
    axis = jacobian[3:6, velocity_index]
    axis /= np.linalg.norm(axis)
    return float(np.degrees(np.arccos(np.clip(abs(np.dot(axis, segment)), 0.0, 1.0))))


def _wrist_displacement_mm_for_roll(chain, roll_deg):
    model = chain.model
    data0 = model.createData()
    q0 = pin.neutral(model)
    pin.forwardKinematics(model, data0, q0)
    wrist_name = chain.joint_names[FOREARM_JOINT_INDEX + 1]
    wrist_id = model.getJointId(wrist_name)
    p0 = data0.oMi[wrist_id].translation.copy()

    q1 = q0.copy()
    q1[FOREARM_JOINT_INDEX] = np.deg2rad(roll_deg)
    data1 = model.createData()
    pin.forwardKinematics(model, data1, q1)
    p1 = data1.oMi[wrist_id].translation
    return float(np.linalg.norm(p1 - p0))


@pytest.mark.parametrize("definition", ARM_DEFINITIONS, ids=lambda d: d.name)
def test_zero_pose_unchanged_by_forearm_axis_correction(definition):
    corrected = get_chain_model(definition)
    uncorrected = _build_chain_without_forearm_axis_patch(definition)
    q = pin.neutral(corrected.model)

    corrected_pose = _tip_pose_mm(corrected, q)
    data_unc = uncorrected.createData()
    pin.forwardKinematics(uncorrected, data_unc, q)
    pin.updateFramePlacements(uncorrected, data_unc)
    uncorrected_pose = data_unc.oMf[uncorrected.getFrameId(definition.tip_frame)].copy()

    np.testing.assert_allclose(
        corrected_pose.translation, uncorrected_pose.translation, atol=1e-9
    )
    np.testing.assert_allclose(
        corrected_pose.rotation, uncorrected_pose.rotation, atol=1e-12
    )

    data_cor = corrected.create_data()
    pin.forwardKinematics(corrected.model, data_cor, q)
    origins_cor = _joint_origins_mm(corrected.model, data_cor, q)
    origins_unc = _joint_origins_mm(uncorrected, data_unc, q)
    for joint_name in corrected.joint_names:
        np.testing.assert_allclose(
            origins_cor[joint_name], origins_unc[joint_name], atol=1e-9
        )


@pytest.mark.parametrize("definition", ARM_DEFINITIONS, ids=lambda d: d.name)
def test_forearm_roll_axis_follows_segment(definition):
    chain = get_chain_model(definition)
    model = chain.model
    data = model.createData()
    q = pin.neutral(model)
    pin.forwardKinematics(model, data, q)
    pin.computeJointJacobians(model, data, q)
    forearm_name = definition.urdf_joint_names[FOREARM_JOINT_INDEX]
    wrist_name = definition.urdf_joint_names[FOREARM_JOINT_INDEX + 1]
    forearm_id = model.getJointId(forearm_name)
    wrist_id = model.getJointId(wrist_name)
    segment = data.oMi[wrist_id].translation - data.oMi[forearm_id].translation
    segment /= np.linalg.norm(segment)
    velocity_index = model.idx_vs[forearm_id]
    jacobian = pin.getJointJacobian(model, data, forearm_id, pin.LOCAL_WORLD_ALIGNED)
    axis = jacobian[3:6, velocity_index]
    axis /= np.linalg.norm(axis)
    angle_deg = np.degrees(np.arccos(np.clip(abs(np.dot(axis, segment)), 0.0, 1.0)))
    assert angle_deg < 1.0


@pytest.mark.parametrize("definition", ARM_DEFINITIONS, ids=lambda d: d.name)
def test_forearm_roll_is_pure_rotation_about_segment(definition):
    chain = get_chain_model(definition)
    displacement_mm = _wrist_displacement_mm_for_roll(chain, ROLL_TEST_DEG)
    assert displacement_mm < 1.0

    model = chain.model
    q0 = pin.neutral(model)
    wrist_name = chain.joint_names[FOREARM_JOINT_INDEX + 1]
    wrist_id = model.getJointId(wrist_name)

    q_plus = q0.copy()
    q_plus[FOREARM_JOINT_INDEX] = np.deg2rad(ROLL_TEST_DEG)
    data_plus = model.createData()
    pin.forwardKinematics(model, data_plus, q_plus)
    p_plus = data_plus.oMi[wrist_id].translation.copy()

    q_back = q0.copy()
    q_back[FOREARM_JOINT_INDEX] = np.deg2rad(-ROLL_TEST_DEG)
    data_back = model.createData()
    pin.forwardKinematics(model, data_back, q_back)
    p_back = data_back.oMi[wrist_id].translation

    np.testing.assert_allclose(p_back, p_plus, atol=1e-6)


@pytest.mark.parametrize("definition", ARM_DEFINITIONS, ids=lambda d: d.name)
def test_forearm_joint_limits_unchanged(definition):
    corrected = get_chain_model(definition)
    uncorrected = _build_chain_without_forearm_axis_patch(definition)
    np.testing.assert_allclose(
        corrected.joint_lower_limits, uncorrected.lowerPositionLimit, atol=1e-15
    )
    np.testing.assert_allclose(
        corrected.joint_upper_limits, uncorrected.upperPositionLimit, atol=1e-15
    )


class _MinimalChain:
    def __init__(self, model, joint_names):
        self.model = model
        self.joint_names = joint_names


def test_raw_urdf_forearm_axis_misalignment_is_about_eleven_degrees():
    """Document measured pre-correction geometry (left 11.0°, right 10.7°)."""
    left = _raw_urdf_forearm_axis_misalignment_deg(LEFT_ARM)
    right = _raw_urdf_forearm_axis_misalignment_deg(RIGHT_ARM)
    assert 10.5 < left < 11.5
    assert 10.2 < right < 11.2
    corrected_left = _wrist_displacement_mm_for_roll(get_chain_model(LEFT_ARM), 30.0)
    corrected_right = _wrist_displacement_mm_for_roll(get_chain_model(RIGHT_ARM), 30.0)
    uncorrected_left = _wrist_displacement_mm_for_roll(
        _MinimalChain(
            _build_chain_without_forearm_axis_patch(LEFT_ARM),
            LEFT_ARM.urdf_joint_names,
        ),
        30.0,
    )
    assert uncorrected_left > 14.0
    assert corrected_left < 1.0
    assert corrected_right < 1.0
