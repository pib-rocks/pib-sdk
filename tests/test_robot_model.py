"""Tests for pib_sdk.robot_model: URDF loading, chain reduction and validation."""

import shutil
from dataclasses import replace

import numpy as np
import pytest

from pib_sdk import robot_model
from pib_sdk.robot_model import (
    DEFAULT_URDF_PATH,
    HEAD,
    LEFT_ARM,
    RIGHT_ARM,
    ArmSide,
    ChainDefinition,
    build_chain_model,
    coerce_arm_side,
    get_arm_model,
    get_chain_model,
    get_head_model,
    set_urdf_path,
)

ARM_DEFINITIONS = [RIGHT_ARM, LEFT_ARM]


def test_bundled_urdf_ships_with_package():
    assert DEFAULT_URDF_PATH.is_file(), "pib_model.urdf must be packaged with pib_sdk"


@pytest.mark.parametrize("definition", ARM_DEFINITIONS + [HEAD], ids=lambda d: d.name)
def test_chain_loads_with_expected_joints(definition):
    chain = get_chain_model(definition)
    assert chain.degrees_of_freedom == len(definition.urdf_joint_names)
    assert chain.joint_names == definition.urdf_joint_names
    assert chain.motor_names == definition.motor_names
    # Reduced model keeps the base-to-tip order of the definition.
    assert tuple(chain.model.names[1:]) == definition.urdf_joint_names


@pytest.mark.parametrize("side", [ArmSide.RIGHT, ArmSide.LEFT, "right", "LEFT"])
def test_get_arm_model_accepts_enum_and_string(side):
    chain = get_arm_model(side)
    assert chain.degrees_of_freedom == 6


def test_get_arm_model_rejects_unknown_side():
    with pytest.raises(ValueError, match="Unknown arm side"):
        get_arm_model("torso")


def test_coerce_arm_side_normalises_strings():
    assert coerce_arm_side("Right") is ArmSide.RIGHT
    assert coerce_arm_side(ArmSide.LEFT) is ArmSide.LEFT


@pytest.mark.parametrize("definition", ARM_DEFINITIONS, ids=lambda d: d.name)
def test_arm_joint_limits_match_urdf(definition):
    lower_deg, upper_deg = get_chain_model(definition).joint_limits_deg
    np.testing.assert_allclose(lower_deg, [-90.0, -90.0, -90.0, -45.0, -90.0, -30.0])
    np.testing.assert_allclose(upper_deg, [90.0, 90.0, 90.0, 90.0, 90.0, 30.0])


def test_models_are_cached_and_scaled_to_millimetres():
    first = get_arm_model(ArmSide.RIGHT)
    assert get_arm_model(ArmSide.RIGHT) is first
    # In millimetres the shoulder joint sits hundreds of units from the base.
    shoulder_offset = np.linalg.norm(first.model.jointPlacements[1].translation)
    assert 100.0 < shoulder_offset < 2000.0


def test_named_configurations_are_exposed_in_degrees():
    chain = get_arm_model(ArmSide.RIGHT)
    np.testing.assert_allclose(chain.named_configuration_deg("zero"), np.zeros(6))
    rest = chain.named_configuration_deg("rest")
    assert rest.shape == (6,)
    with pytest.raises(KeyError, match="available"):
        chain.named_configuration_deg("does-not-exist")


def test_named_configurations_respect_joint_limits():
    for definition in ARM_DEFINITIONS + [HEAD]:
        chain = get_chain_model(definition)
        lower_deg, upper_deg = chain.joint_limits_deg
        for name in definition.named_configurations_deg:
            configuration = chain.named_configuration_deg(name)
            assert np.all(configuration >= lower_deg - 1e-9), f"{definition.name}:{name}"
            assert np.all(configuration <= upper_deg + 1e-9), f"{definition.name}:{name}"


def test_definition_validates_motor_name_count():
    with pytest.raises(ValueError, match="same length"):
        ChainDefinition(
            name="broken",
            urdf_joint_names=("a", "b"),
            motor_names=("only_one",),
            tip_frame="base_link",
        )


def test_missing_joint_raises_with_available_names():
    bad = replace(RIGHT_ARM, urdf_joint_names=RIGHT_ARM.urdf_joint_names[:-1] + ("no_such",))
    with pytest.raises(ValueError, match="no_such"):
        build_chain_model(bad)


def test_missing_tip_frame_raises_with_available_names():
    bad = replace(RIGHT_ARM, tip_frame="no_such_frame")
    with pytest.raises(ValueError, match="no_such_frame"):
        build_chain_model(bad)


def test_missing_urdf_file_raises_file_not_found(tmp_path):
    with pytest.raises(FileNotFoundError, match="URDF not found"):
        build_chain_model(RIGHT_ARM, urdf_path=tmp_path / "nowhere.urdf")


def test_set_urdf_path_switches_the_source(tmp_path):
    copied = tmp_path / "copy.urdf"
    shutil.copy(DEFAULT_URDF_PATH, copied)
    try:
        set_urdf_path(copied)
        chain = get_head_model()
        assert chain.degrees_of_freedom == 2
        assert robot_model._resolve_urdf_path(None) == copied.resolve()
    finally:
        set_urdf_path(DEFAULT_URDF_PATH)
