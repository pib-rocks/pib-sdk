"""Offline tests for pib_sdk.control (no rosbridge connection required).

These cover the pure argument-parsing helpers behind ``Write.move`` /
``Write.set``, the command-line parser, and the naming contract between the
kinematics chains and the control motor groups.
"""

import pytest

from conftest import patch_roslibpy
from pib_sdk import control
from pib_sdk.control import (
    All,
    Write,
    _build_argument_parser,
    _degrees_to_internal_units,
    _expand_motor_specs,
    _expand_token,
    _parse_uniform_move,
    _parse_vector_move,
    default,
    left_arm,
    open_right_hand,
    right_arm,
    right_hand,
)
from pib_sdk.robot_model import HEAD, LEFT_ARM, RIGHT_ARM

ARM_JOINT_COUNT = 6


@pytest.fixture
def fake_roslibpy(monkeypatch):
    return patch_roslibpy(monkeypatch, control)


def test_write_supports_the_context_manager_protocol(fake_roslibpy):
    with Write(host="localhost") as writer:
        assert isinstance(writer, Write)


def test_write_close_is_safe_to_call_more_than_once(fake_roslibpy):
    writer = Write(host="localhost")
    writer.close()
    writer.close()


# --------------------------------------------------------------------------- #
# Kinematics <-> control naming contract                                       #
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize(
    ("definition", "group_name"),
    [(RIGHT_ARM, "right_arm"), (LEFT_ARM, "left_arm"), (HEAD, "head")],
    ids=lambda value: getattr(value, "name", value),
)
def test_chain_motor_names_match_control_groups(definition, group_name):
    """IK output order must feed ``Write.move(<group>, *q_deg)`` unchanged."""
    assert list(definition.motor_names) == control._MOTOR_GROUPS[group_name]


# --------------------------------------------------------------------------- #
# Token expansion                                                              #
# --------------------------------------------------------------------------- #
def test_expand_token_returns_group_motors():
    assert _expand_token(right_arm) == control._MOTOR_GROUPS["right_arm"]


def test_expand_token_all_covers_every_group():
    names = _expand_token(All)
    for group in control._MOTOR_GROUPS.values():
        assert set(group) <= set(names)


def test_expand_token_action_tokens_expand_to_nothing():
    assert _expand_token(open_right_hand) == []
    assert _expand_token(default) == []


def test_expand_motor_specs_mixes_strings_and_tokens():
    names = _expand_motor_specs(["wrist_right", left_arm])
    assert names[0] == "wrist_right"
    assert names[1:] == control._MOTOR_GROUPS["left_arm"]


def test_expand_motor_specs_rejects_unsupported_types():
    with pytest.raises(TypeError, match="Unsupported motor spec"):
        _expand_motor_specs([3.14])


# --------------------------------------------------------------------------- #
# Degree conversion                                                            #
# --------------------------------------------------------------------------- #
def test_degrees_to_internal_units_scales_by_one_hundred():
    assert _degrees_to_internal_units(45.0) == 4500.0
    assert _degrees_to_internal_units(-90.0) == -9000.0


def test_degrees_to_internal_units_rejects_out_of_range():
    with pytest.raises(ValueError, match="between -90 and 90"):
        _degrees_to_internal_units(90.5)


# --------------------------------------------------------------------------- #
# move() argument parsing                                                      #
# --------------------------------------------------------------------------- #
def test_vector_move_one_angle_per_named_motor():
    parsed = _parse_vector_move(["wrist_right", 10.0, "elbow_right", -5.0])
    assert parsed == (["wrist_right", "elbow_right"], [10.0, -5.0])


def test_vector_move_token_consumes_one_angle_per_group_motor():
    angles = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    parsed = _parse_vector_move([right_arm, *angles])
    assert parsed is not None
    motor_names, degrees = parsed
    assert motor_names == control._MOTOR_GROUPS["right_arm"]
    assert degrees == angles


def test_vector_move_falls_back_when_angles_are_missing():
    # Only one angle for a six-motor group: not a vector-mode call.
    assert _parse_vector_move([right_arm, -30.0]) is None


def test_vector_move_rejects_unsupported_selector_type():
    with pytest.raises(TypeError, match="Unsupported arg"):
        _parse_vector_move([object(), 1.0])


def test_uniform_move_applies_one_angle_to_every_motor():
    motor_names, angle = _parse_uniform_move([right_arm, "wrist_left", -30.0])
    assert motor_names == control._MOTOR_GROUPS["right_arm"] + ["wrist_left"]
    assert angle == -30.0


def test_uniform_move_requires_trailing_number():
    with pytest.raises(TypeError, match="trailing degree number"):
        _parse_uniform_move([right_arm, "wrist_left"])


def test_uniform_move_requires_at_least_one_motor():
    with pytest.raises(ValueError, match="No motors specified"):
        _parse_uniform_move([default, 10.0])


def test_move_parsing_supports_ik_star_unpacking():
    """`w.move(right_arm, *q_deg)` with six IK angles is a vector-mode call."""
    q_deg = [12.5, -45.0, 7.5, 30.0, -60.0, 15.0]
    parsed = _parse_vector_move([right_arm, *q_deg])
    assert parsed is not None
    assert parsed[0] == control._MOTOR_GROUPS["right_arm"]
    assert len(parsed[0]) == ARM_JOINT_COUNT


# --------------------------------------------------------------------------- #
# Command-line parser                                                          #
# --------------------------------------------------------------------------- #
def test_cli_parses_move_command():
    parser = _build_argument_parser()
    args = parser.parse_args(["--host", "pib.local", "move", "right_arm", "-20"])
    assert args.host == "pib.local"
    assert args.cmd == "move"
    assert args.names == ["right_arm", "-20"]


def test_cli_parses_set_command_with_pulse_widths():
    parser = _build_argument_parser()
    args = parser.parse_args(
        ["set", "All", "--pulse-width-min", "700", "--pulse-width-max", "2500"]
    )
    assert args.cmd == "set"
    assert args.pulse_width_min == 700
    assert args.pulse_width_max == 2500


def test_hand_tokens_expose_expected_motor_names():
    names = _expand_token(right_hand)
    assert "thumb_right_opposition" in names
    assert all(name.endswith(("_stretch", "_opposition")) for name in names)
