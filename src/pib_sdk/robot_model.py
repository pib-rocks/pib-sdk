"""Kinematic models of the pib robot, built with Pinocchio from the official URDF.

Pinocchio (https://github.com/stack-of-tasks/pinocchio) is the kinematics
backend; it replaces the unmaintained Robotics Toolbox. The robot geometry
comes from pib's official V3 URDF, which ships with the package as
``data/pib_model.urdf`` -- there are no hand-maintained Denavit-Hartenberg tables
any more.

The full-body URDF (36 revolute joints) is loaded once and then *reduced* to
the kinematic chain of interest -- an arm or the head -- by locking every other
joint at zero. Each chain is described by a :class:`ChainDefinition` that names
its URDF joints (base to tip), the pib motor driving each joint, and the URDF
frame used as the chain's tip (end-effector).

Units
-----
URDF lengths are metres, but the public pib-sdk API keeps the previous SDK's
conventions: **millimetres** for positions and **degrees** for joint angles.
Loaded models are therefore scaled once from metres to millimetres; angles are
converted at the API boundary in :mod:`pib_sdk.kinematics`.

Using a different URDF
----------------------
Point the loader at another file with :func:`set_urdf_path` or the
``PIB_URDF_PATH`` environment variable, or pass ``urdf_path`` directly to
:func:`build_chain_model`.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import numpy as np
import pinocchio as pin

# URDFs express lengths in metres; the public API works in millimetres.
MILLIMETRES_PER_METRE = 1000.0

_URDF_PATH_ENVIRONMENT_VARIABLE = "PIB_URDF_PATH"

# pib's official V3 URDF, bundled with the package.
DEFAULT_URDF_PATH = Path(__file__).resolve().parent / "data" / "pib_model.urdf"


class ArmSide(str, Enum):
    """Identifies which of pib's two arms a model or query refers to."""

    RIGHT = "right"
    LEFT = "left"


@dataclass(frozen=True)
class ChainDefinition:
    """Description of one kinematic chain of the robot.

    Attributes
    ----------
    name:
        Human-readable chain name (``"right_arm"``, ``"head"`` ...).
    urdf_joint_names:
        The chain's actuated joints as named in the URDF, ordered base to tip.
    motor_names:
        The pib motor driving each joint, in the same order. These are the
        names understood by :class:`pib_sdk.control.Write`; they differ from
        the URDF names for a few joints (see the mapping in the README).
    tip_frame:
        Name of the URDF link/frame used as the chain's end-effector.
    named_configurations_deg:
        Handy joint configurations in degrees, keyed by name.
    """

    name: str
    urdf_joint_names: tuple[str, ...]
    motor_names: tuple[str, ...]
    tip_frame: str
    named_configurations_deg: Mapping[str, tuple[float, ...]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if len(self.motor_names) != len(self.urdf_joint_names):
            raise ValueError(
                f"{self.name}: motor_names ({len(self.motor_names)}) and "
                f"urdf_joint_names ({len(self.urdf_joint_names)}) must have the same length"
            )
        for config_name, angles_deg in self.named_configurations_deg.items():
            if len(angles_deg) != len(self.urdf_joint_names):
                raise ValueError(
                    f"{self.name}: configuration {config_name!r} has {len(angles_deg)} "
                    f"values, expected {len(self.urdf_joint_names)}"
                )


@dataclass(frozen=True)
class ChainModel:
    """A ready-to-use Pinocchio model of one chain, in millimetres."""

    definition: ChainDefinition
    model: pin.Model
    tip_frame_id: int

    @property
    def degrees_of_freedom(self) -> int:
        return self.model.nq

    @property
    def joint_names(self) -> tuple[str, ...]:
        """URDF joint names, base to tip (also the joint-vector order)."""
        return self.definition.urdf_joint_names

    @property
    def motor_names(self) -> tuple[str, ...]:
        """pib motor names in joint-vector order, ready for ``Write.move``."""
        return self.definition.motor_names

    @property
    def joint_lower_limits(self) -> np.ndarray:
        """Lower joint limits in radians, in joint-vector order."""
        return self.model.lowerPositionLimit.copy()

    @property
    def joint_upper_limits(self) -> np.ndarray:
        """Upper joint limits in radians, in joint-vector order."""
        return self.model.upperPositionLimit.copy()

    @property
    def joint_limits_deg(self) -> tuple[np.ndarray, np.ndarray]:
        """Joint limits as a ``(lower_deg, upper_deg)`` pair of arrays."""
        return np.rad2deg(self.joint_lower_limits), np.rad2deg(self.joint_upper_limits)

    def named_configuration_deg(self, name: str) -> np.ndarray:
        """Return the named joint configuration in degrees."""
        try:
            return np.asarray(self.definition.named_configurations_deg[name], dtype=float)
        except KeyError:
            available = ", ".join(sorted(self.definition.named_configurations_deg))
            raise KeyError(
                f"{self.definition.name} has no configuration {name!r}; "
                f"available: {available or '(none)'}"
            ) from None

    def create_data(self) -> pin.Data:
        """Return a fresh :class:`pinocchio.Data` buffer for this model.

        Each caller should own its own ``Data`` because kinematics routines
        write results into it; the underlying ``Model`` is safe to share.
        """
        return self.model.createData()


# --------------------------------------------------------------------------- #
# Chain definitions                                                            #
# --------------------------------------------------------------------------- #
# In the URDF's zero configuration both arms point straight out to the sides
# (pib's motor zero pose), so URDF joint angles line up with the degrees sent
# to the motors by pib_sdk.control. The "observe" and "rest" postures are
# carried over from the previous SDK releases.
RIGHT_ARM = ChainDefinition(
    name="right_arm",
    urdf_joint_names=(
        "shoulder_vertical_right",
        "shoulder_horizontal_right",
        "upper_arm_right",
        "elbow_right",
        "forearm_right",
        "wrist_right",
    ),
    motor_names=(
        "shoulder_vertical_right",
        "shoulder_horizontal_right",
        "upper_arm_right_rotation",
        "elbow_right",
        "lower_arm_right_rotation",
        "wrist_right",
    ),
    tip_frame="urdf_palm_right",
    named_configurations_deg={
        "zero": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "observe": (40.0, -90.0, -30.0, 40.0, -90.0, 0.0),
        "rest": (90.0, -90.0, 0.0, -40.0, 0.0, 0.0),
    },
)

LEFT_ARM = ChainDefinition(
    name="left_arm",
    urdf_joint_names=(
        "shoulder_vertical_left",
        "shoulder_horizontal_left",
        "upper_arm_left",
        "elbow_left",
        "forearm_left",
        "wrist_left",
    ),
    motor_names=(
        "shoulder_vertical_left",
        "shoulder_horizontal_left",
        "upper_arm_left_rotation",
        "elbow_left",
        "lower_arm_left_rotation",
        "wrist_left",
    ),
    tip_frame="urdf_palm_left",
    named_configurations_deg={
        "zero": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "observe": (-40.0, 90.0, 30.0, 40.0, 90.0, 0.0),
        "rest": (-90.0, 90.0, 0.0, -40.0, 0.0, 0.0),
    },
)

HEAD = ChainDefinition(
    name="head",
    urdf_joint_names=("head_horizontal", "head_vertical"),
    motor_names=("turn_head_motor", "tilt_forward_motor"),
    tip_frame="urdf_camera_link",
    named_configurations_deg={"zero": (0.0, 0.0)},
)

_ARM_DEFINITIONS: dict[ArmSide, ChainDefinition] = {
    ArmSide.RIGHT: RIGHT_ARM,
    ArmSide.LEFT: LEFT_ARM,
}

_FULL_MODEL_CACHE: dict[Path, pin.Model] = {}
_CHAIN_MODEL_CACHE: dict[tuple[str, Path], ChainModel] = {}


# --------------------------------------------------------------------------- #
# URDF loading                                                                 #
# --------------------------------------------------------------------------- #
def set_urdf_path(urdf_path: str | Path) -> None:
    """Point the loader at ``urdf_path`` and clear any cached models."""
    os.environ[_URDF_PATH_ENVIRONMENT_VARIABLE] = str(urdf_path)
    _FULL_MODEL_CACHE.clear()
    _CHAIN_MODEL_CACHE.clear()


def _resolve_urdf_path(urdf_path: str | Path | None) -> Path:
    if urdf_path is not None:
        return Path(urdf_path).resolve()
    return Path(os.environ.get(_URDF_PATH_ENVIRONMENT_VARIABLE, DEFAULT_URDF_PATH)).resolve()


def _load_full_model(path: Path) -> pin.Model:
    """Load (and cache) the complete robot model from the URDF at ``path``."""
    if path not in _FULL_MODEL_CACHE:
        if not path.is_file():
            raise FileNotFoundError(
                f"URDF not found at {path}. The package ships pib's URDF at "
                f"{DEFAULT_URDF_PATH}; use set_urdf_path(...) or the "
                f"{_URDF_PATH_ENVIRONMENT_VARIABLE} environment variable to point at another file."
            )
        _FULL_MODEL_CACHE[path] = pin.buildModelFromUrdf(str(path))
    return _FULL_MODEL_CACHE[path]


def _scale_model_lengths(model: pin.Model, scale: float) -> None:
    """Scale every fixed placement translation in ``model`` (metres -> millimetres).

    All of pib's joints are revolute, so they contribute pure rotations; scaling
    the joint and frame placement translations therefore scales every
    forward-kinematics position by ``scale`` while leaving orientations exact.
    """
    if scale == 1.0:
        return
    for index in range(len(model.jointPlacements)):
        placement = model.jointPlacements[index]
        model.jointPlacements[index] = pin.SE3(placement.rotation, placement.translation * scale)
    for frame in model.frames:
        frame.placement.translation *= scale


def build_chain_model(
    definition: ChainDefinition, urdf_path: str | Path | None = None
) -> ChainModel:
    """Load ``definition``'s chain from the URDF and reduce it to those joints.

    The returned model exposes exactly the chain's actuated degrees of freedom;
    every other joint of the robot is locked at zero. Raises ``ValueError``
    with the available names when the URDF does not match the definition.
    """
    path = _resolve_urdf_path(urdf_path)
    full_model = _load_full_model(path)

    missing = [name for name in definition.urdf_joint_names if not full_model.existJointName(name)]
    if missing:
        available = sorted(name for name in full_model.names if name != "universe")
        raise ValueError(
            f"URDF at {path} does not define these joints for {definition.name}: {missing}. "
            f"Available joints: {available}"
        )

    chain_joint_ids = {full_model.getJointId(name) for name in definition.urdf_joint_names}
    joints_to_lock = [
        joint_id
        for joint_id in range(1, full_model.njoints)  # joint 0 is the universe joint
        if joint_id not in chain_joint_ids
    ]
    model = pin.buildReducedModel(full_model, joints_to_lock, pin.neutral(full_model))
    model.name = definition.name

    reduced_joint_names = tuple(model.names[1:])
    if reduced_joint_names != definition.urdf_joint_names:
        raise ValueError(
            f"URDF joint order for {definition.name} is {reduced_joint_names}, but the "
            f"definition expects {definition.urdf_joint_names}. List the joints base to "
            f"tip as they appear in the URDF's kinematic tree."
        )
    if model.nq != len(definition.urdf_joint_names):
        raise ValueError(
            f"{definition.name}: expected {len(definition.urdf_joint_names)} degrees of "
            f"freedom but the reduced model has {model.nq}; every chain joint must be a "
            f"single-axis revolute joint."
        )

    _scale_model_lengths(model, MILLIMETRES_PER_METRE)

    if not model.existFrame(definition.tip_frame):
        available_frames = sorted(frame.name for frame in model.frames)
        raise ValueError(
            f"Tip frame {definition.tip_frame!r} was not found in the URDF for "
            f"{definition.name}. Available frames: {available_frames}"
        )
    tip_frame_id = model.getFrameId(definition.tip_frame)
    return ChainModel(definition=definition, model=model, tip_frame_id=tip_frame_id)


# --------------------------------------------------------------------------- #
# Cached accessors                                                             #
# --------------------------------------------------------------------------- #
def get_chain_model(definition: ChainDefinition) -> ChainModel:
    """Return the (cached) :class:`ChainModel` for ``definition``."""
    cache_key = (definition.name, _resolve_urdf_path(None))
    if cache_key not in _CHAIN_MODEL_CACHE:
        _CHAIN_MODEL_CACHE[cache_key] = build_chain_model(definition)
    return _CHAIN_MODEL_CACHE[cache_key]


def coerce_arm_side(side: ArmSide | str) -> ArmSide:
    """Return an :class:`ArmSide` for ``side`` given as an enum or a string."""
    if isinstance(side, ArmSide):
        return side
    try:
        return ArmSide(str(side).lower())
    except ValueError as error:
        valid = ", ".join(member.value for member in ArmSide)
        raise ValueError(f"Unknown arm side {side!r}; expected one of: {valid}") from error


def get_arm_model(side: ArmSide | str) -> ChainModel:
    """Return the (cached) :class:`ChainModel` for the requested arm."""
    return get_chain_model(_ARM_DEFINITIONS[coerce_arm_side(side)])


def get_head_model() -> ChainModel:
    """Return the (cached) :class:`ChainModel` for the head (camera chain)."""
    return get_chain_model(HEAD)


if __name__ == "__main__":
    for chain_definition in (RIGHT_ARM, LEFT_ARM, HEAD):
        chain = get_chain_model(chain_definition)
        lower_deg, upper_deg = chain.joint_limits_deg
        print(f"{chain.definition.name}: {chain.degrees_of_freedom} DOF")
        for joint, motor, low, high in zip(
            chain.joint_names, chain.motor_names, lower_deg, upper_deg
        ):
            print(f"  {joint:28s} -> motor {motor:28s} [{low:6.1f}, {high:6.1f}] deg")
