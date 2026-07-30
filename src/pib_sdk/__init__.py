"""pib-sdk: kinematics (Pinocchio + URDF) and rosbridge control for the pib robot.

Quick start
-----------
    from pib_sdk import fk, ik, Write, right_arm

    q_deg = ik("right", xyz=[-400, 100, 900])   # millimetres -> degrees
    pose = fk("right", q_deg)                   # degrees -> pose (mm)

    w = Write(host="localhost")                 # rosbridge on the robot
    w.move(right_arm, *q_deg)
"""

from pib_sdk.control import (
    All,
    Write,
    close_left_hand,
    close_right_hand,
    default,
    head,
    left_arm,
    left_hand,
    open_left_hand,
    open_right_hand,
    right_arm,
    right_hand,
    zero_position,
)
from pib_sdk.kinematics import (
    FK,
    IK,
    ArmKinematics,
    ChainKinematics,
    HeadKinematics,
    camera_pose,
    fk,
    ik,
    pose_from_xyz_rpy,
    pose_to_xyz_rpy,
)
from pib_sdk.robot_model import (
    HEAD,
    LEFT_ARM,
    RIGHT_ARM,
    ArmSide,
    ChainDefinition,
    ChainModel,
    build_chain_model,
    get_arm_model,
    get_chain_model,
    get_head_model,
    set_urdf_path,
)
from pib_sdk.speech import Speak

__version__ = "0.4"

__all__ = [
    # kinematics
    "fk",
    "ik",
    "camera_pose",
    "ArmKinematics",
    "HeadKinematics",
    "ChainKinematics",
    "FK",
    "IK",
    "pose_from_xyz_rpy",
    "pose_to_xyz_rpy",
    # robot model
    "ArmSide",
    "ChainDefinition",
    "ChainModel",
    "RIGHT_ARM",
    "LEFT_ARM",
    "HEAD",
    "build_chain_model",
    "get_chain_model",
    "get_arm_model",
    "get_head_model",
    "set_urdf_path",
    # control
    "Write",
    "All",
    "default",
    "zero_position",
    "right_arm",
    "left_arm",
    "right_hand",
    "left_hand",
    "head",
    "open_left_hand",
    "close_left_hand",
    "open_right_hand",
    "close_right_hand",
    # speech
    "Speak",
    "__version__",
]
