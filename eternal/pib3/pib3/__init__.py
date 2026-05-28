"""
pib3 - Inverse kinematics and trajectory generation for the PIB robot.

This package provides tools to:
1. Convert images to 2D drawing strokes
2. Generate robot trajectories using inverse kinematics
3. Execute trajectories on Webots simulation or real robot

Quick Start:
    >>> from pib3 import Robot, Joint
    >>>
    >>> # Connect and control the robot (uses Tinkerforge direct control by default)
    >>> with Robot(host="172.26.34.149") as robot:
    ...     robot.set_joint(Joint.ELBOW_LEFT, 50.0)
    ...     angle = robot.get_joint(Joint.ELBOW_LEFT)
    >>>
    >>> # Generate and run a drawing trajectory
    >>> import pib3
    >>> trajectory = pib3.generate_trajectory("drawing.png")
    >>> with Robot(host="172.26.34.149") as robot:
    ...     robot.run_trajectory(trajectory)
    >>>
    >>> # AI detection via subsystem API:
    >>> from pib3 import Robot, AIModel
    >>> with Robot(host="...") as robot:
    ...     robot.ai.set_model(AIModel.HAND)
    ...     for hand in robot.ai.get_hand_landmarks():
    ...         print(f"{hand.handedness}: {hand.finger_angles.index:.0f}°")
"""

from pathlib import Path
from typing import Optional, Union

# Core types
from .types import Joint, Sketch, Stroke, HandPose, LEFT_HAND_JOINTS, RIGHT_HAND_JOINTS, AIModel, ImuType

# Configuration
from .config import PaperConfig, IKConfig, ImageConfig, TrajectoryConfig, LowLatencyConfig, RobotConfig

# Core functions
from .image import image_to_sketch
from .trajectory import Trajectory, sketch_to_trajectory

# Backends
from .backends import (
    RobotBackend,
    WebotsBackend,
    RealRobotBackend,
    # Camera/AI types
    Detection,
    HandLandmarks,
    FingerAngles,
    PoseKeypoints,
    BoundingBox,
    CameraFrameReceiver,
    AIDetectionReceiver,
    COCO_LABELS,
    # Audio enums
    AudioOutput,
    AudioInput,
    # Audio device management
    AudioDevice,
    list_audio_devices,
    # TTS
    PiperTTS,
    # Audio utilities
    load_audio_file,
    save_audio_file,
    # Low-latency motor control helpers
    build_motor_mapping,
    PIB_SERVO_CHANNELS,
)

# Convenience aliases
Webots = WebotsBackend
Robot = RealRobotBackend


__version__ = "0.1.4"

__all__ = [
    # Version
    "__version__",
    # Types
    "Joint",
    "Stroke",
    "Sketch",
    "Trajectory",
    "AIModel",
    "ImuType",
    # Config
    "PaperConfig",
    "IKConfig",
    "ImageConfig",
    "TrajectoryConfig",
    "LowLatencyConfig",
    "RobotConfig",
    # Functions
    "image_to_sketch",
    "sketch_to_trajectory",
    "generate_trajectory",
    # Backends
    "RobotBackend",
    "WebotsBackend",
    "RealRobotBackend",
    # Camera/AI types
    "Detection",
    "HandLandmarks",
    "FingerAngles",
    "PoseKeypoints",
    "BoundingBox",
    "CameraFrameReceiver",
    "AIDetectionReceiver",
    "COCO_LABELS",
    # Audio enums
    "AudioOutput",
    "AudioInput",
    # Aliases
    "Webots",
    "Robot",
    # Hand poses
    "HandPose",
    "LEFT_HAND_JOINTS",
    "RIGHT_HAND_JOINTS",
    # Audio device management
    "AudioDevice",
    "list_audio_devices",
    # TTS
    "PiperTTS",
    # Audio utilities
    "load_audio_file",
    "save_audio_file",
    # Low-latency motor control helpers
    "build_motor_mapping",
    "PIB_SERVO_CHANNELS",
]


def generate_trajectory(
    image: Union[str, Path, "np.ndarray", "PIL.Image.Image"],
    output_path: Optional[Union[str, Path]] = None,
    config: Optional[TrajectoryConfig] = None,
    initial_q: Optional[Union["np.ndarray", dict, Trajectory]] = None,
) -> Trajectory:
    """
    Convenience function: Convert image directly to robot trajectory.

    This is a one-shot function that combines image_to_sketch() and
    sketch_to_trajectory() for simple use cases.

    Args:
        image: Input image (file path, numpy array, or PIL Image).
        output_path: If provided, save trajectory JSON to this path.
        config: Full configuration (uses sensible defaults if None).
        initial_q: Initial joint configuration to start IK solving from.
            Can be one of:
            - numpy array of shape (n_joints,) with joint positions in radians
            - dict mapping joint names to positions in radians
            - Trajectory object (uses last waypoint)
            - None (default): use fixed initial pose for drawing

            This enables sequential trajectory execution where each new
            trajectory starts from the robot's current position.

    Returns:
        Trajectory object ready for execution.

    Example:
        >>> import pib3
        >>> trajectory = pib3.generate_trajectory("my_drawing.png")
        >>> trajectory.to_json("output.json")

        >>> # With custom config
        >>> from pib3 import TrajectoryConfig, PaperConfig
        >>> config = TrajectoryConfig(
        ...     paper=PaperConfig(size=0.20, drawing_scale=0.9),
        ... )
        >>> trajectory = pib3.generate_trajectory("drawing.png", config=config)

        >>> # Sequential trajectories (robot draws multiple images)
        >>> traj1 = pib3.generate_trajectory("image1.png")
        >>> traj2 = pib3.generate_trajectory("image2.png", initial_q=traj1)
        >>> traj3 = pib3.generate_trajectory("image3.png", initial_q=traj2)
    """
    if config is None:
        config = TrajectoryConfig()

    # Step 1: Image to sketch
    sketch = image_to_sketch(image, config.image)

    # Step 2: Sketch to trajectory
    trajectory = sketch_to_trajectory(
        sketch, config, initial_q=initial_q
    )

    # Optional: Save to file
    if output_path is not None:
        trajectory.to_json(output_path)

    return trajectory
