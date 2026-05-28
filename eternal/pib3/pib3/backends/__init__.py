"""Robot control backends for pib3 package."""

from .base import RobotBackend
from .webots import WebotsBackend
from .robot import RealRobotBackend, rle_decode, build_motor_mapping, PIB_SERVO_CHANNELS
from .camera import (
    # Dataclasses
    BoundingBox,
    Detection,
    Keypoint,
    FingerAngles,
    HandLandmarks,
    PoseKeypoints,
    AIModelInfo,
    CameraFrame,
    # Enums
    AiModelType,
    Handedness,
    # Receivers
    CameraFrameReceiver,
    AIDetectionReceiver,
    # Utilities
    parse_ai_result,
    # Constants
    COCO_LABELS,
)
from .audio import (
    # Unified audio system - enums
    AudioOutput,
    AudioInput,
    # Device management
    AudioDevice,
    list_audio_devices,
    list_audio_input_devices,
    list_audio_output_devices,
    get_default_audio_input_device,
    get_default_audio_output_device,
    # Playback
    LocalAudioPlayer,
    RobotAudioPlayer,
    # Recording
    LocalAudioRecorder,
    RobotAudioRecorder,
    AudioStreamReceiver,
    # TTS
    PiperTTS,
    # Utilities
    load_audio_file,
    save_audio_file,
    resample_audio,
    DEFAULT_SAMPLE_RATE,
)

__all__ = [
    "RobotBackend",
    "WebotsBackend",
    "RealRobotBackend",
    "rle_decode",
    # Low-latency motor control helpers
    "build_motor_mapping",
    "PIB_SERVO_CHANNELS",
    # Camera/AI dataclasses
    "BoundingBox",
    "Detection",
    "Keypoint",
    "FingerAngles",
    "HandLandmarks",
    "PoseKeypoints",
    "AIModelInfo",
    "CameraFrame",
    # Camera/AI enums
    "AiModelType",
    "Handedness",
    # Camera/AI receivers
    "CameraFrameReceiver",
    "AIDetectionReceiver",
    # Camera/AI utilities
    "parse_ai_result",
    "COCO_LABELS",
    # Unified audio system - enums
    "AudioOutput",
    "AudioInput",
    # Device management
    "AudioDevice",
    "list_audio_devices",
    "list_audio_input_devices",
    "list_audio_output_devices",
    "get_default_audio_input_device",
    "get_default_audio_output_device",
    # Playback
    "LocalAudioPlayer",
    "RobotAudioPlayer",
    # Recording
    "LocalAudioRecorder",
    "RobotAudioRecorder",
    "AudioStreamReceiver",
    # TTS
    "PiperTTS",
    # Utilities
    "load_audio_file",
    "save_audio_file",
    "resample_audio",
    "DEFAULT_SAMPLE_RATE",
]
