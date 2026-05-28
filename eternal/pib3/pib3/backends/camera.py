"""
Camera and AI subsystem for PIB robot.

This module provides typed dataclasses and helpers for consuming camera and AI
streams from the robot. It does NOT provide direct DepthAI access - that runs
on the robot itself via the camera ROS node.

Key Components:
- BoundingBox: Normalized bounding box with utility methods
- Detection: Object detection result with label, confidence, bbox
- HandLandmarks: Hand tracking with 21 landmarks and finger angles
- PoseKeypoints: Body pose with 17 COCO keypoints
- CameraFrame: Frame data with timestamp
- AIModelInfo: Model metadata
- CameraFrameReceiver: Frame buffering helper
- AIDetectionReceiver: Detection buffering with FPS tracking

Hand Landmark Indices (MediaPipe convention):
    0: WRIST
    1-4: THUMB (CMC, MCP, IP, TIP)
    5-8: INDEX (MCP, PIP, DIP, TIP)
    9-12: MIDDLE (MCP, PIP, DIP, TIP)
    13-16: RING (MCP, PIP, DIP, TIP)
    17-20: PINKY (MCP, PIP, DIP, TIP)

Pose Keypoint Indices (COCO convention):
    0: nose, 1: left_eye, 2: right_eye, 3: left_ear, 4: right_ear
    5: left_shoulder, 6: right_shoulder, 7: left_elbow, 8: right_elbow
    9: left_wrist, 10: right_wrist, 11: left_hip, 12: right_hip
    13: left_knee, 14: right_knee, 15: left_ankle, 16: right_ankle

Usage:
    >>> from pib3.backends.camera import AIDetectionReceiver, Detection
    >>>
    >>> receiver = AIDetectionReceiver()
    >>> sub = robot.subscribe_ai_detections(receiver.on_detection)
    >>> time.sleep(5)
    >>> sub.unsubscribe()
    >>>
    >>> # Get typed detections
    >>> for det in receiver.get_detections():
    ...     print(f"{det.label}: {det.confidence:.2f} at {det.bbox}")
    >>>
    >>> # Check performance
    >>> print(f"FPS: {receiver.fps:.1f}, Latency: {receiver.avg_latency_ms:.1f}ms")
"""

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple, Union, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .robot import RealRobotBackend
    from ..types import AIModel

logger = logging.getLogger(__name__)


# ==================== CONSTANTS ====================

# COCO class labels (80 classes)
COCO_LABELS = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
]

# Hand landmark indices (MediaPipe convention)
HAND_WRIST = 0
HAND_THUMB_CMC = 1
HAND_THUMB_MCP = 2
HAND_THUMB_IP = 3
HAND_THUMB_TIP = 4
HAND_INDEX_MCP = 5
HAND_INDEX_PIP = 6
HAND_INDEX_DIP = 7
HAND_INDEX_TIP = 8
HAND_MIDDLE_MCP = 9
HAND_MIDDLE_PIP = 10
HAND_MIDDLE_DIP = 11
HAND_MIDDLE_TIP = 12
HAND_RING_MCP = 13
HAND_RING_PIP = 14
HAND_RING_DIP = 15
HAND_RING_TIP = 16
HAND_PINKY_MCP = 17
HAND_PINKY_PIP = 18
HAND_PINKY_DIP = 19
HAND_PINKY_TIP = 20

# Pose keypoint indices (COCO convention)
POSE_NOSE = 0
POSE_LEFT_EYE = 1
POSE_RIGHT_EYE = 2
POSE_LEFT_EAR = 3
POSE_RIGHT_EAR = 4
POSE_LEFT_SHOULDER = 5
POSE_RIGHT_SHOULDER = 6
POSE_LEFT_ELBOW = 7
POSE_RIGHT_ELBOW = 8
POSE_LEFT_WRIST = 9
POSE_RIGHT_WRIST = 10
POSE_LEFT_HIP = 11
POSE_RIGHT_HIP = 12
POSE_LEFT_KNEE = 13
POSE_RIGHT_KNEE = 14
POSE_LEFT_ANKLE = 15
POSE_RIGHT_ANKLE = 16


# ==================== ENUMS ====================


class AiModelType(str, Enum):
    """Types of AI models available on the robot."""
    DETECTION = "detection"
    POSE = "pose"
    HAND = "hand"
    SEGMENTATION = "instance-segmentation"
    GAZE = "gaze"
    LINES = "lines"


class Handedness(str, Enum):
    """Hand classification."""
    LEFT = "left"
    RIGHT = "right"
    UNKNOWN = "unknown"


# ==================== DATACLASSES ====================


@dataclass
class BoundingBox:
    """
    Normalized bounding box in [0, 1] coordinates.

    Attributes:
        xmin: Left edge (0 = left of image)
        ymin: Top edge (0 = top of image)
        xmax: Right edge (1 = right of image)
        ymax: Bottom edge (1 = bottom of image)
    """
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    @property
    def width(self) -> float:
        """Box width in normalized coordinates."""
        return self.xmax - self.xmin

    @property
    def height(self) -> float:
        """Box height in normalized coordinates."""
        return self.ymax - self.ymin

    @property
    def center(self) -> Tuple[float, float]:
        """Center point (x, y) in normalized coordinates."""
        return ((self.xmin + self.xmax) / 2, (self.ymin + self.ymax) / 2)

    @property
    def area(self) -> float:
        """Box area in normalized coordinates (0 to 1)."""
        return self.width * self.height

    def to_pixels(self, img_width: int, img_height: int) -> Tuple[int, int, int, int]:
        """
        Convert to pixel coordinates.

        Returns:
            Tuple of (x1, y1, x2, y2) in pixels.
        """
        return (
            int(self.xmin * img_width),
            int(self.ymin * img_height),
            int(self.xmax * img_width),
            int(self.ymax * img_height),
        )

    def __repr__(self) -> str:
        cx, cy = self.center
        return f"BoundingBox(center=({cx:.2f}, {cy:.2f}), size=({self.width:.2f}x{self.height:.2f}))"


@dataclass
class Detection:
    """
    Single object detection result.

    Attributes:
        label_id: Numeric class ID (e.g., 0 for person in COCO)
        confidence: Detection confidence (0.0 to 1.0)
        bbox: Bounding box in normalized coordinates
        label: Human-readable class name (auto-resolved from COCO if possible)
        mask_rle: Optional RLE-encoded segmentation mask
    """
    label_id: int
    confidence: float
    bbox: BoundingBox
    label: str = ""
    mask_rle: Optional[Dict] = None

    def __post_init__(self):
        """Auto-resolve label from COCO classes if not provided."""
        if not self.label and 0 <= self.label_id < len(COCO_LABELS):
            self.label = COCO_LABELS[self.label_id]

    @classmethod
    def from_dict(cls, data: dict) -> "Detection":
        """Create Detection from robot's JSON format."""
        bbox_data = data.get("bbox", {})
        bbox = BoundingBox(
            xmin=bbox_data.get("xmin", 0),
            ymin=bbox_data.get("ymin", 0),
            xmax=bbox_data.get("xmax", 0),
            ymax=bbox_data.get("ymax", 0),
        )
        return cls(
            label_id=data.get("label", 0),
            confidence=data.get("confidence", 0.0),
            bbox=bbox,
            label=data.get("label_name", ""),
            mask_rle=data.get("mask_rle"),
        )


@dataclass
class Keypoint:
    """Single keypoint with position and confidence."""
    x: float  # Normalized [0, 1]
    y: float  # Normalized [0, 1]
    confidence: float = 1.0

    def to_pixels(self, img_width: int, img_height: int) -> Tuple[int, int]:
        """Convert to pixel coordinates."""
        return (int(self.x * img_width), int(self.y * img_height))


@dataclass
class FingerAngles:
    """
    Finger bend angles in degrees.

    Calculated from hand landmarks similar to HandTrackerEdge.py.
    Angle of 0° = fully extended, ~90° = bent.

    Attributes:
        thumb_bend: Thumb flexion angle
        thumb_rotation: Thumb rotation relative to palm plane
        index: Index finger bend angle
        middle: Middle finger bend angle
        ring: Ring finger bend angle
        pinky: Pinky finger bend angle
    """
    thumb_bend: float = 0.0
    thumb_rotation: float = 0.0
    index: float = 0.0
    middle: float = 0.0
    ring: float = 0.0
    pinky: float = 0.0

    def to_list(self) -> List[float]:
        """Return angles as list [thumb_bend, thumb_rot, index, middle, ring, pinky]."""
        return [
            self.thumb_bend, self.thumb_rotation,
            self.index, self.middle, self.ring, self.pinky
        ]

    def to_servo_values(self, scale: float = 100.0) -> Dict[str, float]:
        """
        Convert finger angles to servo percentage values.

        Args:
            scale: Maximum servo value (default 100 for percentage)

        Returns:
            Dict mapping finger names to servo values (0 = bent, scale = extended)
        """
        # Invert: 0° extension = 100%, 90° bent = 0%
        def angle_to_pct(angle: float) -> float:
            return max(0, min(scale, scale * (1 - angle / 90.0)))

        return {
            "thumb_opposition": angle_to_pct(self.thumb_rotation),
            "thumb_stretch": angle_to_pct(self.thumb_bend),
            "index": angle_to_pct(self.index),
            "middle": angle_to_pct(self.middle),
            "ring": angle_to_pct(self.ring),
            "pinky": angle_to_pct(self.pinky),
        }


@dataclass
class HandLandmarks:
    """
    Hand tracking result with 21 landmarks.

    Attributes:
        landmarks: Array of shape (21, 2) with normalized [0,1] coordinates
        keypoints: List of 21 Keypoint objects with confidence
        handedness: Left or right hand classification
        confidence: Overall detection confidence
        wrist_position: (x, y) position of wrist in normalized coords
        finger_angles: Calculated finger bend angles
    """
    landmarks: np.ndarray  # Shape (21, 2) or (21, 3)
    keypoints: List[Keypoint] = field(default_factory=list)
    handedness: Handedness = Handedness.UNKNOWN
    confidence: float = 1.0
    finger_angles: Optional[FingerAngles] = None

    def __post_init__(self):
        """Calculate finger angles if not provided."""
        if self.finger_angles is None and len(self.landmarks) >= 21:
            self.finger_angles = self._calculate_finger_angles()

    @property
    def wrist_position(self) -> Tuple[float, float]:
        """Get wrist (landmark 0) position."""
        if len(self.landmarks) > 0:
            return (float(self.landmarks[0][0]), float(self.landmarks[0][1]))
        return (0.0, 0.0)

    def _calculate_finger_angles(self) -> FingerAngles:
        """
        Calculate finger angles from landmarks.

        Algorithm inspired by HandTrackerEdge.py - calculates the angle
        between upper and lower segments of each finger.

        Returns angles where 0° = fully extended, ~90° = bent.
        """
        if len(self.landmarks) < 21:
            return FingerAngles()

        lm = self.landmarks

        def angle_between_vectors(v1: np.ndarray, v2: np.ndarray) -> float:
            """Calculate angle in degrees between two vectors."""
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            if v1_norm < 1e-6 or v2_norm < 1e-6:
                return 0.0
            cos_angle = np.dot(v1, v2) / (v1_norm * v2_norm)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            return math.degrees(math.acos(cos_angle))

        def bend_angle(mcp: int, pip: int, dip: int, tip: int) -> float:
            """
            Calculate finger bend angle in degrees (0° = straight, 180° = curled).

            Vectors run along the proximal and distal phalanx in the same
            nominal direction (MCP->PIP and DIP->TIP). A straight finger
            makes them parallel (~0°); a fully curled finger makes them
            anti-parallel (~180°).
            """
            v1 = lm[pip][:2] - lm[mcp][:2]
            v2 = lm[tip][:2] - lm[dip][:2]
            return angle_between_vectors(v1, v2)

        try:
            # Thumb bend: angle at IP joint
            thumb_bend = bend_angle(
                HAND_THUMB_CMC, HAND_THUMB_MCP, HAND_THUMB_IP, HAND_THUMB_TIP
            )

            # Thumb rotation relative to palm (2-3 vs 0-9)
            vec_thumb_rot = lm[HAND_THUMB_IP][:2] - lm[HAND_THUMB_MCP][:2]
            vec_palm = lm[HAND_MIDDLE_MCP][:2] - lm[HAND_WRIST][:2]
            thumb_rotation = angle_between_vectors(vec_thumb_rot, vec_palm)

            # Finger bend angles
            index_angle = bend_angle(
                HAND_INDEX_MCP, HAND_INDEX_PIP, HAND_INDEX_DIP, HAND_INDEX_TIP
            )
            middle_angle = bend_angle(
                HAND_MIDDLE_MCP, HAND_MIDDLE_PIP, HAND_MIDDLE_DIP, HAND_MIDDLE_TIP
            )
            ring_angle = bend_angle(
                HAND_RING_MCP, HAND_RING_PIP, HAND_RING_DIP, HAND_RING_TIP
            )
            pinky_angle = bend_angle(
                HAND_PINKY_MCP, HAND_PINKY_PIP, HAND_PINKY_DIP, HAND_PINKY_TIP
            )

            return FingerAngles(
                thumb_bend=thumb_bend,
                thumb_rotation=thumb_rotation,
                index=index_angle,
                middle=middle_angle,
                ring=ring_angle,
                pinky=pinky_angle,
            )
        except Exception as e:
            logger.debug(f"Error calculating finger angles: {e}")
            return FingerAngles()

    @classmethod
    def from_keypoints_list(
        cls,
        keypoints: List[dict],
        handedness_value: float = 0.5,
    ) -> "HandLandmarks":
        """
        Create HandLandmarks from robot's keypoints list.

        Args:
            keypoints: List of {"x": float, "y": float, "confidence": float} dicts
            handedness_value: 0.0 = left, 1.0 = right, 0.5 = unknown
        """
        landmarks = np.array([[kp["x"], kp["y"]] for kp in keypoints])
        kp_objects = [
            Keypoint(x=kp["x"], y=kp["y"], confidence=kp.get("confidence", 1.0))
            for kp in keypoints
        ]

        if handedness_value < 0.3:
            handedness = Handedness.LEFT
        elif handedness_value > 0.7:
            handedness = Handedness.RIGHT
        else:
            handedness = Handedness.UNKNOWN

        return cls(
            landmarks=landmarks,
            keypoints=kp_objects,
            handedness=handedness,
        )


@dataclass
class PoseKeypoints:
    """
    Body pose estimation result with 17 COCO keypoints.

    Attributes:
        keypoints: List of 17 Keypoint objects
        confidence: Overall pose confidence
        bbox: Optional bounding box around the person
    """
    keypoints: List[Keypoint]
    confidence: float = 1.0
    bbox: Optional[BoundingBox] = None

    @classmethod
    def from_keypoints_list(cls, keypoints: List[dict], bbox: Optional[dict] = None) -> "PoseKeypoints":
        """Create from robot's keypoints list."""
        kp_objects = [
            Keypoint(x=kp["x"], y=kp["y"], confidence=kp.get("confidence", 1.0))
            for kp in keypoints
        ]

        bbox_obj = None
        if bbox:
            bbox_obj = BoundingBox(
                xmin=bbox.get("xmin", 0),
                ymin=bbox.get("ymin", 0),
                xmax=bbox.get("xmax", 0),
                ymax=bbox.get("ymax", 0),
            )

        return cls(keypoints=kp_objects, bbox=bbox_obj)

    def get_keypoint(self, index: int) -> Optional[Keypoint]:
        """Get keypoint by COCO index (0-16)."""
        if 0 <= index < len(self.keypoints):
            return self.keypoints[index]
        return None

    @property
    def nose(self) -> Optional[Keypoint]:
        return self.get_keypoint(POSE_NOSE)

    @property
    def left_shoulder(self) -> Optional[Keypoint]:
        return self.get_keypoint(POSE_LEFT_SHOULDER)

    @property
    def right_shoulder(self) -> Optional[Keypoint]:
        return self.get_keypoint(POSE_RIGHT_SHOULDER)


@dataclass
class AIModelInfo:
    """
    Information about an AI model available on the robot.

    Attributes:
        name: Model identifier (e.g., "yolov6n")
        type: Model type (detection, pose, hand, etc.)
        description: Human-readable description
        classes: Number of classes (for detection models)
        slug: Luxonis Model Hub slug
        active: Whether this model is currently loaded
    """
    name: str
    type: AiModelType
    description: str = ""
    classes: int = 0
    slug: str = ""
    active: bool = False

    @classmethod
    def from_dict(cls, name: str, data: dict) -> "AIModelInfo":
        """Create from robot's model info dict."""
        return cls(
            name=name,
            type=AiModelType(data.get("type", "detection")),
            description=data.get("description", ""),
            classes=data.get("classes", 0),
            slug=data.get("slug", ""),
            active=data.get("active", False),
        )


@dataclass
class CameraFrame:
    """
    Single camera frame with metadata.

    Attributes:
        jpeg_bytes: Raw JPEG image data
        frame_id: Sequential frame number
        timestamp_ns: Timestamp in nanoseconds
        timestamp: Timestamp as float seconds
    """
    jpeg_bytes: bytes
    frame_id: int = 0
    timestamp_ns: int = 0

    @property
    def timestamp(self) -> float:
        """Get timestamp as seconds."""
        return self.timestamp_ns / 1e9

    def to_numpy(self) -> Optional[np.ndarray]:
        """
        Decode JPEG to numpy array (requires cv2).

        Returns:
            BGR image as numpy array, or None if decoding fails.
        """
        try:
            import cv2
            nparr = np.frombuffer(self.jpeg_bytes, np.uint8)
            return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        except ImportError:
            logger.warning("OpenCV not installed. Cannot decode JPEG.")
            return None
        except Exception as e:
            logger.warning(f"Failed to decode JPEG: {e}")
            return None


# ==================== RECEIVER HELPERS ====================


class CameraFrameReceiver:
    """
    Buffers camera frames for processing.

    Similar to AudioStreamReceiver, this class buffers incoming frames
    and provides easy access to the latest frame or all buffered frames.

    Example:
        >>> receiver = CameraFrameReceiver(max_buffer=30)
        >>> sub = robot.subscribe_camera_image(receiver.on_frame)
        >>> time.sleep(5)
        >>> sub.unsubscribe()
        >>>
        >>> frame = receiver.get_latest()
        >>> if frame:
        ...     print(f"Got frame {frame.frame_id}")
    """

    def __init__(self, max_buffer: int = 30):
        """
        Initialize frame receiver.

        Args:
            max_buffer: Maximum number of frames to buffer.
        """
        self.max_buffer = max_buffer
        self._frames: List[CameraFrame] = []
        self._lock = threading.Lock()
        self._frame_count = 0

    def on_frame(self, jpeg_bytes: bytes) -> None:
        """
        Callback for incoming frame data.

        Pass this method to robot.subscribe_camera_image().
        """
        self._frame_count += 1
        frame = CameraFrame(
            jpeg_bytes=jpeg_bytes,
            frame_id=self._frame_count,
            timestamp_ns=int(time.time() * 1e9),
        )

        with self._lock:
            self._frames.append(frame)
            if len(self._frames) > self.max_buffer:
                self._frames.pop(0)

    def get_latest(self) -> Optional[CameraFrame]:
        """Get the most recent frame, or None if no frames received."""
        with self._lock:
            if self._frames:
                return self._frames[-1]
            return None

    def get_all(self) -> List[CameraFrame]:
        """Get all buffered frames and clear the buffer."""
        with self._lock:
            frames = self._frames.copy()
            self._frames.clear()
            return frames

    def clear(self) -> None:
        """Clear the frame buffer."""
        with self._lock:
            self._frames.clear()

    @property
    def frame_count(self) -> int:
        """Total number of frames received."""
        return self._frame_count

    @property
    def buffer_size(self) -> int:
        """Current number of buffered frames."""
        with self._lock:
            return len(self._frames)


class AIDetectionReceiver:
    """
    Buffers AI detection results with FPS and latency tracking.

    This class processes raw detection dicts from the robot and
    converts them to typed Detection/HandLandmarks/PoseKeypoints objects.

    Usage:
        >>> from pib3 import Robot, AIModel, AIDetectionReceiver
        >>> with Robot(host="...") as robot:
        ...     robot.set_ai_model(AIModel.HAND)
        ...     receiver = AIDetectionReceiver()
        ...     sub = robot.subscribe_ai_detections(receiver.on_detection)
        ...
        ...     # Just works - waits automatically for results
        ...     for hand in receiver.get_hand_landmarks():
        ...         print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
        ...         servos = hand.finger_angles.to_servo_values()
        ...         robot.set_joints({"index_left_stretch": servos["index"]})
        ...
        ...     print(f"FPS: {receiver.fps:.1f}, Latency: {receiver.avg_latency_ms:.1f}ms")
        ...     sub.unsubscribe()
    """

    def __init__(self, max_buffer: int = 100):
        """
        Initialize detection receiver.

        Args:
            max_buffer: Maximum number of detection results to buffer.
        """
        self.max_buffer = max_buffer
        self._results: List[dict] = []  # Raw results
        self._lock = threading.Lock()
        self._new_data = threading.Event()

        # FPS tracking
        self._frame_times: List[float] = []
        self._latencies: List[float] = []
        self._fps_window = 30  # Calculate FPS over last N frames

    def on_detection(self, data: dict) -> None:
        """
        Callback for incoming detection data.

        Pass this method to robot.subscribe_ai_detections().
        """
        now = time.time()

        with self._lock:
            # Store raw result
            self._results.append(data)
            if len(self._results) > self.max_buffer:
                self._results.pop(0)

            # Track timing
            self._frame_times.append(now)
            if len(self._frame_times) > self._fps_window:
                self._frame_times.pop(0)

            # Track latency
            latency_ms = data.get("latency_ms", 0)
            if latency_ms > 0:
                self._latencies.append(latency_ms)
                if len(self._latencies) > self._fps_window:
                    self._latencies.pop(0)

        # Signal that new data is available
        self._new_data.set()

    def _wait_for_data(self, timeout: float) -> None:
        """Wait until data is available or timeout."""
        if timeout <= 0:
            return
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._results:
                    return
            self._new_data.clear()
            remaining = deadline - time.time()
            if remaining > 0:
                self._new_data.wait(timeout=min(0.05, remaining))

    @property
    def fps(self) -> float:
        """Calculate current frames per second."""
        with self._lock:
            if len(self._frame_times) < 2:
                return 0.0
            duration = self._frame_times[-1] - self._frame_times[0]
            if duration < 0.001:
                return 0.0
            return (len(self._frame_times) - 1) / duration

    @property
    def avg_latency_ms(self) -> float:
        """Average inference latency in milliseconds."""
        with self._lock:
            if not self._latencies:
                return 0.0
            return sum(self._latencies) / len(self._latencies)

    def get_latest_raw(self) -> Optional[dict]:
        """Get the most recent raw detection result."""
        with self._lock:
            if self._results:
                return self._results[-1]
            return None

    def get_detections(self, timeout: float = 5.0) -> List[Detection]:
        """
        Get buffered object detections.

        Waits automatically if no results are available yet.

        Args:
            timeout: How long to wait for results if buffer is empty.
                     Use timeout=0 for immediate (non-blocking) return.

        Returns:
            List of Detection objects (may be empty if timeout=0 and no data).
        """
        self._wait_for_data(timeout)
        detections = []
        with self._lock:
            for result in self._results:
                if result.get("type") == "detection":
                    for det_dict in result.get("result", {}).get("detections", []):
                        detections.append(Detection.from_dict(det_dict))
        return detections

    def get_hand_landmarks(self, timeout: float = 5.0) -> List[HandLandmarks]:
        """
        Get buffered hand tracking results.

        Waits automatically if no results are available yet.

        Args:
            timeout: How long to wait for results if buffer is empty.
                     Use timeout=0 for immediate (non-blocking) return.

        Returns:
            List of HandLandmarks objects with finger angles.
        """
        self._wait_for_data(timeout)
        hands = []
        with self._lock:
            for result in self._results:
                if result.get("type") == "hand":
                    keypoints = result.get("result", {}).get("keypoints", [])
                    if keypoints:
                        hands.append(HandLandmarks.from_keypoints_list(keypoints))
        return hands

    def get_poses(self, timeout: float = 5.0) -> List[PoseKeypoints]:
        """
        Get buffered pose estimation results.

        Waits automatically if no results are available yet.

        Args:
            timeout: How long to wait for results if buffer is empty.
                     Use timeout=0 for immediate (non-blocking) return.

        Returns:
            List of PoseKeypoints objects.
        """
        self._wait_for_data(timeout)
        poses = []
        with self._lock:
            for result in self._results:
                if result.get("type") == "pose":
                    # Handle both formats: keypoints list or detections with keypoints
                    res = result.get("result", {})

                    if "keypoints" in res:
                        poses.append(PoseKeypoints.from_keypoints_list(res["keypoints"]))
                    elif "detections" in res:
                        for det in res["detections"]:
                            if "keypoints" in det:
                                poses.append(PoseKeypoints.from_keypoints_list(
                                    det["keypoints"],
                                    bbox=det.get("bbox")
                                ))
        return poses

    def clear(self) -> None:
        """Clear all buffers."""
        with self._lock:
            self._results.clear()
            self._frame_times.clear()
            self._latencies.clear()
        self._new_data.clear()

    @property
    def result_count(self) -> int:
        """Total number of results in buffer."""
        with self._lock:
            return len(self._results)


# ==================== SUBSYSTEM CLASSES ====================


class AISubsystem:
    """
    AI inference subsystem for the robot's OAK-D Lite camera.

    Provides simple access to AI model results without manual subscription management.
    The subsystem automatically handles subscription lifecycle.

    Accessed via `robot.ai`:
        >>> robot.ai.set_model(AIModel.HAND)
        >>> for hand in robot.ai.get_hand_landmarks():
        ...     print(f"{hand.handedness}: {hand.finger_angles.index:.0f}°")
        >>> print(f"FPS: {robot.ai.fps:.1f}")
    """

    def __init__(self, robot: "RealRobotBackend"):
        """
        Initialize AI subsystem.

        Args:
            robot: Parent robot backend instance.
        """
        self._robot = robot
        self._receiver = AIDetectionReceiver()
        self._subscription = None
        self._current_model: Optional[str] = None

    def _ensure_subscribed(self) -> None:
        """Ensure we're subscribed to AI detections."""
        if self._subscription is None and self._robot.is_connected:
            self._subscription = self._robot.subscribe_ai_detections(
                self._receiver.on_detection
            )

    def set_model(self, model: "Union[AIModel, str]", timeout: float = 5.0) -> bool:
        """
        Switch AI model on the OAK-D Lite camera.

        Args:
            model: AI model to load (AIModel enum or string name).
            timeout: Max time to wait for model switch confirmation.

        Returns:
            True if model switch confirmed, False if timeout.

        Example:
            >>> robot.ai.set_model(AIModel.HAND)
            >>> robot.ai.set_model(AIModel.YOLOV8N)
        """
        # Import here to avoid circular import
        from ..types import AIModel

        model_name = model.value if isinstance(model, AIModel) else str(model)
        success = self._robot.set_ai_model(model_name, timeout)
        if success:
            self._current_model = model_name
            self._receiver.clear()  # Clear old results from different model
            self._ensure_subscribed()
        return success

    @property
    def model(self) -> Optional[str]:
        """Currently active AI model name."""
        return self._current_model

    @property
    def fps(self) -> float:
        """Current inference frames per second."""
        return self._receiver.fps

    @property
    def avg_latency_ms(self) -> float:
        """Average inference latency in milliseconds."""
        return self._receiver.avg_latency_ms

    def get_detections(self, timeout: float = 5.0) -> List[Detection]:
        """
        Get object detections from detection models (YOLO, MobileNet-SSD, etc.).

        Waits automatically for results if buffer is empty.

        Args:
            timeout: How long to wait for results. Use 0 for non-blocking.

        Returns:
            List of Detection objects.
        """
        self._ensure_subscribed()
        return self._receiver.get_detections(timeout)

    def get_hand_landmarks(self, timeout: float = 5.0) -> List[HandLandmarks]:
        """
        Get hand tracking results with finger angles.

        Waits automatically for results if buffer is empty.

        Args:
            timeout: How long to wait for results. Use 0 for non-blocking.

        Returns:
            List of HandLandmarks objects with finger angles.

        Example:
            >>> robot.ai.set_model(AIModel.HAND)
            >>> for hand in robot.ai.get_hand_landmarks():
            ...     print(f"{hand.handedness}: index={hand.finger_angles.index:.0f}°")
            ...     servos = hand.finger_angles.to_servo_values()
            ...     robot.set_joints({"index_left_stretch": servos["index"]})
        """
        self._ensure_subscribed()
        return self._receiver.get_hand_landmarks(timeout)

    def get_poses(self, timeout: float = 5.0) -> List[PoseKeypoints]:
        """
        Get body pose estimation results.

        Waits automatically for results if buffer is empty.

        Args:
            timeout: How long to wait for results. Use 0 for non-blocking.

        Returns:
            List of PoseKeypoints objects.
        """
        self._ensure_subscribed()
        return self._receiver.get_poses(timeout)

    def clear(self) -> None:
        """Clear buffered results."""
        self._receiver.clear()

    def stop(self) -> None:
        """Stop AI inference (unsubscribe from detections)."""
        if self._subscription is not None:
            try:
                self._subscription.unsubscribe()
            except Exception:
                pass
            self._subscription = None


class CameraSubsystem:
    """
    RGB camera subsystem for the robot's OAK-D Lite camera.

    Provides access to raw camera frames (separate from AI inference results).

    Accessed via `robot.camera`:
        >>> frame = robot.camera.get_frame()
        >>> if frame:
        ...     img = frame.to_numpy()  # Requires OpenCV
    """

    def __init__(self, robot: "RealRobotBackend"):
        """
        Initialize camera subsystem.

        Args:
            robot: Parent robot backend instance.
        """
        self._robot = robot
        self._receiver = CameraFrameReceiver()
        self._subscription = None

    def _ensure_subscribed(self) -> None:
        """Ensure we're subscribed to camera frames."""
        if self._subscription is None and self._robot.is_connected:
            self._subscription = self._robot.subscribe_camera_image(
                self._receiver.on_frame
            )

    def get_frame(self, timeout: float = 5.0) -> Optional[CameraFrame]:
        """
        Get the latest camera frame.

        Args:
            timeout: How long to wait for a frame if none available.

        Returns:
            CameraFrame object or None if timeout.
        """
        self._ensure_subscribed()
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self._receiver.get_latest()
            if frame:
                return frame
            time.sleep(0.05)
        return self._receiver.get_latest()

    def get_frames(self) -> List[CameraFrame]:
        """Get all buffered frames and clear the buffer."""
        self._ensure_subscribed()
        return self._receiver.get_all()

    @property
    def frame_count(self) -> int:
        """Total number of frames received."""
        return self._receiver.frame_count

    def configure(
        self,
        fps: Optional[int] = None,
        quality: Optional[int] = None,
        resolution: Optional[tuple] = None,
    ) -> None:
        """
        Configure camera settings.

        Args:
            fps: Frames per second (e.g., 30).
            quality: JPEG quality 1-100 (e.g., 80).
            resolution: (width, height) tuple (e.g., (1280, 720)).
        """
        self._robot.set_camera_config(fps, quality, resolution)

    def stop(self) -> None:
        """Stop camera streaming."""
        if self._subscription is not None:
            try:
                self._subscription.unsubscribe()
            except Exception:
                pass
            self._subscription = None




def parse_ai_result(data: dict) -> Union[List[Detection], List[HandLandmarks], List[PoseKeypoints], dict]:
    """
    Parse AI detection result into typed objects based on model type.

    Args:
        data: Raw detection result from robot.

    Returns:
        List of appropriate typed objects, or raw dict if type unknown.
    """
    model_type = data.get("type", "")
    result = data.get("result", {})

    if model_type == "detection" or model_type == "instance-segmentation":
        detections = []
        for det_dict in result.get("detections", []):
            detections.append(Detection.from_dict(det_dict))
        return detections

    elif model_type == "hand":
        keypoints = result.get("keypoints", [])
        if keypoints:
            return [HandLandmarks.from_keypoints_list(keypoints)]
        return []

    elif model_type == "pose":
        poses = []
        if "keypoints" in result:
            poses.append(PoseKeypoints.from_keypoints_list(result["keypoints"]))

        elif "detections" in result:
            for det in result["detections"]:
                if "keypoints" in det:
                    poses.append(PoseKeypoints.from_keypoints_list(
                        det["keypoints"],
                        bbox=det.get("bbox")
                    ))
        return poses

    # Unknown type - return raw
    return result
