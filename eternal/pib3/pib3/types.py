"""Core data types for pib3 package."""

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import List, Optional, Tuple
import numpy as np
from enum import Enum


class ImuType(str, Enum):
    """Types of IMU data streams available."""
    FULL = "full"
    ACCELEROMETER = "accelerometer"
    GYROSCOPE = "gyroscope"


class AIModel(str, Enum):
    """Available AI models on the OAK-D Lite camera.

    Use these enum values instead of strings for better IDE support:
        >>> robot.set_ai_model(AIModel.HAND)
        >>> robot.set_ai_model(AIModel.YOLOV8N)

    String values still work for backward compatibility:
        >>> robot.set_ai_model("hand")  # Also valid
    """

    # Detection models
    MOBILENET_SSD = "mobilenet-ssd"
    YOLOV6N = "yolov6n"
    YOLOV8N = "yolov8n"
    YOLO11S = "yolo11s"
    YOLO11N = "yolo11n"

    # Hand tracking
    HAND = "hand"

    # Pose estimation
    POSE_YOLO = "pose_yolo"
    POSE = "pose"

    # Segmentation
    DEEPLABV3 = "deeplabv3"
    YOLOV8N_SEG = "yolov8n-seg"
    FASTSAM = "fastsam"

    # Gaze estimation
    GAZE = "gaze"

    # Line detection
    LINES = "lines"


class Joint(str, Enum):
    """PIB robot joint names for IDE tab completion.

    Use these enum values instead of strings for better IDE support:
        >>> robot.set_joint(Joint.ELBOW_LEFT, 50.0)
        >>> robot.get_joint(Joint.SHOULDER_VERTICAL_RIGHT)

    String values still work for backward compatibility:
        >>> robot.set_joint("elbow_left", 50.0)  # Also valid
    """

    # Head
    TURN_HEAD = "turn_head_motor"
    TILT_HEAD = "tilt_forward_motor"

    # Left arm
    SHOULDER_VERTICAL_LEFT = "shoulder_vertical_left"
    SHOULDER_HORIZONTAL_LEFT = "shoulder_horizontal_left"
    UPPER_ARM_LEFT_ROTATION = "upper_arm_left_rotation"
    ELBOW_LEFT = "elbow_left"
    LOWER_ARM_LEFT_ROTATION = "lower_arm_left_rotation"
    WRIST_LEFT = "wrist_left"

    # Left hand
    THUMB_LEFT_OPPOSITION = "thumb_left_opposition"
    THUMB_LEFT_STRETCH = "thumb_left_stretch"
    INDEX_LEFT = "index_left_stretch"
    MIDDLE_LEFT = "middle_left_stretch"
    RING_LEFT = "ring_left_stretch"
    PINKY_LEFT = "pinky_left_stretch"

    # Right arm
    SHOULDER_VERTICAL_RIGHT = "shoulder_vertical_right"
    SHOULDER_HORIZONTAL_RIGHT = "shoulder_horizontal_right"
    UPPER_ARM_RIGHT_ROTATION = "upper_arm_right_rotation"
    ELBOW_RIGHT = "elbow_right"
    LOWER_ARM_RIGHT_ROTATION = "lower_arm_right_rotation"
    WRIST_RIGHT = "wrist_right"

    # Right hand
    THUMB_RIGHT_OPPOSITION = "thumb_right_opposition"
    THUMB_RIGHT_STRETCH = "thumb_right_stretch"
    INDEX_RIGHT = "index_right_stretch"
    MIDDLE_RIGHT = "middle_right_stretch"
    RING_RIGHT = "ring_right_stretch"
    PINKY_RIGHT = "pinky_right_stretch"


# Joint groups for convenience
LEFT_HAND_JOINTS: List[Joint] = [
    Joint.THUMB_LEFT_OPPOSITION,
    Joint.THUMB_LEFT_STRETCH,
    Joint.INDEX_LEFT,
    Joint.MIDDLE_LEFT,
    Joint.RING_LEFT,
    Joint.PINKY_LEFT,
]

RIGHT_HAND_JOINTS: List[Joint] = [
    Joint.THUMB_RIGHT_OPPOSITION,
    Joint.THUMB_RIGHT_STRETCH,
    Joint.INDEX_RIGHT,
    Joint.MIDDLE_RIGHT,
    Joint.RING_RIGHT,
    Joint.PINKY_RIGHT,
]


class HandPose(Enum):
    """Hand pose presets (values in percent).

    Physical mapping: 0% = bent/closed, 100% = stretched/open

    Use with robot.set_joints():
        >>> robot.set_joints(HandPose.LEFT_OPEN)
        >>> robot.set_joints(HandPose.RIGHT_CLOSED)

    For partial grip, use the joint lists:
        >>> robot.set_joints({j: 50.0 for j in LEFT_HAND_JOINTS})  # 50% = half grip
    """

    # Values are wrapped in MappingProxyType so callers can't mutate the
    # shared pose dict and corrupt every future set_joints(HandPose.X) call.
    LEFT_OPEN = MappingProxyType({
        "thumb_left_opposition": 100.0,
        "thumb_left_stretch": 100.0,
        "index_left_stretch": 100.0,
        "middle_left_stretch": 100.0,
        "ring_left_stretch": 100.0,
        "pinky_left_stretch": 100.0,
    })

    LEFT_CLOSED = MappingProxyType({
        "thumb_left_opposition": 0.0,
        "thumb_left_stretch": 0.0,
        "index_left_stretch": 0.0,
        "middle_left_stretch": 0.0,
        "ring_left_stretch": 0.0,
        "pinky_left_stretch": 0.0,
    })

    RIGHT_OPEN = MappingProxyType({
        "thumb_right_opposition": 100.0,
        "thumb_right_stretch": 100.0,
        "index_right_stretch": 100.0,
        "middle_right_stretch": 100.0,
        "ring_right_stretch": 100.0,
        "pinky_right_stretch": 100.0,
    })

    RIGHT_CLOSED = MappingProxyType({
        "thumb_right_opposition": 0.0,
        "thumb_right_stretch": 0.0,
        "index_right_stretch": 0.0,
        "middle_right_stretch": 0.0,
        "ring_right_stretch": 0.0,
        "pinky_right_stretch": 0.0,
    })


@dataclass
class Stroke:
    """A single continuous drawing stroke (pen-down motion).

    Represents a sequence of 2D points that form a continuous line
    drawn without lifting the pen.

    Attributes:
        points: Array of shape (N, 2) with normalized [0,1] coordinates.
                (0,0) = top-left of drawing area, (1,1) = bottom-right.
        closed: If True, the stroke forms a closed loop (first and last
                points should be connected).
    """
    points: np.ndarray
    closed: bool = False

    def __post_init__(self):
        """Ensure points is a numpy array with correct shape (N, 2)."""
        self.points = np.asarray(self.points, dtype=np.float64)
        if self.points.ndim == 1:
            if self.points.shape[0] % 2 != 0:
                raise ValueError(
                    f"1D points array must have an even number of elements "
                    f"to reshape to (N, 2), got {self.points.shape[0]}"
                )
            self.points = self.points.reshape(-1, 2)
        if self.points.ndim != 2 or self.points.shape[1] != 2:
            raise ValueError(
                f"Points must have shape (N, 2), got {self.points.shape}"
            )

    def __repr__(self) -> str:
        closed_str = ", closed" if self.closed else ""
        return f"Stroke({len(self.points)} points, length={self.length():.3f}{closed_str})"

    def __len__(self) -> int:
        """Return number of points in the stroke."""
        return len(self.points)

    def length(self) -> float:
        """Calculate total arc length of the stroke."""
        if len(self.points) < 2:
            return 0.0
        diffs = np.diff(self.points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return float(np.sum(distances))

    def reverse(self) -> "Stroke":
        """Return a new stroke with reversed point order."""
        return Stroke(points=self.points[::-1].copy(), closed=self.closed)

    def start(self) -> np.ndarray:
        """Return the starting point of the stroke."""
        return self.points[0]

    def end(self) -> np.ndarray:
        """Return the ending point of the stroke."""
        return self.points[-1]


@dataclass
class Sketch:
    """A collection of strokes extracted from an image.

    Represents a complete drawing as a list of individual strokes,
    typically in optimized drawing order to minimize pen-up travel.

    Attributes:
        strokes: List of Stroke objects in drawing order.
        source_size: Original image dimensions (width, height) in pixels.
                     None if not created from an image.
    """
    strokes: List[Stroke] = field(default_factory=list)
    source_size: Optional[Tuple[int, int]] = None

    def __repr__(self) -> str:
        size_str = f", source={self.source_size[0]}x{self.source_size[1]}" if self.source_size else ""
        return f"Sketch({len(self.strokes)} strokes, {self.total_points()} points{size_str})"

    def __len__(self) -> int:
        """Return number of strokes in the sketch."""
        return len(self.strokes)

    def __iter__(self):
        """Iterate over strokes."""
        return iter(self.strokes)

    def __getitem__(self, idx) -> Stroke:
        """Get stroke by index."""
        return self.strokes[idx]

    def total_points(self) -> int:
        """Return total number of points across all strokes."""
        return sum(len(s) for s in self.strokes)

    def total_length(self) -> float:
        """Return total arc length of all strokes."""
        return sum(s.length() for s in self.strokes)

    def bounds(self) -> Tuple[float, float, float, float]:
        """Return bounding box (min_u, min_v, max_u, max_v) of all strokes.

        Raises:
            ValueError: If the sketch is empty (no strokes or no points).
                Callers should check ``total_points() > 0`` first, or handle
                the exception — returning a dummy ``(0, 0, 1, 1)`` silently
                has caused downstream scaling bugs.
        """
        if not self.strokes:
            raise ValueError("Cannot compute bounds of an empty sketch (no strokes)")
        all_points = np.vstack([s.points for s in self.strokes])
        if all_points.size == 0:
            raise ValueError("Cannot compute bounds of a sketch with no points")
        min_coords = all_points.min(axis=0)
        max_coords = all_points.max(axis=0)
        return (
            float(min_coords[0]),
            float(min_coords[1]),
            float(max_coords[0]),
            float(max_coords[1]),
        )

    def add_stroke(self, stroke: Stroke) -> None:
        """Add a stroke to the sketch."""
        self.strokes.append(stroke)

    def to_dict(self) -> dict:
        """Convert sketch to a JSON-serializable dictionary."""
        return {
            "strokes": [
                {
                    "points": s.points.tolist(),
                    "closed": s.closed,
                }
                for s in self.strokes
            ],
            "source_size": self.source_size,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Sketch":
        """Create a Sketch from a dictionary (e.g., loaded from JSON)."""
        strokes = [
            Stroke(
                points=np.array(s["points"]),
                closed=s.get("closed", False),
            )
            for s in data.get("strokes", [])
        ]
        return cls(
            strokes=strokes,
            source_size=tuple(data["source_size"]) if data.get("source_size") else None,
        )
