"""Configuration dataclasses for pib3 package."""

from dataclasses import dataclass, field
from typing import Dict, Literal, Optional


@dataclass
class PaperConfig:
    """Configuration for the drawing surface (paper).

    Attributes:
        start_x: Paper front edge X position in meters (distance from robot).
        size: Paper width/height in meters (assumes square paper).
        height_z: Table/paper height in meters (Z coordinate).
        center_y: Paper center Y position. If None, auto-calculated based on arm reach.
        drawing_scale: Scale factor for drawing within paper bounds (0.0-1.0).
        lift_height: Pen-up distance in meters when moving between strokes.
    """
    start_x: float = 0.25
    size: float = 0.10
    height_z: float = 0.85
    center_y: Optional[float] = None
    drawing_scale: float = 0.8
    lift_height: float = 0.03


@dataclass
class IKConfig:
    """Configuration for the inverse kinematics solver.

    Attributes:
        max_iterations: Maximum iterations for gradient descent.
        tolerance: Position tolerance in meters.
        step_size: Gradient descent step size.
        damping: Damping factor for damped least squares.
        arm: Which arm to use for drawing ("left" or "right").
        grip_style: Drawing grip style:
            - "index_finger": Use extended index finger as drawing tool (default).
            - "pencil_grip": Clenched fist holding a pencil, tip near pinky base.
    """
    max_iterations: int = 300
    tolerance: float = 0.002
    step_size: float = 0.2
    damping: float = 0.1
    arm: str = "left"
    grip_style: Literal["index_finger", "pencil_grip"] = "index_finger"


@dataclass
class ImageConfig:
    """Configuration for image-to-sketch conversion.

    Attributes:
        threshold: Black/white threshold (0-255).
        auto_foreground: Automatically detect minority pixels as foreground.
        simplify_tolerance: Douglas-Peucker simplification tolerance in pixels.
        min_contour_length: Minimum contour length in pixels.
        min_contour_points: Minimum vertices after simplification.
        margin: Margin/padding in normalized coordinates (0.0-1.0).
        optimize_path_order: Use TSP optimization to minimize pen-up travel.
    """
    threshold: int = 128
    auto_foreground: bool = True
    simplify_tolerance: float = 2.0
    min_contour_length: int = 10
    min_contour_points: int = 3
    margin: float = 0.05
    optimize_path_order: bool = True


@dataclass
class TrajectoryConfig:
    """Full configuration for trajectory generation.

    Combines paper, IK, and image settings with additional parameters.

    Attributes:
        paper: Paper/drawing surface configuration.
        ik: Inverse kinematics solver configuration.
        image: Image processing configuration.
        point_density: Interpolation density for smooth motion.
    """
    paper: PaperConfig = field(default_factory=PaperConfig)
    ik: IKConfig = field(default_factory=IKConfig)
    image: ImageConfig = field(default_factory=ImageConfig)
    point_density: float = 0.01


@dataclass
class LowLatencyConfig:
    """Configuration for low-latency direct Tinkerforge motor control.

    Bypasses ROS/rosbridge for motor commands, sending directly to
    Tinkerforge servo bricklets for reduced latency (~5-20ms vs ~100-200ms).

    This is the default motor control mode. Servo bricklets are
    auto-discovered on connect. Use ``motor_control="ros"`` in the
    Robot constructor to use ROS-based motor control instead.

    Attributes:
        enabled: Enable low-latency mode for motor commands (default: True).
        tinkerforge_host: Tinkerforge brick daemon host (usually robot IP).
        tinkerforge_port: Tinkerforge brick daemon port (default: 4223).
        motor_mapping: Dict mapping motor names to (bricklet_uid, channel) tuples.
            If None, uses auto-discovery (recommended).
        sync_to_ros: If True, update local position cache after setting.
            This ensures get_joint() returns correct values after low-latency sets.
            Note: Does NOT publish to ROS topics (that would cause double commands).
        command_timeout: Timeout for direct motor commands in seconds.
    """
    enabled: bool = True
    tinkerforge_host: Optional[str] = None  # Defaults to RobotConfig.host
    tinkerforge_port: int = 4223
    motor_mapping: Optional[Dict[str, tuple]] = None
    sync_to_ros: bool = True  # Default to True - keeps local cache updated
    command_timeout: float = 0.5


@dataclass
class RobotConfig:
    """Configuration for real robot connection.

    Attributes:
        host: Robot IP address.
        port: Rosbridge websocket port.
        timeout: Connection timeout in seconds.
        low_latency: Advanced Tinkerforge motor control settings.
            Motor commands use Tinkerforge by default (enabled=True).
            Set enabled=False to use ROS for motor control instead.
    """
    host: str = "172.26.34.149"
    port: int = 9090
    timeout: float = 5.0
    low_latency: LowLatencyConfig = field(default_factory=LowLatencyConfig)
