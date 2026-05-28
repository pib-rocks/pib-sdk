"""Real robot backend via rosbridge for pib3 package."""

import json
import logging
import math
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple, Union

import numpy as np

logger = logging.getLogger(__name__)

# Lazy import: roslibpy is optional, only needed for the real robot backend.
# connect() raises a clear error if it is not installed.
try:
    import roslibpy
except ImportError:
    roslibpy = None  # type: ignore[assignment]

from .base import RobotBackend
from .audio import AudioOutput, AudioInput, RobotAudioPlayer, RobotAudioRecorder, DEFAULT_SAMPLE_RATE
from ..config import RobotConfig, LowLatencyConfig
from ..types import ImuType, AIModel

# Type alias for Tinkerforge motor mapping: motor_name -> (bricklet_uid, channel)
TinkerforgeMotorMapping = Dict[str, Tuple[str, int]]


# Standard PIB servo channel assignments (same wiring for all robots).
# Maps motor names to (bricklet_number, channel) tuples.
# Bricklet UIDs differ per robot, but bricklet/channel assignments are consistent.
#
# Reference UIDs (from webapp / experts):
#   Servo Bricklet 1: 2cPP  -> Right arm + hand
#   Servo Bricklet 2: 2cPm  -> Shoulders (horizontal + vertical), head
#   Servo Bricklet 3: 2cPQ  -> Left arm + hand
PIB_SERVO_CHANNELS = {
    # Servo Bricklet 1 - Right arm + hand
    "upper_arm_right_rotation": (1, 9),
    "elbow_right": (1, 8),
    "lower_arm_right_rotation": (1, 7),
    "wrist_right": (1, 6),
    "thumb_right_opposition": (1, 0),
    "thumb_right_stretch": (1, 1),
    "index_right_stretch": (1, 2),
    "middle_right_stretch": (1, 3),
    "ring_right_stretch": (1, 4),
    "pinky_right_stretch": (1, 5),

    # Servo Bricklet 2 - Shoulders, head
    "shoulder_horizontal_right": (2, 0),
    "shoulder_vertical_right": (2, 1),
    "turn_head_motor": (2, 4),
    "tilt_forward_motor": (2, 5),
    "shoulder_horizontal_left": (2, 8),
    "shoulder_vertical_left": (2, 9),

    # Servo Bricklet 3 - Left arm + hand
    "upper_arm_left_rotation": (3, 9),
    "elbow_left": (3, 8),
    "lower_arm_left_rotation": (3, 7),
    "wrist_left": (3, 6),
    "thumb_left_opposition": (3, 0),
    "thumb_left_stretch": (3, 1),
    "index_left_stretch": (3, 2),
    "middle_left_stretch": (3, 3),
    "ring_left_stretch": (3, 4),
    "pinky_left_stretch": (3, 5),
}

# Known servo bricklet UIDs from the webapp (experts' reference).
# Used to validate auto-discovery ordering.
# Maps bricklet role number -> expected UID.
KNOWN_SERVO_UIDS = {
    1: "2cPP",  # Right arm + hand
    2: "2cPm",  # Shoulders (horizontal + vertical), head
    3: "2cPQ",  # Left arm + hand
}


def build_motor_mapping(
    servo1_uid: str,
    servo2_uid: str,
    servo3_uid: str,
) -> TinkerforgeMotorMapping:
    """
    Build motor mapping from servo bricklet UIDs.

    Uses the standard PIB wiring where:
    - servo1: Right arm + right hand
    - servo2: Shoulders (horizontal + vertical), head
    - servo3: Left arm + left hand

    Args:
        servo1_uid: UID of servo bricklet for right arm + hand.
        servo2_uid: UID of servo bricklet for shoulders + head.
        servo3_uid: UID of servo bricklet for left arm + hand.

    Returns:
        Complete motor mapping dict for LowLatencyConfig.

    Example:
        >>> uids = robot.discover_servo_bricklets()
        >>> # Determine which UID is which by testing
        >>> mapping = build_motor_mapping(
        ...     servo1_uid=uids[0],  # Right arm
        ...     servo2_uid=uids[1],  # Shoulders + head
        ...     servo3_uid=uids[2],  # Left arm
        ... )
        >>> robot.configure_motor_mapping(mapping)
    """
    return {
        # Servo 1 - Right arm + hand
        "upper_arm_right_rotation": (servo1_uid, 9),
        "elbow_right": (servo1_uid, 8),
        "lower_arm_right_rotation": (servo1_uid, 7),
        "wrist_right": (servo1_uid, 6),
        "thumb_right_opposition": (servo1_uid, 0),
        "thumb_right_stretch": (servo1_uid, 1),
        "index_right_stretch": (servo1_uid, 2),
        "middle_right_stretch": (servo1_uid, 3),
        "ring_right_stretch": (servo1_uid, 4),
        "pinky_right_stretch": (servo1_uid, 5),

        # Servo 2 - Shoulders + head
        "shoulder_horizontal_right": (servo2_uid, 0),
        "shoulder_vertical_right": (servo2_uid, 1),
        "turn_head_motor": (servo2_uid, 4),
        "tilt_forward_motor": (servo2_uid, 5),
        "shoulder_horizontal_left": (servo2_uid, 8),
        "shoulder_vertical_left": (servo2_uid, 9),

        # Servo 3 - Left arm + hand
        "upper_arm_left_rotation": (servo3_uid, 9),
        "elbow_left": (servo3_uid, 8),
        "lower_arm_left_rotation": (servo3_uid, 7),
        "wrist_left": (servo3_uid, 6),
        "thumb_left_opposition": (servo3_uid, 0),
        "thumb_left_stretch": (servo3_uid, 1),
        "index_left_stretch": (servo3_uid, 2),
        "middle_left_stretch": (servo3_uid, 3),
        "ring_left_stretch": (servo3_uid, 4),
        "pinky_left_stretch": (servo3_uid, 5),
    }


class _CompositeImuSubscription:
    """Bundle accel + gyro subscriptions behind a single .unsubscribe()."""

    def __init__(self, *topics):
        self._topics = topics

    def unsubscribe(self) -> None:
        for topic in self._topics:
            try:
                topic.unsubscribe()
            except Exception as exc:
                logger.debug("IMU topic unsubscribe failed: %s", exc)


class RealRobotBackend(RobotBackend):
    """
    Control the real PIB robot.

    By default, motor commands are sent directly to Tinkerforge servo
    bricklets for low latency (~5-20 ms). Servo bricklets are
    auto-discovered on connect — no manual configuration needed.

    ROS/rosbridge is still connected for audio, camera, and AI
    subsystems. To use ROS for motor control instead, pass
    ``motor_mode="ros"``.

    Note:
        Uses joint_limits_robot.yaml for percentage <-> radians conversion.
        Calibrate with: python -m pib3.tools.calibrate_joints

    Example:
        >>> from pib3 import Robot, Joint
        >>> with Robot(host="172.26.34.149") as robot:
        ...     # Motor commands go directly to Tinkerforge (default)
        ...     robot.set_joint(Joint.ELBOW_LEFT, 50.0)
        ...     angle = robot.get_joint(Joint.ELBOW_LEFT)
        ...
        ...     # Save and restore pose
        ...     saved_pose = robot.get_joints()
        ...     robot.set_joints(saved_pose)

        >>> # Use ROS for motor control instead:
        >>> with Robot(host="172.26.34.149", motor_mode="ros") as robot:
        ...     robot.set_joint(Joint.ELBOW_LEFT, 50.0)
    """

    # Use robot-specific joint limits (requires calibration for percentage mode)
    JOINT_LIMITS_FILE = "joint_limits_robot.yaml"

    # Default timeout for waiting for joint data from ROS (seconds)
    DEFAULT_GET_JOINTS_TIMEOUT = 5.0

    # Default Tinkerforge motion configuration (units: 0.01°/s, 0.01°/s², 0.01°/s²).
    # 15000 ≈ 150°/s, which is ~2/3 of the slower MG996R/DS3225MG servos and
    # gives smooth, non-jerky motion. Full-throttle (0 = no limit) is neither
    # safe for fingers nor kind to the gearboxes.
    DEFAULT_MOTION_VELOCITY = 15000
    DEFAULT_MOTION_ACCELERATION = 15000
    DEFAULT_MOTION_DECELERATION = 15000

    def __init__(
        self,
        host: str = "172.26.34.149",
        port: int = 9090,
        timeout: float = 5.0,
        motor_mode: str = "direct",
    ):
        """
        Initialize real robot backend.

        Args:
            host: Robot IP address.
            port: Rosbridge websocket port.
            timeout: Connection timeout in seconds.
            motor_mode: Motor control mode (default: ``"direct"``).
                - ``"direct"``: Send motor commands directly to Tinkerforge
                  servo bricklets for low latency (~5-20 ms). Bricklets are
                  auto-discovered on connect. ROS is still connected for
                  audio, camera, and AI subsystems.
                - ``"ros"``: Send motor commands via ROS/rosbridge
                  (~100-200 ms latency). Use this if Tinkerforge is
                  unavailable or you need ROS-based motor control.
        """
        super().__init__()
        self.host = host
        self.port = port
        self.timeout = timeout
        self._client = None
        self._service = None
        self._motor_settings_service = None
        self._position_subscriber = None
        self._motor_settings_subscriber = None
        # Joint positions received from robot via /joint_trajectory topic
        self._joint_positions: Dict[str, float] = {}
        self._joint_positions_lock = threading.Lock()
        # Motor settings received from robot via /motor_settings topic
        self._motor_settings: Dict[str, Dict] = {}
        self._motor_settings_lock = threading.Lock()

        # Unified audio system
        self._robot_audio_player: Optional[RobotAudioPlayer] = None
        self._robot_audio_recorder: Optional[RobotAudioRecorder] = None

        # Tinkerforge direct motor control
        self._low_latency_config = LowLatencyConfig(
            enabled=(motor_mode == "direct"),
        )
        self._tinkerforge_conn = None
        self._tinkerforge_servos: Dict[str, "BrickletServoV2"] = {}
        self._tinkerforge_motor_map: TinkerforgeMotorMapping = {}
        # Cache enabled state to avoid redundant USB calls: {(uid, channel): bool}
        self._servo_enabled_cache: Dict[Tuple[str, int], bool] = {}
        # Cache last-known (velocity, acceleration, deceleration) per channel
        # so _set_motor_direct can skip a USB round-trip on every call.
        self._motion_config_cache: Dict[Tuple[str, int], Tuple[int, int, int]] = {}
        # Callback-based position reached tracking
        # Reverse map: (bricklet_uid, channel) -> motor_name
        self._tinkerforge_reverse_map: Dict[Tuple[str, int], str] = {}
        # Events signalled by CALLBACK_POSITION_REACHED: motor_name -> Event
        self._position_reached_events: Dict[str, threading.Event] = {}

        # Subsystems (lazy-initialized)
        self._ai_subsystem = None
        self._camera_subsystem = None
        self._audio_subsystem = None

    # ==================== SUBSYSTEM PROPERTIES ====================

    @property
    def ai(self) -> "AISubsystem":
        """
        AI inference subsystem for object detection, hand tracking, and pose estimation.

        Example:
            >>> robot.ai.set_model(AIModel.HAND)
            >>> for hand in robot.ai.get_hand_landmarks():
            ...     print(f"{hand.handedness}: {hand.finger_angles.index:.0f}°")
            >>> print(f"FPS: {robot.ai.fps:.1f}")
        """
        if self._ai_subsystem is None:
            from .camera import AISubsystem
            self._ai_subsystem = AISubsystem(self)
        return self._ai_subsystem

    @property
    def camera(self) -> "CameraSubsystem":
        """
        RGB camera subsystem for raw frame access.

        Example:
            >>> frame = robot.camera.get_frame()
            >>> if frame:
            ...     img = frame.to_numpy()  # Requires OpenCV
        """
        if self._camera_subsystem is None:
            from .camera import CameraSubsystem
            self._camera_subsystem = CameraSubsystem(self)
        return self._camera_subsystem

    @classmethod
    def from_config(cls, config: RobotConfig) -> "RealRobotBackend":
        """Create backend from RobotConfig."""
        motor_mode = "direct" if config.low_latency.enabled else "ros"
        backend = cls(
            host=config.host,
            port=config.port,
            timeout=config.timeout,
            motor_mode=motor_mode,
        )
        # Apply advanced low-latency settings if provided
        backend._low_latency_config = config.low_latency
        return backend

    def _to_backend_format(self, radians: np.ndarray) -> np.ndarray:
        """Convert radians to centidegrees."""
        return np.round(np.degrees(radians) * 100).astype(int)

    def _from_backend_format(self, centidegrees: np.ndarray) -> np.ndarray:
        """Convert centidegrees to radians."""
        return np.radians(np.asarray(centidegrees) / 100.0)

    def _radians_to_centidegrees(self, radians: float) -> int:
        """Convert single value from radians to centidegrees."""
        return round(math.degrees(radians) * 100)

    def _centidegrees_to_radians(self, centidegrees: float) -> float:
        """Convert single value from centidegrees to radians."""
        return math.radians(centidegrees / 100.0)

    def connect(self) -> None:
        """Establish connection to robot via rosbridge websocket."""
        if roslibpy is None:
            raise ImportError(
                "roslibpy is required for real robot connection. "
                "Install with: pip install roslibpy"
            )

        self._client = roslibpy.Ros(host=self.host, port=self.port)

        try:
            self._client.run(timeout=self.timeout)
        except Exception as e:
            # Handle Twisted reactor issues (ReactorNotRestartable)
            # This commonly happens in Jupyter notebooks
            if "ReactorNotRestartable" in str(type(e).__name__) or "ReactorNotRestartable" in str(e):
                raise ConnectionError(
                    "Cannot reconnect: Twisted reactor cannot be restarted. "
                    "In Jupyter notebooks, you must restart the kernel to reconnect. "
                    "Alternatively, keep a single Robot connection open for the session."
                ) from e
            raise

        # Wait for connection
        start = time.time()
        while not self._client.is_connected and (time.time() - start) < self.timeout:
            time.sleep(0.1)

        if not self._client.is_connected:
            self._client = None
            raise ConnectionError(
                f"Failed to connect to robot at {self.host}:{self.port}. "
                f"Check that rosbridge_server is running."
            )

        # Initialize service clients
        self._service = roslibpy.Service(
            self._client,
            '/apply_joint_trajectory',
            'datatypes/ApplyJointTrajectory'
        )
        self._motor_settings_service = roslibpy.Service(
            self._client,
            '/apply_motor_settings',
            'datatypes/ApplyMotorSettings'
        )

        # Subscribe to /joint_trajectory for position feedback
        # Robot publishes current positions when motors move
        self._position_subscriber = roslibpy.Topic(
            self._client,
            '/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory'
        )
        self._position_subscriber.subscribe(self._on_joint_trajectory)

        # Subscribe to /motor_settings for settings feedback
        self._motor_settings_subscriber = roslibpy.Topic(
            self._client,
            '/motor_settings',
            'datatypes/MotorSettings'
        )
        self._motor_settings_subscriber.subscribe(self._on_motor_settings)

        # Connect Tinkerforge if low-latency mode is enabled
        if self._low_latency_config.enabled:
            self._connect_tinkerforge()

    def _on_motor_settings(self, message: dict) -> None:
        """Callback for motor settings updates from /motor_settings topic."""
        motor_name = message.get('motor_name')
        if motor_name:
            with self._motor_settings_lock:
                self._motor_settings[motor_name] = message

    def _on_joint_trajectory(self, message: dict) -> None:
        """Callback for position updates from /joint_trajectory topic.

        The robot publishes current positions when motors move.
        Message format: trajectory_msgs/msg/JointTrajectory
        """
        joint_names = message.get('joint_names', [])
        points = message.get('points', [])

        if joint_names and points:
            try:
                positions = points[0].get('positions', [])
                with self._joint_positions_lock:
                    for i, name in enumerate(joint_names):
                        if i < len(positions):
                            # Convert centidegrees to radians
                            centidegrees = float(positions[i])
                            self._joint_positions[name] = self._centidegrees_to_radians(centidegrees)
            except (ValueError, IndexError, TypeError):
                pass

    # ==================== LOW-LATENCY TINKERFORGE METHODS ====================

    def _connect_tinkerforge(self) -> None:
        """Connect to Tinkerforge brick daemon for direct motor control.

        Establishes connection and either uses the provided motor mapping
        or auto-discovers servo bricklets.
        """
        try:
            from tinkerforge.ip_connection import IPConnection
            from tinkerforge.bricklet_servo_v2 import BrickletServoV2
        except ImportError:
            logger.warning(
                "tinkerforge package not installed. Falling back to ROS for motor control. "
                "Install with: pip install tinkerforge"
            )
            self._low_latency_config.enabled = False
            return

        tf_host = self._low_latency_config.tinkerforge_host or self.host
        tf_port = self._low_latency_config.tinkerforge_port

        try:
            self._tinkerforge_conn = IPConnection()
            self._tinkerforge_conn.connect(tf_host, tf_port)
            logger.info(f"Connected to Tinkerforge daemon at {tf_host}:{tf_port}")

            motor_mapping = self._low_latency_config.motor_mapping

            if motor_mapping:
                # Use explicitly provided mapping
                self._tinkerforge_motor_map = motor_mapping
                self._init_servo_bricklets()
            else:
                # Auto-discover servo bricklets and build mapping
                self._auto_discover_servos()

        except Exception as e:
            logger.warning(
                f"Failed to connect to Tinkerforge at {tf_host}:{tf_port}: {e}. "
                f"Falling back to ROS for motor control."
            )
            self._low_latency_config.enabled = False
            self._tinkerforge_conn = None

    def _auto_discover_servos(self) -> None:
        """Auto-discover servo bricklets and build the full motor mapping.

        Enumerates Tinkerforge Servo Bricklet V2 devices, sorts them by
        physical stack position, and assigns them to bricklet roles.

        Empirically verified position order (lowest to highest):
        - position[0] -> servo2 (shoulders + head)
        - position[1] -> servo3 (left arm + hand)
        - position[2] -> servo1 (right arm + hand)
        """
        if self._tinkerforge_conn is None:
            return

        discovered: List[Tuple[str, str]] = []  # (uid, position)
        expected_bricklets = 3
        discovery_timeout = 1.0
        discovered_event = threading.Event()
        discovered_lock = threading.Lock()

        def enumerate_callback(uid, connected_uid, position, hardware_version,
                               firmware_version, device_identifier, enumeration_type):
            # Servo Bricklet V2 device identifier is 2157
            if device_identifier != 2157:
                return
            with discovered_lock:
                if any(u == uid for u, _ in discovered):
                    return
                discovered.append((uid, position))
                logger.info(f"Discovered Servo Bricklet V2: UID={uid}, position={position}")
                if len(discovered) >= expected_bricklets:
                    discovered_event.set()

        self._tinkerforge_conn.register_callback(
            self._tinkerforge_conn.CALLBACK_ENUMERATE,
            enumerate_callback
        )
        self._tinkerforge_conn.enumerate()

        # Return as soon as we've seen the expected bricklets, else wait
        # at most discovery_timeout for late responses.
        discovered_event.wait(timeout=discovery_timeout)

        if len(discovered) != 3:
            self._discovered_servo_uids = [uid for uid, _ in discovered]
            logger.warning(
                f"Expected 3 Servo Bricklets, found {len(discovered)}. "
                f"Auto-mapping disabled. Falling back to ROS for motor control. "
                f"Provide motor_mapping via LowLatencyConfig for manual configuration."
            )
            return

        # Sort by physical stack position.
        # Empirically verified order: position-sorted index maps to
        # [servo2 (shoulders/head), servo3 (left arm), servo1 (right arm)].
        discovered.sort(key=lambda x: x[1])

        for i, (uid, pos) in enumerate(discovered):
            logger.info(
                f"Enumerated servo [{i}]: UID={uid}, position='{pos}'"
            )

        servo2_uid = discovered[0][0]  # Shoulders + head
        servo3_uid = discovered[1][0]  # Left arm + hand
        servo1_uid = discovered[2][0]  # Right arm + hand

        # Validate against known UIDs when available
        discovered_uids = {uid for uid, _ in discovered}
        known_uids = set(KNOWN_SERVO_UIDS.values())
        if discovered_uids == known_uids:
            expected = {
                1: KNOWN_SERVO_UIDS[1],
                2: KNOWN_SERVO_UIDS[2],
                3: KNOWN_SERVO_UIDS[3],
            }
            actual = {1: servo1_uid, 2: servo2_uid, 3: servo3_uid}

            if actual != expected:
                logger.warning(
                    f"BRICKLET ORDER MISMATCH! "
                    f"Assigned: servo1={servo1_uid}, servo2={servo2_uid}, servo3={servo3_uid}  "
                    f"Expected: servo1={expected[1]}, servo2={expected[2]}, servo3={expected[3]}  "
                    f"Overriding with known UID mapping."
                )
                servo1_uid = expected[1]
                servo2_uid = expected[2]
                servo3_uid = expected[3]
            else:
                logger.info("Bricklet UIDs match known reference — ordering verified.")

        # Store in role order: servo1, servo2, servo3
        self._discovered_servo_uids = [servo1_uid, servo2_uid, servo3_uid]

        logger.info(
            f"Auto-mapped servo bricklets: "
            f"servo1(right arm)={servo1_uid}, "
            f"servo2(shoulders/head)={servo2_uid}, "
            f"servo3(left arm)={servo3_uid}"
        )

        self._tinkerforge_motor_map = build_motor_mapping(
            servo1_uid, servo2_uid, servo3_uid
        )
        self._init_servo_bricklets()

    def discover_servo_bricklets(self, timeout: float = 1.0) -> List[str]:
        """
        Discover connected Tinkerforge servo bricklets and return their UIDs.

        This is a convenience method to help configure motor_mapping.
        Connect to the robot first, then call this to find available bricklet UIDs.

        Args:
            timeout: Time to wait for enumeration responses (seconds).

        Returns:
            List of discovered Servo Bricklet V2 UIDs.

        Example:
            >>> with pib3.Robot(host="172.26.34.149") as robot:
            ...     # First, connect to Tinkerforge
            ...     robot._connect_tinkerforge()
            ...     # Discover available bricklets
            ...     uids = robot.discover_servo_bricklets()
            ...     print(f"Found servo bricklets: {uids}")
            ...     # Then configure mapping based on your robot's wiring
            ...     robot.configure_motor_mapping({
            ...         "elbow_left": (uids[0], 8),
            ...         # ... etc
            ...     })
        """
        if self._tinkerforge_conn is None:
            # Try to connect if not already connected
            if not self._low_latency_config.enabled:
                self._low_latency_config.enabled = True
            self._connect_tinkerforge()

        if self._tinkerforge_conn is None:
            logger.warning("Cannot discover bricklets: Tinkerforge not connected")
            return []

        discovered = []

        def enumerate_callback(uid, connected_uid, position, hardware_version,
                               firmware_version, device_identifier, enumeration_type):
            # Servo Bricklet V2 device identifier is 2157
            if device_identifier == 2157 and uid not in discovered:
                discovered.append(uid)
                logger.info(f"Discovered Servo Bricklet V2: UID={uid}, position={position}")

        self._tinkerforge_conn.register_callback(
            self._tinkerforge_conn.CALLBACK_ENUMERATE,
            enumerate_callback
        )
        self._tinkerforge_conn.enumerate()

        # Wait for enumeration responses
        time.sleep(timeout)

        return discovered

    @property
    def discovered_servo_uids(self) -> List[str]:
        """Get list of discovered Servo Bricklet UIDs from last auto-discovery."""
        return getattr(self, '_discovered_servo_uids', [])

    def _init_servo_bricklets(self, auto_configure: bool = True) -> None:
        """Initialize servo bricklet objects from the motor mapping.

        Args:
            auto_configure: If True, automatically configure all servo channels
                with default PWM and motion settings after initialization.
        """
        if self._tinkerforge_conn is None:
            return

        try:
            from tinkerforge.bricklet_servo_v2 import BrickletServoV2
        except ImportError:
            return

        # Get unique bricklet UIDs from the mapping
        unique_uids = set()
        for motor_name, (uid, channel) in self._tinkerforge_motor_map.items():
            unique_uids.add(uid)

        # Create servo bricklet objects
        for uid in unique_uids:
            try:
                servo = BrickletServoV2(uid, self._tinkerforge_conn)
                self._tinkerforge_servos[uid] = servo
                logger.debug(f"Initialized Servo Bricklet V2: {uid}")
            except Exception as e:
                logger.error(f"Failed to initialize Servo Bricklet {uid}: {e}")

        # Build reverse map and register position-reached callbacks
        self._tinkerforge_reverse_map.clear()
        self._position_reached_events.clear()
        for motor_name, (uid, channel) in self._tinkerforge_motor_map.items():
            self._tinkerforge_reverse_map[(uid, channel)] = motor_name
            self._position_reached_events[motor_name] = threading.Event()

        for uid, servo in self._tinkerforge_servos.items():
            self._register_position_reached_callback(uid, servo)

        # Auto-configure all channels with default settings
        if auto_configure and self._tinkerforge_servos:
            # Initialize cache for all mapped motors as False (unknown/disabled)
            self._servo_enabled_cache.clear()
            self._motion_config_cache.clear()
            self.configure_all_servo_channels()
            logger.info("Auto-configured all servo channels with default settings")

    def _register_position_reached_callback(self, uid: str, servo) -> None:
        """Register CALLBACK_POSITION_REACHED on a servo bricklet.

        Uses the Tinkerforge callback mechanism instead of polling to detect
        when a servo channel has reached its target position.

        Args:
            uid: Bricklet UID (used to resolve motor name via reverse map).
            servo: BrickletServoV2 instance.
        """
        from tinkerforge.bricklet_servo_v2 import BrickletServoV2

        def on_position_reached(servo_channel, position):
            motor_name = self._tinkerforge_reverse_map.get((uid, servo_channel))
            if motor_name is not None:
                event = self._position_reached_events.get(motor_name)
                if event is not None:
                    event.set()
                    logger.debug(
                        f"Position reached callback: {motor_name} "
                        f"at {position} centideg (channel={servo_channel})"
                    )

        servo.register_callback(
            BrickletServoV2.CALLBACK_POSITION_REACHED,
            on_position_reached,
        )

        # Enable the callback for each channel mapped to this bricklet
        for (mapped_uid, channel), motor_name in self._tinkerforge_reverse_map.items():
            if mapped_uid == uid:
                try:
                    servo.set_position_reached_callback_configuration(channel, True)
                    logger.debug(
                        f"Enabled position-reached callback for {motor_name} "
                        f"(bricklet={uid}, channel={channel})"
                    )
                except Exception as e:
                    logger.error(
                        f"Failed to enable position-reached callback for "
                        f"{motor_name}: {e}"
                    )

    def _disconnect_tinkerforge(self) -> None:
        """Disconnect from Tinkerforge brick daemon."""
        if self._tinkerforge_conn is not None:
            try:
                self._tinkerforge_conn.disconnect()
            except Exception:
                pass
            self._tinkerforge_conn = None
            self._tinkerforge_servos.clear()
            self._tinkerforge_motor_map.clear()
            self._servo_enabled_cache.clear()
            self._motion_config_cache.clear()
            self._tinkerforge_reverse_map.clear()
            self._position_reached_events.clear()

    @property
    def low_latency_available(self) -> bool:
        """Check if low-latency mode is available and connected."""
        return (
            self._low_latency_config.enabled
            and self._tinkerforge_conn is not None
            and bool(self._tinkerforge_motor_map)
            and bool(self._tinkerforge_servos)
        )

    @property
    def low_latency_enabled(self) -> bool:
        """Get whether low-latency mode is currently enabled."""
        return self._low_latency_config.enabled

    @low_latency_enabled.setter
    def low_latency_enabled(self, value: bool) -> None:
        """Enable or disable low-latency mode at runtime.

        When enabling, a Tinkerforge connection is attempted (requires an
        active ROS connection so that ``self.host`` is reachable). The flag
        is only set to True if the connection actually succeeds — otherwise
        a RuntimeError is raised so callers don't silently end up with a
        "enabled" flag pointing at nothing.
        """
        if value and not self._low_latency_config.enabled:
            if not self.is_connected:
                raise RuntimeError(
                    "Cannot enable low-latency mode: not connected to robot. "
                    "Call connect() (or use the Robot() context manager) first."
                )
            # Set the flag before attempting the connection so _connect_tinkerforge
            # sees the intent; roll it back if the attempt fails.
            self._low_latency_config.enabled = True
            try:
                if self._tinkerforge_conn is None:
                    self._connect_tinkerforge()
            except Exception:
                self._low_latency_config.enabled = False
                raise
            if not self.low_latency_available:
                self._low_latency_config.enabled = False
                raise RuntimeError(
                    "Failed to enable low-latency mode: Tinkerforge daemon "
                    "reachable? servo bricklets discovered? Check "
                    "`low_latency_available` for the connection-state diagnosis."
                )
        elif not value:
            self._low_latency_config.enabled = False

    @property
    def low_latency_sync_to_ros(self) -> bool:
        """Get whether low-latency mode syncs positions back to ROS topics."""
        return self._low_latency_config.sync_to_ros

    @low_latency_sync_to_ros.setter
    def low_latency_sync_to_ros(self, value: bool) -> None:
        """Set whether to update local position cache after direct control.

        When True, the local position cache is updated after low-latency motor
        commands, ensuring get_joint() returns correct values. This does NOT
        publish to ROS topics (that would cause double motor commands).
        """
        self._low_latency_config.sync_to_ros = value

    def configure_motor_mapping(
        self,
        mapping: TinkerforgeMotorMapping,
        reinitialize: bool = True,
    ) -> None:
        """
        Configure Tinkerforge motor-to-bricklet mapping at runtime.

        This allows setting up motor mappings after connection, useful when
        the mapping isn't known at construction time.

        Args:
            mapping: Dict mapping motor names to (bricklet_uid, channel) tuples.
                Example: {"elbow_left": ("ABC", 0), "wrist_left": ("ABC", 1)}
            reinitialize: If True, reinitialize servo bricklet connections.

        Example:
            >>> robot.configure_motor_mapping({
            ...     "shoulder_vertical_left": ("XYZ", 0),
            ...     "shoulder_horizontal_left": ("XYZ", 1),
            ...     "upper_arm_left_rotation": ("XYZ", 2),
            ...     "elbow_left": ("XYZ", 3),
            ...     "lower_arm_left_rotation": ("XYZ", 4),
            ...     "wrist_left": ("XYZ", 5),
            ... })
        """
        self._tinkerforge_motor_map.update(mapping)

        if reinitialize and self._tinkerforge_conn is not None:
            self._init_servo_bricklets()

    def configure_servo_channel(
        self,
        motor_name: str,
        pulse_width_min: int = 700,
        pulse_width_max: int = 2500,
        velocity: int = 9000,
        acceleration: int = 9000,
        deceleration: int = 9000,
        period: int = 19500,
        degree_min: int = -9000,
        degree_max: int = 9000,
    ) -> bool:
        """
        Configure Tinkerforge servo channel settings for a motor.

        This configures the PWM pulse width range and motion parameters
        for a specific motor. Should be called once after connecting,
        before using low-latency mode.

        Args:
            motor_name: Name of the motor to configure.
            pulse_width_min: Minimum PWM pulse width in microseconds (default: 700).
            pulse_width_max: Maximum PWM pulse width in microseconds (default: 2500).
            velocity: Maximum velocity in 0.01°/s (default: 9000 = 90°/s).
            acceleration: Acceleration in 0.01°/s² (default: 9000).
            deceleration: Deceleration in 0.01°/s² (default: 9000).
            period: PWM period in microseconds (default: 19500 = 19.5ms).
            degree_min: Min abstract angle in 0.01° (default: -9000 = -90°).
            degree_max: Max abstract angle in 0.01° (default: 9000 = 90°).

        Returns:
            True if configuration was successful.

        Example:
            >>> robot.configure_servo_channel("elbow_left",
            ...     pulse_width_min=700,
            ...     pulse_width_max=2500,
            ...     velocity=9000,
            ...     acceleration=9000,
            ...     deceleration=9000
            ... )
        """
        if motor_name not in self._tinkerforge_motor_map:
            logger.warning(f"Motor {motor_name} not in Tinkerforge mapping")
            return False

        uid, channel = self._tinkerforge_motor_map[motor_name]
        servo = self._tinkerforge_servos.get(uid)

        if servo is None:
            logger.warning(f"Servo bricklet {uid} not initialized")
            return False

        try:
            servo.set_period(channel, period)
            servo.set_degree(channel, degree_min, degree_max)
            servo.set_pulse_width(channel, pulse_width_min, pulse_width_max)
            servo.set_motion_configuration(channel, velocity, acceleration, deceleration)
            self._motion_config_cache[(uid, channel)] = (
                int(velocity), int(acceleration), int(deceleration)
            )
            logger.debug(
                f"Configured servo {motor_name}: pulse_width=[{pulse_width_min}, {pulse_width_max}], "
                f"motion=[{velocity}, {acceleration}, {deceleration}], "
                f"period={period}, degrees=[{degree_min}, {degree_max}]"
            )
            return True
        except Exception as e:
            logger.error(f"Failed to configure servo {motor_name}: {e}")
            return False

    def configure_all_servo_channels(
        self,
        velocity: Optional[int] = None,
        acceleration: Optional[int] = None,
        deceleration: Optional[int] = None,
    ) -> bool:
        """
        Configure motion parameters for all mapped servo channels.

        Sets velocity, acceleration, and deceleration for all motors.
        Does NOT change degree ranges, pulse widths, or period — those are
        left at whatever the robot's firmware has configured.

        Args:
            velocity: Maximum velocity in 0.01°/s.
                None (default) → ``DEFAULT_MOTION_VELOCITY`` (15000 ≈ 150°/s).
                Pass 0 explicitly to remove the limit (servo runs full-speed).
            acceleration: Acceleration in 0.01°/s².
                None → ``DEFAULT_MOTION_ACCELERATION`` (15000).
                Pass 0 to disable ramping.
            deceleration: Deceleration in 0.01°/s².
                None → ``DEFAULT_MOTION_DECELERATION`` (15000).
                Pass 0 to disable ramping.

        Returns:
            True if all channels were configured successfully.
        """
        if velocity is None:
            velocity = self.DEFAULT_MOTION_VELOCITY
        if acceleration is None:
            acceleration = self.DEFAULT_MOTION_ACCELERATION
        if deceleration is None:
            deceleration = self.DEFAULT_MOTION_DECELERATION

        all_success = True
        for motor_name in self._tinkerforge_motor_map:
            uid, channel = self._tinkerforge_motor_map[motor_name]
            servo = self._tinkerforge_servos.get(uid)
            if servo is None:
                logger.warning(f"Servo bricklet {uid} not initialized")
                all_success = False
                continue
            try:
                servo.set_motion_configuration(channel, velocity, acceleration, deceleration)
                self._motion_config_cache[(uid, channel)] = (
                    int(velocity), int(acceleration), int(deceleration)
                )
                logger.debug(
                    f"Configured motion for {motor_name}: "
                    f"velocity={velocity}, accel={acceleration}, decel={deceleration}"
                )
            except Exception as e:
                logger.error(f"Failed to configure motion for {motor_name}: {e}")
                all_success = False
        return all_success

    def _set_motor_direct(
        self,
        motor_name: str,
        position_centidegrees: int,
        velocity_centideg: Optional[int] = None,
    ) -> bool:
        """
        Set motor position directly via Tinkerforge (low-latency mode).

        Args:
            motor_name: Name of the motor.
            position_centidegrees: Target position in centidegrees (1/100 degree).
            velocity_centideg: Optional velocity in centidegrees/second.

        Returns:
            True if command was sent successfully.
        """
        if motor_name not in self._tinkerforge_motor_map:
            logger.debug(
                f"Motor {motor_name} not in Tinkerforge mapping, falling back to ROS"
            )
            return False

        uid, channel = self._tinkerforge_motor_map[motor_name]
        servo = self._tinkerforge_servos.get(uid)

        if servo is None:
            logger.warning(f"Servo bricklet {uid} not initialized")
            return False

        try:
            # Tinkerforge Servo V2 position is in units of 0.01° (same as centidegrees)
            # Velocity is in 0.01°/s

            # Enable the servo channel if not already enabled
            # Caching enabled state avoids redundant USB traffic (which is significant at high rates)
            cache_key = (uid, channel)
            if not self._servo_enabled_cache.get(cache_key, False):
                servo.set_enable(channel, True)
                self._servo_enabled_cache[cache_key] = True


            # Set velocity if provided (overrides motion_configuration velocity).
            # Cache the last-known (velocity, accel, decel) tuple per channel
            # so we skip both the get_motion_configuration USB round-trip and
            # the set_motion_configuration call when nothing changed.
            if velocity_centideg is not None:
                new_velocity = abs(int(velocity_centideg))
                cached = self._motion_config_cache.get(cache_key)
                if cached is None:
                    current = servo.get_motion_configuration(channel)
                    cached = (
                        getattr(current, 'velocity', 0),
                        getattr(current, 'acceleration', 0),
                        getattr(current, 'deceleration', 0),
                    )
                if cached[0] != new_velocity:
                    new_cfg = (new_velocity, cached[1], cached[2])
                    servo.set_motion_configuration(channel, *new_cfg)
                    self._motion_config_cache[cache_key] = new_cfg
                else:
                    self._motion_config_cache[cache_key] = cached

            # Set position.  Event clearing is handled by _verify_positions
            # (not here) to avoid a race where a stale callback from a
            # previous command sets the event between clear and set_position.
            servo.set_position(channel, position_centidegrees)

            logger.debug(
                f"Direct motor set: {motor_name} -> {position_centidegrees} centideg "
                f"(bricklet={uid}, channel={channel})"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to set motor {motor_name} directly: {e}")
            return False

    def _get_motor_direct(self, motor_name: str) -> Optional[int]:
        """
        Get motor position directly via Tinkerforge (low-latency mode).

        Reads the actual servo position from the potentiometer feedback,
        not the commanded position.

        Args:
            motor_name: Name of the motor.

        Returns:
            Current position in centidegrees (1/100 degree), or None if unavailable.
        """
        if motor_name not in self._tinkerforge_motor_map:
            return None

        uid, channel = self._tinkerforge_motor_map[motor_name]
        servo = self._tinkerforge_servos.get(uid)

        if servo is None:
            return None

        try:
            # get_current_position returns the actual measured position
            # from the servo's potentiometer feedback in centidegrees
            position = servo.get_current_position(channel)

            logger.debug(
                f"Direct motor read: {motor_name} = {position} centideg "
                f"(bricklet={uid}, channel={channel})"
            )
            return position

        except Exception as e:
            logger.error(f"Failed to read motor {motor_name} directly: {e}")
            return None

    def disconnect(self) -> None:
        """Close connection to robot."""
        # Stop subsystems
        if self._ai_subsystem is not None:
            try:
                self._ai_subsystem.stop()
            except Exception:
                pass
            self._ai_subsystem = None

        if self._camera_subsystem is not None:
            try:
                self._camera_subsystem.stop()
            except Exception:
                pass
            self._camera_subsystem = None

        # Disconnect Tinkerforge first
        self._disconnect_tinkerforge()

        if self._position_subscriber is not None:
            try:
                self._position_subscriber.unsubscribe()
            except Exception:
                pass
            self._position_subscriber = None

        if self._motor_settings_subscriber is not None:
            try:
                self._motor_settings_subscriber.unsubscribe()
            except Exception:
                pass
            self._motor_settings_subscriber = None

        if self._client is not None:
            try:
                self._client.terminate()
            except Exception:
                pass
            self._client = None
            self._service = None
            self._motor_settings_service = None

        with self._joint_positions_lock:
            self._joint_positions.clear()

        with self._motor_settings_lock:
            self._motor_settings.clear()

    @property
    def is_connected(self) -> bool:
        """Check if connected to robot."""
        return self._client is not None and self._client.is_connected

    def set_motor_settings(
        self,
        motors: Union[str, List[str], "Joint"],
        use_defaults: bool = False,
        timeout: float = 5.0,
        **settings,
    ) -> bool:
        """
        Apply motor settings (velocity, acceleration, etc.) to one or more motors.

        Uses the /apply_motor_settings ROS service.

        Args:
            motors: Motor name(s) to configure. Can be:
                - Single motor name: "elbow_left"
                - Joint enum: Joint.ELBOW_LEFT
                - List of names: ["elbow_left", "wrist_left"]
                - Group name: "left_arm", "right_hand", "head" (from MOTOR_GROUPS)
            use_defaults: If True, merge with DEFAULT_MOTOR_SETTINGS first.
            timeout: Service call timeout in seconds.
            **settings: Motor settings to apply. Available options:
                - turned_on (bool): Enable/disable motor
                - visible (bool): Motor visibility
                - invert (bool): Invert motor direction
                - velocity (int): Motor velocity (default: 16000)
                - acceleration (int): Acceleration (default: 10000)
                - deceleration (int): Deceleration (default: 5000)
                - pulse_width_min (int): Min PWM pulse width (default: 700)
                - pulse_width_max (int): Max PWM pulse width (default: 2500)
                - period (int): PWM period (default: 19500)
                - rotation_range_min (int): Min angle in centidegrees (default: -9000)
                - rotation_range_max (int): Max angle in centidegrees (default: 9000)

        Returns:
            True if settings were applied successfully to all motors.

        Example:
            >>> # Set velocity for single motor
            >>> robot.set_motor_settings(Joint.ELBOW_LEFT, velocity=8000)

            >>> # Set velocity for entire arm
            >>> robot.set_motor_settings("left_arm", velocity=8000, acceleration=5000)

            >>> # Apply default settings to all hand motors
            >>> robot.set_motor_settings("left_hand", use_defaults=True)

            >>> # Multiple motors
            >>> robot.set_motor_settings(["elbow_left", "wrist_left"], velocity=10000)
        """
        from ..dh_model import MOTOR_GROUPS, DEFAULT_MOTOR_SETTINGS
        from ..types import Joint

        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        if self._motor_settings_service is None:
            raise RuntimeError("Motor settings service not initialized")

        # Resolve motor names
        motor_names: List[str] = []

        if isinstance(motors, Joint):
            motor_names = [motors.value]
        elif isinstance(motors, str):
            # Check if it's a group name
            if motors in MOTOR_GROUPS:
                motor_names = MOTOR_GROUPS[motors]
            else:
                motor_names = [motors]
        elif isinstance(motors, list):
            for m in motors:
                if isinstance(m, Joint):
                    motor_names.append(m.value)
                elif isinstance(m, str):
                    if m in MOTOR_GROUPS:
                        motor_names.extend(MOTOR_GROUPS[m])
                    else:
                        motor_names.append(m)
                else:
                    raise TypeError(f"Unsupported motor type: {type(m)}")
        else:
            raise TypeError(f"Unsupported motors argument type: {type(motors)}")

        # Build settings dict
        if use_defaults:
            final_settings = dict(DEFAULT_MOTOR_SETTINGS)
            final_settings.update(settings)
        else:
            final_settings = settings

        if not final_settings:
            logger.warning("No settings provided to set_motor_settings()")
            return True

        # Apply settings to each motor
        all_success = True
        for motor_name in motor_names:
            motor_settings = {"motor_name": motor_name}
            motor_settings.update(final_settings)

            request = roslibpy.ServiceRequest({"motor_settings": motor_settings})

            try:
                response = self._motor_settings_service.call(request, timeout=timeout)
                applied = bool(response.get("settings_applied", False))
                persisted = bool(response.get("settings_persisted", False))
                success = applied or persisted

                if not success:
                    logger.warning(
                        f"Motor settings not applied for {motor_name}: {response}"
                    )
                    all_success = False
                else:
                    logger.debug(f"Motor settings applied for {motor_name}")

            except Exception as e:
                logger.error(f"Failed to apply motor settings for {motor_name}: {e}")
                all_success = False

        return all_success

    def get_motor_settings(
        self,
        motors: Optional[Union[str, List[str], "Joint"]] = None,
        use_defaults: bool = False,
    ) -> Dict[str, Dict]:
        """
        Get motor settings for one or more motors.

        Returns cached settings received from the /motor_settings topic.
        If no settings have been received yet, returns defaults if requested.

        Args:
            motors: Motor name(s) to query. Can be:
                - None: Return all cached settings
                - Single motor name: "elbow_left"
                - Joint enum: Joint.ELBOW_LEFT
                - List of names: ["elbow_left", "wrist_left"]
                - Group name: "left_arm", "right_hand", "head" (from MOTOR_GROUPS)
            use_defaults: If True, return DEFAULT_MOTOR_SETTINGS for motors
                         that have no cached settings.

        Returns:
            Dict mapping motor names to their settings dicts.
            Settings may include: turned_on, visible, invert, velocity,
            acceleration, deceleration, pulse_width_min, pulse_width_max,
            period, rotation_range_min, rotation_range_max.

        Example:
            >>> # Get all cached settings
            >>> settings = robot.get_motor_settings()

            >>> # Get settings for specific motor
            >>> settings = robot.get_motor_settings(Joint.ELBOW_LEFT)

            >>> # Get settings for arm group, with defaults for missing motors
            >>> settings = robot.get_motor_settings("left_arm", use_defaults=True)
        """
        from ..dh_model import MOTOR_GROUPS, DEFAULT_MOTOR_SETTINGS
        from ..types import Joint

        # Resolve motor names
        motor_names: Optional[List[str]] = None

        if motors is not None:
            motor_names = []
            if isinstance(motors, Joint):
                motor_names = [motors.value]
            elif isinstance(motors, str):
                if motors in MOTOR_GROUPS:
                    motor_names = MOTOR_GROUPS[motors]
                else:
                    motor_names = [motors]
            elif isinstance(motors, list):
                for m in motors:
                    if isinstance(m, Joint):
                        motor_names.append(m.value)
                    elif isinstance(m, str):
                        if m in MOTOR_GROUPS:
                            motor_names.extend(MOTOR_GROUPS[m])
                        else:
                            motor_names.append(m)
                    else:
                        raise TypeError(f"Unsupported motor type: {type(m)}")
            else:
                raise TypeError(f"Unsupported motors argument type: {type(motors)}")

        # Get cached settings
        result: Dict[str, Dict] = {}

        with self._motor_settings_lock:
            if motor_names is None:
                # Return all cached settings
                result = {k: dict(v) for k, v in self._motor_settings.items()}
            else:
                # Return settings for specified motors
                for name in motor_names:
                    if name in self._motor_settings:
                        result[name] = dict(self._motor_settings[name])
                    elif use_defaults:
                        result[name] = {"motor_name": name, **DEFAULT_MOTOR_SETTINGS}

        return result

    def _get_joint_radians(
        self,
        motor_name: str,
        timeout: Optional[float] = None,
    ) -> Optional[float]:
        """
        Get current position of a single joint in radians.

        When low-latency mode is enabled and the motor is in the Tinkerforge
        mapping, reads directly from the servo bricklet. Otherwise, uses
        the ROS subscription cache.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").
            timeout: Max time to wait for position data (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).
                    Note: timeout is ignored in low-latency mode (reads are instant).

        Returns:
            Current position in radians, or None if unavailable.
        """
        # Try low-latency direct read first
        if self.low_latency_enabled and self.low_latency_available:
            centidegrees = self._get_motor_direct(motor_name)
            if centidegrees is not None:
                radians = self._centidegrees_to_radians(centidegrees)
                # Update local cache for consistency
                if self._low_latency_config.sync_to_ros:
                    with self._joint_positions_lock:
                        self._joint_positions[motor_name] = radians
                return radians
            # Fall through to ROS if motor not in Tinkerforge mapping

        # Fall back to ROS subscription cache
        if timeout is None:
            timeout = self.DEFAULT_GET_JOINTS_TIMEOUT

        start = time.time()
        while (time.time() - start) < timeout:
            with self._joint_positions_lock:
                if motor_name in self._joint_positions:
                    return self._joint_positions[motor_name]
            time.sleep(0.05)  # Poll every 50ms

        with self._joint_positions_lock:
            return self._joint_positions.get(motor_name)

    def _get_joints_radians(
        self,
        motor_names: Optional[List[str]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints in radians.

        When low-latency mode is enabled, reads directly from Tinkerforge
        servo bricklets for mapped motors. Motors not in the mapping fall
        back to the ROS subscription cache.

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available positions (from mapping + ROS cache).
            timeout: Max time to wait for position data (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).
                    Note: timeout only applies to ROS fallback reads.

        Returns:
            Dict mapping motor names to positions in radians.
            May contain fewer joints than requested if timeout expires.
        """
        result: Dict[str, float] = {}
        remaining_motors: Optional[List[str]] = None

        # Try low-latency direct reads first
        if self.low_latency_enabled and self.low_latency_available:
            if motor_names is None:
                # Read all motors in the Tinkerforge mapping
                motors_to_read = list(self._tinkerforge_motor_map.keys())
            else:
                motors_to_read = motor_names

            for motor_name in motors_to_read:
                centidegrees = self._get_motor_direct(motor_name)
                if centidegrees is not None:
                    radians = self._centidegrees_to_radians(centidegrees)
                    result[motor_name] = radians
                    # Update local cache for consistency
                    if self._low_latency_config.sync_to_ros:
                        with self._joint_positions_lock:
                            self._joint_positions[motor_name] = radians

            # Determine which motors still need ROS fallback
            if motor_names is not None:
                remaining_motors = [m for m in motor_names if m not in result]
                if not remaining_motors:
                    return result  # All motors read via low-latency
            elif result:
                # If no specific motors requested, merge with ROS cache
                with self._joint_positions_lock:
                    for name, pos in self._joint_positions.items():
                        if name not in result:
                            result[name] = pos
                return result

        # Fall back to ROS subscription cache for remaining motors
        if timeout is None:
            timeout = self.DEFAULT_GET_JOINTS_TIMEOUT

        # Determine which joints we're waiting for
        if remaining_motors is not None:
            expected_joints = set(remaining_motors)
        elif motor_names is not None:
            expected_joints = set(motor_names)
        else:
            expected_joints = None

        # Wait for requested joints to have data
        start = time.time()
        while (time.time() - start) < timeout:
            with self._joint_positions_lock:
                if expected_joints is None:
                    # Return all available if we have any
                    if self._joint_positions:
                        result.update(self._joint_positions)
                        return result
                else:
                    available_joints = set(self._joint_positions.keys())
                    if expected_joints <= available_joints:
                        # All expected joints are available
                        for name in expected_joints:
                            result[name] = self._joint_positions[name]
                        return result
            time.sleep(0.05)  # Poll every 50ms

        # Timeout expired - return whatever we have
        with self._joint_positions_lock:
            if motor_names is None:
                result.update(self._joint_positions)
            else:
                for name in (remaining_motors or motor_names):
                    if name in self._joint_positions:
                        result[name] = self._joint_positions[name]

        return result

    def _verify_positions(
        self,
        target_positions: Dict[str, float],
        unit: str,
        timeout: float,
        tolerance: float,
    ) -> bool:
        """Verify joints reached target positions.

        For low-latency (Tinkerforge) motors, waits on CALLBACK_POSITION_REACHED
        events instead of polling. Falls back to the base class polling for
        ROS-controlled motors or when low-latency is not available.
        """
        if not self.low_latency_available:
            return super()._verify_positions(target_positions, unit, timeout, tolerance)

        start_time = time.time()

        # Separate low-latency and ROS motors
        ll_motors: Dict[str, float] = {}
        ros_motors: Dict[str, float] = {}
        for name, target in target_positions.items():
            if name in self._tinkerforge_motor_map:
                ll_motors[name] = target
            else:
                ros_motors[name] = target

        # Wait on position-reached events for low-latency motors.
        #
        # Events are cleared HERE (not in _set_motor_direct) to avoid a race
        # where a stale callback from a previous command sets the event
        # between clear() and set_position().  By the time we reach this
        # point, all set_position() calls have been sent and any stale
        # callbacks from the USB/IP pipeline (~1-2 ms round-trip) have long
        # been delivered, so clear() safely discards only stale state.
        for name, target in ll_motors.items():
            event = self._position_reached_events.get(name)
            if event is None:
                continue

            # Discard stale callbacks from previous commands.
            event.clear()

            # Quick check: motor may already be at the target position,
            # in which case the callback won't fire at all.
            current_pos = self.get_joint(name, unit=unit)
            if current_pos is not None and abs(current_pos - target) <= tolerance:
                continue

            remaining = timeout - (time.time() - start_time)
            if remaining <= 0:
                return False

            if not event.wait(timeout=remaining):
                # Timeout — check if we're within tolerance anyway (the
                # callback may not fire if the motor was already very
                # close to the target).
                current_pos = self.get_joint(name, unit=unit)
                if current_pos is None or abs(current_pos - target) > tolerance:
                    return False

        # Fall back to base-class polling for any ROS-only motors
        if ros_motors:
            remaining = timeout - (time.time() - start_time)
            if remaining <= 0:
                return False
            return super()._verify_positions(ros_motors, unit, remaining, tolerance)

        return True

    # Default velocity for ROS motor movements (centidegrees per second).
    # Used only for ROS-based commands; low-latency (Tinkerforge) commands
    # use the hardware's configured velocity (velocity=0 means no limit).
    DEFAULT_VELOCITY_CENTIDEG = 10000  # 100 degrees/sec

    def _set_joints_impl(
        self,
        positions_radians: Dict[str, float],
        velocity_centideg: Optional[float] = None,
        low_latency: Optional[bool] = None,
    ) -> bool:
        """
        Set joint positions via apply_joint_trajectory service or direct Tinkerforge.

        When low_latency mode is enabled and available, sends commands directly
        to Tinkerforge servo bricklets, bypassing the ROS/rosbridge stack for
        reduced latency (~5-20ms vs ~100-200ms).

        Args:
            positions_radians: Dict mapping motor names to positions in radians.
            velocity_centideg: Velocity in centidegrees/sec. If None, uses the
                hardware's configured velocity for low-latency mode, or
                DEFAULT_VELOCITY_CENTIDEG for ROS mode.
            low_latency: Override low_latency setting for this call.
                - None: Use configured default (LowLatencyConfig.enabled)
                - True: Force low-latency mode (if available)
                - False: Force ROS mode

        Returns:
            True if all joints were set successfully.

        Note:
            When using low_latency mode, the new position will NOT be visible
            in ROS topics unless sync_to_ros is enabled in LowLatencyConfig.
        """
        if not self.is_connected:
            return False

        # Determine whether to use low-latency mode
        use_low_latency = (
            low_latency if low_latency is not None
            else self._low_latency_config.enabled
        )
        use_low_latency = use_low_latency and self.low_latency_available

        # For ROS commands, always need a velocity value
        ros_velocity = velocity_centideg if velocity_centideg is not None else self.DEFAULT_VELOCITY_CENTIDEG

        # For low-latency (Tinkerforge), only override velocity if explicitly provided.
        # None means "use whatever the hardware already has configured" (typically no limit).
        tf_velocity = int(velocity_centideg) if velocity_centideg is not None else None

        all_successful = True
        motors_via_ros = {}  # Motors that couldn't be set directly

        if use_low_latency:
            # Try direct Tinkerforge control first
            for joint_name, position in positions_radians.items():
                centidegrees = self._radians_to_centidegrees(position)

                success = self._set_motor_direct(
                    joint_name,
                    centidegrees,
                    velocity_centideg=tf_velocity,
                )

                if not success:
                    # Motor not in Tinkerforge mapping, queue for ROS
                    motors_via_ros[joint_name] = position
                elif self._low_latency_config.sync_to_ros:
                    # Optionally sync to ROS topic for visibility
                    self._sync_position_to_ros(joint_name, centidegrees, ros_velocity)

            # Fall back to ROS for motors not in Tinkerforge mapping
            if motors_via_ros:
                ros_success = self._set_joints_via_ros(motors_via_ros, ros_velocity)
                all_successful = all_successful and ros_success

            return all_successful

        # Standard ROS path
        return self._set_joints_via_ros(positions_radians, ros_velocity)

    def _set_joints_via_ros(
        self,
        positions_radians: Dict[str, float],
        velocity_centideg: float,
    ) -> bool:
        """
        Set joint positions via ROS apply_joint_trajectory service.

        Sends each joint as a separate service call because the PIB robot's
        ROS implementation processes one joint per call.

        Args:
            positions_radians: Dict mapping motor names to positions in radians.
            velocity_centideg: Velocity in centidegrees/sec.

        Returns:
            True if all joints were set successfully.
        """


        all_successful = True

        # Send each joint as a separate service call
        for joint_name, position in positions_radians.items():
            centidegrees = self._radians_to_centidegrees(position)

            message = {
                'joint_trajectory': {
                    'header': {
                        'stamp': {'sec': 0, 'nanosec': 0},
                        'frame_id': '',
                    },
                    'joint_names': [joint_name],
                    'points': [{
                        'positions': [float(centidegrees)],
                        'velocities': [float(velocity_centideg)],
                        'accelerations': [],
                        'effort': [],
                        'time_from_start': {'sec': 0, 'nanosec': 0},
                    }],
                }
            }

            logger.debug(
                f"Sending to {joint_name}: position={centidegrees} centideg "
                f"({position:.4f} rad), velocity={velocity_centideg}"
            )

            try:
                request = roslibpy.ServiceRequest(message)
                result = self._service.call(request, timeout=self.timeout)
                success = result.get('successful', False)
                logger.debug(f"  Result: {result}")
                if not success:
                    all_successful = False
            except Exception as e:
                logger.debug(f"  Exception: {e}")
                all_successful = False

        return all_successful

    def _sync_position_to_ros(
        self,
        motor_name: str,
        centidegrees: int,
        velocity_centideg: float,
    ) -> None:
        """
        Update internal position cache after direct motor set.

        NOTE: We do NOT publish to /joint_trajectory topic because that topic
        is bidirectional - publishing would trigger a second motor command
        through the ROS node, causing the motor to move twice.

        Instead, we just update our local position cache so get_joint() returns
        the correct value after a low-latency set.
        """
        # Update local position cache (convert centidegrees back to radians)
        radians = self._centidegrees_to_radians(centidegrees)
        with self._joint_positions_lock:
            self._joint_positions[motor_name] = radians

        logger.debug(f"Updated local cache for {motor_name}: {centidegrees} centideg")

    def _execute_waypoints(
        self,
        joint_names: List[str],
        waypoints: np.ndarray,
        rate_hz: float,
        progress_callback: Optional[Callable[[int, int], None]],
    ) -> bool:
        """Execute waypoints on real robot.

        Uses low-latency Tinkerforge path when available for reduced latency.
        Falls back to ROS service calls for motors not in the Tinkerforge mapping.
        """
        if not self.is_connected:
            return False

        use_low_latency = self.low_latency_available
        period = 1.0 / rate_hz
        total = len(waypoints)
        motor_names = list(joint_names)
        fail_count = 0

        for i, point in enumerate(waypoints):
            if self._stopped:
                logger.warning(f"Trajectory aborted at waypoint {i}/{total} (emergency stop)")
                return False

            # Waypoints are already in centidegrees from _to_backend_format
            positions_centideg = [float(point[j]) for j in range(len(motor_names))]

            if use_low_latency:
                # Fast path: send directly to Tinkerforge servo bricklets
                ros_fallback_names = []
                ros_fallback_positions = []

                for name, centideg in zip(motor_names, positions_centideg):
                    success = self._set_motor_direct(name, int(centideg))
                    if not success:
                        # Motor not in Tinkerforge mapping, queue for ROS
                        ros_fallback_names.append(name)
                        ros_fallback_positions.append(centideg)
                    elif self._low_latency_config.sync_to_ros:
                        radians = self._centidegrees_to_radians(centideg)
                        with self._joint_positions_lock:
                            self._joint_positions[name] = radians

                # Send remaining motors via ROS
                if ros_fallback_names:
                    message = {
                        'joint_trajectory': {
                            'header': {
                                'stamp': {'sec': 0, 'nanosec': 0},
                                'frame_id': '',
                            },
                            'joint_names': ros_fallback_names,
                            'points': [{
                                'positions': ros_fallback_positions,
                                'velocities': [],
                                'accelerations': [],
                                'effort': [],
                                'time_from_start': {'sec': 0, 'nanosec': 0},
                            }],
                        }
                    }
                    try:
                        request = roslibpy.ServiceRequest(message)
                        self._service.call(request, timeout=self.timeout)
                    except Exception as e:
                        fail_count += 1
                        logger.warning(f"Waypoint {i + 1}/{total} ROS fallback failed: {e}")
            else:
                # Standard ROS path
                message = {
                    'joint_trajectory': {
                        'header': {
                            'stamp': {'sec': 0, 'nanosec': 0},
                            'frame_id': '',
                        },
                        'joint_names': motor_names,
                        'points': [{
                            'positions': positions_centideg,
                            'velocities': [],
                            'accelerations': [],
                            'effort': [],
                            'time_from_start': {'sec': 0, 'nanosec': 0},
                        }],
                    }
                }
                try:
                    request = roslibpy.ServiceRequest(message)
                    self._service.call(request, timeout=self.timeout)
                except Exception as e:
                    fail_count += 1
                    logger.warning(f"Waypoint {i + 1}/{total} failed: {e}")

            if progress_callback:
                progress_callback(i + 1, total)

            time.sleep(period)

        if fail_count > 0:
            logger.error(
                f"Trajectory execution: {fail_count}/{total} waypoints failed"
            )
        # Succeed only if every waypoint was dispatched successfully. A
        # partial run should surface as a failure so callers don't silently
        # get a half-executed trajectory.
        return fail_count == 0

    # ==================== CAMERA METHODS ====================

    def subscribe_camera_image(
        self,
        callback: Callable[[bytes], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to camera image stream from OAK-D Lite.

        The camera publishes hardware-encoded MJPEG frames. The callback
        receives raw JPEG bytes after base64 decoding.

        Streaming only runs while subscribed (on-demand activation).

        Note:
            Data is transmitted as base64-encoded JSON. Binary CBOR transfer
            is not currently supported by roslibpy. For high-performance
            applications, consider using rosbridge directly with CBOR encoding.

        Args:
            callback: Called with raw JPEG bytes for each frame.

        Returns:
            Topic object (call .unsubscribe() when done to stop streaming).

        Example:
            >>> def on_frame(jpeg_bytes):
            ...     with open("frame.jpg", "wb") as f:
            ...         f.write(jpeg_bytes)
            >>> sub = robot.subscribe_camera_image(on_frame)
            >>> # ... later ...
            >>> sub.unsubscribe()
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        import base64


        topic = roslibpy.Topic(
            self._client,
            '/camera/image/compressed',
            'sensor_msgs/msg/CompressedImage',
        )

        def parse_and_forward(msg):
            # Data is base64-encoded JPEG in sensor_msgs/CompressedImage
            data = msg.get('data', '')
            if isinstance(data, str) and data:
                jpeg_bytes = base64.b64decode(data)
                callback(jpeg_bytes)

        topic.subscribe(parse_and_forward)
        return topic

    def subscribe_camera_legacy(
        self,
        callback: Callable[[str], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to legacy base64-encoded camera stream.

        This is the backward-compatible endpoint. For new code,
        use subscribe_camera_image() with CBOR for better performance.

        Args:
            callback: Called with base64-encoded JPEG string.

        Returns:
            Topic object (call .unsubscribe() when done).
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        topic = roslibpy.Topic(
            self._client,
            '/camera_topic',
            'std_msgs/msg/String'
        )

        def parse_and_forward(msg):
            callback(msg.get('data', ''))

        topic.subscribe(parse_and_forward)
        return topic

    def set_camera_config(
        self,
        fps: Optional[int] = None,
        quality: Optional[int] = None,
        resolution: Optional[tuple] = None,
    ) -> None:
        """
        Configure camera settings.

        Note: Changes may cause brief stream interruption (~100-200ms)
        as the pipeline rebuilds.

        Args:
            fps: Frames per second (e.g., 30).
            quality: JPEG quality 1-100 (e.g., 80).
            resolution: (width, height) tuple (e.g., (1280, 720)).
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        config = {}
        if fps is not None:
            config['fps'] = fps
        if quality is not None:
            config['quality'] = quality
        if resolution is not None:
            config['resolution'] = list(resolution)

        if not config:
            return

        topic = roslibpy.Topic(
            self._client,
            '/camera/config',
            'std_msgs/msg/String'
        )
        topic.publish({'data': json.dumps(config)})

    # ==================== AI DETECTION METHODS ====================

    def subscribe_ai_detections(
        self,
        callback: Callable[[dict], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to AI detection results from OAK-D Lite.

        Inference only runs while subscribed (on-demand activation).
        Results format depends on the currently loaded model type
        (detection, pose, hand, segmentation, etc.).

        For typed results, use with AIDetectionReceiver from pib3.backends.camera:

            >>> from pib3.backends import AIDetectionReceiver
            >>> receiver = AIDetectionReceiver()
            >>> sub = robot.subscribe_ai_detections(receiver.on_detection)
            >>> time.sleep(5)
            >>> sub.unsubscribe()
            >>>
            >>> # Get typed Detection objects
            >>> for det in receiver.get_detections():
            ...     print(f"{det.label}: {det.confidence:.2f}")
            >>>
            >>> # Get typed HandLandmarks with finger angles
            >>> for hand in receiver.get_hand_landmarks():
            ...     print(f"{hand.handedness}: angles={hand.finger_angles}")
            >>>
            >>> # Check FPS/latency
            >>> print(f"FPS: {receiver.fps:.1f}, Latency: {receiver.avg_latency_ms:.1f}ms")

        Args:
            callback: Called with detection dict containing:
                - model: str - Model name (e.g., "yolov6n", "hand", "pose_yolo")
                - type: str - "detection", "hand", "pose", "instance-segmentation"
                - frame_id: int - Frame sequence number
                - timestamp_ns: int - Timestamp in nanoseconds
                - latency_ms: float - Inference latency
                - result: dict - Model-specific results:
                    - detection: {"detections": [{"label", "confidence", "bbox"}]}
                    - hand: {"keypoints": [...], "finger_angles": {...}, "handedness": {...}}
                    - pose: {"keypoints": [...]} or {"detections": [...with keypoints...]}

        Returns:
            Topic object (call .unsubscribe() when done to stop inference).

        Example:
            >>> def on_detection(data):
            ...     if data['type'] == 'detection':
            ...         for det in data['result']['detections']:
            ...             print(f"Found class {det['label']} at {det['bbox']}")
            ...     elif data['type'] == 'hand':
            ...         angles = data['result'].get('finger_angles', {})
            ...         print(f"Index angle: {angles.get('index', 0):.1f}°")
            >>> sub = robot.subscribe_ai_detections(on_detection)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        topic = roslibpy.Topic(
            self._client,
            '/camera/ai/detections',
            'std_msgs/msg/String'
        )

        def parse_and_forward(msg):
            data = json.loads(msg.get('data', '{}'))
            callback(data)

        topic.subscribe(parse_and_forward)
        return topic

    def get_available_ai_models(self, timeout: float = 5.0) -> dict:
        """
        Get list of available AI models on the robot.

        Args:
            timeout: Max time to wait for response in seconds.

        Returns:
            Dict mapping model names to their info:
            {
                "mobilenet-ssd": {
                    "type": "detection",
                    "input_size": [300, 300],
                    "fps": 30,
                    "description": "Fast object detection"
                },
                ...
            }
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        result = {}
        event = threading.Event()

        def on_models(msg):
            nonlocal result
            result = json.loads(msg.get('data', '{}'))
            event.set()

        topic = roslibpy.Topic(
            self._client,
            '/camera/ai/available_models',
            'std_msgs/msg/String'
        )
        topic.subscribe(on_models)
        event.wait(timeout=timeout)
        topic.unsubscribe()
        return result

    def set_ai_model(
        self,
        model: Union[AIModel, str],
        timeout: float = 5.0,
    ) -> bool:
        """
        Switch AI model on the OAK-D Lite camera (synchronous).

        This method waits for confirmation that the model has been
        successfully loaded before returning.

        Note: Model switching causes brief interruption (~200-500ms)
        as the pipeline rebuilds with the new neural network.

        Args:
            model: AI model to load. Can be an AIModel enum value or string.
                Use AIModel enum for IDE autocompletion:
                    >>> robot.set_ai_model(AIModel.HAND)
                    >>> robot.set_ai_model(AIModel.YOLOV8N)
                Or use string names:
                    >>> robot.set_ai_model("hand")
                    >>> robot.set_ai_model("yolov8n")
            timeout: Maximum time to wait for model switch confirmation
                (default: 5.0 seconds).

        Returns:
            True if model switch confirmed, False if timeout occurred.

        Example:
            >>> from pib3 import Robot, AIModel
            >>> with Robot(host="...") as robot:
            ...     robot.set_ai_model(AIModel.HAND)
            ...     robot.set_ai_model(AIModel.YOLOV8N)
            ...     robot.set_ai_model("mobilenet-ssd")  # String also works
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")

        # Convert AIModel enum to string value
        model_name = model.value if isinstance(model, AIModel) else str(model)

        import threading


        model_ready = threading.Event()

        # Subscribe to current model updates first
        model_topic = roslibpy.Topic(
            self._client,
            '/camera/ai/current_model',
            'std_msgs/msg/String'
        )

        def on_model_update(msg):
            data = json.loads(msg.get('data', '{}'))
            current_model = data.get('name', data.get('model', ''))
            # Check if the target model is now active
            if model_name in current_model or current_model in model_name:
                model_ready.set()

        model_topic.subscribe(on_model_update)

        # Publish the model change request
        config_topic = roslibpy.Topic(
            self._client,
            '/ai/config',
            'std_msgs/msg/String'
        )
        config_topic.publish({'data': json.dumps({'model': model_name})})

        # Wait for confirmation
        success = model_ready.wait(timeout=timeout)

        # Clean up subscription
        model_topic.unsubscribe()

        return success

    def set_ai_config(
        self,
        model: Optional[Union[AIModel, str]] = None,
        confidence: Optional[float] = None,
        segmentation_mode: Optional[str] = None,
        segmentation_target_class: Optional[int] = None,
    ) -> None:
        """
        Configure AI inference settings.

        Args:
            model: Model to switch to (AIModel enum or string).
            confidence: Detection confidence threshold (0.0-1.0).
            segmentation_mode: "bbox" (lightweight) or "mask" (detailed RLE).
            segmentation_target_class: Class ID for mask mode segmentation.

        Note: Model/confidence changes cause pipeline rebuild (~200-500ms).
              Segmentation mode changes are instant (output format only).
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        config = {}
        if model is not None:
            # Convert AIModel enum to string value
            config['model'] = model.value if isinstance(model, AIModel) else str(model)
        if confidence is not None:
            config['confidence'] = confidence
        if segmentation_mode is not None:
            config['segmentation_mode'] = segmentation_mode
        if segmentation_target_class is not None:
            config['segmentation_target_class'] = segmentation_target_class

        if not config:
            return

        topic = roslibpy.Topic(
            self._client,
            '/ai/config',
            'std_msgs/msg/String'
        )
        topic.publish({'data': json.dumps(config)})

    def subscribe_current_ai_model(
        self,
        callback: Callable[[dict], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to current AI model info updates.

        Args:
            callback: Called with model info dict when model changes:
                {
                    "name": "mobilenet-ssd",
                    "type": "detection",
                    "input_size": [300, 300],
                    "fps": 30,
                    "description": "..."
                }

        Returns:
            Topic object (call .unsubscribe() when done).
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        topic = roslibpy.Topic(
            self._client,
            '/camera/ai/current_model',
            'std_msgs/msg/String'
        )

        def parse_and_forward(msg):
            data = json.loads(msg.get('data', '{}'))
            callback(data)

        topic.subscribe(parse_and_forward)
        return topic

    # ==================== AUDIO METHODS ====================

    def subscribe_audio_stream(
        self,
        callback: Callable[[List[int]], None],
    ) -> "roslibpy.Topic":
        """
        Subscribe to audio stream from robot's microphone array.

        Receives raw PCM audio data captured from the microphone.
        The callback receives int16 samples that can be played back
        or processed.

        Audio Format:
            - Sample rate: 16000 Hz (16kHz)
            - Channels: 1 (Mono)
            - Bit depth: 16-bit signed integers
            - Chunk size: ~1024 samples per message

        Args:
            callback: Called with list of int16 audio samples for each chunk.

        Returns:
            Topic object (call .unsubscribe() when done to stop streaming).

        Example:
            >>> import numpy as np
            >>> audio_buffer = []
            >>> def on_audio(samples):
            ...     audio_buffer.extend(samples)
            ...     print(f"Received {len(samples)} samples")
            >>> sub = robot.subscribe_audio_stream(on_audio)
            >>> # ... record for some time ...
            >>> sub.unsubscribe()
            >>> # Convert to numpy array for processing
            >>> audio_data = np.array(audio_buffer, dtype=np.int16)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        topic = roslibpy.Topic(
            self._client,
            '/audio_stream',
            'std_msgs/msg/Int16MultiArray'
        )

        def parse_and_forward(msg):
            # Int16MultiArray message format:
            # { "layout": {...}, "data": [int16, int16, ...] }
            data = msg.get('data', [])
            if data:
                callback(data)

        topic.subscribe(parse_and_forward)
        return topic

    def play_audio_from_speech(
        self,
        text: str,
        language: str = "en",
        wait: bool = True,
        timeout: float = 30.0,
    ) -> bool:
        """
        Play text-to-speech audio on the robot.

        Uses the robot's TTS service to synthesize and play speech.
        This is useful for making the robot speak without needing
        to send audio data.

        Args:
            text: Text to speak.
            language: Language code (e.g., "en" for English, "de" for German).
            wait: If True, wait for speech to complete. If False, return immediately.
            timeout: Max time to wait for service response (seconds).

        Returns:
            True if speech was initiated successfully.

        Example:
            >>> robot.play_audio_from_speech("Hello, I am PIB!")
            >>> robot.play_audio_from_speech("Hallo!", language="de")
            >>> # Fire and forget
            >>> robot.play_audio_from_speech("Working...", wait=False)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        service = roslibpy.Service(
            self._client,
            '/play_audio_from_speech',
            'datatypes/srv/PlayAudioFromSpeech'
        )

        request = roslibpy.ServiceRequest({
            'speech': text,
            'language': language,
            'join': wait,
        })

        try:
            result = service.call(request, timeout=timeout)
            return result.get('success', True)
        except Exception as e:
            logger.warning(f"TTS service call failed: {e}")
            return False

    def play_audio_from_file(
        self,
        filepath: str,
        wait: bool = True,
        timeout: float = 30.0,
    ) -> bool:
        """
        Play a WAV audio file that exists on the robot's filesystem.

        Note: This requires the audio file to already exist on the robot.
        There is currently no way to upload audio files via ROS topics.
        Use this for pre-installed sounds or files transferred separately.

        Args:
            filepath: Absolute path to the WAV file on the robot's filesystem.
            wait: If True (join=True), wait for playback to finish.
                 If False (join=False), return immediately while audio plays.
            timeout: Max time to wait for service response (seconds).

        Returns:
            True if playback was initiated successfully.

        Example:
            >>> # Play a pre-installed sound effect
            >>> robot.play_audio_from_file("/home/pib/sounds/startup.wav")
            >>> # Play without waiting
            >>> robot.play_audio_from_file("/home/pib/sounds/notification.wav", wait=False)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        service = roslibpy.Service(
            self._client,
            '/play_audio_from_file',
            'datatypes/srv/PlayAudioFromFile'
        )

        request = roslibpy.ServiceRequest({
            'filepath': filepath,
            'join': wait,
        })

        try:
            result = service.call(request, timeout=timeout)
            return result.get('success', True)
        except Exception as e:
            logger.warning(f"Play audio file service call failed: {e}")
            return False

    # ==================== IMU METHODS ====================

    def subscribe_imu(
        self,
        callback: Callable[[dict], None],
        data_type: Union[str, ImuType] = "full",
    ):
        """
        Subscribe to IMU data from OAK-D Lite BMI270.

        Streaming only runs while subscribed (on-demand activation).

        Data types use individual topics:
        - "full": Combined accel + gyro from both topics, merged on each
          new message (latest value of the other channel is reused).
        - "accelerometer": Vector3Stamped from /camera/imu/accelerometer
        - "gyroscope": Vector3Stamped from /camera/imu/gyroscope

        Args:
            callback: Called with IMU data dict. For "full" mode the dict
                has ``header``, ``linear_acceleration`` and
                ``angular_velocity`` keys.
            data_type: One of "full", "accelerometer", "gyroscope".
                Also accepts ImuType enum members.

        Returns:
            Subscription handle (call .unsubscribe() when done to stop
            streaming). For "full" mode this handle cancels both underlying
            topic subscriptions.

        Example:
            >>> def on_imu(data):
            ...     accel = data['linear_acceleration']
            ...     gyro = data['angular_velocity']
            ...     print(f"Accel: x={accel['x']:.2f} m/s², Gyro: z={gyro.get('z', 0):.2f} rad/s")
            >>> sub = robot.subscribe_imu(on_imu, data_type=ImuType.FULL)
        """
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        # Handle Enum or string
        dtype_str = data_type.value if isinstance(data_type, ImuType) else data_type

        valid_types = [ImuType.FULL.value, ImuType.ACCELEROMETER.value, ImuType.GYROSCOPE.value]
        if dtype_str not in valid_types:
            raise ValueError(f"data_type must be one of: {valid_types}")

        # IMU data comes from individual topics:
        # - /camera/imu/accelerometer (geometry_msgs/msg/Vector3Stamped)
        # - /camera/imu/gyroscope (geometry_msgs/msg/Vector3Stamped)
        # The combined /camera/imu topic may not always publish data.

        if dtype_str == ImuType.FULL.value:
            # Combine both streams: subscribe to accel + gyro, buffer the
            # latest reading of each and emit a merged payload whenever a
            # new message arrives on either channel.
            accel_topic = roslibpy.Topic(
                self._client,
                '/camera/imu/accelerometer',
                'geometry_msgs/msg/Vector3Stamped',
            )
            gyro_topic = roslibpy.Topic(
                self._client,
                '/camera/imu/gyroscope',
                'geometry_msgs/msg/Vector3Stamped',
            )

            lock = threading.Lock()
            state = {'accel': None, 'gyro': None, 'header': None}

            def emit_locked():
                callback({
                    'header': state['header'] or {},
                    'linear_acceleration': state['accel'] or {},
                    'angular_velocity': state['gyro'] or {},
                })

            def on_accel(msg):
                with lock:
                    state['accel'] = msg.get('vector', {})
                    state['header'] = msg.get('header', state['header'])
                    emit_locked()

            def on_gyro(msg):
                with lock:
                    state['gyro'] = msg.get('vector', {})
                    state['header'] = msg.get('header', state['header'])
                    emit_locked()

            accel_topic.subscribe(on_accel)
            gyro_topic.subscribe(on_gyro)

            return _CompositeImuSubscription(accel_topic, gyro_topic)

        elif dtype_str == ImuType.ACCELEROMETER.value:
            # Subscribe to accelerometer topic directly
            topic = roslibpy.Topic(
                self._client,
                '/camera/imu/accelerometer',
                'geometry_msgs/msg/Vector3Stamped'
            )
            topic.subscribe(callback)
            return topic
        else:
            # Subscribe to gyroscope topic directly
            topic = roslibpy.Topic(
                self._client,
                '/camera/imu/gyroscope',
                'geometry_msgs/msg/Vector3Stamped'
            )
            topic.subscribe(callback)
            return topic

    def set_imu_frequency(self, frequency: int) -> None:
        """
        Set IMU sampling frequency.

        The BMI270 IMU only supports a fixed set of frequencies and will
        round down to the nearest valid value:

        ========== ==================
        Requested  Actual (BMI270)
        ========== ==================
        25 Hz      25 Hz
        50 Hz      50 Hz
        100 Hz     100 Hz
        200 Hz     200 Hz
        250 Hz     250 Hz (max)
        ========== ==================

        Any value is accepted; the sensor rounds down automatically.
        A warning is logged when a non-standard value is requested.

        Args:
            frequency: Desired frequency in Hz.
        """
        valid_frequencies = [25, 50, 100, 200, 250]
        if frequency not in valid_frequencies:
            logger.warning(
                f"IMU frequency {frequency} Hz is not a standard BMI270 frequency. "
                f"The sensor will round down to the nearest of: "
                f"{', '.join(map(str, valid_frequencies))} Hz."
            )

        if not self.is_connected:
            raise ConnectionError("Not connected to robot")


        topic = roslibpy.Topic(
            self._client,
            '/imu/config',
            'std_msgs/msg/String'
        )
        topic.publish({'data': json.dumps({'frequency': frequency})})

    # ==================== UNIFIED AUDIO OVERRIDES ====================

    @property
    def default_audio_output(self) -> AudioOutput:
        """Default audio output for real robot: ROBOT."""
        return AudioOutput.ROBOT

    def _get_robot_audio_player(self) -> Optional[RobotAudioPlayer]:
        """Get or create robot audio player."""
        if self._robot_audio_player is None and self.is_connected:
            try:
                self._robot_audio_player = RobotAudioPlayer(self._client)
            except Exception as e:
                logger.warning(f"Failed to create robot audio player: {e}")
        return self._robot_audio_player

    def _play_on_robot(
        self,
        data: np.ndarray,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        block: bool = True,
    ) -> bool:
        """
        Play audio on robot speakers via /audio_playback topic.

        Args:
            data: Audio data as int16 numpy array.
            sample_rate: Sample rate in Hz (default: 16000).
            block: If True, wait for estimated playback duration.

        Returns:
            True if audio was sent successfully.
        """
        if not self.is_connected:
            logger.warning("Cannot play on robot: not connected")
            return False

        player = self._get_robot_audio_player()
        if player is None:
            logger.warning("Robot audio player not available")
            return False

        return player.play(data, sample_rate, block=block)

    @property
    def default_audio_input(self) -> AudioInput:
        """Default audio input for real robot: ROBOT."""
        return AudioInput.ROBOT

    def _get_robot_audio_recorder(self) -> Optional[RobotAudioRecorder]:
        """Get or create robot audio recorder."""
        if self._robot_audio_recorder is None and self.is_connected:
            try:
                self._robot_audio_recorder = RobotAudioRecorder(self._client)
            except Exception as e:
                logger.warning(f"Failed to create robot audio recorder: {e}")
        return self._robot_audio_recorder

    def _record_from_robot(
        self,
        duration: float,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ) -> Optional[np.ndarray]:
        """
        Record audio from robot's microphone via /audio_input topic.

        Args:
            duration: Recording duration in seconds.
            sample_rate: Sample rate in Hz (default: 16000).

        Returns:
            Audio data as numpy array of int16 samples, or None if failed.
        """
        if not self.is_connected:
            logger.warning("Cannot record from robot: not connected")
            return None

        recorder = self._get_robot_audio_recorder()
        if recorder is None:
            logger.warning("Robot audio recorder not available")
            return None

        try:
            return recorder.record(duration, sample_rate)
        except Exception as e:
            logger.warning(f"Robot audio recording failed: {e}")
            return None

    def speak(
        self,
        text: str,
        output: Optional[AudioOutput] = None,
        voice: Optional[str] = None,
        block: bool = True,
        use_robot_tts: Optional[bool] = None,
        language: str = "de",
    ) -> bool:
        """
        Synthesize and play text-to-speech.

        On the real robot, prefers the on-board ``/play_audio_from_speech``
        ROS service (synthesized on the robot, no audio streamed over
        rosbridge). Falls back to local Piper synthesis + audio transport
        when the robot service is unavailable or ``output`` targets LOCAL.

        Args:
            text: Text to speak.
            output: Playback destination. If None, uses backend default
                (ROBOT for RealRobotBackend).
            voice: Piper voice model (only used for local synthesis).
            block: If True, wait for playback to complete.
            use_robot_tts: Force robot-side TTS (True), force local Piper
                (False), or auto-decide (None, default). Auto prefers robot
                TTS whenever the output includes ROBOT.
            language: Language code for the robot's TTS service (e.g. "en",
                "de"). Ignored for local Piper, which picks language via
                ``voice``.

        Returns:
            True on success.
        """
        if output is None:
            output = self._audio_output or self.default_audio_output

        wants_robot_path = output in (AudioOutput.ROBOT, AudioOutput.LOCAL_AND_ROBOT)
        try_robot_tts = use_robot_tts if use_robot_tts is not None else wants_robot_path

        if try_robot_tts and self.is_connected:
            try:
                ok = self.play_audio_from_speech(
                    text, language=language, wait=block
                )
                if ok and output == AudioOutput.ROBOT:
                    return True
                if ok and output == AudioOutput.LOCAL_AND_ROBOT:
                    # Robot side already played; also play locally via Piper.
                    return super().speak(text, output=AudioOutput.LOCAL, voice=voice, block=block)
                # On failure, fall through to the base-class Piper path.
            except Exception as e:
                logger.warning(f"Robot TTS service failed ({e}); falling back to local Piper")

        return super().speak(text, output=output, voice=voice, block=block)


# ==================== RLE DECODER HELPER ====================

def rle_decode(rle: dict) -> np.ndarray:
    """
    Decode RLE-encoded segmentation mask to numpy array.

    Decodes the format produced by the robot's on-board ``rle_encode()``:
    a dict with 'runs' (run lengths), 'values' (pixel values per run),
    and 'shape' [height, width].

    Args:
        rle: Dict with 'runs', 'values', and 'shape' keys.

    Returns:
        Mask as numpy array of shape (height, width).

    Example:
        >>> def on_detection(data):
        ...     if data['type'] == 'segmentation':
        ...         result = data['result']
        ...         if result.get('mode') == 'mask':
        ...             mask = rle_decode(result['mask_rle'])
        ...             print(f"Mask shape: {mask.shape}")
    """
    shape = rle.get("shape", [0, 0])
    runs = rle.get("runs", [])
    values = rle.get("values", [])

    total_pixels = int(shape[0]) * int(shape[1])
    if not runs or not values or total_pixels == 0:
        return np.zeros(shape, dtype=np.uint8)

    pixels = np.repeat(
        np.asarray(values, dtype=np.uint8),
        np.asarray(runs, dtype=np.int64),
    )

    if pixels.size < total_pixels:
        pixels = np.concatenate([
            pixels,
            np.zeros(total_pixels - pixels.size, dtype=np.uint8),
        ])
    elif pixels.size > total_pixels:
        pixels = pixels[:total_pixels]

    return pixels.reshape(shape)
