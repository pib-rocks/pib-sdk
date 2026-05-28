"""Webots simulator backend for pib3 package."""

import logging
import time
from typing import Any, Callable, Dict, List, Optional

import numpy as np

logger = logging.getLogger(__name__)

from .base import RobotBackend


# Mapping from trajectory joint names to Webots motor device names
JOINT_TO_WEBOTS_MOTOR = {
    "turn_head_motor": "head_horizontal",
    "tilt_forward_motor": "head_vertical",
    "shoulder_vertical_left": "shoulder_vertical_left",
    "shoulder_horizontal_left": "shoulder_horizontal_left",
    "upper_arm_left_rotation": "upper_arm_left",
    "elbow_left": "elbow_left",
    "lower_arm_left_rotation": "forearm_left",
    "wrist_left": "wrist_left",
    "thumb_left_opposition": "thumb_left_opposition",
    "thumb_left_stretch": "thumb_left_distal",
    "index_left_stretch": "index_left_distal",
    "middle_left_stretch": "middle_left_distal",
    "ring_left_stretch": "ring_left_distal",
    "pinky_left_stretch": "pinky_left_distal",
    "shoulder_vertical_right": "shoulder_vertical_right",
    "shoulder_horizontal_right": "shoulder_horizontal_right",
    "upper_arm_right_rotation": "upper_arm_right",
    "elbow_right": "elbow_right",
    "lower_arm_right_rotation": "forearm_right",
    "wrist_right": "wrist_right",
    "thumb_right_opposition": "thumb_right_opposition",
    "thumb_right_stretch": "thumb_right_distal",
    "index_right_stretch": "index_right_distal",
    "middle_right_stretch": "middle_right_distal",
    "ring_right_stretch": "ring_right_distal",
    "pinky_right_stretch": "pinky_right_distal",
}

# Finger joints that have both proximal and distal motors in Webots.
# The real robot has a single motor per finger with a construction train
# (mechanical linkage) that bends both proximal and distal joints equally.
# To act as a faithful digital twin, Webots always commands both joints to
# the same position on write and reads only the distal sensor (since both
# joints are always identical).  Do NOT decouple these joints.
# Webots finger joints: 0° = open, 90° = closed (range 0 to π/2 radians)
FINGER_PROXIMAL_MOTORS = {
    "thumb_left_stretch": "thumb_left_proximal",
    "index_left_stretch": "index_left_proximal",
    "middle_left_stretch": "middle_left_proximal",
    "ring_left_stretch": "ring_left_proximal",
    "pinky_left_stretch": "pinky_left_proximal",
    "thumb_right_stretch": "thumb_right_proximal",
    "index_right_stretch": "index_right_proximal",
    "middle_right_stretch": "middle_right_proximal",
    "ring_right_stretch": "ring_right_proximal",
    "pinky_right_stretch": "pinky_right_proximal",
}


class WebotsBackend(RobotBackend):
    """
    Execute trajectories in Webots simulator.

    Must be instantiated from within a Webots controller script.

    Webots interprets motor positions as relative to the base position at
    simulation time zero. On connect, the backend reads each joint's initial
    position and stores it as a per-joint offset. All position commands are
    then converted from absolute radians to Webots-relative positions using
    these offsets. After ``_reset_to_zero()`` the offsets become 0.0 (since
    all joints are driven to absolute zero), so commanding 0 rad targets
    the proto-defined zero.

    When reading joint positions, the backend waits for motor readings to
    stabilize (same value twice) to ensure accurate readings when motors
    are in motion. Use the `timeout` parameter in get_joints() to control
    how long to wait (default: 5.0 seconds).

    Example:
        # In your Webots controller file:
        from pib3.backends import WebotsBackend

        with WebotsBackend() as backend:
            backend.run_trajectory("trajectory.json")

            # Read joint positions (waits up to 5s for stabilization)
            joints = backend.get_joints()

            # Read with custom timeout
            joints = backend.get_joints(timeout=2.0)
    """

    def __init__(
        self,
        step_ms: int = 50,
    ):
        """
        Initialize Webots backend.

        Args:
            step_ms: Time step per waypoint in milliseconds.
        """
        super().__init__()
        self.step_ms = step_ms
        self._robot = None
        self._timestep = None
        self._motors: Dict[str, Any] = {}
        self._proximal_motors: Dict[str, Any] = {}
        # Per-joint offsets: the initial position of each joint at simulation
        # start (base position at timepoint zero).  Webots setPosition() is
        # relative to this base, so we subtract the offset when commanding
        # and add it back when reading.
        self._home_offsets: Dict[str, float] = {}

    def _to_backend_format(self, radians: np.ndarray) -> np.ndarray:
        """Convert absolute radians to Webots-relative positions.

        After _reset_to_zero() all offsets are 0.0, so this is an identity.
        """
        # For trajectory playback, offsets are per-column (per joint).
        # After reset they are all 0 so this is effectively a no-op.
        return radians

    def _from_backend_format(self, values: np.ndarray) -> np.ndarray:
        """Convert Webots-relative positions to absolute radians."""
        return values

    def connect(self) -> None:
        """Initialize Webots robot and motors."""
        try:
            from controller import Robot
        except ImportError:
            raise ImportError(
                "Webots controller module not found. "
                "This backend must be used from within a Webots controller script."
            )

        self._robot = Robot()
        self._timestep = int(self._robot.getBasicTimeStep())

        # Initialize all motors (only once)
        if len(self._motors) == 0:
            self._motors = {}
            
            for joint_name, motor_name in JOINT_TO_WEBOTS_MOTOR.items():
                motor = self._robot.getDevice(motor_name)
                if motor is not None:
                    self._motors[joint_name] = motor

                    # Enable once, forever. Readout will be available at each timestep.
                    sensor = motor.getPositionSensor()
                    if sensor is not None:
                        sensor.enable(self._timestep)
        
        # Initialize proximal finger motors (only once)
        if len(self._proximal_motors) == 0:
            self._proximal_motors = {}
            for joint_name, proximal_motor_name in FINGER_PROXIMAL_MOTORS.items():
                motor = self._robot.getDevice(proximal_motor_name)
                if motor is not None:
                    self._proximal_motors[joint_name] = motor
                    logger.debug(f"Initialized proximal motor: {joint_name} -> {proximal_motor_name}")
                    sensor = motor.getPositionSensor()
                    if sensor is not None:
                        sensor.enable(self._timestep)
                else:
                    logger.warning(f"Proximal motor not found: {proximal_motor_name}")

        # Read initial joint positions (base position at timepoint zero).
        # Webots setPosition() targets are relative to this base, so we
        # record the offsets before resetting to absolute zero.
        self._read_home_offsets()

        # Drive all motors to absolute zero, then clear offsets (since
        # the joints are now at 0 rad and positions become absolute).
        self._reset_to_zero()

    def _read_home_offsets(self) -> None:
        """Read and store each joint's initial position as its home offset.

        Must be called after sensors are enabled and before _reset_to_zero().
        The offsets represent the base position at simulation timepoint zero.
        """
        # Step once so sensors produce valid readings
        self._robot.step(self._timestep)

        self._home_offsets = {}
        for name, motor in self._motors.items():
            sensor = motor.getPositionSensor()
            if sensor is not None:
                pos = sensor.getValue()
                self._home_offsets[name] = pos
                if abs(pos) > 0.01:
                    logger.info(
                        f"Joint {name} initial offset: {pos:.4f} rad "
                        f"({pos * 180 / np.pi:.1f} deg)"
                    )
            else:
                self._home_offsets[name] = 0.0

    def _reset_to_zero(self) -> None:
        """Reset all motors to absolute zero and wait for convergence.

        Uses the stored home offsets to command the correct Webots-relative
        position that drives each joint to absolute 0 rad.  After convergence
        the offsets are cleared (set to 0.0) so that subsequent commands are
        in absolute coordinates.
        """
        # Log non-zero starting positions
        for name, offset in self._home_offsets.items():
            if abs(offset) > 0.01:
                logger.warning(
                    f"Joint {name} starts at {offset:.4f} rad "
                    f"({offset * 180 / np.pi:.1f} deg) — resetting to zero"
                )

        # Command all motors to absolute zero.
        # Webots target = absolute_target - home_offset = 0.0 - offset = -offset
        for name, motor in self._motors.items():
            offset = self._home_offsets.get(name, 0.0)
            motor.setPosition(-offset)
        for motor in self._proximal_motors.values():
            motor.setPosition(0.0)

        # Step simulation until all motors converge (or timeout)
        tolerance = 0.01  # radians
        max_steps = int(5000 / self._timestep)  # 5 seconds max

        for _ in range(max_steps):
            self._robot.step(self._timestep)

            all_at_zero = True
            for name, motor in self._motors.items():
                sensor = motor.getPositionSensor()
                if sensor is not None:
                    # The sensor reads Webots-relative position;
                    # absolute = reading + home_offset.
                    offset = self._home_offsets.get(name, 0.0)
                    absolute_pos = sensor.getValue() + offset
                    if abs(absolute_pos) > tolerance:
                        all_at_zero = False
                        break

            if all_at_zero:
                logger.info("All motors reached zero position.")
                # Offsets are now consumed — joints are at absolute 0,
                # so future commands need no offset adjustment.
                self._home_offsets = {name: 0.0 for name in self._home_offsets}
                return

        logger.warning("Timeout waiting for motors to reach zero position.")
        # Clear offsets even on timeout so the system remains usable
        self._home_offsets = {name: 0.0 for name in self._home_offsets}

    def disconnect(self) -> None:
        """Cleanup (no-op for Webots, robot lifecycle managed by simulator)."""
        # self._motors = {}
        pass

    @property
    def is_connected(self) -> bool:
        """Check if robot is initialized."""
        return self._robot is not None

    # Default timeout for waiting for motor stabilization (seconds)
    DEFAULT_GET_JOINTS_TIMEOUT = 5.0

    def _get_joint_radians(
        self,
        motor_name: str,
        timeout: Optional[float] = None,
    ) -> Optional[float]:
        """
        Get current position of a single joint in absolute radians.

        Reads the Webots-relative sensor value and adds the home offset
        to return the absolute position.

        For finger joints, only the distal motor sensor is read.  The real
        robot uses a single motor with a construction train that bends both
        proximal and distal joints equally, so they always have the same
        angle.  Webots mirrors this by always commanding both joints to the
        same value (see ``_set_joints_impl``), making the distal reading
        sufficient.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").
            timeout: Max time to wait for motor to stabilize (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).

        Returns:
            Current position in absolute radians, or None if unavailable.
        """
        if not self.is_connected:
            return None

        if motor_name in self._motors:
            motor = self._motors[motor_name]
            sensor = motor.getPositionSensor()
            if sensor is not None:
                if timeout is None:
                    timeout = self.DEFAULT_GET_JOINTS_TIMEOUT
                offset = self._home_offsets.get(motor_name, 0.0)
                start = time.time()
                webots_pos_old = sensor.getValue()
                while (time.time() - start) < timeout:
                    self._robot.step(self._timestep)
                    webots_pos = sensor.getValue()
                    # Check if motor has stabilized (same reading twice)
                    if abs(webots_pos - webots_pos_old) < 0.0001:
                        return webots_pos + offset
                    else:
                        webots_pos_old = webots_pos
            return None

        return None

    def _get_joints_radians(
        self,
        motor_names: Optional[List[str]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints in radians.

        Reads all requested joints simultaneously after waiting for the
        simulation to stabilize. This avoids the inconsistency of reading
        joints sequentially (where each read advances the simulation,
        potentially moving joints that were already read).

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available joints.
            timeout: Max time to wait for all motors to stabilize (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).

        Returns:
            Dict mapping motor names to positions in radians.
        """
        if not self.is_connected:
            return {}

        if timeout is None:
            timeout = self.DEFAULT_GET_JOINTS_TIMEOUT

        names_to_query = motor_names if motor_names is not None else list(self._motors.keys())

        # Filter to names that have valid motors and sensors
        valid_names = []
        for name in names_to_query:
            if name in self._motors:
                sensor = self._motors[name].getPositionSensor()
                if sensor is not None:
                    valid_names.append(name)

        if not valid_names:
            return {}

        # Require several consecutive stable reads before accepting the
        # value — a single stable step can be a velocity zero-crossing mid
        # motion and give a false positive.
        required_stable = self.STABILITY_CONSECUTIVE_READS
        stable_threshold = self.STABILITY_THRESHOLD_RAD

        prev_readings: Dict[str, float] = {}
        for name in valid_names:
            sensor = self._motors[name].getPositionSensor()
            prev_readings[name] = sensor.getValue()

        stable_count = 0
        start = time.time()
        while (time.time() - start) < timeout:
            self._robot.step(self._timestep)

            all_stable = True
            current_readings: Dict[str, float] = {}
            for name in valid_names:
                sensor = self._motors[name].getPositionSensor()
                val = sensor.getValue()
                current_readings[name] = val
                if abs(val - prev_readings.get(name, float('inf'))) >= stable_threshold:
                    all_stable = False

            if all_stable:
                stable_count += 1
                if stable_count >= required_stable:
                    result = {}
                    for name in valid_names:
                        offset = self._home_offsets.get(name, 0.0)
                        result[name] = current_readings[name] + offset
                    return result
            else:
                stable_count = 0

            prev_readings = current_readings

        # Timeout — return best readings we have
        result = {}
        for name in valid_names:
            offset = self._home_offsets.get(name, 0.0)
            result[name] = prev_readings[name] + offset
        return result

    def _set_joints_impl(
        self,
        positions_radians: Dict[str, float],
        velocity_centideg: Optional[int] = None,
    ) -> bool:
        """Set joint positions in absolute radians.

        Converts absolute radians to Webots-relative positions by subtracting
        each joint's home offset.  For finger joints, both proximal and distal
        motors are set to the same position for coupled movement.

        Args:
            positions_radians: Dict mapping motor names to positions in radians.
            velocity_centideg: Ignored in Webots (motor velocity is set via
                Webots motor API, not centidegrees).
        """
        if not self.is_connected:
            return False

        for joint_name, position in positions_radians.items():
            if joint_name in self._motors:
                offset = self._home_offsets.get(joint_name, 0.0)
                webots_pos = position - offset
                logger.debug(f"Setting {joint_name} to {position:.4f} rad absolute "
                             f"(webots={webots_pos:.4f}, offset={offset:.4f})")
                self._motors[joint_name].setPosition(webots_pos)

                if joint_name in self._proximal_motors:
                    self._proximal_motors[joint_name].setPosition(webots_pos)

        # Step simulation once to initiate movement
        self._robot.step(self._timestep)
        return True

    def _verify_positions(
        self,
        target_positions: Dict[str, float],
        unit: str,
        timeout: float,
        tolerance: float,
    ) -> bool:
        """
        Verify joints reached target positions by stepping the simulation.

        Overrides base class to continuously step Webots simulation until
        motors reach their targets, providing blocking behavior that matches
        the real robot API.

        Args:
            target_positions: Dict of target positions (in specified unit).
            unit: Unit of the target positions ("percent", "deg", or "rad").
            timeout: Max time to wait (seconds).
            tolerance: Acceptable error (in same unit).

        Returns:
            True if all joints are within tolerance.
        """
        import math

        # Convert targets and per-joint tolerance to radians for comparison.
        # In percent mode, 1% means different absolute angles for each joint
        # (joints have different calibrated ranges), so tolerance must be
        # computed per joint using the same linear map as _percent_to_radians.
        targets_rad: Dict[str, float] = {}
        tolerances_rad: Dict[str, float] = {}

        if unit == "percent":
            for name, pos in target_positions.items():
                targets_rad[name] = self._percent_to_radians(name, pos)
                # 1% of the joint's calibrated range, applied symmetrically.
                span_rad = abs(
                    self._percent_to_radians(name, 100.0)
                    - self._percent_to_radians(name, 0.0)
                )
                tolerances_rad[name] = tolerance * 0.01 * span_rad
        elif unit == "deg":
            tol_rad = math.radians(tolerance)
            for name, pos in target_positions.items():
                targets_rad[name] = math.radians(pos)
                tolerances_rad[name] = tol_rad
        else:  # rad
            for name, pos in target_positions.items():
                targets_rad[name] = pos
                tolerances_rad[name] = tolerance

        start_time = time.time()
        max_steps = int(timeout * 1000 / self._timestep)  # Convert timeout to max simulation steps
        required_stable = self.VERIFY_CONSECUTIVE_READS
        stable_count = 0

        # Always step at least once before accepting a result — otherwise a
        # request that is already within tolerance of the current (pre-move)
        # position would return True before the motor had a chance to act.
        stepped = False

        for _ in range(max_steps):
            # Step simulation to let motors move (and register the new target)
            if self._robot.step(self._timestep) == -1:
                return False
            stepped = True

            # Check if all joints are within per-joint tolerance
            all_within_tolerance = True
            for joint_name, target_rad in targets_rad.items():
                if joint_name not in self._motors:
                    continue

                motor = self._motors[joint_name]
                sensor = motor.getPositionSensor()
                if sensor is None:
                    continue

                offset = self._home_offsets.get(joint_name, 0.0)
                current_pos = sensor.getValue() + offset
                error = abs(current_pos - target_rad)

                if error > tolerances_rad[joint_name]:
                    all_within_tolerance = False
                    break

            if all_within_tolerance:
                stable_count += 1
                if stable_count >= required_stable:
                    return True
            else:
                stable_count = 0

            # Also check wall-clock timeout
            if (time.time() - start_time) >= timeout:
                return False

        return stepped and stable_count >= required_stable

    def _execute_waypoints(
        self,
        joint_names: List[str],
        waypoints: np.ndarray,
        rate_hz: float,
        progress_callback: Optional[Callable[[int, int], None]],
    ) -> bool:
        """Execute waypoints in Webots.

        Waypoints are in absolute radians.  Each position is converted to
        Webots-relative by subtracting the joint's home offset.
        """
        if not self.is_connected:
            return False

        # Build index mapping
        joint_indices = {}
        for i, name in enumerate(joint_names):
            if name in self._motors:
                joint_indices[name] = i

        step_ms = max(int(1000 / rate_hz), self._timestep)
        total = len(waypoints)

        for i, point in enumerate(waypoints):
            if self._stopped:
                logger.warning(f"Trajectory aborted at waypoint {i}/{total} (emergency stop)")
                return False

            for name, idx in joint_indices.items():
                position = point[idx]
                offset = self._home_offsets.get(name, 0.0)
                webots_pos = position - offset
                self._motors[name].setPosition(webots_pos)

                if name in self._proximal_motors:
                    self._proximal_motors[name].setPosition(webots_pos)

            self._robot.step(step_ms)

            if progress_callback:
                progress_callback(i + 1, total)

        return True

    # ==================== UNIFIED AUDIO OVERRIDES ====================

    def _is_webots(self) -> bool:
        """
        Webots backend returns True.

        This causes ROBOT and LOCAL_AND_ROBOT to resolve to LOCAL only,
        avoiding duplicate playback in simulation.
        """
        return True
