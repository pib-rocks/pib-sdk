"""Trajectory generation via inverse kinematics for pib3 package."""

import json
import warnings
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple, Union

import numpy as np

from .config import IKConfig, PaperConfig, TrajectoryConfig
from .types import Sketch, Stroke


# Joint indices for left and right arms
LEFT_ARM_JOINTS = {
    'shoulder_vertical': 2,
    'shoulder_horizontal': 3,
    'upper_arm': 4,
    'elbow': 5,
    'forearm': 6,
    'wrist': 7,
}

RIGHT_ARM_JOINTS = {
    'shoulder_vertical': 19,
    'shoulder_horizontal': 20,
    'upper_arm': 21,
    'elbow': 22,
    'forearm': 23,
    'wrist': 24,
}

# Left index finger joints (for kinematic chain)
LEFT_FINGER_JOINTS = [11, 12]
RIGHT_FINGER_JOINTS = [28, 29]

# TCP link names for index finger drawing
LEFT_TCP_LINK = 'urdf_finger_distal_2'
RIGHT_TCP_LINK = 'urdf_finger_distal_7'

# TCP link names for pencil grip drawing (palm-based)
LEFT_TCP_LINK_PENCIL = 'urdf_palm_left'
RIGHT_TCP_LINK_PENCIL = 'urdf_palm_right'

# Motor name mapping for URDF joints to Webots/real robot motor names
URDF_TO_MOTOR_NAME = {
    # Head
    0: 'turn_head_motor',
    1: 'tilt_forward_motor',
    # Left arm
    2: 'shoulder_vertical_left',
    3: 'shoulder_horizontal_left',
    4: 'upper_arm_left_rotation',
    5: 'elbow_left',
    6: 'lower_arm_left_rotation',
    7: 'wrist_left',
    # Left fingers
    8: 'thumb_left_opposition',
    9: 'thumb_left_stretch',
    10: 'thumb_left_stretch',
    11: 'index_left_stretch',
    12: 'index_left_stretch',
    13: 'middle_left_stretch',
    14: 'middle_left_stretch',
    15: 'ring_left_stretch',
    16: 'ring_left_stretch',
    17: 'pinky_left_stretch',
    18: 'pinky_left_stretch',
    # Right arm
    19: 'shoulder_vertical_right',
    20: 'shoulder_horizontal_right',
    21: 'upper_arm_right_rotation',
    22: 'elbow_right',
    23: 'lower_arm_right_rotation',
    24: 'wrist_right',
    # Right fingers
    25: 'thumb_right_opposition',
    26: 'thumb_right_stretch',
    27: 'thumb_right_stretch',
    28: 'index_right_stretch',
    29: 'index_right_stretch',
    30: 'middle_right_stretch',
    31: 'middle_right_stretch',
    32: 'ring_right_stretch',
    33: 'ring_right_stretch',
    34: 'pinky_right_stretch',
    35: 'pinky_right_stretch',
}


@dataclass
class Trajectory:
    """Robot joint trajectory for executing a drawing.

    Stores joint positions in canonical Webots motor radians.

    Attributes:
        joint_names: Names of joints in order (36 for PIB).
        waypoints: Array of shape (N, num_joints) with joint positions in radians.
        metadata: Additional info (paper position, IK stats, etc.)
    """
    joint_names: List[str]
    waypoints: np.ndarray
    metadata: dict = field(default_factory=dict)

    # Constants
    FORMAT_VERSION = "1.0"
    UNIT = "radians"
    COORDINATE_FRAME = "webots"  # Canonical format = Webots motor radians

    def __repr__(self) -> str:
        n_joints = len(self.joint_names)
        n_waypoints = len(self.waypoints)
        arm = self.metadata.get("arm", "?")
        rate = self.metadata.get("success_rate")
        rate_str = f", success={rate:.0%}" if rate is not None else ""
        return f"Trajectory({n_waypoints} waypoints, {n_joints} joints, arm={arm}{rate_str})"

    def __len__(self) -> int:
        """Return number of waypoints."""
        return len(self.waypoints)

    def __post_init__(self):
        """Ensure waypoints is a numpy array."""
        self.waypoints = np.asarray(self.waypoints, dtype=np.float64)

    def to_webots_format(self) -> np.ndarray:
        """Convert waypoints to Webots motor positions (identity).

        Waypoints are already in absolute radians.  Per-joint offsets from
        the Webots starting position are applied by the backend, not here.
        """
        return self.waypoints

    def to_robot_format(self) -> np.ndarray:
        """Convert waypoints to real robot format (centidegrees)."""
        return np.round(np.degrees(self.waypoints) * 100).astype(int)

    def to_json(self, path: Union[str, Path]) -> None:
        """Save trajectory to JSON file with unit metadata."""
        data = {
            "format_version": self.FORMAT_VERSION,
            "unit": self.UNIT,
            "coordinate_frame": self.COORDINATE_FRAME,
            "joint_names": self.joint_names,
            "waypoints": self.waypoints.tolist(),
            "metadata": {
                **self.metadata,
                "created_at": datetime.now(timezone.utc).isoformat(),
                "offsets": {
                    "description": "Canonical format is absolute radians. "
                    "Per-joint offsets from Webots starting position are "
                    "applied by the backend at playback time.",
                },
            },
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

    @classmethod
    def from_json(cls, path: Union[str, Path]) -> "Trajectory":
        """Load trajectory from JSON file.

        Validates the file shape up front so a slightly malformed file fails
        loudly (with a pointer to what's wrong) instead of silently degrading
        downstream — e.g. a missing ``arm`` metadata field misrouting joints
        in sequential-trajectory code.
        """
        with open(path) as f:
            data = json.load(f)

        if not isinstance(data, dict):
            raise ValueError(
                f"Trajectory file {path} must contain a JSON object at the top level, "
                f"got {type(data).__name__}."
            )

        # Version guard — unknown future versions may carry incompatible fields.
        version = data.get("format_version")
        if version is not None and str(version) != cls.FORMAT_VERSION:
            logger.warning(
                f"Trajectory {path} was written with format_version={version!r}; "
                f"this package expects {cls.FORMAT_VERSION!r}. Proceeding, but "
                f"field semantics may not match."
            )

        unit = data.get("unit")
        if unit is not None and unit != cls.UNIT:
            raise ValueError(
                f"Unsupported trajectory unit {unit!r} (expected {cls.UNIT!r})."
            )

        # Accept legacy "points"/"link_names" aliases but require one to exist.
        joint_names = data.get("joint_names", data.get("link_names"))
        if joint_names is None:
            raise ValueError(
                f"Trajectory file {path} is missing 'joint_names' (or legacy 'link_names')."
            )
        if not isinstance(joint_names, list) or not all(isinstance(n, str) for n in joint_names):
            raise ValueError(
                f"Trajectory file {path}: 'joint_names' must be a list of strings."
            )

        raw_waypoints = data.get("waypoints", data.get("points"))
        if raw_waypoints is None:
            raise ValueError(
                f"Trajectory file {path} is missing 'waypoints' (or legacy 'points')."
            )

        try:
            waypoints = np.asarray(raw_waypoints, dtype=np.float64)
        except (TypeError, ValueError) as e:
            raise ValueError(
                f"Trajectory file {path}: 'waypoints' is not a numeric 2-D array ({e})."
            ) from e

        if waypoints.size == 0:
            waypoints = waypoints.reshape(0, len(joint_names))
        elif waypoints.ndim != 2:
            raise ValueError(
                f"Trajectory file {path}: 'waypoints' must be 2-D "
                f"(N waypoints × M joints); got shape {waypoints.shape}."
            )
        elif waypoints.shape[1] != len(joint_names):
            raise ValueError(
                f"Trajectory file {path}: waypoint width {waypoints.shape[1]} "
                f"does not match joint_names length {len(joint_names)}."
            )

        metadata = data.get("metadata", {})
        if not isinstance(metadata, dict):
            raise ValueError(
                f"Trajectory file {path}: 'metadata' must be an object, "
                f"got {type(metadata).__name__}."
            )

        return cls(
            joint_names=joint_names,
            waypoints=waypoints,
            metadata=metadata,
        )

    def validate(self, backend: "RobotBackend") -> List[Dict]:
        """
        Check all waypoints against a backend's joint limits.

        Returns a list of violations. An empty list means the trajectory
        is fully within limits and safe to execute.

        Args:
            backend: A RobotBackend instance (WebotsBackend or RealRobotBackend)
                whose joint limits will be used for validation.

        Returns:
            List of dicts, each describing a violation:
            ``{"waypoint": int, "joint": str, "value": float,
              "min": float, "max": float}``

        Example:
            >>> traj = pib3.generate_trajectory("drawing.png")
            >>> with pib3.Robot(host="...") as robot:
            ...     violations = traj.validate(robot)
            ...     if violations:
            ...         print(f"{len(violations)} limit violations found!")
            ...         for v in violations[:5]:
            ...             print(f"  wp {v['waypoint']}: {v['joint']} = "
            ...                   f"{v['value']:.3f} rad, limits [{v['min']:.3f}, {v['max']:.3f}]")
            ...     else:
            ...         robot.run_trajectory(traj)
        """
        limits = backend._get_joint_limits()
        violations = []

        for j, name in enumerate(self.joint_names):
            joint_limits = limits.get(name)
            if joint_limits is None:
                continue
            min_rad = joint_limits.get("min")
            max_rad = joint_limits.get("max")
            if min_rad is None or max_rad is None:
                continue

            # Handle inverted ranges (e.g. Webots finger joints where min > max)
            lo, hi = min(min_rad, max_rad), max(min_rad, max_rad)

            col = self.waypoints[:, j]
            out_of_range = np.where((col < lo - 1e-6) | (col > hi + 1e-6))[0]
            for wp_idx in out_of_range:
                violations.append({
                    "waypoint": int(wp_idx),
                    "joint": name,
                    "value": float(col[wp_idx]),
                    "min": lo,
                    "max": hi,
                })

        return violations


def _get_urdf_path() -> Path:
    """Get path to bundled URDF file."""
    return Path(__file__).parent / "resources" / "pib_model.urdf"


def _get_resources_path() -> Path:
    """Get path to resources directory."""
    return Path(__file__).parent / "resources"


def _prepare_urdf_with_absolute_paths() -> Path:
    """
    Prepare URDF with absolute paths for mesh files.

    The bundled URDF uses relative paths (stl/filename.stl) which need to be
    resolved to absolute paths for roboticstoolbox to load the meshes correctly.

    Returns:
        Path to a temporary URDF file with absolute paths.
    """
    import tempfile
    import re

    urdf_path = _get_urdf_path()
    resources_path = _get_resources_path()

    # Read the URDF content
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()

    # Replace relative stl/ paths with absolute paths
    # Match patterns like: filename="stl/urdf_body.stl"
    def replace_mesh_path(match):
        relative_path = match.group(1)
        absolute_path = resources_path / relative_path
        return f'filename="{absolute_path}"'

    urdf_content = re.sub(
        r'filename="(stl/[^"]+)"',
        replace_mesh_path,
        urdf_content
    )

    # Write to a temporary file
    temp_dir = tempfile.gettempdir()
    temp_urdf = Path(temp_dir) / "pib_model_resolved.urdf"
    with open(temp_urdf, 'w') as f:
        f.write(urdf_content)

    return temp_urdf


def _patch_np_disp_if_missing() -> None:
    """Add np.disp back if it's been removed (numpy 2.x).

    roboticstoolbox still references np.disp in a few code paths. We do this
    on demand rather than unconditionally so the monkey-patch only happens
    when the environment actually needs it.
    """
    if not hasattr(np, "disp"):
        np.disp = lambda x: print(x)


def _rtb_call_with_np_disp(fn, *args, **kwargs):
    """Call a roboticstoolbox function, retrying with the np.disp patch if needed.

    Narrow dance: don't touch numpy unless the failure actually points at
    a missing np.disp. If any AttributeError without 'disp' in the message
    bubbles up, let it propagate.
    """
    try:
        return fn(*args, **kwargs)
    except AttributeError as e:
        if "disp" not in str(e):
            raise
        _patch_np_disp_if_missing()
        return fn(*args, **kwargs)


@lru_cache(maxsize=1)
def _load_robot():
    """Load robot model from bundled URDF (memoized — parsed once per process)."""

    def _attempt():
        import roboticstoolbox as rtb
        urdf_path = _get_urdf_path()
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
        resolved_urdf = _prepare_urdf_with_absolute_paths()
        return rtb.ERobot.URDF(str(resolved_urdf))

    try:
        return _attempt()
    except AttributeError as e:
        # numpy 2.x dropped np.disp; rtb still references it.
        if "disp" not in str(e):
            raise
        _patch_np_disp_if_missing()
        return _attempt()
    except ImportError:
        raise ImportError(
            "roboticstoolbox-python is required for trajectory generation. "
            "Install with: pip install roboticstoolbox-python"
        )


def _get_joint_limits(robot) -> List[Tuple[float, float]]:
    """Extract joint limits from robot model."""
    limits = []
    for link in robot.links:
        if link.isjoint and link.jindex is not None:
            qlim = link.qlim
            if qlim is not None and len(qlim) == 2:
                limits.append((qlim[0], qlim[1]))
            else:
                limits.append((-np.pi, np.pi))
    return limits


def _solve_ik_point(
    robot,
    target_pos: np.ndarray,
    q_init: np.ndarray,
    end_link: str,
    tool_offset,
    arm_joint_indices: List[int],
    joint_limits: List[Tuple[float, float]],
    config: IKConfig,
    target_orientation: Optional[np.ndarray] = None,
    orientation_weight: float = 0.5,
) -> Tuple[np.ndarray, bool]:
    """
    Solve IK using roboticstoolbox's ikine_LM() with expert DH parameters.

    Uses the calibrated DH model from dh_model.py with Levenberg-Marquardt
    optimization. Targets are transformed to shoulder-relative coordinates
    to ensure compatibility between URDF world frame and DH model.

    Only the arm joints are used as IK degrees of freedom.  Finger joints
    stay fixed — they are part of the kinematic chain (for forward
    kinematics to the TCP) but are never moved by the solver.

    Args:
        robot: URDF robot model (used for shoulder position).
        target_pos: Target position [x, y, z] in meters (world frame).
        q_init: Initial joint configuration.
        end_link: End effector link name.
        tool_offset: SE3 transform from end link to tool tip.
        arm_joint_indices: URDF indices of the arm joints to actuate.
        joint_limits: Joint limits (unused - DH model has built-in limits).
        config: IK configuration.
        target_orientation: Optional target orientation as rotation matrix (3x3).
        orientation_weight: Weight for orientation error (0-1).

    Returns:
        (q_solution, success)
    """
    from spatialmath import SE3

    from .dh_model import get_dh_robot, get_shoulder_position

    # Determine which arm based on joint indices
    arm = "left" if arm_joint_indices[0] < 10 else "right"

    # Get shoulder position to compute relative target
    shoulder_pos = get_shoulder_position(robot, arm)

    # Compute target relative to shoulder, convert to mm
    target_rel_mm = (np.array(target_pos) - shoulder_pos) * 1000

    # Get calibrated DH robot
    dh_robot = get_dh_robot(arm, tool_offset)

    # Build target SE3 and mask for ikine_LM
    use_orientation = target_orientation is not None and orientation_weight > 0
    if use_orientation:
        target_se3 = SE3.Rt(target_orientation, target_rel_mm)
        mask = [1, 1, 1, 1, 1, 1]
    else:
        # Position-only IK (most common for drawing)
        target_se3 = SE3(target_rel_mm)
        mask = [1, 1, 1, 0, 0, 0]

    # Extract initial guess from current arm configuration
    q0 = np.array([q_init[idx] for idx in arm_joint_indices])

    try:
        sol = _rtb_call_with_np_disp(
            dh_robot.ikine_LM,
            target_se3,
            q0=q0,
            mask=mask,
            tol=config.tolerance * 1000,  # convert m tolerance to mm
            ilimit=config.max_iterations,
        )

        if not sol.success:
            return q_init, False

        # Build full configuration with DH solution
        q_solution = q_init.copy()
        for i, idx in enumerate(arm_joint_indices):
            q_solution[idx] = sol.q[i]

        return q_solution, True

    except Exception as e:
        import logging
        logging.getLogger(__name__).warning(
            f"IK solver exception for target {target_pos}: {e}"
        )
        return q_init, False


def _interpolate_failed_points(
    q_traj: List[np.ndarray],
    success_flags: List[bool],
) -> np.ndarray:
    """Interpolate joint values for failed IK points."""
    n = len(q_traj)
    if n == 0:
        return np.array(q_traj)

    q_array = np.array(q_traj)

    i = 0
    while i < n:
        if not success_flags[i]:
            fail_start = i
            fail_end = i
            while fail_end < n and not success_flags[fail_end]:
                fail_end += 1

            q_before = q_array[fail_start - 1] if fail_start > 0 else None
            q_after = q_array[fail_end] if fail_end < n else None

            if q_before is not None and q_after is not None:
                for j in range(fail_start, fail_end):
                    t = (j - fail_start + 1) / (fail_end - fail_start + 1)
                    q_array[j] = (1 - t) * q_before + t * q_after
            elif q_before is not None:
                for j in range(fail_start, fail_end):
                    q_array[j] = q_before
            elif q_after is not None:
                for j in range(fail_start, fail_end):
                    q_array[j] = q_after

            i = fail_end
        else:
            i += 1

    return q_array


def _map_to_3d(
    u: float,
    v: float,
    paper: PaperConfig,
) -> Tuple[float, float, float]:
    """Map normalized (u, v) to 3D world coordinates."""
    scaled_size = paper.size * paper.drawing_scale
    offset = (paper.size - scaled_size) / 2.0

    x_paper = u * scaled_size + offset
    y_paper = v * scaled_size + offset

    world_x = paper.start_x + paper.size - y_paper
    world_y = (paper.center_y - paper.size / 2.0) + x_paper
    world_z = paper.height_z

    return world_x, world_y, world_z


def _interpolate_stroke_points(
    stroke: Stroke,
    density: float = 0.01,
) -> List[Tuple[float, float]]:
    """Interpolate stroke points for smooth motion."""
    points = stroke.points
    if len(points) < 2:
        return [(points[0, 0], points[0, 1])] if len(points) == 1 else []

    path = []
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]
        dist = np.linalg.norm(p2 - p1)
        n_steps = max(2, int(dist / density) + 1)
        for s in np.linspace(0, 1, n_steps):
            interp = p1 + s * (p2 - p1)
            path.append((float(interp[0]), float(interp[1])))

    return path


def _set_initial_arm_pose(
    q: np.ndarray,
    arm: str = "left",
    grip_style: str = "index_finger",
) -> np.ndarray:
    """Set initial arm pose for drawing.

    Args:
        q: Joint configuration array to modify.
        arm: Which arm to use ("left" or "right").
        grip_style: Drawing grip style ("index_finger" or "pencil_grip").

    Returns:
        Modified joint configuration array.
    """
    arm_joints = LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS

    # Neutral mid-range pose with all joints at ~1.0 rad for maximum flexibility
    q[arm_joints['shoulder_vertical']] = 1.0
    q[arm_joints['shoulder_horizontal']] = 0.0
    q[arm_joints['upper_arm']] = 0.0
    q[arm_joints['elbow']] = 1.0
    q[arm_joints['wrist']] = 1.0

    # Forearm rotation sets wrist orientation:
    # - index_finger: horizontal wrist (0.0)
    # - pencil_grip: vertical wrist (π/2)
    if grip_style == "pencil_grip":
        q[arm_joints['forearm']] = np.pi / 2
    else:
        q[arm_joints['forearm']] = 0.0

    if grip_style == "pencil_grip":
        # Pencil grip: all fingers curled in power grip (thumb over fingers)
        if arm == "left":
            # Thumb wrapped over
            q[8] = 1.0   # thumb_left_opposition
            q[9] = 1.0   # thumb_left_stretch (proximal)
            q[10] = 1.0  # thumb_left_stretch (distal)
            # All fingers curled
            for idx in [11, 12, 13, 14, 15, 16, 17, 18]:
                q[idx] = 1.0
        else:
            # Right hand
            q[25] = 1.0  # thumb_right_opposition
            q[26] = 1.0  # thumb_right_stretch (proximal)
            q[27] = 1.0  # thumb_right_stretch (distal)
            for idx in [28, 29, 30, 31, 32, 33, 34, 35]:
                q[idx] = 1.0
    else:
        # Index finger drawing: extend index, curl others
        if arm == "left":
            # Thumb partially bent
            q[8] = 0.5
            q[9] = 1.0
            q[10] = 1.0
            # Middle, ring, pinky curled
            for idx in [13, 14, 15, 16, 17, 18]:
                q[idx] = 1.0
        else:
            # Right hand fingers
            q[25] = 0.5
            q[26] = 1.0
            q[27] = 1.0
            for idx in [30, 31, 32, 33, 34, 35]:
                q[idx] = 1.0

    return q


def sketch_to_trajectory(
    sketch: Sketch,
    config: Optional[TrajectoryConfig] = None,
    progress_callback: Optional[Callable[[int, int, bool], None]] = None,
    initial_q: Optional[Union[np.ndarray, Dict[str, float], "Trajectory"]] = None,
) -> Trajectory:
    """
    Convert a Sketch to a robot Trajectory using inverse kinematics.

    Args:
        sketch: Sketch object containing strokes to draw.
        config: Trajectory configuration. Uses defaults if None.
        progress_callback: Optional callback(current_point, total_points, success).
        initial_q: Initial joint configuration to start IK solving from.
            Can be one of:
            - numpy array of shape (n_joints,) with joint positions in radians
            - dict mapping joint names to positions in radians
            - Trajectory object (uses last waypoint)
            - None (default): use fixed initial pose for drawing

            This allows sequential trajectory execution where each new
            trajectory starts from the end position of the previous one.

    Returns:
        Trajectory object with joint positions for each waypoint.

    Raises:
        ImportError: If roboticstoolbox is not installed.
        RuntimeError: If no IK solutions are found.

    Example:
        >>> from pib3 import image_to_sketch, sketch_to_trajectory
        >>> sketch = image_to_sketch("drawing.png")
        >>> trajectory = sketch_to_trajectory(sketch)
        >>> trajectory.to_json("output.json")

        >>> # Sequential trajectories
        >>> traj1 = sketch_to_trajectory(sketch1)
        >>> traj2 = sketch_to_trajectory(sketch2, initial_q=traj1)  # Start from traj1 end
    """
    import spatialmath as sm

    if config is None:
        config = TrajectoryConfig()

    # Load robot
    robot = _load_robot()
    joint_limits = _get_joint_limits(robot)

    # Determine arm configuration
    arm = config.ik.arm
    grip_style = config.ik.grip_style

    if arm == "left":
        arm_joints = LEFT_ARM_JOINTS
        tcp_link_finger = LEFT_TCP_LINK
        tcp_link_pencil = LEFT_TCP_LINK_PENCIL
    else:
        arm_joints = RIGHT_ARM_JOINTS
        tcp_link_finger = RIGHT_TCP_LINK
        tcp_link_pencil = RIGHT_TCP_LINK_PENCIL

    arm_joint_indices = list(arm_joints.values())

    # Configure TCP and tool offset based on grip style.
    # Only the 6 arm joints are IK degrees of freedom; finger joints stay
    # fixed at the pose set by _set_initial_arm_pose.  Forward kinematics
    # still runs through the finger chain to reach the TCP.
    if grip_style == "pencil_grip":
        tcp_link = tcp_link_pencil
        tool_offset = sm.SE3(0.04, -0.06, 0)  # 40mm forward, 60mm toward pinky
    else:
        # Index finger drawing: use finger tip as TCP
        tcp_link = tcp_link_finger
        tool_offset = sm.SE3(0, 0.027, 0)  # 27mm in Y (finger tip direction)

    # Initialize configuration
    if initial_q is not None:
        # Use provided initial configuration
        if isinstance(initial_q, Trajectory):
            # Expand trajectory back to full robot configuration by mapping
            # each motor name to all URDF indices that share that name
            # (e.g. 'index_left_stretch' drives URDF indices 11 and 12).
            q_current = np.zeros(robot.n)
            q_current = _set_initial_arm_pose(q_current, arm, grip_style)
            last_wp = initial_q.waypoints[-1]
            motor_to_indices: Dict[str, List[int]] = {}
            for idx, name in URDF_TO_MOTOR_NAME.items():
                motor_to_indices.setdefault(name, []).append(idx)
            for j, name in enumerate(initial_q.joint_names):
                if j >= len(last_wp):
                    break
                for urdf_idx in motor_to_indices.get(name, []):
                    q_current[urdf_idx] = last_wp[j]
        elif isinstance(initial_q, dict):
            # Convert dict of joint names to array.
            # Build a mapping from motor name -> list of URDF indices, because
            # coupled finger joints share the same motor name for both proximal
            # and distal URDF joints (e.g. indices 11 & 12 are both
            # "index_left_stretch") and both must be set to the same value.
            q_current = np.zeros(robot.n)
            q_current = _set_initial_arm_pose(q_current, arm, grip_style)  # Start with defaults
            joint_name_to_indices: Dict[str, List[int]] = {}
            for idx, name in URDF_TO_MOTOR_NAME.items():
                joint_name_to_indices.setdefault(name, []).append(idx)
            for name, value in initial_q.items():
                if name in joint_name_to_indices:
                    for idx in joint_name_to_indices[name]:
                        q_current[idx] = value
        else:
            # Assume numpy array (full robot configuration)
            q_current = np.asarray(initial_q, dtype=np.float64).copy()
            if q_current.shape[0] != robot.n:
                raise ValueError(
                    f"initial_q array has {q_current.shape[0]} elements, "
                    f"expected {robot.n}"
                )
    else:
        # Default: start from fixed initial pose
        q_current = np.zeros(robot.n)
        q_current = _set_initial_arm_pose(q_current, arm, grip_style)
    robot.q = q_current

    # Get initial TCP position
    T_start = robot.fkine(q_current, end=tcp_link)
    if isinstance(T_start, np.ndarray):
        T_start = sm.SE3(T_start)
    T_start = T_start * tool_offset
    start_pos = T_start.t

    # Work on a copy of the paper config to avoid mutating the caller's config
    paper = deepcopy(config.paper)
    if paper.center_y is None:
        # Auto-place paper in front of the active arm (negative y is left).
        paper.center_y = -0.34 if arm == "left" else 0.34
    if paper.start_x is None:
        # Auto-place paper so the active arm's TCP sits at its near edge.
        paper.start_x = float(start_pos[0] - paper.size / 2.0)

    # Generate 3D trajectory points
    trajectory_3d = []
    for stroke in sketch.strokes:
        dense_points = _interpolate_stroke_points(stroke, config.point_density)
        if not dense_points:
            continue

        # Lift before stroke
        u0, v0 = dense_points[0]
        x, y, z = _map_to_3d(u0, v0, paper)
        trajectory_3d.append((x, y, z + paper.lift_height, True))

        # Draw stroke
        for u, v in dense_points:
            x, y, z = _map_to_3d(u, v, paper)
            trajectory_3d.append((x, y, z, False))

        # Lift after stroke
        u_end, v_end = dense_points[-1]
        x, y, z = _map_to_3d(u_end, v_end, paper)
        trajectory_3d.append((x, y, z + paper.lift_height, True))

    # Solve IK
    q_traj = []
    success_flags = []
    success_count = 0
    fail_count = 0
    total_points = len(trajectory_3d)

    # Orientation constraints
    # For drawing on paper, position accuracy is most important
    # Both grip styles use position-only IK for maximum robustness
    target_orientation = None
    orientation_weight = 0.0

    for i, (x, y, z, is_lift) in enumerate(trajectory_3d):
        q_sol, success = _solve_ik_point(
            robot,
            np.array([x, y, z]),
            q_current,
            tcp_link,
            tool_offset,
            arm_joint_indices,
            joint_limits,
            config.ik,
            target_orientation=target_orientation,
            orientation_weight=orientation_weight,
        )

        if success:
            q_current = q_sol
            success_count += 1
        else:
            fail_count += 1

        q_traj.append(q_current.copy())
        success_flags.append(success)

        if progress_callback:
            progress_callback(i + 1, total_points, success)

    if success_count == 0:
        raise RuntimeError("No successful IK solutions found. Check paper position and robot reach.")

    # Interpolate failed points
    if fail_count > 0:
        q_array = _interpolate_failed_points(q_traj, success_flags)
    else:
        q_array = np.array(q_traj)

    # Filter trajectory to only include the drawing arm + hand joints and
    # deduplicate coupled finger joints. Real-robot fingers have a single
    # motor driving both proximal and distal URDF joints via a mechanical
    # linkage; the Webots digital twin mirrors this coupling in
    # FINGER_PROXIMAL_MOTORS. IK never actuates finger joints (they stay at
    # the pose set by _set_initial_arm_pose), so the proximal and distal
    # columns are guaranteed to be identical — we emit only one column per
    # unique motor name.
    if arm == "left":
        candidate_indices = list(range(2, 19))   # left arm (2-7) + hand (8-18)
    else:
        candidate_indices = list(range(19, 36))  # right arm (19-24) + hand (25-35)

    drawing_joint_indices: List[int] = []
    joint_names: List[str] = []
    seen_motors = set()
    for urdf_idx in candidate_indices:
        motor_name = URDF_TO_MOTOR_NAME.get(urdf_idx, f"joint_{urdf_idx}")
        if motor_name in seen_motors:
            continue
        seen_motors.add(motor_name)
        drawing_joint_indices.append(urdf_idx)
        joint_names.append(motor_name)

    q_array = q_array[:, drawing_joint_indices]

    # Create trajectory
    trajectory = Trajectory(
        joint_names=joint_names,
        waypoints=q_array,
        metadata={
            "source": "pib3",
            "robot_model": "pib",
            "tcp_link": tcp_link,
            "arm": arm,
            "grip_style": grip_style,
            "total_points": len(q_array),
            "success_count": success_count,
            "fail_count": fail_count,
            "success_rate": success_count / total_points if total_points > 0 else 0,
            "paper_config": {
                "start_x": paper.start_x,
                "size": paper.size,
                "center_y": paper.center_y,
                "height_z": paper.height_z,
                "drawing_scale": paper.drawing_scale,
            },
        },
    )

    return trajectory
