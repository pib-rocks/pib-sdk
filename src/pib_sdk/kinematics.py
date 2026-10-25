"""Forward and inverse kinematics for pib, powered by Pinocchio.

Joint angles in **radians** are the canonical unit throughout this module;
use the ``*_deg`` wrappers when you have degrees. Positions remain in
**millimetres**.

Public entry points:

* :func:`fk` / :func:`ik` -- one-call forward and inverse kinematics for an arm.
* :func:`get_hand_position_xyz` -- hand-tip XYZ from commanded arm joint positions.
* :class:`ArmKinematics` / :class:`HeadKinematics` -- reusable solvers holding a
  single Pinocchio buffer.
* :func:`camera_pose` -- pose of the head camera for a given pan/tilt.
* :class:`FK` / :class:`IK` -- drop-in classes matching the previous SDK.

Each angle-taking function has a ``*_deg`` sibling that converts degrees to
radians (inputs) or radians to degrees (outputs) and delegates in one line.

Example (same pose, radians vs degrees)::

    >>> import numpy as np
    >>> from pib_sdk.kinematics import fk, fk_deg
    >>> q_rad = [0.0, np.deg2rad(45), 0.0, 0.0, np.deg2rad(90), 0.0]
    >>> q_deg = [0, 45, 0, 0, 90, 0]
    >>> np.allclose(fk("right", q_rad).translation, fk_deg("right", q_deg).translation)
    True

Layer conventions
-----------------
+------------------+---------------------------+
| Layer            | Angles / positions        |
+==================+===========================+
| ``pib_sdk``      | radians; positions mm     |
| ROS              | radians                   |
| pib API          | centidegrees              |
| STS driver       | ticks                     |
+------------------+---------------------------+

Orientation is given as roll/pitch/yaw in radians (or degrees via ``*_deg``
helpers), applied as ``R = Rz(yaw) . Ry(pitch) . Rx(roll)`` (ZYX, matching
the previous Robotics Toolbox implementation).

Poses are :class:`pinocchio.SE3` values in pib's base frame; read
``pose.translation`` (mm) and ``pose.rotation`` (3x3), or use
:func:`pose_to_xyz_rpy` for ``(xyz, rpy_rad)``.
"""

from __future__ import annotations

import time
from collections.abc import Iterable, Sequence

import numpy as np
import pinocchio as pin

from pib_sdk.robot_model import (
    ArmSide,
    ChainModel,
    get_arm_model,
    get_head_model,
)

# Error-twist layout used throughout: [x, y, z, rot_x, rot_y, rot_z] in a
# world-aligned frame. The first three entries are position, the last three
# orientation, matching the mask ordering of the previous implementation.
_POSITION_ONLY_MASK: tuple[int, ...] = (1, 1, 1, 0, 0, 0)
_FULL_POSE_MASK: tuple[int, ...] = (1, 1, 1, 1, 1, 1)

# The pose error mixes position (millimetres) and orientation (radians). In the
# least-squares step the orientation rows are scaled by this characteristic
# length (roughly the arm's reach in mm) so that neither position nor
# orientation dominates the solve.
_ORIENTATION_WEIGHT: float = 250.0

# Model-frame ``+Y`` points behind the chest (``-Y`` is forward out of the body).
_BEHIND_BODY_Y_THRESHOLD_MM: float = 600.0
# Margin on the sampled maximum reach when classifying workspace failures.
_REACH_MARGIN: float = 0.05


class InverseKinematicsError(ValueError):
    """IK could not produce a pose-accurate, limit-respecting joint vector."""


class InverseKinematicsWorkspaceError(InverseKinematicsError):
    """The target lies beyond the arm's reachable workspace (distance from base)."""


class InverseKinematicsBehindBodyError(InverseKinematicsError):
    """The target lies behind the torso (+Y in the model base frame)."""


class InverseKinematicsJointLimitError(InverseKinematicsError):
    """The pose is reachable only with joints outside mechanical limits."""


class InverseKinematicsConvergenceError(InverseKinematicsError):
    """The iterative solver did not drive the pose error below ``tolerance``."""


def _degrees_to_radians(values: Sequence[float]) -> np.ndarray:
    return np.deg2rad(np.asarray(values, dtype=float))


def _radians_to_degrees(values: Sequence[float]) -> np.ndarray:
    return np.rad2deg(np.asarray(values, dtype=float))


def pose_from_xyz_rpy(xyz: Sequence[float], rpy_rad: Sequence[float] | None = None) -> pin.SE3:
    """Build an :class:`pinocchio.SE3` from a position (mm) and optional RPY (rad)."""
    translation = np.asarray(xyz, dtype=float)
    if translation.shape != (3,):
        raise ValueError("xyz must have exactly 3 elements")
    if rpy_rad is None:
        return pin.SE3(np.eye(3), translation)
    roll, pitch, yaw = np.asarray(rpy_rad, dtype=float)
    rotation = (
        pin.utils.rotate("z", yaw) @ pin.utils.rotate("y", pitch) @ pin.utils.rotate("x", roll)
    )
    return pin.SE3(rotation, translation)


def pose_from_xyz_rpy_deg(xyz: Sequence[float], rpy_deg: Sequence[float] | None = None) -> pin.SE3:
    """Build an SE3 from a position (mm) and optional RPY (deg)."""
    return pose_from_xyz_rpy(xyz, _degrees_to_radians(rpy_deg) if rpy_deg is not None else None)


def pose_to_xyz_rpy(pose: pin.SE3) -> tuple[np.ndarray, np.ndarray]:
    """Decompose an SE3 pose into ``(xyz_mm, rpy_rad)`` in the ZYX convention."""
    rotation = pose.rotation
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    pitch = np.arctan2(-rotation[2, 0], np.hypot(rotation[0, 0], rotation[1, 0]))
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    return pose.translation.copy(), np.array([roll, pitch, yaw], dtype=float)


def pose_to_xyz_rpy_deg(pose: pin.SE3) -> tuple[np.ndarray, np.ndarray]:
    """Decompose an SE3 pose into ``(xyz_mm, rpy_deg)`` in the ZYX convention."""
    xyz, rpy_rad = pose_to_xyz_rpy(pose)
    return xyz, _radians_to_degrees(rpy_rad)


# Fixed rotation from pib's model base frame to REP-103 (+90 deg about Z).
_MODEL_TO_REP103: pin.SE3 = pin.SE3(pin.utils.rotate("z", np.pi / 2.0), np.zeros(3))
_REP103_TO_MODEL: pin.SE3 = pin.SE3(pin.utils.rotate("z", -np.pi / 2.0), np.zeros(3))


def to_rep103(value: pin.SE3 | Sequence[float]) -> pin.SE3 | np.ndarray:
    """Convert a position or pose from pib's model frame to REP-103.

    The model frame uses ``+Z`` up, ``-Y`` forward (out of the chest), and ``+X``
    as the robot's own left. REP-103 is that frame rotated by ``-90`` degrees
    about ``Z``, so model → REP-103 applies ``+90`` degrees about ``Z``:

    ``X_rep = -Y_model``, ``Y_rep = +X_model``, ``Z_rep = Z_model``.

    Parameters
    ----------
    value:
        Either a 3-vector position in millimetres or a :class:`pinocchio.SE3`
        pose (translation in mm, rotation in the model base frame). In both
        cases translation and rotation are converted.

    Returns
    -------
    np.ndarray or pin.SE3
        The same kind of value expressed in the REP-103 base frame.

    Example
    -------
    A point on the left palm at zero configuration is
    ``(626.2, 27.5, 994.5)`` mm in the model frame::

        >>> import numpy as np
        >>> from pib_sdk.kinematics import pose_from_xyz_rpy, to_rep103
        >>> model_pose = pose_from_xyz_rpy([626.2, 27.5, 994.5])
        >>> rep_pose = to_rep103(model_pose)
        >>> np.round(rep_pose.translation, 1)
        array([-27.5, 626.2, 994.5])

    See :func:`from_rep103` for the inverse map.
    """
    if isinstance(value, pin.SE3):
        return _MODEL_TO_REP103 * value
    translation = np.asarray(value, dtype=float)
    if translation.shape != (3,):
        raise ValueError("position must have exactly 3 elements")
    return (_MODEL_TO_REP103.rotation @ translation).copy()


def from_rep103(value: pin.SE3 | Sequence[float]) -> pin.SE3 | np.ndarray:
    """Convert a position or pose from REP-103 back to pib's model frame.

    This is the inverse of :func:`to_rep103` (rotation by ``-90`` degrees about
    ``Z``: ``X_model = +Y_rep``, ``Y_model = -X_rep``, ``Z_model = Z_rep``).

    Parameters and return type match :func:`to_rep103`, but directions are
    reversed.
    """
    if isinstance(value, pin.SE3):
        return _REP103_TO_MODEL * value
    translation = np.asarray(value, dtype=float)
    if translation.shape != (3,):
        raise ValueError("position must have exactly 3 elements")
    return (_REP103_TO_MODEL.rotation @ translation).copy()


class ChainKinematics:
    """Forward and inverse kinematics for one kinematic chain.

    Reuse an instance to avoid rebuilding the model and its solver buffer.
    Each instance owns one mutable Pinocchio ``data`` object; do not call
    :meth:`inverse` or :meth:`forward` on the same instance from multiple
    threads concurrently. Sequential solves on one instance are independent
    (each call finishes before the next mutates ``data``).

    See :class:`ArmKinematics` and :class:`HeadKinematics` for ready-made
    chains, or construct directly from any :class:`ChainModel`.
    """

    def __init__(self, chain: ChainModel):
        self._chain = chain
        self._data: pin.Data = chain.create_data()
        self._joint_lower_limits: np.ndarray = chain.joint_lower_limits
        self._joint_upper_limits: np.ndarray = chain.joint_upper_limits
        self._max_reach_mm: float | None = None
        self.last_inverse_stats: dict[str, float | int | bool] | None = None

    @property
    def degrees_of_freedom(self) -> int:
        return self._chain.degrees_of_freedom

    @property
    def joint_names(self) -> list[str]:
        """URDF joint names in joint-vector order."""
        return list(self._chain.joint_names)

    @property
    def motor_names(self) -> list[str]:
        """pib motor names in joint-vector order (see ``pib_sdk.control``)."""
        return list(self._chain.motor_names)

    @property
    def joint_limits_rad(self) -> tuple[np.ndarray, np.ndarray]:
        """Joint limits as a ``(lower_rad, upper_rad)`` pair of arrays."""
        return self._joint_lower_limits.copy(), self._joint_upper_limits.copy()

    @property
    def joint_limits_deg(self) -> tuple[np.ndarray, np.ndarray]:
        """Joint limits as a ``(lower_deg, upper_deg)`` pair of arrays."""
        return self._chain.joint_limits_deg

    def named_configuration(self, name: str) -> np.ndarray:
        """Return a named joint configuration (``"zero"``, ``"rest"`` ...) in radians."""
        return self._chain.named_configuration_deg(name) * (np.pi / 180.0)

    def named_configuration_deg(self, name: str) -> np.ndarray:
        """Return a named joint configuration (``"zero"``, ``"rest"`` ...) in degrees."""
        return self._chain.named_configuration_deg(name)

    # -- forward kinematics -------------------------------------------------- #

    def forward(self, joint_angles_rad: Iterable[float]) -> pin.SE3:
        """Return the tip pose (translation in mm) for joint angles in radians."""
        joint_angles_rad = np.asarray(list(joint_angles_rad), dtype=float)
        if joint_angles_rad.size != self.degrees_of_freedom:
            raise ValueError(
                f"Expected {self.degrees_of_freedom} joint values, got {joint_angles_rad.size}"
            )
        return self._tip_pose(joint_angles_rad)

    def forward_deg(self, joint_angles_deg: Iterable[float]) -> pin.SE3:
        """Return the tip pose for joint angles in degrees."""
        return self.forward(_degrees_to_radians(list(joint_angles_deg)))

    # -- inverse kinematics -------------------------------------------------- #

    def inverse(
        self,
        xyz: Sequence[float],
        rpy_rad: Sequence[float] | None = None,
        initial_guess_rad: Iterable[float] | None = None,
        *,
        rpy_deg: Sequence[float] | None = None,
        initial_guess_deg: Iterable[float] | None = None,
        tolerance: float = 1e-4,
        max_iterations: int = 200,
        mask: Sequence[float] | None = None,
        restarts: int = 50,
        respect_limits: bool = True,
    ) -> np.ndarray:
        """Return joint angles (radians) that reach the requested pose.

        Parameters
        ----------
        xyz:
            Target position in millimetres.
        rpy_rad:
            Target orientation (roll, pitch, yaw in radians). If ``None``,
            orientation is ignored (position-only IK).
        rpy_deg:
            Same as ``rpy_rad`` but in degrees; returns joint angles in **degrees**
            (legacy callers). Do not pass both ``rpy_rad`` and ``rpy_deg``.
        initial_guess_rad:
            Starting joint configuration in radians (defaults to all zeros).
        initial_guess_deg:
            Starting guess in degrees; only with ``rpy_deg`` / degree return path.
        tolerance:
            Convergence threshold on the (masked, weighted) pose-error norm;
            position components are in millimetres.
        max_iterations:
            Maximum solver iterations per attempt.
        mask:
            Optional 6-element mask over ``[x, y, z, rot_x, rot_y, rot_z]``
            selecting which world-frame pose-error components to drive to zero.
            Overrides the automatic position-only / full-pose choice.
        restarts:
            Number of random in-limit restarts attempted if the first solve
            does not converge. The initial guess is always tried first.
        respect_limits:
            When ``True`` (default), only solutions within the mechanical joint
            limits are accepted, so the result is always usable by
            :meth:`pib_sdk.control.Write.move`. Set to ``False`` for an
            unconstrained solution.

        Raises
        ------
        ValueError
            If ``mask`` is malformed or radian/degree arguments are mixed.
        InverseKinematicsWorkspaceError
            Target farther from the base than the sampled maximum reach.
        InverseKinematicsBehindBodyError
            Target with model-frame ``+Y`` behind the torso (see module constants).
        InverseKinematicsJointLimitError
            With ``respect_limits=True``, only out-of-limit solutions converged.
        InverseKinematicsConvergenceError
            No attempt met ``tolerance`` within ``max_iterations`` (and restarts).

        After each call, :attr:`last_inverse_stats` holds ``iterations``,
        ``restarts_used``, ``pose_error_norm`` (mm-scale position part when
        position-only), and ``elapsed_ms`` for the winning attempt.
        """
        if rpy_deg is not None or initial_guess_deg is not None:
            if rpy_rad is not None or initial_guess_rad is not None:
                raise ValueError("pass either radian or degree angle arguments, not both")
            return self.inverse_deg(
                xyz=xyz,
                rpy_deg=rpy_deg,
                initial_guess_deg=initial_guess_deg,
                tolerance=tolerance,
                max_iterations=max_iterations,
                mask=mask,
                restarts=restarts,
                respect_limits=respect_limits,
            )
        target_pose = pose_from_xyz_rpy(xyz, rpy_rad)
        error_mask = self._resolve_mask(rpy_rad, mask)
        if initial_guess_rad is None:
            initial_guess_rad = np.zeros(self.degrees_of_freedom)
        else:
            initial_guess_rad = np.asarray(list(initial_guess_rad), dtype=float)

        solve_started = time.perf_counter()
        solution_rad, converged, diagnostics = self._solve_with_restarts(
            target_pose,
            error_mask,
            initial_guess_rad,
            tolerance,
            max_iterations,
            restarts,
            respect_limits,
        )
        elapsed_ms = (time.perf_counter() - solve_started) * 1000.0
        self.last_inverse_stats = {
            "iterations": diagnostics["iterations"],
            "restarts_used": diagnostics["restarts_used"],
            "pose_error_norm": diagnostics["pose_error_norm"],
            "elapsed_ms": elapsed_ms,
            "converged": converged,
        }
        if not converged:
            self._raise_inverse_failure(
                target_pose.translation,
                respect_limits=respect_limits,
                diagnostics=diagnostics,
            )
        if respect_limits:
            solution_rad = np.clip(solution_rad, self._joint_lower_limits, self._joint_upper_limits)
        return solution_rad.copy()

    def inverse_deg(
        self,
        xyz: Sequence[float],
        rpy_deg: Sequence[float] | None = None,
        initial_guess_deg: Iterable[float] | None = None,
        tolerance: float = 1e-4,
        max_iterations: int = 200,
        mask: Sequence[float] | None = None,
        restarts: int = 50,
        respect_limits: bool = True,
    ) -> np.ndarray:
        """Return joint angles (degrees) that reach the requested pose."""
        rpy_rad = _degrees_to_radians(rpy_deg) if rpy_deg is not None else None
        guess_rad = (
            _degrees_to_radians(list(initial_guess_deg)) if initial_guess_deg is not None else None
        )
        return _radians_to_degrees(
            self.inverse(
                xyz=xyz,
                rpy_rad=rpy_rad,
                initial_guess_rad=guess_rad,
                tolerance=tolerance,
                max_iterations=max_iterations,
                mask=mask,
                restarts=restarts,
                respect_limits=respect_limits,
            )
        )

    # -- internal helpers ---------------------------------------------------- #

    def _tip_pose(self, joint_angles_rad: np.ndarray) -> pin.SE3:
        model, data = self._chain.model, self._data
        frame_id = self._chain.tip_frame_id
        pin.forwardKinematics(model, data, joint_angles_rad)
        pin.updateFramePlacement(model, data, frame_id)
        return data.oMf[frame_id].copy()

    def _resolve_mask(
        self, rpy_rad: Sequence[float] | None, mask: Sequence[float] | None
    ) -> np.ndarray:
        if mask is None:
            chosen = _POSITION_ONLY_MASK if rpy_rad is None else _FULL_POSE_MASK
        else:
            if len(mask) != 6:
                raise ValueError("mask must have exactly 6 elements")
            chosen = tuple(mask)
        return np.asarray(chosen, dtype=bool)

    def _pose_error(
        self, joint_angles_rad: np.ndarray, target: pin.SE3, mask: np.ndarray
    ) -> np.ndarray:
        current = self._tip_pose(joint_angles_rad)
        error_twist = np.concatenate(
            [
                target.translation - current.translation,
                pin.log3(target.rotation @ current.rotation.T),
            ]
        )
        return error_twist[mask]

    def _solve_once(
        self,
        target: pin.SE3,
        mask: np.ndarray,
        seed_rad: np.ndarray,
        tolerance: float,
        max_iterations: int,
    ) -> tuple[np.ndarray, bool, float, int]:
        """Levenberg-Marquardt closed-loop IK from a single seed.

        Each step solves ``(J'WJ + lambda*I) dq = J'W e``, where ``W`` scales
        the orientation rows to be comparable to the position rows (see
        ``_ORIENTATION_WEIGHT``). The ``lambda*I`` term penalises joint
        velocity, which keeps the step well-conditioned and holds joints with
        little influence on the task (such as the wrist roll under a
        position-only target) close to the seed instead of letting them drift.
        ``lambda`` adapts Levenberg-Marquardt style: it shrinks on an accepted
        step and grows on a rejected one.
        """
        model, frame_id = self._chain.model, self._chain.tip_frame_id
        identity = np.eye(self.degrees_of_freedom)
        weights = np.array(
            [
                1.0,
                1.0,
                1.0,
                _ORIENTATION_WEIGHT,
                _ORIENTATION_WEIGHT,
                _ORIENTATION_WEIGHT,
            ]
        )[mask]
        joint_angles_rad = np.array(seed_rad, dtype=float)
        damping = 1e-2
        iterations = 0

        for _ in range(max_iterations):
            iterations += 1
            error = self._pose_error(joint_angles_rad, target, mask)
            error_norm = float(np.linalg.norm(error))
            if error_norm < tolerance:
                return joint_angles_rad, True, error_norm, iterations

            jacobian = pin.computeFrameJacobian(
                model, self._data, joint_angles_rad, frame_id, pin.LOCAL_WORLD_ALIGNED
            )[mask, :]
            weighted_jacobian = weights[:, None] * jacobian
            weighted_error = weights * error
            step = np.linalg.solve(
                weighted_jacobian.T @ weighted_jacobian + damping * identity,
                weighted_jacobian.T @ weighted_error,
            )
            candidate = pin.integrate(model, joint_angles_rad, step)

            candidate_error = np.linalg.norm(self._pose_error(candidate, target, mask))
            if candidate_error < error_norm:
                joint_angles_rad = candidate
                damping = max(damping * 0.5, 1e-6)
            else:
                damping = min(damping * 4.0, 1e6)

        final_error = float(np.linalg.norm(self._pose_error(joint_angles_rad, target, mask)))
        return joint_angles_rad, final_error < tolerance, final_error, iterations

    def _solve_with_restarts(
        self,
        target: pin.SE3,
        mask: np.ndarray,
        initial_guess_rad: np.ndarray,
        tolerance: float,
        max_iterations: int,
        restarts: int,
        respect_limits: bool,
    ) -> tuple[np.ndarray, bool, dict[str, float | int | bool]]:
        def is_acceptable(joint_angles_rad: np.ndarray, converged: bool) -> bool:
            if not converged:
                return False
            return (not respect_limits) or self._within_limits(joint_angles_rad)

        out_of_limit_converged = False
        restarts_used = 0

        solution, converged, best_error, iterations = self._solve_once(
            target, mask, initial_guess_rad, tolerance, max_iterations
        )
        if converged and respect_limits and not self._within_limits(solution):
            out_of_limit_converged = True
        if is_acceptable(solution, converged):
            return (
                solution,
                True,
                {
                    "iterations": iterations,
                    "restarts_used": 0,
                    "pose_error_norm": best_error,
                    "out_of_limit_converged": out_of_limit_converged,
                },
            )
        best_solution = solution
        best_iterations = iterations

        random_generator = np.random.default_rng(seed=0)
        lower, upper = self._joint_lower_limits, self._joint_upper_limits
        for attempt in range(restarts):
            restarts_used = attempt + 1
            random_seed = lower + (upper - lower) * random_generator.random(self.degrees_of_freedom)
            solution, converged, error, iterations = self._solve_once(
                target, mask, random_seed, tolerance, max_iterations
            )
            if converged and respect_limits and not self._within_limits(solution):
                out_of_limit_converged = True
            if is_acceptable(solution, converged):
                return (
                    solution,
                    True,
                    {
                        "iterations": iterations,
                        "restarts_used": restarts_used,
                        "pose_error_norm": error,
                        "out_of_limit_converged": out_of_limit_converged,
                    },
                )
            if error < best_error:
                best_solution, best_error, best_iterations = solution, error, iterations
        return (
            best_solution,
            False,
            {
                "iterations": best_iterations,
                "restarts_used": restarts_used,
                "pose_error_norm": best_error,
                "out_of_limit_converged": out_of_limit_converged,
            },
        )

    def _max_reach_from_base_mm(self) -> float:
        if self._max_reach_mm is not None:
            return self._max_reach_mm
        lower, upper = self._joint_lower_limits, self._joint_upper_limits
        rng = np.random.default_rng(seed=17)
        max_distance = 0.0
        for _ in range(2000):
            joint_angles_rad = lower + (upper - lower) * rng.random(self.degrees_of_freedom)
            distance = float(np.linalg.norm(self._tip_pose(joint_angles_rad).translation))
            max_distance = max(max_distance, distance)
        self._max_reach_mm = max_distance
        return max_distance

    def _raise_inverse_failure(
        self,
        target_xyz: np.ndarray,
        *,
        respect_limits: bool,
        diagnostics: dict[str, float | int | bool],
    ) -> None:
        y_mm = float(target_xyz[1])
        if y_mm > _BEHIND_BODY_Y_THRESHOLD_MM:
            raise InverseKinematicsBehindBodyError(
                f"Target Y={y_mm:.1f} mm is behind the body "
                f"(model +Y; threshold {_BEHIND_BODY_Y_THRESHOLD_MM:.0f} mm)"
            )
        reach = self._max_reach_from_base_mm()
        distance = float(np.linalg.norm(target_xyz))
        if distance > reach * (1.0 + _REACH_MARGIN):
            raise InverseKinematicsWorkspaceError(
                f"Target is {distance:.1f} mm from the base, beyond sampled reach "
                f"{reach:.1f} mm (+{_REACH_MARGIN * 100:.0f}% margin)"
            )
        if respect_limits and diagnostics.get("out_of_limit_converged"):
            raise InverseKinematicsJointLimitError(
                "Target pose converged only outside mechanical joint limits"
            )
        within = " within joint limits" if respect_limits else ""
        raise InverseKinematicsConvergenceError(
            f"Inverse kinematics failed to converge for the requested pose{within}"
        )

    def _within_limits(self, joint_angles_rad: np.ndarray, tolerance: float = 1e-6) -> bool:
        return bool(
            np.all(joint_angles_rad >= self._joint_lower_limits - tolerance)
            and np.all(joint_angles_rad <= self._joint_upper_limits + tolerance)
        )


class ArmKinematics(ChainKinematics):
    """Forward and inverse kinematics for one of pib's arms.

    Example
    -------
    >>> import numpy as np
    >>> arm = ArmKinematics("right")
    >>> pose = arm.forward([0, np.deg2rad(45), 0, 0, np.deg2rad(90), 0])
    >>> joints_rad = arm.inverse(xyz=[-400, 100, 900])
    """

    def __init__(self, side: ArmSide | str = ArmSide.RIGHT):
        super().__init__(get_arm_model(side))


class HeadKinematics(ChainKinematics):
    """Forward kinematics of pib's head; the tip frame is the camera link."""

    def __init__(self) -> None:
        super().__init__(get_head_model())

    def camera_pose(self, pan_rad: float = 0.0, tilt_rad: float = 0.0) -> pin.SE3:
        """Pose of the camera frame in the base frame for a given head posture (rad).

        The orientation includes :data:`~pib_sdk.robot_model.CAMERA_FRAME_CORRECTION`
        (180° about camera +Y) applied when the URDF is loaded, so optical +Z
        points away from the body until the Onshape export is fixed (PR-2045).
        """
        return self.forward([pan_rad, tilt_rad])

    def camera_pose_deg(self, pan_deg: float = 0.0, tilt_deg: float = 0.0) -> pin.SE3:
        """Pose of the camera frame for pan/tilt in degrees."""
        return self.camera_pose(np.deg2rad(pan_deg), np.deg2rad(tilt_deg))


# --------------------------------------------------------------------------- #
# One-call convenience wrappers                                                #
# --------------------------------------------------------------------------- #
def fk(side: ArmSide | str, q_rad: Iterable[float]) -> pin.SE3:
    """Forward kinematics of an arm in a single call (joint angles in radians).

    Example
    -------
    >>> import numpy as np
    >>> pose = fk("right", [0, np.deg2rad(45), 0, 0, np.deg2rad(90), 0])
    >>> pose.translation  # palm position in mm
    """
    return ArmKinematics(side).forward(q_rad)


def fk_deg(side: ArmSide | str, q_deg: Iterable[float]) -> pin.SE3:
    """Forward kinematics with joint angles in degrees."""
    return fk(side, np.deg2rad(q_deg))


def get_hand_position_xyz(
    side: ArmSide | str,
    *,
    telemetry_host: str = "localhost",
    telemetry_port: int = 9090,
    telemetry=None,
) -> tuple[float, float, float]:
    """Return the hand-tip XYZ of one arm from currently commanded joint positions.

    Joint positions are read in **degrees** from telemetry and converted
    internally. The returned ``(x, y, z)`` is in **millimetres** in pib's base
    frame (the translation of :meth:`ArmKinematics.forward`).

    Joints are read via the SDK's position reader and reflect the last
    **commanded** position, not live encoder feedback. A stalled or
    hand-moved joint is therefore not detected.

    Parameters
    ----------
    side:
        ``"left"`` or ``"right"`` (or :class:`~pib_sdk.robot_model.ArmSide`).
    telemetry_host, telemetry_port:
        Used only when ``telemetry`` is omitted, to construct a
        :class:`~pib_sdk.telemetry.Telemetry` client.
    telemetry:
        Optional position source. If given, the caller owns its lifecycle
        (used by offline tests to inject a fake). If omitted, a client is
        opened and closed inside this call.
    """
    arm = ArmKinematics(side)

    def xyz_from(source) -> tuple[float, float, float]:
        positions = source.get_positions_deg(arm.motor_names)
        missing = [name for name in arm.motor_names if name not in positions]
        if missing:
            raise KeyError(
                f"Missing motor position for {missing[0]!r}; needed motors {list(arm.motor_names)}"
            )
        q_deg = [positions[m] for m in arm.motor_names]
        pose = arm.forward_deg(q_deg)
        return tuple(pose.translation)

    if telemetry is None:
        from pib_sdk.telemetry import Telemetry

        with Telemetry(host=telemetry_host, port=telemetry_port) as owned:
            return xyz_from(owned)
    return xyz_from(telemetry)


def get_hand_position_xyz_deg(
    side: ArmSide | str,
    *,
    telemetry_host: str = "localhost",
    telemetry_port: int = 9090,
    telemetry=None,
) -> tuple[float, float, float]:
    """Alias of :func:`get_hand_position_xyz` (telemetry is always in degrees)."""
    return get_hand_position_xyz(
        side,
        telemetry_host=telemetry_host,
        telemetry_port=telemetry_port,
        telemetry=telemetry,
    )


def ik(
    side: ArmSide | str,
    *,
    xyz: Sequence[float],
    rpy_rad: Sequence[float] | None = None,
    **solver_options,
) -> np.ndarray:
    """Inverse kinematics of an arm in a single call (returns radians).

    Example
    -------
    >>> q_rad = ik("left", xyz=[400, 100, 900])

    Keyword options are forwarded to :meth:`ChainKinematics.inverse`
    (``initial_guess_rad``, ``tolerance``, ``max_iterations``, ``mask``,
    ``restarts``, ``respect_limits``).
    """
    return ArmKinematics(side).inverse(xyz=xyz, rpy_rad=rpy_rad, **solver_options)


def ik_deg(
    side: ArmSide | str,
    *,
    xyz: Sequence[float],
    rpy_deg: Sequence[float] | None = None,
    **solver_options,
) -> np.ndarray:
    """Inverse kinematics with orientation in degrees (returns degrees)."""
    return ArmKinematics(side).inverse_deg(xyz=xyz, rpy_deg=rpy_deg, **solver_options)


def camera_pose(
    pan_rad: float = 0.0,
    tilt_rad: float = 0.0,
    *,
    pan_deg: float | None = None,
    tilt_deg: float | None = None,
) -> pin.SE3:
    """Pose of the head camera in the base frame (translation in mm, angles in rad).

    The returned pose uses the corrected camera link frame (see
    :data:`~pib_sdk.robot_model.CAMERA_FRAME_CORRECTION`); this layer is
    temporary until PR-2045 fixes the bundled URDF export.

    Legacy callers may pass ``pan_deg`` / ``tilt_deg`` instead of radian arguments.

    Example
    -------
    >>> import numpy as np
    >>> pose = camera_pose(pan_rad=np.deg2rad(30), tilt_rad=np.deg2rad(-10))
    """
    if pan_deg is not None or tilt_deg is not None:
        return camera_pose_deg(
            pan_deg if pan_deg is not None else np.rad2deg(pan_rad),
            tilt_deg if tilt_deg is not None else np.rad2deg(tilt_rad),
        )
    return HeadKinematics().camera_pose(pan_rad, tilt_rad)


def camera_pose_deg(pan_deg: float = 0.0, tilt_deg: float = 0.0) -> pin.SE3:
    """Pose of the head camera for pan/tilt in degrees."""
    return HeadKinematics().camera_pose(np.deg2rad(pan_deg), np.deg2rad(tilt_deg))


# --------------------------------------------------------------------------- #
# Backwards-compatible aliases (previous SDK spelling)                         #
# --------------------------------------------------------------------------- #
class FK:
    """Drop-in replacement for the previous SDK's ``FK`` class."""

    def __init__(self, side: ArmSide | str = ArmSide.RIGHT):
        self._kinematics = ArmKinematics(side)
        self.joint_names = self._kinematics.joint_names

    def pose(
        self,
        q_rad: Iterable[float] | None = None,
        *,
        q_deg: Iterable[float] | None = None,
    ) -> pin.SE3:
        """Compute the end-effector pose for a joint vector in radians.

        Legacy callers may pass ``q_deg`` instead of a radian vector.
        """
        if q_deg is not None:
            return self.pose_deg(q_deg)
        return self._kinematics.forward(q_rad)

    def pose_deg(self, q_deg: Iterable[float]) -> pin.SE3:
        """Compute the end-effector pose for a joint vector in degrees."""
        return self._kinematics.forward_deg(q_deg)


class IK:
    """Drop-in replacement for the previous SDK's ``IK`` class."""

    def __init__(self, side: ArmSide | str = ArmSide.RIGHT):
        self._kinematics = ArmKinematics(side)

    def solve(
        self,
        xyz: Sequence[float],
        rpy_rad: Sequence[float] | None = None,
        q0_rad: Iterable[float] | None = None,
        *,
        rpy_deg: Sequence[float] | None = None,
        q0_deg: Iterable[float] | None = None,
        tol: float = 1e-4,
        max_steps: int = 100,
        custom_mask: Sequence[float] | None = None,
    ) -> np.ndarray:
        """Return joint angles (radians) for the requested pose.

        Legacy callers may pass ``rpy_deg`` / ``q0_deg`` to receive degrees.
        """
        if rpy_deg is not None or q0_deg is not None:
            return self.solve_deg(
                xyz=xyz,
                rpy_deg=rpy_deg,
                q0_deg=q0_deg,
                tol=tol,
                max_steps=max_steps,
                custom_mask=custom_mask,
            )
        return self._kinematics.inverse(
            xyz=xyz,
            rpy_rad=rpy_rad,
            initial_guess_rad=q0_rad,
            tolerance=tol,
            max_iterations=max_steps,
            mask=custom_mask,
        )

    def solve_deg(
        self,
        xyz: Sequence[float],
        rpy_deg: Sequence[float] | None = None,
        q0_deg: Iterable[float] | None = None,
        tol: float = 1e-4,
        max_steps: int = 100,
        custom_mask: Sequence[float] | None = None,
    ) -> np.ndarray:
        """Return joint angles (degrees) for the requested pose."""
        return self._kinematics.inverse_deg(
            xyz=xyz,
            rpy_deg=rpy_deg,
            initial_guess_deg=q0_deg,
            tolerance=tol,
            max_iterations=max_steps,
            mask=custom_mask,
        )
