from importlib import import_module
from typing import Iterable, Literal, Sequence, Optional
import numpy as np
from spatialmath import SE3
from roboticstoolbox import DHRobot


def _to_rad(seq_deg: Sequence[float]) -> np.ndarray:
    #Degrees → radians (returns NumPy array)
    return np.deg2rad(np.asarray(seq_deg, dtype=float))


def _to_deg(seq_rad: Sequence[float]) -> np.ndarray:
    #Radians → degrees (returns NumPy array)
    return np.rad2deg(np.asarray(seq_rad, dtype=float))


def _get_robot(side: Literal["right", "left"]) -> DHRobot:
    #Instantiate either pib_right() or pib_left() from DH_model.pib_DH.
    mod = import_module("pib_utils.DH_model.pib_DH")
    cls_name = {"right": "pib_right", "left": "pib_left"}[side.lower()]
    return getattr(mod, cls_name)()


# Forward kinematics

class FK:
    def __init__(self, side: Literal["right", "left"] = "right"):
        self.robot: DHRobot = _get_robot(side)
        self.joint_names = [f"theta{i+1}" for i in range(self.robot.n)]

    def pose(self, q_deg: Iterable[float]) -> SE3:
        #Compute end-effector SE3 pose for a joint vector (degrees) raises ValueError  if the length of `q_deg` is not equal to DOF.
        q_deg = list(q_deg)
        if len(q_deg) != self.robot.n:
            raise ValueError(f"Expected {self.robot.n} joint values, got {len(q_deg)}")
        return self.robot.fkine(_to_rad(q_deg))


# Inverse kinematics

class IK:
    def __init__(self, side: Literal["right", "left"] = "right"):
        self.robot: DHRobot = _get_robot(side)

    def solve(
        self,
        xyz: Sequence[float],
        rpy_deg: Optional[Sequence[float]] = None,
        q0_deg: Optional[Iterable[float]] = None,
        tol: float = 1e-4,
        max_steps: int = 100,
        custom_mask: Optional[Sequence[float]] = None,
    ) -> np.ndarray:
        """
        Return joint angles (degrees) for the requested pose.

        Parameters
        ----------
        xyz : (3,) sequence
            Target position in millimetres.
        rpy_deg : (3,) sequence, optional
            Target orientation (roll, pitch, yaw in degrees).  If ``None``,
            orientation is ignored (position-only IK).
        q0_deg : initial guess in degrees (defaults to robot.qz).
        tol, max_steps : convergence settings passed to ikine_LM().
        custom_mask : optional 6-element mask overriding the automatic one raises ValueError if the solver fails to converge.
        """
        # Build target SE3
        T = SE3(*xyz) if rpy_deg is None else SE3(*xyz) * SE3.RPY(*rpy_deg, unit="deg")

        # Default mask
        mask = [1, 1, 1, 0, 0, 0] if rpy_deg is None else [1, 1, 1, 1, 1, 1]
        if custom_mask is not None:
            if len(custom_mask) != 6:
                raise ValueError("custom_mask must have 6 elements")
            mask = list(custom_mask)

        # Initial guess
        q0 = self.robot.qz if q0_deg is None else _to_rad(q0_deg)

        # Solve
        sol = self.robot.ikine_LM(
            T,
            q0=q0,
            tol=tol,
            ilimit=max_steps,
            mask=mask,
        )
        if not sol.success:
            raise ValueError(f"IK failed: {sol.reason}")
        return _to_deg(sol.q)



# One-liner convenience functions

def fk(side: Literal["right", "left"], q_deg: Iterable[float]) -> SE3:
    """
    One-call forward kinematics.

    Example
    -------
    >>> pose = fk("right", [0, 45, 0, 0, 90, 0])
    """
    return FK(side).pose(q_deg)


def ik(
    side: Literal["right", "left"],
    *,
    xyz: Sequence[float],
    rpy_deg: Optional[Sequence[float]] = None,
    **kw,
) -> np.ndarray:
    """
    One-call inverse kinematics.

    Example
    -------
    >>> q = ik("left", xyz=[150, 0, 350])
    """
    return IK(side).solve(xyz=xyz, rpy_deg=rpy_deg, **kw)
