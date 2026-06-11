from importlib import import_module
from typing import Iterable, Literal, Sequence, Optional
import numpy as np
from spatialmath import SE3
import pinocchio as pin


def _to_rad(seq_deg: Sequence[float]) -> np.ndarray:
    #Degrees → radians (returns NumPy array)
    return np.deg2rad(np.asarray(seq_deg, dtype=float))


def _to_deg(seq_rad: Sequence[float]) -> np.ndarray:
    #Radians → degrees (returns NumPy array)
    return np.rad2deg(np.asarray(seq_rad, dtype=float))


def _get_robot(side: Literal["right", "left"]):
    #Instantiate either pib_right() or pib_left() from DH_model.pib_DH.
    mod = import_module("pib_sdk.pib_DH")
    cls_name = {"right": "pib_right", "left": "pib_left"}[side.lower()]
    return getattr(mod, cls_name)()

def _pin_to_se3(T: pin.SE3) -> SE3:
    """Konvertierung von pinocchio SE3 -> spatialmath SE3."""
    M = np.eye(4)
    M[:3, :3] = T.rotation
    M[:3,  3] = T.translation
    return SE3(M)
# Forward kinematics

class FK:
    def __init__(self, side: Literal["right", "left"] = "right"):
        self.robot = _get_robot(side)
        self.joint_names = [f"theta{i+1}" for i in range(self.robot.n)]

    def pose(self, q_deg: Iterable[float]) -> SE3:
        #Compute end-effector SE3 pose for a joint vector (degrees) raises ValueError  if the length of `q_deg` is not equal to DOF.
        q_deg = list(q_deg)
        if len(q_deg) != self.robot.n:
            raise ValueError(f"Expected {self.robot.n} joint values, got {len(q_deg)}")
        q = _to_rad(q_deg)
        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        
        frame_id = self.robot.model.getFrameId(self.robot.ee_frame_name)
        return _pin_to_se3(self.robot.data.oMf[frame_id])


# Inverse kinematics

class IK:
    def __init__(self, side: Literal["right", "left"] = "right"):
        self.robot = _get_robot(side)

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
        # Zielpose aufbauen
        if rpy_deg is None:
            T_target = pin.SE3(np.eye(3), np.array(xyz, dtype=float))
        else:
            T_target = pin.SE3(
                pin.rpy2matrix(*np.radians(rpy_deg)),
                np.array(xyz, dtype=float),
            )

        # Maske (position-only vs. vollständig)
        if custom_mask is not None:
            if len(custom_mask) != 6:
                raise ValueError("custom_mask must have 6 elements")
            mask = np.array(custom_mask, dtype=float)
        else:
            mask = np.array([1, 1, 1, 0, 0, 0] if rpy_deg is None else [1, 1, 1, 1, 1, 1], dtype=float)

        # Startwert
        q = self.robot.qz.copy() if q0_deg is None else _to_rad(q0_deg)

        frame_id = self.robot.model.getFrameId(self.robot.ee_frame_name)
        model, data = self.robot.model, self.robot.data

        # Levenberg-Marquardt Loop
        damping = 1e-6
        for _ in range(max_steps):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)

            T_current = data.oMf[frame_id]
            err_se3   = T_target.actInv(T_current)          # Fehler im lokalen Frame
            err_vec   = pin.log6(err_se3).vector * mask      # 6D Fehlervektor, maskiert

            if np.linalg.norm(err_vec) < tol:
                return _to_deg(q)

            J_full = pin.computeFrameJacobian(
                model, data, q, frame_id, pin.LOCAL
            )
            J = J_full * mask[:, None]                       # Zeilen maskieren

            # LM-Schritt: q += (JᵀJ + λI)⁻¹ Jᵀ err
            JtJ = J.T @ J
            q  -= np.linalg.solve(JtJ + damping * np.eye(model.nq), J.T @ err_vec)
            q   = pin.normalize(model, q)                    # Gelenkgrenzen einhalten

        raise ValueError("IK failed: max_steps reached without convergence")



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
