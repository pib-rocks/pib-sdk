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

def _pin_to_se3(T) -> SE3:
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

	def solve(self, xyz: Sequence[float], rpy_deg: Optional[Sequence[float]] = None, q0_deg: Optional[Iterable[float]] = None, tol: float = 0.02, max_steps: int = 100, custom_mask: Optional[Sequence[float]] = None, n_restarts=100, seed=None) -> np.ndarray:
	# Zielpose aufbauen
		# Build target SE3
		T_target= SE3(*xyz) if rpy_deg is None else SE3(*xyz) * SE3.RPY(*rpy_deg, unit="deg")
		# Default mask
		mask = [1, 1, 1, 0, 0, 0] if rpy_deg is None else [1, 1, 1, 1, 1, 1]
		if custom_mask is not None:
			if len(custom_mask) != 6:
				raise ValueError("custom_mask must have 6 elements")
			mask = list(custom_mask)
		model, data = self.robot.model, self.robot.data
		frame_id = model.getFrameId(self.robot.ee_frame_name)
		lo, hi = model.lowerPositionLimit, model.upperPositionLimit
		rng = np.random.default_rng(seed)
		#Solve-Loop, Warning: Solving is still partially with errors (inconsistent, convergence not enough, cause is unknown)
		starts = []
		if q0_deg is not None:
			starts.append(_to_rad(q0_deg))
		starts.append(self.robot.qz)
		if hasattr(self.robot, "q_rest"):
			starts.append(self.robot.q_rest)
		if hasattr(self.robot, "q_observe"):
			starts.append(self.robot.q_observe)
		while len(starts) < n_restarts:
			starts.append(lo + rng.random(model.nq) * (hi - lo))
		best_q, best_err = None, np.inf
		for q0 in starts[:n_restarts]:
			q = q0.copy()
			lam = 1e-3
			stall = 0
			pin.forwardKinematics(model, data, q)
			pin.updateFramePlacements(model, data)
			err = T_target.t - data.oMf[frame_id].translation
			err_norm = np.linalg.norm(err)
			for _ in range(max_steps):
				if err_norm < tol:
					return np.rad2deg(q)
				J6 = pin.computeFrameJacobian(model, data, q, frame_id, pin.WORLD)
				J = J6[:3, :]
				dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
				dq = np.clip(dq, -np.deg2rad(15), np.deg2rad(15))
				q_trial = pin.integrate(model, q, dq)
				q_trial = np.clip(q_trial, lo, hi)
				pin.forwardKinematics(model, data, q_trial)
				pin.updateFramePlacements(model, data)
				err_trial = T_target.t - data.oMf[frame_id].translation
				err_trial_norm = np.linalg.norm(err_trial)
				if err_trial_norm < err_norm:
					q, err, err_norm = q_trial, err_trial, err_trial_norm
					lam = max(lam * 0.5, 1e-6)
					stall = 0
				else:
					lam = min(lam * 10, 1e3)
					stall += 1
					if stall > 15:
						break
				if err_norm < best_err:
					best_err, best_q = err_norm, q.copy()
		raise ValueError(
		f"IK failed after {n_restarts} restarts; bestes Residuum = "
		f"{best_err*1000:.2f} mm (target {np.round(xyz, 4)})."
		)


# One-liner convenience functions

def fk(side: Literal["right", "left"], q_deg: Iterable[float]) -> SE3:
	return FK(side).pose(q_deg)


def ik(side: Literal["right", "left"], *, xyz: Sequence[float], rpy_deg: Optional[Sequence[float]] = None, **kw) -> np.ndarray:
	return IK(side).solve(xyz=xyz, rpy_deg=rpy_deg, **kw)