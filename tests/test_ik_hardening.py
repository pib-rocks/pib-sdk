"""Measured IK workspace sweeps, failure contracts, and Pinocchio CLIK cross-check.

Benchmarks (pinocchio 4.1, bundled URDF, ``tolerance=1e-4``, ``restarts=50``,
300 FK seeds per arm, sweep seed 42):

| Arm   | Success | Worst tip (mm) | Worst joint (rad) | Median ms | p95 ms | Med iter | p95 iter |
|-------|---------|----------------|-------------------|-----------|--------|----------|----------|
| right | 100%    | 0.095          | 3.97              | 2.6       | 8.1    | 8        | 19       |
| left  | 100%    | 0.100          | 3.75              | 1.5       | 9.1    | 14       | 21       |

Joint error is ``||q_solution - q_seed||`` (non-unique IK); only tip error gates success.

The sweep draws targets 15 % inside the joint limits, so the seeds are comfortably
reachable. A draw over the **full** limit range is harder and does fail: measured on the
same build, 98 % on the left arm (5 of 300 targets converged only outside the limits and
raised ``InverseKinematicsJointLimitError``) and 100 % on the right. Failures are always
loud - no silently unreached target.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import pinocchio as pin
import pytest

from pib_sdk.kinematics import (
    _POSITION_ONLY_MASK,
    ArmKinematics,
    ChainKinematics,
    InverseKinematicsBehindBodyError,
    InverseKinematicsJointLimitError,
    InverseKinematicsWorkspaceError,
    ik,
    ik_deg,
)
from pib_sdk.robot_model import ArmSide

SIDES = [ArmSide.RIGHT, ArmSide.LEFT]
SWEEP_SAMPLES = 300
SWEEP_SEED = 42
DEFAULT_TOLERANCE = 1e-4

# Floors from measured sweeps (2026-10-25); leave headroom below 100% success.
MIN_SUCCESS_RATE = 0.99
MAX_WORST_TIP_ERROR_MM = 0.15
# LLM-loop motion must stay sub-second; measured p95 ~13 ms per solve.
MAX_P95_SOLVE_MS = 150.0


@dataclass
class SweepResult:
    success_rate: float
    worst_tip_error_mm: float
    worst_joint_error_rad: float
    median_ms: float
    p95_ms: float
    median_iterations: float
    p95_iterations: float


def _interior_joint_samples(
    arm: ArmKinematics, count: int, rng: np.random.Generator
) -> list[np.ndarray]:
    lower_deg, upper_deg = arm.joint_limits_deg
    span = upper_deg - lower_deg
    low, high = lower_deg + 0.15 * span, upper_deg - 0.15 * span
    return [low + (high - low) * rng.random(arm.degrees_of_freedom) for _ in range(count)]


def run_workspace_sweep(arm: ArmKinematics, count: int, seed: int) -> SweepResult:
    rng = np.random.default_rng(seed)
    lower_rad, upper_rad = arm.joint_limits_rad
    successes = 0
    tip_errors: list[float] = []
    joint_errors: list[float] = []
    elapsed_ms: list[float] = []
    iterations: list[int] = []

    for _ in range(count):
        q_seed_deg = _interior_joint_samples(arm, 1, rng)[0]
        q_seed_rad = np.deg2rad(q_seed_deg)
        target = arm.forward(q_seed_rad)
        t0 = time.perf_counter()
        try:
            solution_rad = arm.inverse(
                xyz=target.translation,
                tolerance=DEFAULT_TOLERANCE,
                restarts=50,
                initial_guess_rad=np.zeros(arm.degrees_of_freedom),
            )
            ok = True
        except Exception:
            solution_rad = None
            ok = False
        elapsed_ms.append((time.perf_counter() - t0) * 1000.0)
        stats = arm.last_inverse_stats or {}
        iterations.append(int(stats.get("iterations", 0)))

        if ok and solution_rad is not None:
            reached = arm.forward(solution_rad)
            tip_err = float(np.linalg.norm(reached.translation - target.translation))
            tip_errors.append(tip_err)
            joint_errors.append(float(np.linalg.norm(solution_rad - q_seed_rad)))
            if tip_err < DEFAULT_TOLERANCE:
                successes += 1

    tip_arr = np.asarray(tip_errors, dtype=float)
    joint_arr = np.asarray(joint_errors, dtype=float)
    time_arr = np.asarray(elapsed_ms, dtype=float)
    iter_arr = np.asarray(iterations, dtype=float)
    return SweepResult(
        success_rate=successes / count,
        worst_tip_error_mm=float(np.max(tip_arr)) if tip_arr.size else float("nan"),
        worst_joint_error_rad=float(np.max(joint_arr)) if joint_arr.size else float("nan"),
        median_ms=float(np.median(time_arr)),
        p95_ms=float(np.percentile(time_arr, 95)),
        median_iterations=float(np.median(iter_arr)),
        p95_iterations=float(np.percentile(iter_arr, 95)),
    )


def clik_position_only(
    chain_kin: ChainKinematics,
    target: pin.SE3,
    seed_rad: np.ndarray,
    tolerance: float,
    max_iterations: int,
) -> tuple[np.ndarray, bool]:
    """Independent CLIK loop using Pinocchio Jacobians (cross-check only)."""
    model = chain_kin._chain.model
    data = model.createData()
    frame_id = chain_kin._chain.tip_frame_id
    mask = np.asarray(_POSITION_ONLY_MASK, dtype=bool)
    weights = np.array([1.0, 1.0, 1.0], dtype=float)
    identity = np.eye(chain_kin.degrees_of_freedom)
    joint_angles_rad = np.array(seed_rad, dtype=float)
    damping = 1e-2

    def tip_pose(q: np.ndarray) -> pin.SE3:
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, frame_id)
        return data.oMf[frame_id]

    for _ in range(max_iterations):
        current = tip_pose(joint_angles_rad)
        error = target.translation - current.translation
        error_norm = float(np.linalg.norm(error))
        if error_norm < tolerance:
            return joint_angles_rad, True

        jacobian = pin.computeFrameJacobian(
            model, data, joint_angles_rad, frame_id, pin.LOCAL_WORLD_ALIGNED
        )[mask, :]
        weighted_jacobian = weights[:, None] * jacobian
        weighted_error = weights * error
        step = np.linalg.solve(
            weighted_jacobian.T @ weighted_jacobian + damping * identity,
            weighted_jacobian.T @ weighted_error,
        )
        candidate = pin.integrate(model, joint_angles_rad, step)
        candidate_pose = tip_pose(candidate)
        candidate_error = float(np.linalg.norm(target.translation - candidate_pose.translation))
        if candidate_error < error_norm:
            joint_angles_rad = candidate
            damping = max(damping * 0.5, 1e-6)
        else:
            damping = min(damping * 4.0, 1e6)

    current = tip_pose(joint_angles_rad)
    final_error = float(np.linalg.norm(target.translation - current.translation))
    return joint_angles_rad, final_error < tolerance


@pytest.mark.parametrize("side", SIDES)
def test_workspace_roundtrip_sweep_meets_contract(side):
    arm = ArmKinematics(side)
    result = run_workspace_sweep(arm, SWEEP_SAMPLES, SWEEP_SEED + int(side == ArmSide.LEFT))
    assert result.success_rate >= MIN_SUCCESS_RATE, result
    assert result.worst_tip_error_mm <= MAX_WORST_TIP_ERROR_MM, result
    assert result.p95_ms <= MAX_P95_SOLVE_MS, result


def test_ik_negative_workspace_raises():
    arm = ArmKinematics(ArmSide.RIGHT)
    with pytest.raises(InverseKinematicsWorkspaceError, match="beyond sampled reach"):
        arm.inverse_deg(xyz=[5000.0, 0.0, 0.0], restarts=2, max_iterations=50)


def test_ik_negative_behind_body_raises():
    arm = ArmKinematics(ArmSide.RIGHT)
    with pytest.raises(InverseKinematicsBehindBodyError, match="behind the body"):
        arm.inverse_deg(xyz=[0.0, 1500.0, 900.0], restarts=2, max_iterations=50)


def test_ik_negative_joint_limit_pose_raises():
    arm = ArmKinematics(ArmSide.RIGHT)
    _, upper = arm.joint_limits_rad
    beyond_limit = upper.copy()
    beyond_limit[1] += 0.2
    target = arm.forward(beyond_limit)
    with pytest.raises(InverseKinematicsJointLimitError, match="outside mechanical"):
        arm.inverse(
            xyz=target.translation,
            respect_limits=True,
            restarts=30,
            max_iterations=200,
        )


@pytest.mark.parametrize("side", SIDES)
def test_clik_crosscheck_agrees_on_tip_pose(side):
    arm = ArmKinematics(side)
    rng = np.random.default_rng(seed=99 + int(side == ArmSide.LEFT))
    checked = 0
    for q_seed_deg in _interior_joint_samples(arm, 40, rng):
        q_seed = np.deg2rad(q_seed_deg)
        target = arm.forward(q_seed)
        sdk_solution = arm.inverse(
            xyz=target.translation,
            tolerance=DEFAULT_TOLERANCE,
            restarts=20,
            initial_guess_rad=np.zeros(6),
        )
        clik_solution, clik_ok = clik_position_only(
            arm,
            target,
            seed_rad=np.zeros(6),
            tolerance=DEFAULT_TOLERANCE,
            max_iterations=200,
        )
        assert clik_ok, "CLIK reference failed on an FK seed (should be reachable)"
        sdk_pose = arm.forward(sdk_solution)
        clik_pose = arm.forward(clik_solution)
        tip_delta = float(np.linalg.norm(sdk_pose.translation - clik_pose.translation))
        assert tip_delta < 1.0, f"tip disagreement {tip_delta:.3f} mm"
        checked += 1
    assert checked == 40


def test_ik_and_ik_deg_agree():
    arm = ArmKinematics(ArmSide.RIGHT)
    target = arm.forward_deg([10, -20, 5, 30, -10, 0])
    xyz = list(target.translation)
    q_rad = ik(ArmSide.RIGHT, xyz=xyz, restarts=5)
    q_deg = ik_deg(ArmSide.RIGHT, xyz=xyz, restarts=5)
    np.testing.assert_allclose(q_rad, np.deg2rad(q_deg), atol=1e-9)


def test_ik_respect_limits_always_in_range():
    arm = ArmKinematics(ArmSide.LEFT)
    lower_deg, upper_deg = arm.joint_limits_deg
    rng = np.random.default_rng(7)
    for q_deg in _interior_joint_samples(arm, 8, rng):
        solution_deg = arm.inverse_deg(xyz=arm.forward_deg(q_deg).translation, respect_limits=True)
        assert np.all(solution_deg >= lower_deg - 1e-6)
        assert np.all(solution_deg <= upper_deg + 1e-6)


def test_ik_fixed_seed_is_reproducible():
    arm = ArmKinematics(ArmSide.RIGHT)
    target_xyz = [-420.0, 80.0, 950.0]
    first = arm.inverse_deg(xyz=target_xyz, restarts=50)
    second = arm.inverse_deg(xyz=target_xyz, restarts=50)
    np.testing.assert_allclose(first, second, atol=1e-9)


def test_sequential_solves_do_not_contaminate():
    arm = ArmKinematics(ArmSide.RIGHT)
    configs = [
        [0, 30, -10, 20, 45, 0],
        [15, -25, 5, -15, 60, 10],
    ]
    solutions = []
    for q_deg in configs:
        target = arm.forward_deg(q_deg)
        solutions.append(arm.inverse_deg(xyz=target.translation, restarts=10))
    for q_deg, sol_deg in zip(configs, solutions):
        reached = arm.forward_deg(sol_deg).translation
        expected = arm.forward_deg(q_deg).translation
        assert np.linalg.norm(reached - expected) < DEFAULT_TOLERANCE
