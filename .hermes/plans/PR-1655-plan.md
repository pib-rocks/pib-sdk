# PR-1655: pib-sdk get_hand_position_xyz(side) — combine motor read + forward kinematics

Ticket: https://pib-rocks.atlassian.net/browse/PR-1655
Repo: pib-rocks/pib-sdk — local at /home/pib/opencode/pib-sdk, branch `PR-1655` (from main).
Scope: implement the SDK read-side helper. Do NOT merge to develop. Do NOT push.

## Goal
Add `get_hand_position_xyz(side)` to pib-sdk that (1) reads every motor of the chosen arm's
kinematic chain via the SDK's existing position reader, (2) maps them in correct URDF joint
order, and (3) runs forward kinematics to return the hand tip XYZ (mm). Make it testable
offline (injectable position source, fake roslibpy), and unit-verify it matches `fk(side, q)`.

## Verified facts (read from this repo's source)
1. `src/pib_sdk/kinematics.py`:
   - `ArmKinematics(side)` — `side` is `ArmSide | str` ("left"/"right").
   - `.motor_names` → list of pib motor names in joint-vector (URDF) order.
   - `.forward(joint_angles_deg)` → `pin.SE3`; `.translation` = [x,y,z] mm.
   - `.joint_names` → URDF joint names in same order.
   - `pose_to_xyz_rpy(pose)` → `(xyz_mm, rpy_deg)`.
   - `fk(side, q_deg)` = one-call ArmKinematics forward.
2. `src/pib_sdk/telemetry.py`:
   - `Telemetry(host, port, get_position_service, motor_current_topic)`.
   - `.get_positions_deg(motor_names)` → `{name: deg}` (batches get_joint_position).
   - `.get_position_deg(motor_name)` → float degrees.
   - `.close()` and context manager `__enter__/__exit__`.
3. Tests are OFFLINE via a shared fake roslibpy in `tests/conftest.py` exposing
   `patch_roslibpy(monkeypatch, module)` → `(topics, services)`, and `services`
   is a dict keyed by service name with `.responses[motor_name] = {...}`. See
   `tests/test_telemetry.py`, `tests/test_kinematics.py`.
4. RUN tests: `cd /home/pib/opencode/pib-sdk && PYTHONPATH=src python3 -m pytest tests/ -q`.
   Baseline green (48 passed on test_kinematics, test_telemetry, test_robot_model).

## Design
Add the function in `src/pib_sdk/kinematics.py` OR a new small module. Recommended: put
`get_hand_position_xyz` in `src/pib_sdk/kinematics.py` (it is FK-related), importing
`Telemetry` only lazily/optionally so kinematics stays usable without roslibpy if needed.
Signature:

```python
def get_hand_position_xyz(
    side: ArmSide | str,
    *,
    telemetry_host: str = "localhost",
    telemetry_port: int = 9090,
    telemetry: Telemetry | None = None,
    axes: str = "xyz",          # optional; default xyz
) -> tuple[float, float, float]:
```

Behaviour:
- If `telemetry` is not provided, construct `Telemetry(host=telemetry_host, port=telemetry_port)`,
  read positions, and close it (use it as a context manager).
- If `telemetry` is provided, use it (caller manages lifecycle) — this is how offline tests inject
  a fake.
- Build `q = [positions[m] for m in arm.motor_names]` (assert every motor present).
- `pose = arm.forward(q)`; return `tuple(pose.translation)` (mm).
- Docstring must state mm/degree units and the caveat that values are the last COMMANDED
  position, not live encoder feedback.

Keep it minimal; do not add eager `Telemetry` import at module top if kinematics currently
imports no roslibpy (check). Prefer lazy import inside the function.

## Phases
### Phase 1 — function + export + docstring
- Add `get_hand_position_xyz` to `src/pib_sdk/kinematics.py`.
- Export it from `src/pib_sdk/__init__.py` and add to `__all__`.
- Run `PYTHONPATH=src python3 -c "import pib_sdk; print(hasattr(pib_sdk,'get_hand_position_xyz'))"`.

### Phase 2 — offline unit tests
Add `tests/test_hand_position.py` using the conftest fake:
1. `test_returned_xyz_matches_fk_for_known_q`: inject a fake Telemetry whose
   `get_positions_deg` returns a known dict; assert `get_hand_position_xyz(side, telemetry=fake)`
   equals `fk(side, q).translation` (as tuples) — proving it uses the same FK.
2. `test_uses_arm_motor_names_in_order`: assert the fake Telemetry was called with exactly
   `ArmKinematics(side).motor_names` (spy the call argument).
3. `test_raises_if_motor_missing`: fake returns position dict missing one arm motor →
   expect a clear error (KeyError or ValueError) naming the missing motor.
4. `test_default_host_port_construct_telemetry`: (optional) if it does not require network,
   use the conftest fake to confirm default construction path works; skip if it would need a
   real rosbridge. If risky, assert instead that the injected-path works and document that
   the default path creates a Telemetry(host, port).

Match the conftest patching pattern exactly (mirror test_telemetry.py: `patch_roslibpy(monkeypatch, telemetry_module)`).

## Hard constraints
1. Stay on branch `PR-1655`. Do NOT merge to develop. Do NOT push.
2. Prefer a lazy `Telemetry` import so `pib_sdk.kinematics` has no hard roslibpy import-time dep
   (VERIFIED: `kinematics.py` currently imports NO roslibpy — only numpy, pinocchio, and
   pib_sdk.robot_model items). Use `from pib_sdk.telemetry import Telemetry` INSIDE the function,
   not at module top.
3. Do NOT change existing API (`fk`, `ik`, `ArmKinematics`, `Telemetry` signatures).
4. Do NOT touch motor control, rosbridge, or the robot model. Pure read-side addition.
5. You MAY commit on `PR-1655` (`feat: PR-1655 ...`). Do NOT commit `.hermes/plans/**`. Do NOT
   commit if tests fail.
6. Report REAL test counts (do not fabricate).

## Verify (orchestrator after each phase)
- `git branch --show-current` == PR-1655
- `PYTHONPATH=src python3 -m pytest tests/test_hand_position.py -q` → all pass
- Full `PYTHONPATH=src python3 -m pytest tests/ -q` → pass, no regressions
- `PYTHONPATH=src python3 -c "import pib_sdk; assert hasattr(pib_sdk,'get_hand_position_xyz')"`
- `git diff --stat` shows only the new function, __init__ export, and the new test file.
- No merge/push (origin/main sha unchanged).