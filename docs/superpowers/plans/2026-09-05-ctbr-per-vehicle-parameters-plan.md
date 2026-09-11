# CTBR Per-Vehicle Parameters Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split CTBR ROS parameters into shared controller, shared trajectory, and ID-selected per-vehicle controller calibration namespaces.

**Architecture:** Add a small parameter resolver in `ctbr_controller.py`. `CtbrControllerNode` reads `/ctbr_controller`, `/ctbr_trajectory`, and `/ctbr_controller_cf<ID>` after its vehicle ID is known; `MultiCtbrControllerNode` only reads global scheduling values. Move existing YAML fields to these roots without changing `crazyflies.yaml`, trajectory mathematics, ROS topics, or CSV fields.

**Tech Stack:** ROS1 `rospy`, Python 3, PyYAML, standalone `runpy` test harnesses.

**Spec:** `docs/superpowers/specs/2026-09-05-ctbr-per-vehicle-parameters-design.md`

## Global Constraints

- Use `/ctbr_controller_cf2`, `/ctbr_controller_cf4`, `/ctbr_controller_cf5`; do not use hyphens in ROS parameter names.
- `crazyflies.yaml` remains the sole source for ID, URI, `ctbr_enabled`, and `orbit_phase_rad`.
- All reference geometry/timing belong to `/ctbr_trajectory`; only phase offset comes from `crazyflies.yaml`.
- Per-CF blocks contain calibration/tuning only; state/EKF safety and fleet coordination remain shared.
- Preserve existing dirty worktree changes and do not create a commit without explicit user authorization.

---

### Task 1: Test and implement a parameter namespace resolver

**Files:**

- Modify: `ros_ws/src/crazyswarm/scripts/ctbr_controller.py:45-55, 789-1240`
- Modify: `ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py:1-130`

**Interfaces:**

- Produce `vehicle_parameter_namespace(vehicle_id) -> str`, which returns `/ctbr_controller_cf<ID>` for a positive ID.
- Produce `CtbrParameterResolver(get_param, has_param, vehicle_id)` with `global_param(name, default)`, `trajectory_param(name, default)`, and `vehicle_param(name, default)` methods.
- Reject missing or non-dictionary `/ctbr_controller_cf<ID>` roots with an error that names both `CF<ID>` and the full expected path.

- [ ] **Step 1: Write failing resolver tests**

Add these tests before production code:

```python
def test_vehicle_parameter_namespace_uses_cf_id_suffix():
    module = _controller_module()
    assert module.vehicle_parameter_namespace(2) == "/ctbr_controller_cf2"
    assert module.vehicle_parameter_namespace(4) == "/ctbr_controller_cf4"


def test_parameter_resolver_keeps_global_trajectory_and_cf_calibration_separate():
    module = _controller_module()
    values = {
        "/ctbr_controller_cf2": {},
        "/ctbr_controller_cf4": {},
        "/ctbr_controller/control_rate_hz": 60.0,
        "/ctbr_trajectory/figure_eight_radius_m": 0.8,
        "/ctbr_controller_cf2/mass_kg": 0.0434,
        "/ctbr_controller_cf4/mass_kg": 0.0460,
    }
    resolver = module.CtbrParameterResolver(
        lambda name, default=None: values.get(name, default),
        lambda name: name in values,
        vehicle_id=4,
    )
    assert resolver.global_param("control_rate_hz") == 60.0
    assert resolver.trajectory_param("figure_eight_radius_m") == 0.8
    assert resolver.vehicle_param("mass_kg") == 0.0460


def test_parameter_resolver_rejects_missing_vehicle_calibration_block():
    module = _controller_module()
    try:
        module.CtbrParameterResolver(lambda *_args: None, lambda _name: False, 5)
    except ValueError as error:
        assert "CF5" in str(error)
        assert "/ctbr_controller_cf5" in str(error)
    else:
        raise AssertionError("missing CF5 parameter block must be rejected")
```

- [ ] **Step 2: Verify RED**

Run the three new tests through `runpy`. Expected failure: the namespace function and resolver are undefined.

- [ ] **Step 3: Implement the minimal resolver**

Add `GLOBAL_CTBR_PARAMETER_ROOT = "/ctbr_controller"`, `TRAJECTORY_PARAMETER_ROOT = "/ctbr_trajectory"`, a missing-default sentinel, `vehicle_parameter_namespace()`, and `CtbrParameterResolver`. Validate the vehicle root with `has_param(root)` and `get_param(root)` before any vehicle field read.

- [ ] **Step 4: Verify GREEN**

Re-run the three resolver tests. Expected: all pass.

### Task 2: Restructure YAML and test enabled-CF coverage

**Files:**

- Modify: `ros_ws/src/crazyswarm/config/ctbr_controller.yaml:1-150`
- Modify: `ros_ws/src/crazyswarm/scripts/test_vehicle_config.py:1-100`

**Interfaces:**

- Produce top-level maps `ctbr_controller`, `ctbr_trajectory`, `ctbr_controller_cf2`, `ctbr_controller_cf4`, `ctbr_controller_cf5`.
- Every enabled `crazyflies.yaml` ID must have a matching `ctbr_controller_cf<ID>` map.

- [ ] **Step 1: Write a failing layout test**

Add a test loading both YAML files, deriving `enabled_ids` from entries where `ctbr_enabled` is true, and asserting each has a `ctbr_controller_cf%d` map containing positive `mass_kg`, valid thrust limits, and three-element `position_gain`. Also assert `ctbr_trajectory` exists, while `mass_kg` and `trajectory_mode` no longer occur under `ctbr_controller`.

- [ ] **Step 2: Verify RED**

Run only the new layout test. Expected failure: the trajectory and per-CF maps do not yet exist.

- [ ] **Step 3: Move YAML keys by ownership**

Move fields into exactly these groups:

```text
ctbr_controller:
  fleet scheduling; mocap/EKF health and filters; gravity; voltage admission;
  log and Path settings

ctbr_trajectory:
  trajectory mode, geometry, formation values, all reference timing,
  target heights, landing values, phase-specific tilt limits

ctbr_controller_cf<ID>:
  mass; thrust limits; phase minimum thrust; default tilt/body-rate limits;
  position/velocity/integral/attitude gains; V2 Omega_c settings;
  voltage-to-raw-PWM reference and scaling
```

Copy current per-vehicle calibration values into CF2, CF4, and CF5 as initial values. Keep `ctbr_visualization` unchanged and do not add controller tuning to `crazyflies.yaml`.

- [ ] **Step 4: Verify GREEN**

Run every `test_*` function in `test_vehicle_config.py`. Expected: all pass.

### Task 3: Route controller reads to explicit namespaces

**Files:**

- Modify: `ros_ws/src/crazyswarm/scripts/ctbr_controller.py:802-1240, 2065-2145`
- Modify: `ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py:1-130`
- Modify: `ros_ws/src/crazyswarm/launch/ctbr_controller.launch:7-13` only if root-level YAML loading needs adjustment; otherwise do not edit it.

**Interfaces:**

- `CtbrControllerNode` owns `self.params: CtbrParameterResolver`.
- `_vector_param` accepts a resolver reader callable plus a field name, retaining `as_vector` validation.
- `MultiCtbrControllerNode` reads scheduler/logging values only from `/ctbr_controller`.

- [ ] **Step 1: Write failing controller-construction routing tests**

Extend the existing ROS stubs with a complete dictionary-backed parameter server (`get_param`, `has_param`, `Publisher`, `Subscriber`, `Timer`, `Duration`, `on_shutdown`, logging no-ops). Parse the migrated YAML into absolute parameter names and construct `CtbrControllerNode` twice with CF2 and CF4 vehicle entries plus `auto_timer=False` and a temporary logger. Assert each node’s `controller.config.mass`, `position_gain`, and `max_command_thrust` come from its own `ctbr_controller_cf<ID>` block, while both nodes share the same trajectory mode and control rate. This catches the production mistake of leaving a `~mass_kg` or `~trajectory_mode` read behind.

Remove `/ctbr_controller_cf5` from the fake parameter server and assert construction for CF5 fails with a message containing `/ctbr_controller_cf5`.

- [ ] **Step 2: Verify RED**

Run only these routing tests. Expected failure: the existing controller uses private `~` paths, which are intentionally absent from the fake server.

- [ ] **Step 3: Implement field routing**

After `CtbrControllerNode` parses its `vehicle_id`, construct `self.params`. Replace all private parameter reads according to this mapping:

```text
global_param: target confirmation, rate, mocap/EKF/safety/filter settings,
              gravity, voltage admission, logging and Path values
trajectory_param: every CircularTrajectoryConfig field except orbit_phase_rad
vehicle_param: mass, thrust limits, minimum flight thrust, default limits,
               gains, V2 settings, raw-PWM voltage calibration
```

Pass `self.orbit_phase_rad` from `crazyflies.yaml` unchanged to `CircularTrajectoryConfig`. Translate resolver `ValueError` into `rospy.ROSInitException` during node setup. Update the coordinator’s fleet count, log directory, rate, and pause threshold to absolute `/ctbr_controller/...` reads. Retain `/crazyflies`, `~cf_id`, and `~cf_prefix` compatibility reads.

- [ ] **Step 4: Verify GREEN**

Run all `test_*` functions in `test_ctbr_controller_v2.py`. Expected: all pass.

### Task 4: Full regression verification and operator handoff

**Files:**

- Modify only files from Tasks 1–3 if a regression test identifies a defect.

- [ ] **Step 1: Run syntax and YAML checks**

Run:

```bash
python3 -m py_compile \
  ros_ws/src/crazyswarm/scripts/ctbr_controller.py \
  ros_ws/src/crazyswarm/scripts/ctbr_trajectory.py \
  ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py \
  ros_ws/src/crazyswarm/scripts/test_vehicle_config.py
python3 - <<'PY'
import yaml
with open('ros_ws/src/crazyswarm/config/ctbr_controller.yaml') as handle:
    config = yaml.safe_load(handle)
assert {'ctbr_controller', 'ctbr_trajectory', 'ctbr_controller_cf2',
        'ctbr_controller_cf4', 'ctbr_controller_cf5'} <= set(config)
print('parameter YAML parsed')
PY
```

- [ ] **Step 2: Run the full standalone suite**

Run:

```bash
python3 - <<'PY'
import runpy
files = [
    'ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py',
    'ros_ws/src/crazyswarm/scripts/test_ctbr_trajectory_smoothstep.py',
    'ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py',
    'ros_ws/src/crazyswarm/scripts/test_vehicle_config.py',
]
count = 0
for filename in files:
    module = runpy.run_path(filename)
    tests = [(name, value) for name, value in module.items()
             if name.startswith('test_') and callable(value)]
    for _name, test in tests:
        test()
    count += len(tests)
print('%d tests passed' % count)
PY
git diff --check
```

Expected: every test passes and `git diff --check` reports no whitespace error.

- [ ] **Step 3: Include the addition workflow in the handoff**

Document that adding CF6 requires both an enabled identity entry in `crazyflies.yaml` and a complete `ctbr_controller_cf6` block in `ctbr_controller.yaml`; no Python source edit is required.
