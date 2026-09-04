# 多无人机 CTBR 圆周编队 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将单机 CTBR 任务扩展为由 `crazyflies.yaml` 配置的两架及以上飞机同步圆周编队任务，并提供合并 CSV、多机绘图和 RViz 多 Path 显示。

**Architecture:** 保留现有 `GeometricCtbrController`、NOKOV/EKF 状态混合和 C++ CTBR 桥接；在 Python 节点中创建每架飞机的运行时对象，由一个全局定时器统一推进任务时钟。每架飞机独立发布 `/cf<ID>/cmd_ctbr` 和 `/cf<ID>/path`，所有控制周期写入同一任务级 CSV；可视化按 `vehicle_id` 分组并保持单机 CSV 兼容。

**Tech Stack:** ROS Noetic/rospy、Python 3、NumPy、SciPy、Matplotlib、PyYAML、C++ Crazyswarm server、RViz。

**Spec:** `docs/superpowers/specs/2026-09-04-multi-ctbr-formation-design.md`

## Global Constraints

- 仅使用一条 Crazyradio；所有飞机 URI 必须使用同一 radio/channel，连接和 CRTP 调度由 `crazyswarm_server` 负责。
- 位姿和姿态继续使用 NOKOV；EKF 只参与当前已实现的速度/加速度混合，不替换 `R_WB`。
- 世界坐标圆心固定为 `[0, 0]`，半径为 `1.0 m`，每架高度为自身有效 NOKOV 起点 `z0 + 1.0 m`。
- `ctbr_enabled: true` 的条目参加多机任务；配置解析必须拒绝重复 id、重复 URI、缺失 phase 和不同 Radio/channel。
- 所有飞机未完成电压预检或任一飞机状态失效时不得继续发送有效推力；任务异常时向所有飞机发送零推力。
- 保留现有单机话题名称和 `~cf_id`/`~cf_prefix` 兼容行为。

---

### Task 1: 扩展多机配置解析与轨迹参数

**Files:**
- Modify: `ros_ws/src/crazyswarm/scripts/vehicle_config.py`
- Modify: `ros_ws/src/crazyswarm/scripts/ctbr_trajectory.py`
- Modify: `ros_ws/src/crazyswarm/launch/crazyflies.yaml`
- Modify: `ros_ws/src/crazyswarm/config/ctbr_controller.yaml`
- Test: `ros_ws/src/crazyswarm/scripts/test_vehicle_config.py`
- Test: `ros_ws/src/crazyswarm/scripts/test_ctbr_trajectory_smoothstep.py`

**Interfaces:**
- `validate_vehicle_entries(entries, require_ctbr=True, min_ctbr=1) -> list[dict]` validates id, URI, radio/channel and `orbit_phase_rad`.
- `select_vehicle_entries(entries, cf_id=None) -> list[dict]` returns all enabled entries in multi mode or one explicit entry for legacy mode.
- `CircularTrajectoryConfig` gains `circle_center_xy` and `orbit_phase_rad` fields.
- `CircularFlightTrajectory.reset(start_position, start_yaw, now, orbit_phase_rad=None)` stores the per-vehicle orbit phase.
- `CircularFlightTrajectory._circle_target(elapsed)` applies the stored phase to position, velocity, acceleration and jerk.
- `CircularFlightTrajectory._desired_yaw(...)` supports `face_partner` by using the configured opposite phase; otherwise keeps the existing center-facing behavior.

- [ ] **Step 1: Write failing configuration tests**

Add tests that pass two entries with the same `channel` and radio prefix, assert both are returned in id order, assert phases `0` and `pi` are preserved, and assert duplicate id/URI or mismatched channel raises `ValueError`.

- [ ] **Step 2: Run the configuration tests and verify the expected failure**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_vehicle_config.py -q`

Expected: FAIL because multi-entry validation and selection functions do not exist.

- [ ] **Step 3: Implement multi-entry validation and selection**

Keep `select_vehicle_entry` as a wrapper for legacy callers. Add validation that normalizes explicit `uri`, checks `0 <= id <= 255`, requires unique ids/URIs, requires one shared `radio://<radio>/<channel>/...` prefix and finite `orbit_phase_rad` for CTBR entries. Preserve the existing legacy URI fallback for entries without `uri`.

- [ ] **Step 4: Write failing trajectory phase/yaw tests**

Add tests that instantiate two trajectories with `orbit_phase_rad=0` and `math.pi`, evaluate the same elapsed time, assert their XY targets differ by exactly `2*radius`, their z targets equal their own `z0 + takeoff_height_m`, and their `face_partner` yaws differ by `pi`.

- [ ] **Step 5: Run the trajectory tests and verify the expected failure**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_ctbr_trajectory_smoothstep.py -q`

Expected: FAIL because the trajectory configuration has no per-vehicle phase or partner-facing yaw.

- [ ] **Step 6: Implement phase-aware circular references**

Extend only the circular reference calculations; do not change the quintic smoothstep derivatives. Keep the existing absolute-center behavior for single-aircraft callers by defaulting `circle_center_xy` to the current start-position offset result.

- [ ] **Step 7: Update YAML defaults**

Configure cf2 with URI `radio://0/80/2M/E7E7E7E702`, phase `0.0`, and cf4 with URI `radio://0/80/2M/E7E7E7E704`, phase `3.141592653589793`, both `ctbr_enabled: true`, `type: default`; set the global circle center to `[0.0, 0.0]` and retain radius `1.0`.

- [ ] **Step 8: Run Task 1 tests**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_vehicle_config.py ros_ws/src/crazyswarm/scripts/test_ctbr_trajectory_smoothstep.py -q`

Expected: PASS, with all existing smoothstep and height-correction tests still green.

---

### Task 2: Introduce per-vehicle runtime state and synchronized multi-vehicle control

**Files:**
- Modify: `ros_ws/src/crazyswarm/scripts/ctbr_controller.py`
- Modify: `ros_ws/src/crazyswarm/launch/ctbr_controller.launch`
- Test: `ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py`
- Test: `ros_ws/src/crazyswarm/scripts/test_vehicle_config.py`

**Interfaces:**
- `VehicleRuntime` stores `vehicle_id`, `uri`, `prefix`, config, trajectory, geometric controller, latest state/EKF/battery, preflight status, path history and command publisher.
- `CtbrControllerNode._load_vehicle_runtimes() -> dict[int, VehicleRuntime]` creates one runtime per enabled YAML entry.
- `CtbrControllerNode._timer_callback(event)` advances one global mission clock and calls `_step_vehicle(runtime, mission_now)` for every runtime.
- `CtbrControllerNode._all_preflight_ready() -> bool` gates mission start.
- `CtbrControllerNode._abort_all(reason)` publishes zero CTBR to every runtime and records the shared reason.

- [ ] **Step 1: Write failing multi-runtime tests**

Add ROS-stub tests for: two enabled entries produce two runtime objects and two `/cf<ID>/cmd_ctbr` publishers; mission start remains false until both preflight flags are ready; an invalid state in one runtime makes `_abort_all` publish zero commands to both; explicit `~cf_id` still selects one vehicle.

- [ ] **Step 2: Run the controller tests and verify the expected failure**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py -q`

Expected: FAIL because the node currently owns only `self.vehicle_config`, one timer path and one command publisher.

- [ ] **Step 3: Refactor single-vehicle initialization into `VehicleRuntime`**

Move per-aircraft fields currently stored on `CtbrControllerNode` into the runtime object without changing callback semantics. Keep `GeometricCtbrController.compute`, `blend_kinematic_feedback`, velocity filtering, battery preflight and all safety limits unchanged.

- [ ] **Step 4: Register per-vehicle subscribers and publishers**

For each runtime subscribe to `/cf<ID>/mocap_state`, `/cf<ID>/ekf_kinematics`, `/cf<ID>/battery`; callbacks capture the runtime id and reject data for other ids. Create `/cf<ID>/cmd_ctbr` and `/cf<ID>/path` publishers with queue size 1/latch behavior matching the existing node.

- [ ] **Step 5: Implement global mission gating and synchronized stepping**

Use one `mission_start_time` and one ROS timer. Before start, publish zero CTBR for every runtime. Once all runtime preflight checks and mocap states are valid, call each trajectory with the same mission elapsed time. Each runtime keeps independent integrators and derivative filters.

- [ ] **Step 6: Implement all-aircraft abort and shutdown**

Any state timeout, invalid quaternion, position bound violation, or trajectory exception calls `_abort_all(reason)`; subsequent timer ticks keep all outputs zero. Shutdown sends zero CTBR to all publishers, closes the logger, and publishes final Path messages.

- [ ] **Step 7: Keep legacy single-aircraft selection**

If `~cf_id` is present, create only that runtime and retain the old single-aircraft log prefix. If it is absent, create all `ctbr_enabled` runtimes and require at least two for formation mode.

- [ ] **Step 8: Run controller tests and syntax checks**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py -q` and `python3 -m py_compile ros_ws/src/crazyswarm/scripts/ctbr_controller.py ros_ws/src/crazyswarm/scripts/ctbr_trajectory.py`

Expected: PASS and exit code 0.

---

### Task 3: Implement one task-level merged CSV logger

**Files:**
- Modify: `ros_ws/src/crazyswarm/scripts/ctbr_controller.py`
- Test: `ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py`

**Interfaces:**
- `MultiFlightCsvLogger(directory, vehicle_metadata) -> logger` creates `multi_ctbr_YYYYmmdd_HHMMSS.csv`.
- `MultiFlightCsvLogger.write(runtime, now, target, command, control_state, formation_state, abort_reason)` writes one row for one vehicle.
- `MultiFlightCsvLogger.path` exposes the complete CSV path for the startup log.
- Existing `FlightCsvLogger.FIELDS` remains the base schema; merged fields append `mission_time_s`, `vehicle_id`, `radio_uri`, `orbit_phase_rad`, `formation_state`, and `global_abort_reason`.

- [ ] **Step 1: Write failing merged-log tests**

Create two synthetic runtimes and write one cycle for each. Assert one header, two data rows, both ids and URIs, identical mission time, and preservation of `position_error_*`, `desired_force_*`, EKF and CTBR fields. Assert an invalid runtime row contains NaN in only that runtime’s state columns.

- [ ] **Step 2: Run the logger tests and verify the expected failure**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py -q -k merged_log`

Expected: FAIL because the controller creates one `FlightCsvLogger` per process and has no formation metadata.

- [ ] **Step 3: Implement merged logger and row assembly**

Reuse `_base_log_row`, `_with_xyz`, and existing command fields. Add metadata columns without changing units or names of the existing 91 columns. Flush after each global cycle so visualization can read an active log safely.

- [ ] **Step 4: Wire the logger into synchronized stepping**

Write exactly one row per vehicle per timer cycle, including preflight/hold/aborted rows. Record the same `global_abort_reason` for all runtimes after an all-aircraft abort.

- [ ] **Step 5: Run logger and controller tests**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py -q`

Expected: PASS.

---

### Task 4: Extend CSV visualization for multiple vehicles

**Files:**
- Modify: `ros_ws/src/crazyswarm/scripts/ctbr_visualization.py`
- Modify: `ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py`

**Interfaces:**
- `load_multi_log(path) -> dict[int, dict[str, np.ndarray]]` groups rows by `vehicle_id`; legacy files without that column return one group.
- `load_plot_data(log_path, max_abs_position_m, vehicle_id=None)` returns either one selected group or all groups.
- `create_figure(pyplot, vehicle_ids=None)` creates per-vehicle line sets while preserving separate velocity and acceleration axes.
- `update_figure(handles, data, title)` updates all vehicle lines and legends.
- CLI adds `--vehicle-id INTEGER`; default renders all vehicles from a merged CSV.

- [ ] **Step 1: Write failing multi-CSV visualization tests**

Generate a temporary merged CSV with two vehicles and assert grouping returns two ids, each with its own position/target/error arrays, and `vehicle_id` filtering returns only the requested group. Assert legacy single CSV still returns one group.

- [ ] **Step 2: Run the visualization tests and verify the expected failure**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py -q`

Expected: FAIL because `load_plot_data` currently assumes one vehicle and no `vehicle_id` column.

- [ ] **Step 3: Implement grouping and selected-vehicle loading**

Group rows by integer `vehicle_id`, use `mission_time_s` when present and `control_time_s` otherwise, preserve invalid-sample gaps per vehicle, and keep all existing attitude trace, velocity and acceleration calculations.

- [ ] **Step 4: Implement multi-line figure updates**

Assign stable colors by sorted vehicle id; use solid lines for measured values, dashed lines for target/reference values, and legends labeled `cf<ID>`. Keep the existing five-panel layout and separate velocity/acceleration plots.

- [ ] **Step 5: Update real-time file selection**

Make `latest_ctbr_log` prefer `multi_ctbr_*.csv`; real-time refresh follows that one file rather than switching between per-vehicle files. Explicit positional CSV paths remain supported.

- [ ] **Step 6: Run visualization tests and headless static plotting**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py -q` and `MPLBACKEND=Agg python3 ros_ws/src/crazyswarm/scripts/ctbr_visualization.py ros_ws/src/crazyswarm/scripts/ctbr_logs/cf2_ctbr_20260904_231538.csv --static --no-show --output /tmp/ctbr_single.png`

Expected: PASS and a generated PNG for the legacy single-aircraft CSV.

---

### Task 5: Add RViz Path displays and launch configuration

**Files:**
- Modify: `ros_ws/src/crazyswarm/launch/test.rviz`
- Modify: `ros_ws/src/crazyswarm/launch/ctbr_controller.launch`
- Modify: `ros_ws/src/crazyswarm/CMakeLists.txt`
- Test: `ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py`

**Interfaces:**
- Each runtime publishes a `nav_msgs/Path` on `/cf<ID>/path` in frame `world`.
- Launch loads all `crazyflies.yaml` entries once and starts one multi-vehicle controller node.
- RViz contains one Path display per configured current vehicle and uses distinct colors.

- [ ] **Step 1: Write a launch/configuration regression check**

Add a lightweight test that parses `crazyflies.yaml`, checks every enabled id has a corresponding `/cf<ID>/path` topic string in the RViz config, and checks `ctbr_controller.launch` starts exactly one controller node.

- [ ] **Step 2: Run the check and verify the expected failure**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py -q -k path`

Expected: FAIL because RViz currently contains only `/cf2/path` and the configuration is single-aircraft.

- [ ] **Step 3: Update RViz Path displays**

Keep `/cf2/path`, add `/cf4/path` with a distinct color, and document that future ids require adding a display. Retain the existing TF, point cloud and world frame settings.

- [ ] **Step 4: Update launch comments/parameters**

Change the launch comments to describe multi-vehicle selection, keep one `ctbr_controller.py` node, and avoid adding extra radio/server processes.

- [ ] **Step 5: Ensure install targets include all changed runtime files**

Keep the existing script/config install entries and add no generated artifacts. Verify `CMakeLists.txt` installs the controller, trajectory, visualization and `vehicle_config.py` used by the launch.

- [ ] **Step 6: Run configuration checks**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py -q -k path` and `git diff --check`.

Expected: PASS with no whitespace errors.

---

### Task 6: End-to-end offline verification and handoff

**Files:**
- Test: `ros_ws/src/crazyswarm/scripts/test_vehicle_config.py`
- Test: `ros_ws/src/crazyswarm/scripts/test_ctbr_trajectory_smoothstep.py`
- Test: `ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py`
- Test: `ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py`

- [ ] **Step 1: Run all Python tests**

Run: `python3 -m pytest ros_ws/src/crazyswarm/scripts/test_vehicle_config.py ros_ws/src/crazyswarm/scripts/test_ctbr_trajectory_smoothstep.py ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py -q`

Expected: PASS with zero failures.

- [ ] **Step 2: Compile all modified Python scripts**

Run: `python3 -m py_compile ros_ws/src/crazyswarm/scripts/vehicle_config.py ros_ws/src/crazyswarm/scripts/ctbr_trajectory.py ros_ws/src/crazyswarm/scripts/ctbr_controller.py ros_ws/src/crazyswarm/scripts/ctbr_visualization.py`

Expected: exit code 0.

- [ ] **Step 3: Validate the final YAML and required multi-vehicle invariants**

Run: `python3 - <<'PY'\nimport yaml\nfrom pathlib import Path\nfrom ros_ws.src.crazyswarm.scripts.vehicle_config import validate_vehicle_entries\nentries = yaml.safe_load(Path('ros_ws/src/crazyswarm/launch/crazyflies.yaml').read_text())['crazyflies']\nselected = validate_vehicle_entries(entries)\nassert len(selected) >= 2\nassert {int(e['id']) for e in selected} >= {2, 4}\nprint('validated ids:', [int(e['id']) for e in selected])\nPY`

Expected: prints validated ids including 2 and 4.

- [ ] **Step 4: Review diff and preserve unrelated user changes**

Run: `git diff --stat` and `git status --short`; verify only the planned files changed in this feature and do not reset or overwrite existing unrelated work.

- [ ] **Step 5: Commit implementation**

Run: `git add ros_ws/src/crazyswarm/config/ctbr_controller.yaml ros_ws/src/crazyswarm/launch/crazyflies.yaml ros_ws/src/crazyswarm/launch/ctbr_controller.launch ros_ws/src/crazyswarm/launch/test.rviz ros_ws/src/crazyswarm/scripts/vehicle_config.py ros_ws/src/crazyswarm/scripts/ctbr_trajectory.py ros_ws/src/crazyswarm/scripts/ctbr_controller.py ros_ws/src/crazyswarm/scripts/ctbr_visualization.py ros_ws/src/crazyswarm/scripts/test_vehicle_config.py ros_ws/src/crazyswarm/scripts/test_ctbr_trajectory_smoothstep.py ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py ros_ws/src/crazyswarm/scripts/test_ctbr_visualization.py ros_ws/src/crazyswarm/CMakeLists.txt && git commit -m "feat: add multi-vehicle CTBR circular formation"`

