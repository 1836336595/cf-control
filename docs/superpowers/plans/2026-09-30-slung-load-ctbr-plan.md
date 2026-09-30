# MATLAB 吊运 CTBR 接入实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 接入 `load` 负载刚体、负载几何中心换算和 MATLAB 三机吊运 CTBR 控制链路。

**Architecture:** 服务端发布 `/load/mocap_state` 原始刚体状态；Python 端将上表面中心换算为几何中心，并由独立的 slung-load 模块完成负载外环、张力分配和绳向控制。现有起飞/降落状态机继续负责安全阶段，运输阶段切换到吊运控制器。

**Tech Stack:** ROS1、C++11、Python 3、NumPy、YAML、现有 Crazyswarm CTBR 消息。

**Spec:** `docs/superpowers/specs/2026-09-30-slung-load-ctbr-design.md`

## Global Constraints

- 负载刚体名称固定为 `load`。
- 负载刚体原点是上表面中心；几何中心偏移由负载高度计算。
- 三机列顺序固定为 CF3、CF4、CF5。
- ROS z-up 与 MATLAB z-down 的转换只能在边界层处理一次。
- 负载状态无效时不得发布正常运输推力。

---

### Task 1: Add regression tests for payload geometry and mapping

**Files:**
- Modify: `ros_ws/src/crazyswarm/scripts/test_ctbr_controller_v2.py`
- Create: `ros_ws/src/crazyswarm/scripts/test_slung_load_controller.py`

- [ ] Test raw top-surface pose to geometric-center conversion.
- [ ] Test fixed CF3/CF4/CF5 attachment ordering.
- [ ] Test z-down to z-up attachment conversion.
- [ ] Test invalid payload state prevents transport output.

### Task 2: Add payload and vehicle YAML configuration

**Files:**
- Create: `ros_ws/src/crazyswarm/config/slung_payload.yaml`
- Create: `ros_ws/src/crazyswarm/config/ctbr_vehicle.yaml`
- Modify: `ros_ws/src/crazyswarm/launch/ctbr_controller.launch`
- Modify: `ros_ws/src/crazyswarm/launch/crazyflies.yaml`

- [ ] Move MATLAB payload constants into the payload file.
- [ ] Add `payload_rigid_body: load` and payload topic settings.
- [ ] Add explicit CF3/CF4/CF5 attachment indices and retain project-specific vehicle calibration.
- [ ] Load all YAML files in a deterministic order.

### Task 3: Publish the load MocapState from the server

**Files:**
- Modify: `ros_ws/src/crazyswarm/src/crazyswarm_server.cpp`
- Modify: `ros_ws/src/crazyswarm/CMakeLists.txt`
- Modify: `ros_ws/src/crazyswarm/package.xml`

- [ ] Read the configured payload rigid-body name.
- [ ] Advertise `/load/mocap_state`.
- [ ] Publish valid and invalid states using the existing estimator.
- [ ] Keep Crazyflie external-pose broadcasting unchanged.

### Task 4: Implement the MATLAB slung-load controller

**Files:**
- Create: `ros_ws/src/crazyswarm/scripts/slung_load_controller.py`
- Modify: `ros_ws/src/crazyswarm/scripts/ctbr_controller.py`

- [ ] Implement payload geometry conversion and attachment mapping.
- [ ] Implement payload position and attitude outer loops.
- [ ] Implement rank-checked tension allocation.
- [ ] Implement link-direction control and per-vehicle force construction.
- [ ] Convert the MATLAB z-down force equations into ROS z-up exactly once.
- [ ] Use the existing geometric CTBR attitude/rate output path with the resulting per-vehicle force.

### Task 5: Integrate state machine, logging, and visualization

**Files:**
- Modify: `ros_ws/src/crazyswarm/scripts/ctbr_controller.py`
- Modify: `ros_ws/src/crazyswarm/scripts/ctbr_visualization.py`
- Modify: `README.md`

- [ ] Gate transport phase on valid load state.
- [ ] Add payload and per-link diagnostics to CSV.
- [ ] Preserve existing single-CF and non-transport modes.
- [ ] Document the `load` rigid body and required launch order.

### Task 6: Verification

**Files:**
- Modify: relevant test files only if failures expose missing coverage.

- [ ] Run all existing Python behavioral tests.
- [ ] Run new slung-load tests.
- [ ] Run Python compilation and YAML parsing.
- [ ] Build the C++ workspace target if ROS/catkin is available.
- [ ] Verify no stale `path_max_poses` or old formation-only assumptions remain in the transport path.

