#!/usr/bin/env python3
"""Offline MuJoCo validation of the real mission/controller, without ROS/radio.

Only ROS I/O is stubbed by the existing test utilities. Mission logic, payload
pose observer and CTBR control laws come from ctbr_controller.py. Sampled ideal
EKF velocity passes through the real host filter. Full firmware EKF is not modeled.
"""

import argparse
from collections import deque
import csv
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import yaml

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from slung_load_controller import payload_ground_clearance
from slung_simulated_telemetry import SampledEkfTelemetry


def resolve_simulation_settings(payload, pose_delay_s=None, command_delay_s=None,
                                time_limit_s=None):
    settings = dict(
        pose_delay_s=.01, command_delay_s=.005, pose_sample_rate_hz=100.0,
        ekf_feedback_mode="sampled_filtered", ekf_sample_rate_hz=50.0,
        ekf_delay_s=.02, time_limit_s=180.0,
        payload_rotation_noise_std_rad=[0.,0.,0.], pose_noise_seed=20261016,
    )
    settings.update(payload.get("simulation", {}))
    for name, value in (("pose_delay_s", pose_delay_s),
                        ("command_delay_s", command_delay_s),
                        ("time_limit_s", time_limit_s)):
        if value is not None:
            settings[name] = float(value)
    for name in ("pose_delay_s", "command_delay_s", "ekf_delay_s"):
        if not np.isfinite(settings[name]) or settings[name] < 0:
            raise ValueError(name + " must be finite and nonnegative")
    for name in ("pose_sample_rate_hz", "ekf_sample_rate_hz", "time_limit_s"):
        if not np.isfinite(settings[name]) or settings[name] <= 0:
            raise ValueError(name + " must be finite and positive")
    if settings["ekf_feedback_mode"] not in ("sampled_filtered", "delayed_truth"):
        raise ValueError("ekf_feedback_mode must be sampled_filtered or delayed_truth")
    noise = np.asarray(settings["payload_rotation_noise_std_rad"], dtype=float)
    if noise.shape != (3,) or not np.all(np.isfinite(noise)) or np.any(noise < 0):
        raise ValueError("payload_rotation_noise_std_rad must contain three finite nonnegative values")
    settings["payload_rotation_noise_std_rad"] = noise.tolist()
    return settings


def payload_rotation_measurement(rotation, noise_std, rng):
    """Perturb only the observed attitude, keeping physical top-origin position."""
    rotation = np.asarray(rotation, dtype=float)
    std = np.asarray(noise_std, dtype=float)
    if not np.any(std):
        return rotation.copy()
    vector = rng.normal(size=3) * std
    angle = np.linalg.norm(vector)
    skew = np.array([[0.,-vector[2],vector[1]], [vector[2],0.,-vector[0]], [-vector[1],vector[0],0.]])
    perturbation = (np.eye(3) + skew if angle < 1e-12 else
                    np.eye(3) + np.sin(angle)/angle*skew + (1-np.cos(angle))/angle**2*(skew@skew))
    return rotation @ perturbation


def observation_window_failure(handoff_time, final_time, required_seconds):
    if handoff_time is None:
        return "handoff_timeout"
    if final_time - handoff_time < required_seconds - 1e-9:
        return "post_handoff_window_incomplete"
    return None


def is_planned_validation_landing(enabled, landing_started_time, reasons):
    """Only the latched, explicitly named three-vehicle validation stop is planned."""
    return bool(
        enabled and landing_started_time is not None
        and np.isfinite(landing_started_time) and len(reasons) == 3
        and all(str(reason).startswith("payload_load: flight_validation_complete:")
                for reason in reasons)
    )


def validation_terminal_failure(phases):
    """A planned normal landing never excuses an emergency or abort phase."""
    if "aborted" in phases:
        return "aborted"
    if "emergency_landing" in phases:
        return "emergency_landing"
    return None


def is_expected_controlled_fault_descent(enabled, reason, phases, landing_started_time,
                                         payload_fresh):
    """Opt-in test continuation for one fresh height-fault supported descent."""
    return bool(
        enabled and payload_fresh and landing_started_time is not None
        and np.isfinite(landing_started_time)
        and str(reason).startswith("payload_load: height_drop:")
        and len(phases) == 3
        and all(phase in ("landing", "landing_settle", "landed") for phase in phases)
    )


def validation_window_failure(handoff_time, landing_started_time, required_seconds, completed):
    if handoff_time is None:
        return "handoff_timeout"
    if landing_started_time is None:
        return "validation_landing_not_started"
    if landing_started_time - handoff_time < required_seconds - 1e-9:
        return "validation_transport_window_incomplete"
    if not completed:
        return "validation_landing_incomplete"
    return None


def set_validation_ground_reference(trajectories, ground_center_z):
    """Correct replay-only ground origins after reset at airborne TAKEUP anchors."""
    for trajectory in trajectories:
        trajectory.start_position[2] = float(ground_center_z)


class ValidationLandingMonitor:
    """Require physical touchdown and continuous settled zero-thrust commands."""

    def __init__(self, ground_center_z, ground_tolerance_m=.03,
                 vertical_velocity_limit_mps=.15, zero_hold_s=.5):
        self.ground_center_z = float(ground_center_z)
        self.ground_tolerance_m = float(ground_tolerance_m)
        self.vertical_velocity_limit_mps = float(vertical_velocity_limit_mps)
        self.zero_hold_s = float(zero_hold_s)
        self.ready_since = None
        self.completed = False

    def update(self, now, states, thrusts):
        try:
            positions = np.asarray([state["position"] for state in states], dtype=float)
            velocities = np.asarray([state["velocity"] for state in states], dtype=float)
            thrusts = np.asarray(thrusts, dtype=float)
            ready = bool(
                np.isfinite(now) and positions.shape == (3, 3)
                and velocities.shape == (3, 3) and thrusts.shape == (3,)
                and np.all(np.isfinite(positions)) and np.all(np.isfinite(velocities))
                and np.all(np.isfinite(thrusts)) and np.all(thrusts == 0.0)
                and np.all(np.abs(positions[:, 2] - self.ground_center_z)
                           <= self.ground_tolerance_m)
                and np.all(np.abs(velocities[:, 2]) < self.vertical_velocity_limit_mps)
            )
        except (KeyError, TypeError, ValueError):
            ready = False
        if not ready:
            self.ready_since = None
            self.completed = False
            return False
        if self.ready_since is None or now < self.ready_since:
            self.ready_since = float(now)
        self.completed = now - self.ready_since >= self.zero_hold_s - 1e-9
        return self.completed


PLANT_SETTING_NAMES = (
    "onboard_controller", "motor_model", "vehicle_inertia", "arm_length",
    "thrust_time_constant", "motor_thrust_efficiency", "motor_yaw_torque_coefficient",
    "brushless_motor_time_constant_s", "brushless_motor_speed_gain_radps",
    "brushless_speed_normalization_radps", "brushless_rotor_inertia_kgm2",
    "brushless_thrust_coefficients", "brushless_torque_coefficients",
    "bridge_collective_pwm_reference", "bridge_collective_pwm_limit",
    "bridge_thrust_curve_exponent", "bridge_thrust_raw_scale", "motor_pwm_max",
    "motor_available_command_fraction", "mellinger_rate_hz",
    "mellinger_attitude_gains", "mellinger_rate_gains", "mellinger_rate_derivative_gain",
    "mellinger_yaw_integral_gain", "mellinger_yaw_integral_limit",
    "onboard_pid_rate_hz", "onboard_pid_rate_kp", "onboard_pid_rate_ki",
    "onboard_pid_rate_kd", "onboard_pid_rate_integral_limit", "onboard_pid_yaw_gains",
    "onboard_pid_yaw_integral_limit", "payload_rotational_damping",
)


def build_physics_config(base, manager, simulation, rate_bandwidth):
    """Resolve plant assumptions separately from the unchanged flight controller."""
    settings = {}
    for name in PLANT_SETTING_NAMES:
        default = getattr(base, name)
        value = simulation.get(name, default)
        settings[name] = (tuple(float(item) for item in value) if isinstance(default, tuple)
                          else str(value) if isinstance(default, str) else float(value))
    cfg = manager.slung_controller.config
    physical_lengths = np.asarray(simulation.get("physical_link_lengths_m",cfg.link_lengths_m),dtype=float)
    if physical_lengths.shape != (3,) or not np.all(np.isfinite(physical_lengths)) or np.any(physical_lengths <= 0):
        raise ValueError("physical_link_lengths_m must contain three finite positive lengths")
    return replace(
        base, **settings, control_rate_hz=manager.rate_hz,
        rate_loop_bandwidth=tuple(rate_bandwidth),
        vehicles=tuple(replace(v, mass=n.controller.config.mass,
                               max_total_thrust=n.controller.config.max_total_thrust,
                               max_command_thrust=n.controller.config.max_command_thrust)
                       for v, n in zip(base.vehicles, manager.vehicles)),
        cable_lengths=tuple(float(value) for value in physical_lengths), payload_mass=cfg.payload_mass_kg,
        payload_size=tuple(cfg.payload_size_m),
        attachment_points=tuple(map(tuple, cfg.attachment_points_m.T)),
    )


def actuator_trace_fields(model, index):
    # This is the physical state at the row's sample instant. The newly
    # computed command will enter the plant after command_delay_s; keep the
    # held/applied command alongside it rather than implying instant response.
    result = {"actual_thrust_n": float(model._actual_thrust[index]),
              "motor_model": model.config.motor_model,
              "actuator_state_time_s": float(model.data.time) if hasattr(model, "data") else float("nan"),
              "applied_thrust_command_n": float("nan"),
              "motor_at_upper_limit_count": 0,
              "collective_raw_thrust": float("nan")}
    actuator = model._onboard_actuators[index] if model._onboard_actuators else None
    if actuator is not None:
        result["collective_raw_thrust"] = int(actuator.raw_collective)
        result["applied_thrust_command_n"] = float(getattr(actuator, "applied_thrust_command", float("nan")))
        result["motor_at_upper_limit_count"] = int(np.sum(
            actuator.motor_pwm >= getattr(actuator, "motor_pwm_limit", 65535)))
    for prefix, attribute in (("motor_pwm", "motor_pwm"), ("motor_speed_radps", "motor_speeds"),
                              ("motor_force_n", "motor_forces"),
                              ("motor_reaction_torque_nm", "motor_reaction_torques")):
        values = getattr(actuator, attribute) if actuator is not None else np.full(4, np.nan)
        result.update({prefix + "_" + str(i + 1): float(value) for i, value in enumerate(values)})
    return result


def run_simulation(directory, parameter_overrides=None, pose_delay_s=None,
                   command_delay_s=None, rate_bandwidth=(35.0, 35.0, 20.0),
                   post_handoff_s=None, time_limit_s=None, initial_tilt_deg=0.0,
                   mocap_rate_alpha=0.2, render=False, record=None,
                   target_payload_height_m=None, verify_controlled_fault_descent=False):
    """Return measurable handoff outcomes and save truth + real controller CSVs."""
    import mujoco

    sys.path.insert(0, str(Path(__file__).resolve().parents[5] / "simulate_uav" / "udaan"))
    from udaan.crazyswarm_slung_parameters import default_config
    from udaan.models.mujoco.crazyswarm_slung import CrazySwarmSlungModel
    from udaan.control.crazyswarm_slung import VehicleCommand

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    module = _controller_module()
    original_time = module.time.monotonic
    manager, anchors = _slung_manager(module, directory, parameter_overrides)
    validation_enabled = bool(manager.payload_flight_validation_enabled)
    validation_transport_s = float(manager.payload_flight_validation_transport_s)
    simulation = resolve_simulation_settings(
        module.rospy.get_param("/slung_payload"), pose_delay_s, command_delay_s, time_limit_s,
    )
    pose_delay_s = float(simulation["pose_delay_s"])
    command_delay_s = float(simulation["command_delay_s"])
    time_limit_s = float(simulation["time_limit_s"])
    if post_handoff_s is None:
        post_handoff_s = float(module.rospy.get_param(
            "/slung_payload/post_handoff_s", 60.0
        ))
    if not np.isfinite(post_handoff_s) or post_handoff_s <= 0.0:
        raise ValueError("post_handoff_s must be positive and finite")
    # The replay's observation window owns the end time. Prevent the normal
    # mission trajectory (which defaults to a 30 s hover) from entering its
    # landing phase before the requested post-handoff window has elapsed.
    for vehicle in manager.vehicles:
        if vehicle.trajectory_config is not None:
            vehicle.trajectory_config.hover_duration_s = max(
                float(vehicle.trajectory_config.hover_duration_s), post_handoff_s + 1.0
            )
        if vehicle.trajectory is not None:
            vehicle.trajectory.config.hover_duration_s = max(
                float(vehicle.trajectory.config.hover_duration_s), post_handoff_s + 1.0
            )
    if target_payload_height_m is None:
        configured_height = module.rospy.get_param(
            "/slung_payload/target_payload_height_m", None
        )
        if configured_height is not None:
            target_payload_height_m = float(configured_height)
    # Begin with the configured spaced equilibrium geometry; the previous
    # 12 cm radial fallback placed CF4/CF5 only ~24 cm apart at initialization.
    anchors = manager._payload_takeup_positions()
    for vehicle, position in zip(manager.vehicles, anchors):
        vehicle.trajectory.reset(position, 0.0, 0.0)
        vehicle.trajectory.phase = "height_correction"
        vehicle.trajectory.phase_start_time = 0.0
    if target_payload_height_m is not None:
        target_payload_height_m = float(target_payload_height_m)
        if not np.isfinite(target_payload_height_m) or target_payload_height_m <= 0.0:
            raise ValueError("target_payload_height_m must be positive and finite")
        # The manager computes its final payload reference from each aircraft's
        # takeoff_position minus the cable length. The aircraft already start
        # at taut TAKEUP anchors in this replay, so overwrite only that final
        # reference; otherwise takeoff_height_m is added a second time.
        for vehicle, cable_length in zip(
                manager.vehicles, manager.slung_controller.config.link_lengths_m):
            target_z = target_payload_height_m + float(cable_length)
            vehicle.trajectory.takeoff_position[2] = target_z
            vehicle.trajectory.height_correction_position[2] = target_z
            vehicle.trajectory.circle_entry_start_position[2] = target_z
    snapshot = {root: module.rospy.get_param("/" + root) for root in (
        "ctbr_controller", "ctbr_trajectory", "slung_payload",
        "ctbr_controller_cf3", "ctbr_controller_cf4", "ctbr_controller_cf5",
    ) if module.rospy.has_param("/" + root)}
    snapshot["simulation_runtime"] = dict(
        simulation, post_handoff_s=float(post_handoff_s),
        target_payload_height_m=target_payload_height_m,
        effective_hover_duration_s=float(manager.vehicles[0].trajectory.config.hover_duration_s),
        ekf_host_filter_cutoff_hz=float(manager.vehicles[0].ekf_velocity_filter.cutoff_hz),
        ekf_host_filter_max_dt_s=float(manager.vehicles[0].ekf_velocity_filter.max_dt),
    )
    base = default_config()
    config = build_physics_config(base, manager, simulation, rate_bandwidth)
    validation_ground_center_z = float(config.body_size[2]) / 2.0
    if validation_enabled or verify_controlled_fault_descent:
        set_validation_ground_reference(
            [vehicle.trajectory for vehicle in manager.vehicles], validation_ground_center_z)
    snapshot["simulation_runtime"].update(
        validation_enabled=validation_enabled,
        validation_transport_s=validation_transport_s,
        verify_controlled_fault_descent=bool(verify_controlled_fault_descent),
        validation_ground_reference_assumption=(
            "trajectory.start_position.z uses the physical MuJoCo ground body center; "
            "TAKEUP positions and targets are preserved"
            if validation_enabled or verify_controlled_fault_descent else None),
        validation_ground_center_z_m=(validation_ground_center_z
                                    if validation_enabled or verify_controlled_fault_descent else None),
        validation_zero_hold_s=.5,
        validation_ground_tolerance_m=.03,
        validation_vertical_velocity_limit_mps=.15,
    )
    snapshot["simulation_runtime"]["effective_plant"] = {
        name: asdict(config)[name] for name in PLANT_SETTING_NAMES
    }
    snapshot["simulation_runtime"]["effective_plant"]["vehicle_masses_kg"] = config.vehicle_masses.tolist()
    snapshot["simulation_runtime"]["effective_plant"]["thrust_reference_n"] = [v.max_total_thrust for v in config.vehicles]
    snapshot["simulation_runtime"]["effective_plant"]["physical_link_lengths_m"] = list(config.cable_lengths)
    (directory / "input_parameters.yaml").write_text(yaml.safe_dump(snapshot, sort_keys=False))
    for index, vehicle in enumerate(manager.vehicles):
        vehicle.thrust_raw_scale = config.bridge_thrust_raw_scale[index]
    model = CrazySwarmSlungModel(
        render=render,
        record=str(record) if record is not None else None,
        config=config,
    )
    clock_time = 0.0
    original_ros_time = module.rospy.Time.now
    original_logerr = getattr(module.rospy, "logerr", None)
    module.time.monotonic = lambda: clock_time
    module.rospy.Time.now = lambda: SimpleNamespace(to_sec=lambda: 100.0 + clock_time)
    module.rospy.logerr = lambda *_args: None
    manager._publish_payload_path = lambda _state: None
    manager.fleet_gate_open = True
    manager.last_tick = 0.0
    pose_queue, command_queue = deque(), deque()
    next_pose_time = 0.0
    pose_period = 1.0 / float(simulation["pose_sample_rate_hz"])
    ekf_mode = simulation["ekf_feedback_mode"]
    ekf_link = (SampledEkfTelemetry(simulation["ekf_sample_rate_hz"], simulation["ekf_delay_s"])
                if ekf_mode == "sampled_filtered" else None)
    last_pose_time = 0.0
    pose_noise_rng = np.random.default_rng(int(simulation["pose_noise_seed"]))
    handoff_time = None
    minimum_clearance = float("inf")
    maximum_xy_error = 0.0
    maximum_body_rate = 0.0
    maximum_tilt = 0.0
    minimum_vehicle_distance = float("inf")
    saturation_ticks = post_ticks = 0
    phases = []
    failure = None
    terminal_reason = ""
    validation_landing_started_time = None
    controlled_fault_landing_started_time = None
    validation_monitor = ValidationLandingMonitor(validation_ground_center_z)
    trace = []
    commands = [VehicleCommand(v.mass * 9.80665, np.zeros(3),
                               np.array([0.0, 0.0, v.mass * 9.80665]))
                for v in config.vehicles]

    previous_pose = None
    filtered_rates = np.zeros((3, 3))

    def sample_pose():
        nonlocal previous_pose
        sampled = float(model.data.time)
        states = model.vehicle_states()
        if previous_pose is not None:
            previous_time, previous_states = previous_pose
            dt = sampled - previous_time
            for i, (state, old) in enumerate(zip(states, previous_states)):
                # Match the real server: body-frame quaternion increment,
                # followed by the alpha=0.2 causal low-pass at the pose rate.
                relative = old["rotation"].T @ state["rotation"]
                quat = np.empty(4)
                mujoco.mju_mat2Quat(quat, relative.ravel())
                if quat[0] < 0:
                    quat *= -1
                sine_half = np.linalg.norm(quat[1:])
                raw = (2*quat[1:]/dt if sine_half < 1e-6 else
                       quat[1:]/sine_half * (2*np.arctan2(sine_half, quat[0])/dt))
                if previous_time == 0.0:
                    filtered_rates[i] = raw
                else:
                    filtered_rates[i] += mocap_rate_alpha * (raw-filtered_rates[i])
                state["body_rate"] = filtered_rates[i].copy()
        previous_pose = sampled, states
        pose_queue.append((sampled, model.payload_state(), states))

    def deliver_poses(now):
        nonlocal last_pose_time
        while pose_queue and pose_queue[0][0] + pose_delay_s <= now + 1e-10:
            sampled, payload, states = pose_queue.popleft()
            last_pose_time = sampled
            quat = np.empty(4)
            observed_rotation = payload_rotation_measurement(
                payload["rotation"], simulation["payload_rotation_noise_std_rad"], pose_noise_rng)
            mujoco.mju_mat2Quat(quat, observed_rotation.ravel())
            top = payload["position"] + payload["rotation"] @ np.array([0, 0, config.payload_size[2] / 2])
            vector = lambda v: SimpleNamespace(x=v[0], y=v[1], z=v[2])
            manager._payload_state_callback(SimpleNamespace(
                valid=True, derivatives_valid=True,
                header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 100.0 + sampled)),
                pose=SimpleNamespace(position=vector(top), orientation=SimpleNamespace(
                    w=quat[0], x=quat[1], y=quat[2], z=quat[3])),
                twist=SimpleNamespace(linear=vector(payload["velocity"]), angular=vector(payload["body_rate"])),
                acceleration=vector(payload["acceleration"]),
            ))
            for vehicle, state in zip(manager.vehicles, states):
                vehicle.latest_state = dict(
                    state, valid=True, derivatives_valid=True, filter_derivatives_valid=True,
                    filtered_velocity=state["velocity"], filtered_acceleration=state["acceleration"],
                    received_time=sampled + pose_delay_s, mocap_sample_time_s=100.0 + sampled,
                )
                if ekf_mode == "delayed_truth":
                    vehicle.latest_ekf_state = dict(
                        position=state["position"], filtered_velocity=state["velocity"],
                        filtered_acceleration=state["acceleration"], filter_derivatives_valid=True,
                        received_time=sampled + pose_delay_s,
                    )

    try:
        for i, (qpos, vehicle) in enumerate(zip(model._vehicle_qpos, manager.vehicles)):
            model.data.qpos[qpos:qpos + 3] = anchors[i]
            angle = np.deg2rad(initial_tilt_deg) * (1 if i % 2 == 0 else -1)
            model.data.qpos[qpos + 3:qpos + 7] = [np.cos(angle / 2), np.sin(angle / 2), 0, 0]
            vehicle.preflight_voltage_ready = True
            vehicle.last_tick = vehicle.start_time = 0.0
            vehicle._publish_path = lambda _state: None
            vehicle._publish_command = lambda _command: None
            vehicle._publish_zero = lambda: None
        mujoco.mj_forward(model.model, model.data)
        for vehicle, state in zip(manager.vehicles, model.vehicle_states()):
            vehicle.latest_state = dict(
                state, valid=True, derivatives_valid=True, filter_derivatives_valid=True,
                filtered_velocity=state["velocity"], filtered_acceleration=state["acceleration"],
                received_time=0.0, mocap_sample_time_s=100.0,
            )
            if ekf_mode == "delayed_truth":
                vehicle.latest_ekf_state = dict(
                    position=state["position"], filtered_velocity=state["velocity"],
                    filtered_acceleration=state["acceleration"], filter_derivatives_valid=True,
                    received_time=0.0,
                )
            else:
                vehicle.latest_ekf_state = None
        # Initialize motor-force state as well as the collective diagnostic.
        model.reset_actuators([c.thrust for c in commands])
        sample_pose()
        if ekf_link is not None:
            ekf_link.sample_if_due(float(model.data.time), model.vehicle_states())
        next_pose_time = pose_period
        for tick in range(int(time_limit_s * manager.rate_hz)):
            now = float(model.data.time)
            clock_time = now
            deliver_poses(now)
            if ekf_link is not None:
                ekf_link.deliver(now, manager.vehicles)
            manager._timer_callback(None)
            phase = manager.vehicles[0].flight_phase
            if not phases or phases[-1] != phase:
                phases.append(phase)
            reasons = [getattr(vehicle.trajectory, "emergency_landing_reason", "")
                       for vehicle in manager.vehicles]
            latched_landing_time = manager.payload_validation_landing_started_time
            planned_landing = is_planned_validation_landing(
                validation_enabled, latched_landing_time, reasons)
            if planned_landing:
                validation_landing_started_time = float(latched_landing_time)
                terminal_reason = reasons[0]
            if manager.global_abort_reason:
                failure = "aborted"
                terminal_reason = manager.global_abort_reason
                break
            controlled_fault = getattr(manager, "payload_controlled_exit_reason", "")
            if (verify_controlled_fault_descent and controlled_fault_landing_started_time is None
                    and str(controlled_fault).startswith("payload_load: height_drop:")):
                controlled_fault_landing_started_time = manager.payload_landing_start_time
            expected_fault_descent = is_expected_controlled_fault_descent(
                verify_controlled_fault_descent, controlled_fault,
                [vehicle.trajectory.phase for vehicle in manager.vehicles],
                controlled_fault_landing_started_time, manager._payload_ready(now))
            if controlled_fault and not controlled_fault.startswith("payload_load: flight_validation_complete:"):
                terminal_reason = controlled_fault
                if not expected_fault_descent:
                    failure = "controlled_landing_fault"
                    break
            terminal_failure = (validation_terminal_failure(
                [vehicle.flight_phase for vehicle in manager.vehicles])
                if validation_enabled or verify_controlled_fault_descent
                else phase if phase in ("emergency_landing", "aborted") else None)
            if terminal_failure is not None:
                failure = terminal_failure
                terminal_reason = next((reason for reason in reasons if reason and not
                    str(reason).startswith("payload_load: flight_validation_complete:")), failure)
                break
            if not model._all_states_finite():
                failure = "nonfinite_simulation_state"
                break
            payload = model.payload_state()
            states = model.vehicle_states()
            minimum_vehicle_distance = min(minimum_vehicle_distance, *[
                float(np.linalg.norm(states[a]["position"] - states[b]["position"]))
                for a, b in ((0,1),(0,2),(1,2))])
            clearance = payload_ground_clearance(payload["position"], payload["rotation"], config.payload_size)
            if handoff_time is None and manager.payload_last_takeup_reference is not None:
                xy = manager.payload_last_takeup_reference["positions"][:, :2]
                maximum_xy_error = max(maximum_xy_error, float(np.max(np.linalg.norm(
                    np.array([s["position"][:2] for s in states]) - xy, axis=1))))
            if handoff_time is None and phase == "tension_ramp":
                handoff_time = now
            latest_commands = [VehicleCommand(
                v.last_control_command["collective_thrust"],
                v.last_control_command["body_rate_command"],
                v.last_control_command["desired_force"],
            ) for v in manager.vehicles]
            payload_rotation = payload["rotation"]
            payload_roll = float(np.arctan2(payload_rotation[2, 1], payload_rotation[2, 2]))
            payload_pitch = float(np.arcsin(np.clip(-payload_rotation[2, 0], -1.0, 1.0)))
            payload_yaw = float(np.arctan2(payload_rotation[1, 0], payload_rotation[0, 0]))
            saturated = False
            for index, (vehicle, state, command) in enumerate(zip(manager.vehicles, states, latest_commands)):
                saturated |= bool(np.any(np.abs(command.body_rate) >= vehicle.controller.config.max_body_rate - 1e-6))
                trace.append(dict(
                    control_time_s=now, flight_phase=phase, vehicle_id=vehicle.vehicle_id,
                    handoff_elapsed_s=now-handoff_time if handoff_time is not None else -1.0,
                    pose_age_s=now-last_pose_time, payload_height_m=payload["position"][2],
                    payload_clearance_m=clearance, command_thrust_n=command.thrust,
                    desired_force_z_n=command.desired_force_world[2],
                    payload_roll_rad=payload_roll,
                    payload_pitch_rad=payload_pitch,
                    payload_yaw_rad=payload_yaw,
                    **actuator_trace_fields(model, index),
                    **{f"payload_body_rate_{a}": v for a, v in zip("xyz", payload["body_rate"])},
                    **{f"position_{a}": v for a, v in zip("xyz", state["position"])},
                    **{f"body_rate_{a}": v for a, v in zip("xyz", state["body_rate"])},
                    **{f"command_rate_{a}": v for a, v in zip("xyz", command.body_rate)},
                ))
            if handoff_time is not None and not planned_landing and not expected_fault_descent:
                post_ticks += 1
                saturation_ticks += int(saturated)
                minimum_clearance = min(minimum_clearance, clearance)
                maximum_body_rate = max(maximum_body_rate, max(float(np.max(np.abs(s["body_rate"]))) for s in states))
                maximum_tilt = max(maximum_tilt, max(float(np.arccos(np.clip(s["rotation"][2, 2], -1, 1))) for s in states))
                if clearance < 0.03:
                    failure = "payload_dropped"
                    break
                if (not validation_enabled and not verify_controlled_fault_descent
                        and now - handoff_time >= post_handoff_s):
                    break
            if (planned_landing or expected_fault_descent) and validation_monitor.update(
                    now, states, [command.thrust for command in latest_commands]):
                break
            command_queue.append((now + command_delay_s, latest_commands))
            deadline = (tick + 1) / manager.rate_hz
            while deadline - model.data.time > 1e-10:
                while command_queue and command_queue[0][0] <= model.data.time + 1e-10:
                    commands = command_queue.popleft()[1]
                endpoint = min(deadline, next_pose_time)
                if ekf_link is not None:
                    endpoint = min(endpoint, ekf_link.next_sample_time)
                if command_queue:
                    endpoint = min(endpoint, command_queue[0][0])
                model.advance_physics(commands, endpoint - model.data.time)
                if model.data.time >= next_pose_time - 1e-10:
                    sample_pose()
                    next_pose_time += pose_period
                if (ekf_link is not None and
                        model.data.time >= ekf_link.next_sample_time - 1e-10):
                    ekf_link.sample_if_due(float(model.data.time), model.vehicle_states())
        if verify_controlled_fault_descent and failure is None:
            if controlled_fault_landing_started_time is None:
                failure = "controlled_fault_descent_not_started"
            elif not validation_monitor.completed:
                failure = "controlled_fault_descent_incomplete"
        elif validation_enabled and failure is None:
            failure = validation_window_failure(
                handoff_time, validation_landing_started_time, validation_transport_s,
                validation_monitor.completed)
        elif handoff_time is None and failure is None:
            failure = "handoff_timeout"
        elif not validation_enabled and failure is None and phase != "hover":
            failure = "hover_not_reached"
        if not validation_enabled and not verify_controlled_fault_descent and failure is None:
            failure = observation_window_failure(handoff_time, float(model.data.time), post_handoff_s)
        saturation_fraction = saturation_ticks / max(post_ticks, 1)
        target = next(iter(manager.last_transport_overrides.values()), {}).get("payload_target_position")
        position_error = (float(np.linalg.norm(model.payload_state()["position"] - target))
                          if target is not None else None)
        if failure is None:
            if minimum_clearance < 0.15:
                failure = "insufficient_clearance"
            elif minimum_vehicle_distance < manager.payload_transport_guard.min_distance_m:
                failure = "unsafe_vehicle_distance"
            elif saturation_fraction > 0.05:
                failure = "sustained_rate_saturation"
            elif maximum_xy_error > 0.25:
                failure = "prelift_xy_error"
            elif not validation_enabled and not verify_controlled_fault_descent and (
                    position_error is None or not np.isfinite(position_error) or position_error > 0.10):
                failure = "hover_position_error"
        final_vehicle_states = model.vehicle_states()
        final_thrusts = [float(vehicle.last_control_command["collective_thrust"])
                         for vehicle in manager.vehicles]
        result = dict(
            success=failure is None, failure=failure, control_rate_hz=manager.rate_hz,
            success_category=(None if failure is not None else
                              "controlled_fault_descent" if verify_controlled_fault_descent else
                              "short_flight_validation" if validation_enabled else "post_handoff_hover"),
            verify_controlled_fault_descent=bool(verify_controlled_fault_descent),
            controlled_fault_landing_started_time_s=controlled_fault_landing_started_time,
            controlled_fault_descent_completed=bool(
                verify_controlled_fault_descent and validation_monitor.completed and failure is None),
            validation_enabled=validation_enabled,
            validation_transport_s=validation_transport_s,
            validation_landing_started_time_s=validation_landing_started_time,
            observed_transport_before_landing_s=(
                None if validation_landing_started_time is None or handoff_time is None
                else validation_landing_started_time - handoff_time),
            validation_completed=bool(validation_enabled and validation_monitor.completed and failure is None),
            all_vehicles_zero=bool(all(thrust == 0.0 for thrust in final_thrusts)),
            final_vehicle_z_m=[float(state["position"][2]) for state in final_vehicle_states],
            final_vehicle_vz_mps=[float(state["velocity"][2]) for state in final_vehicle_states],
            terminal_reason=terminal_reason or failure or "post_handoff_window_complete",
            link_calibration_enabled=bool(manager.payload_link_calibration_enabled),
            link_calibration_applied=bool(manager.payload_link_calibration_applied),
            link_calibration_status=manager.payload_link_calibrator.status,
            link_calibration_reason=manager.payload_link_calibrator.reason,
            nominal_link_lengths_m=manager.payload_nominal_link_lengths.tolist(),
            effective_link_lengths_m=manager.slung_controller.config.link_lengths_m.tolist(),
            physical_link_lengths_m=list(config.cable_lengths),
            onboard_controller=config.onboard_controller,
            motor_model=config.motor_model,
            effective_plant=snapshot["simulation_runtime"]["effective_plant"],
            ekf_feedback_mode=ekf_mode,
            ekf_sample_rate_hz=(float(simulation["ekf_sample_rate_hz"])
                                if ekf_link is not None else 1.0 / pose_period),
            ekf_delay_s=(float(simulation["ekf_delay_s"]) if ekf_link is not None else pose_delay_s),
            ekf_host_filter_cutoff_hz=(float(manager.vehicles[0].ekf_velocity_filter.cutoff_hz)
                                      if ekf_link is not None else None),
            ekf_sample_frames=(ekf_link.sample_count if ekf_link is not None else None),
            ekf_delivered_frames=(ekf_link.delivered_count if ekf_link is not None else None),
            observed_post_handoff_s=(None if handoff_time is None
                                     else max(0.0, float(model.data.time) - handoff_time)),
            mellinger_attitude_gains=list(config.mellinger_attitude_gains),
            mellinger_rate_gains=list(config.mellinger_rate_gains),
            mellinger_rate_derivative_gain=config.mellinger_rate_derivative_gain,
            physics_timestep_s=config.timestep, pose_sample_rate_hz=1.0/pose_period,
            pose_delay_s=pose_delay_s, command_delay_s=command_delay_s,
            rate_loop_bandwidth=list(rate_bandwidth), initial_tilt_deg=initial_tilt_deg,
            handoff_time_s=handoff_time, final_time_s=float(model.data.time), phases=phases,
            minimum_clearance_m=minimum_clearance if np.isfinite(minimum_clearance) else None,
            maximum_prelift_xy_error_m=maximum_xy_error,
            maximum_body_rate_rps=maximum_body_rate, maximum_tilt_deg=float(np.rad2deg(maximum_tilt)),
            minimum_vehicle_distance_m=minimum_vehicle_distance if np.isfinite(minimum_vehicle_distance) else None,
            mocap_rate_alpha=mocap_rate_alpha,
            target_payload_height_m=target_payload_height_m,
            post_handoff_s=post_handoff_s,
            payload_safety_reason=manager.payload_safety_reason,
            rate_saturation_fraction=saturation_fraction,
            final_payload_target_m=target.tolist() if target is not None else None,
            final_payload_position_error_m=position_error,
            final_payload_height_m=float(model.payload_state()["position"][2]),
            controller_csv=manager.logger.path,
        )
        if trace:
            with (directory / "simulation_trace.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(trace[0]))
                writer.writeheader()
                writer.writerows(trace)
        (directory / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    finally:
        module.time.monotonic = original_time
        module.rospy.Time.now = original_ros_time
        if original_logerr is None:
            del module.rospy.logerr
        else:
            module.rospy.logerr = original_logerr
        manager.logger.close()
        model.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, help="ROS parameter YAML overlay, used only in simulation")
    parser.add_argument("--pose-delay", type=float, default=None, help="Override simulation.pose_delay_s")
    parser.add_argument("--command-delay", type=float, default=None, help="Override simulation.command_delay_s")
    parser.add_argument("--rate-bandwidth", type=float, nargs=3, default=(35.0, 35.0, 20.0))
    parser.add_argument("--initial-tilt", type=float, default=0.0)
    parser.add_argument("--render", action="store_true", help="Open the MuJoCo window during replay")
    parser.add_argument("--record", type=Path, help="Save the MuJoCo window as GIF or MP4")
    parser.add_argument(
        "--target-payload-height", type=float, default=None,
        help="Override the final payload-center height in this replay (m)",
    )
    args = parser.parse_args()
    overrides = yaml.safe_load(args.candidate.read_text()) if args.candidate else None
    summary = run_simulation(args.output, overrides, args.pose_delay, args.command_delay,
                             args.rate_bandwidth, initial_tilt_deg=args.initial_tilt,
                             render=args.render, record=args.record,
                             target_payload_height_m=args.target_payload_height)
    print(json.dumps(summary, indent=2))
    sys.exit(0 if summary["success"] else 1)
