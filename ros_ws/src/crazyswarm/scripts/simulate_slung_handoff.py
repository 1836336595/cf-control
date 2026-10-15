#!/usr/bin/env python3
"""Offline MuJoCo validation of the real mission/controller, without ROS/radio.

Only ROS I/O is stubbed by the existing test utilities. Mission logic, payload
pose observer and CTBR control laws come from ctbr_controller.py. Sampled ideal
EKF velocity passes through the real host filter. Full firmware EKF is not modeled.
"""

import argparse
from collections import deque
import csv
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import yaml

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from slung_load_controller import payload_ground_clearance


def run_simulation(directory, parameter_overrides=None, pose_delay_s=0.01,
                   command_delay_s=0.005, rate_bandwidth=(35.0, 35.0, 20.0),
                   post_handoff_s=None, time_limit_s=90.0, initial_tilt_deg=0.0,
                   mocap_rate_alpha=0.2, render=False, record=None,
                   target_payload_height_m=None):
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
    (directory / "input_parameters.yaml").write_text(yaml.safe_dump(snapshot, sort_keys=False))
    base = default_config()
    cfg = manager.slung_controller.config
    config = replace(
        base, control_rate_hz=manager.rate_hz, rate_loop_bandwidth=tuple(rate_bandwidth),
        vehicles=tuple(replace(v, mass=n.controller.config.mass)
                       for v, n in zip(base.vehicles, manager.vehicles)),
        cable_lengths=tuple(cfg.link_lengths_m), payload_mass=cfg.payload_mass_kg,
        payload_size=tuple(cfg.payload_size_m),
        attachment_points=tuple(map(tuple, cfg.attachment_points_m.T)),
    )
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
    pose_period = 0.01
    last_pose_time = 0.0
    handoff_time = None
    minimum_clearance = float("inf")
    maximum_xy_error = 0.0
    maximum_body_rate = 0.0
    maximum_tilt = 0.0
    minimum_vehicle_distance = float("inf")
    saturation_ticks = post_ticks = 0
    phases = []
    failure = None
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
            mujoco.mju_mat2Quat(quat, payload["rotation"].ravel())
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
            vehicle.latest_ekf_state = dict(
                position=state["position"], filtered_velocity=state["velocity"],
                filtered_acceleration=state["acceleration"], filter_derivatives_valid=True,
                received_time=0.0,
            )
        model._actual_thrust[:] = [c.thrust for c in commands]
        sample_pose()
        next_pose_time = pose_period
        for tick in range(int(time_limit_s * manager.rate_hz)):
            now = float(model.data.time)
            clock_time = now
            deliver_poses(now)
            manager._timer_callback(None)
            phase = manager.vehicles[0].flight_phase
            if not phases or phases[-1] != phase:
                phases.append(phase)
            if phase in ("emergency_landing", "aborted"):
                failure = phase
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
                    **{f"payload_body_rate_{a}": v for a, v in zip("xyz", payload["body_rate"])},
                    **{f"position_{a}": v for a, v in zip("xyz", state["position"])},
                    **{f"body_rate_{a}": v for a, v in zip("xyz", state["body_rate"])},
                    **{f"command_rate_{a}": v for a, v in zip("xyz", command.body_rate)},
                ))
            if handoff_time is not None:
                post_ticks += 1
                saturation_ticks += int(saturated)
                minimum_clearance = min(minimum_clearance, clearance)
                maximum_body_rate = max(maximum_body_rate, max(float(np.max(np.abs(s["body_rate"]))) for s in states))
                maximum_tilt = max(maximum_tilt, max(float(np.arccos(np.clip(s["rotation"][2, 2], -1, 1))) for s in states))
                if clearance < 0.03:
                    failure = "payload_dropped"
                    break
                if now - handoff_time >= post_handoff_s:
                    break
            command_queue.append((now + command_delay_s, latest_commands))
            deadline = (tick + 1) / manager.rate_hz
            while deadline - model.data.time > 1e-10:
                while command_queue and command_queue[0][0] <= model.data.time + 1e-10:
                    commands = command_queue.popleft()[1]
                endpoint = min(deadline, next_pose_time)
                if command_queue:
                    endpoint = min(endpoint, command_queue[0][0])
                model.advance_physics(commands, endpoint - model.data.time)
                if model.data.time >= next_pose_time - 1e-10:
                    sample_pose()
                    next_pose_time += pose_period
        if handoff_time is None and failure is None:
            failure = "handoff_timeout"
        elif failure is None and phase != "hover":
            failure = "hover_not_reached"
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
            elif position_error is None or not np.isfinite(position_error) or position_error > 0.10:
                failure = "hover_position_error"
        result = dict(
            success=failure is None, failure=failure, control_rate_hz=manager.rate_hz,
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
    parser.add_argument("--pose-delay", type=float, default=0.01)
    parser.add_argument("--command-delay", type=float, default=0.005)
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
