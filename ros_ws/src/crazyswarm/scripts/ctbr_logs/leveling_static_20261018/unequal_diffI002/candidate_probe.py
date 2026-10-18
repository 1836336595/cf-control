#!/usr/bin/env python3
"""Frozen-source, physical rotor-efficiency leveling diagnostic; artifacts only."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parents[1]
WORKSPACE = SCRIPTS.parents[4]


def analyze(directory, summary):
    trace = list(csv.DictReader((directory / "simulation_trace.csv").open()))
    control = list(csv.DictReader(Path(summary["controller_csv"]).open()))
    truth = [r for r in trace if r["vehicle_id"] == "3"]
    handoff = summary["handoff_time_s"]
    result = {key: summary[key] for key in ("success", "failure", "handoff_time_s",
        "final_time_s", "observed_post_handoff_s", "maximum_body_rate_rps",
        "rate_saturation_fraction", "final_payload_position_error_m",
        "minimum_clearance_m", "minimum_vehicle_distance_m", "terminal_reason")}
    result["final_truth_rpy_deg"] = np.rad2deg([
        float(truth[-1]["payload_" + a + "_rad"]) for a in ("roll", "pitch", "yaw")]).tolist()
    result["windows"] = []
    if handoff is None:
        return result
    control_by_key = {(r["control_time_s"], r["vehicle_id"]): r for r in control}
    for lower, upper in ((15, 20), (20, 30), (30, 40), (50, 60)):
        selected = [r for r in truth if lower <= float(r["control_time_s"]) - handoff <= upper]
        if not selected:
            continue
        rpy = np.rad2deg([[float(r["payload_" + a + "_rad"]) for a in ("roll", "pitch", "yaw")]
                         for r in selected])
        window = dict(start_after_handoff_s=lower, end_after_handoff_s=upper,
                      sample_count=len(selected), truth_rpy_mean_deg=rpy.mean(axis=0).tolist(),
                      truth_rpy_std_deg=rpy.std(axis=0).tolist(),
                      truth_rpy_peak_to_peak_deg=np.ptp(rpy, axis=0).tolist(), vehicles={})
        for vehicle_id in ("3", "4", "5"):
            selected_vehicle = [r for r in trace if r["vehicle_id"] == vehicle_id
                and lower <= float(r["control_time_s"]) - handoff <= upper]
            errors, holds, positions, commands, actuals = [], [], [], [], []
            for row in selected_vehicle:
                crow = control_by_key.get((row["control_time_s"], vehicle_id))
                if crow is None:
                    continue
                try:
                    hold = np.array([float(crow["position_hold_position_" + a]) for a in "xyz"])
                    position = np.array([float(row["position_" + a]) for a in "xyz"])
                except (KeyError, ValueError):
                    continue
                if np.all(np.isfinite(hold)):
                    errors.append(position - hold)
                    holds.append(hold)
                    positions.append(position)
                    commands.append(float(row["command_thrust_n"]))
                    actuals.append(float(row["actual_thrust_n"]))
            if errors:
                window["vehicles"][vehicle_id] = dict(
                    matched_samples=len(errors),
                    truth_minus_hold_mean_m=np.mean(errors, axis=0).tolist(),
                    truth_minus_hold_std_m=np.std(errors, axis=0).tolist(),
                    hold_reference_mean_m=np.mean(holds, axis=0).tolist(),
                    truth_position_mean_m=np.mean(positions, axis=0).tolist(),
                    command_thrust_mean_n=float(np.mean(commands)),
                    actual_thrust_mean_n=float(np.mean(actuals)))
        result["windows"].append(window)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", required=True)
    parser.add_argument("--efficiencies", type=float, nargs=3, default=[1., 1., 1.])
    parser.add_argument("--onset", choices=("from_start", "handoff"), default="from_start")
    parser.add_argument("--hold-scale", type=float)
    parser.add_argument("--post-handoff", type=float, default=60.)
    parser.add_argument("--hold-integral-ki", type=float, default=0.)
    parser.add_argument("--attitude-bandwidth", type=float, nargs=3)
    args = parser.parse_args()
    source = HERE / "source_snapshot"
    sys.path.insert(0, str(WORKSPACE / "simulate_uav" / "udaan"))
    sys.path.insert(0, str(source))
    import test_ctbr_controller_v2 as helper
    helper.CONTROLLER_CONFIG_PATH = HERE / "config_snapshot" / "ctbr_controller.yaml"
    import simulate_slung_handoff as replay
    candidate_path = SCRIPTS / "ctbr_logs/transport_reference_20261018/cascade_observed_q_damping000_timing_noise/candidate.yaml"
    overrides = yaml.safe_load(candidate_path.read_text())
    simulation = overrides["slung_payload"]["simulation"]
    simulation["motor_thrust_efficiency"] = args.efficiencies if args.onset == "from_start" else [1., 1., 1.]
    if args.hold_scale is not None:
        overrides["slung_payload"]["transport_position_hold_gain_scale"] = args.hold_scale
    if args.attitude_bandwidth is not None:
        overrides["slung_payload"]["attitude_bandwidth_hz"] = args.attitude_bandwidth
    output = HERE / args.case
    output.mkdir(exist_ok=True)
    (output / "candidate.yaml").write_text(yaml.safe_dump(overrides, sort_keys=False))
    (output / "candidate.json").write_text(json.dumps(overrides, indent=2) + "\n")
    metadata = dict(command=sys.argv, actual_motor_thrust_efficiency=args.efficiencies,
        onset=args.onset, source_snapshot=str(source), config_snapshot=str(HERE / "config_snapshot"),
        baseline_candidate=str(candidate_path), physical_state_or_completion_forcing=False,
        physical_disturbance="Native per-vehicle efficiency multiplies four rotor lift forces and resulting roll/pitch moments; Graefe reactive yaw torque is unchanged",
        frame="ROS world z-up; payload R = Rz(yaw) Ry(pitch) Rx(roll); truth body-center positions",
        cable_lengths_m=[.85] * 3, payload_mass_kg=.023)
    metadata["differential_hold_integral"] = dict(
        ki_n_per_m_s=args.hold_integral_ki, force_norm_cap_n=.03,
        force_rate_norm_cap_nps=.002, projection="Remove per-axis fleet mean; joint fleet scaling",
        injection="desired_force_override plus b; desired_force_dot_override plus actual bdot",
        common_load_support_trim="Unchanged production implementation")
    holder = {}
    bias_trace = []
    original_manager = replay._slung_manager
    def capture_manager(*positional, **named):
        module = positional[0]
        manager, anchors = original_manager(*positional, **named)
        holder["manager"] = manager
        metadata["vehicle_masses_kg"] = [v.controller.config.mass for v in manager.vehicles]
        metadata["payload_attachment_points_m"] = manager.slung_controller.config.attachment_points_m.tolist()
        if args.hold_integral_ki > 0.:
            original_transport = manager._transport_overrides
            bias = np.zeros((3, 3))
            def with_differential_hold_integral(now):
                nonlocal bias
                outputs = original_transport(now)
                if not outputs or any(id(v) not in outputs for v in manager.vehicles):
                    return outputs
                targets = [outputs[id(v)] for v in manager.vehicles]
                elapsed = now - manager.payload_transport_start_time
                learning_start = max(manager.payload_attitude_reference_s,
                    manager.payload_tension_ramp_s + manager.payload_reference_lift_s)
                reasons = []
                if elapsed < learning_start:
                    reasons.append("reference_incomplete")
                if (manager.global_abort_reason or manager.payload_fault_since is not None or
                        manager.payload_landing_start_time is not None):
                    reasons.append("landing_or_fault")
                errors = []
                for vehicle, target in zip(manager.vehicles, targets):
                    state = vehicle.latest_state
                    control_state = vehicle._build_ekf_control_state(state, now)
                    if not (vehicle._state_ready(now) and state.get("valid", False) and
                            control_state.get("derivatives_valid", True) and
                            control_state.get("filter_derivatives_valid", False)):
                        reasons.append("invalid_state_or_derivatives")
                    errors.append(np.asarray(state["position"]) - target["position_hold_position"])
                    command = getattr(vehicle, "last_control_command", None)
                    if command is None:
                        reasons.append("missing_previous_command")
                        continue
                    cfg = vehicle.controller.config
                    raw_force = np.asarray(command["raw_desired_force"])
                    limited_force, _ = module._limit_force_with_derivative(
                        raw_force, np.zeros(3), cfg.max_command_thrust,
                        target.get("max_tilt_rad", cfg.max_tilt_rad))
                    if (not np.allclose(raw_force, limited_force, atol=1e-9, rtol=0.) or
                            command["collective_thrust"] >= cfg.max_command_thrust - .01 or
                            command["collective_thrust"] <= 0. or
                            np.any(np.abs(command["raw_body_rate_command"]) >= cfg.max_body_rate)):
                        reasons.append("previous_vehicle_limit_active")
                errors = np.asarray(errors)
                if not np.all(np.isfinite(errors)):
                    reasons.append("nonfinite_error")
                differential_error = errors - np.mean(errors, axis=0)
                dt = float(manager.payload_control_dt)
                derivative = np.zeros((3, 3))
                if not reasons:
                    requested_derivative = -args.hold_integral_ki * differential_error
                    rate_scale = min(1., .002 / max(float(np.max(np.linalg.norm(
                        requested_derivative, axis=1))), 1e-30))
                    proposed = bias + dt * rate_scale * requested_derivative
                    proposed *= min(1., .03 / max(float(np.max(np.linalg.norm(
                        proposed, axis=1))), 1e-30))
                    # A second common scale on the increment preserves both
                    # the norm ball and the rate cap after joint projection.
                    increment = proposed - bias
                    increment *= min(1., dt * .002 / max(float(np.max(np.linalg.norm(
                        increment, axis=1))), 1e-30))
                    derivative = increment / dt
                    bias = bias + increment
                for index, target in enumerate(targets):
                    target["desired_force_override"] = target["desired_force_override"] + bias[index]
                    target["desired_force_dot_override"] = target["desired_force_dot_override"] + derivative[index]
                bias_trace.append(dict(control_time_s=float(now), handoff_elapsed_s=float(elapsed),
                    dt_s=dt, learning=not reasons, freeze_reasons=";".join(sorted(set(reasons))),
                    differential_error_m=differential_error.tolist(), bias_n=bias.tolist(),
                    bias_dot_nps=derivative.tolist(), sum_bias_n=bias.sum(axis=0).tolist(),
                    sum_bias_dot_nps=derivative.sum(axis=0).tolist()))
                return outputs
            manager._transport_overrides = with_differential_hold_integral
        return manager, anchors
    replay._slung_manager = capture_manager
    if args.onset == "handoff":
        from udaan.models.mujoco.crazyswarm_slung import CrazySwarmSlungModel
        original_apply = CrazySwarmSlungModel._apply_commands
        def apply_with_efficiency(model, commands, dt):
            manager = holder.get("manager")
            if manager is not None and manager.payload_transport_start_time is not None:
                metadata.setdefault("actual_efficiency_onset_time_s", float(model.data.time))
                for actuator, efficiency in zip(model._onboard_actuators, args.efficiencies):
                    actuator.efficiency = efficiency
            return original_apply(model, commands, dt)
        CrazySwarmSlungModel._apply_commands = apply_with_efficiency
    (output / "run_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    summary = replay.run_simulation(output, parameter_overrides=overrides,
        post_handoff_s=args.post_handoff)
    if bias_trace:
        (output / "differential_hold_bias_trace.json").write_text(json.dumps(bias_trace) + "\n")
        metadata["differential_hold_integral"].update(
            first_learning_elapsed_s=next((r["handoff_elapsed_s"] for r in bias_trace if r["learning"]), None),
            learning_ticks=sum(r["learning"] for r in bias_trace), trace_ticks=len(bias_trace),
            final_bias_n=bias_trace[-1]["bias_n"],
            maximum_bias_norm_n=max(float(np.max(np.linalg.norm(r["bias_n"], axis=1))) for r in bias_trace),
            maximum_bias_rate_norm_nps=max(float(np.max(np.linalg.norm(r["bias_dot_nps"], axis=1))) for r in bias_trace),
            maximum_sum_bias_norm_n=max(float(np.linalg.norm(r["sum_bias_n"])) for r in bias_trace),
            maximum_sum_bias_rate_norm_nps=max(float(np.linalg.norm(r["sum_bias_dot_nps"])) for r in bias_trace))
    (output / "run_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    metrics = analyze(output, summary)
    (output / "leveling_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2), flush=True)


if __name__ == "__main__":
    main()
