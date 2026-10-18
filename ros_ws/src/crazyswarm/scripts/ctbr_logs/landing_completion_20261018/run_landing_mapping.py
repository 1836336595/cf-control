#!/usr/bin/env python3
"""Offline full-mission landing replay with physical motor calibration error.

This runner writes artifacts only. It imports either the frozen pre-fix sources
or current production sources and uses the same frozen parameter set for both.
The native motor_thrust_efficiency multiplies all four real rotor forces (and
their moments), preserving rotor dynamics, ground contacts, and rope mechanics.
No simulator state, reference, phase, touchdown result, or completion flag is
forced by this runner. The replay's existing physical 0.007 m ground-origin
assumption is retained and recorded in its input_parameters.yaml.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import yaml


HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parents[1]
WORKSPACE = SCRIPTS.parents[4]


def finite_values(rows, name):
    values = []
    for row in rows:
        try:
            value = float(row[name])
        except (KeyError, ValueError, TypeError):
            continue
        if np.isfinite(value):
            values.append(value)
    return values


def analyze(output, summary):
    with (output / "simulation_trace.csv").open() as stream:
        trace = list(csv.DictReader(stream))
    with Path(summary["controller_csv"]).open() as stream:
        control = list(csv.DictReader(stream))
    result = {name: summary[name] for name in (
        "success", "failure", "handoff_time_s", "validation_landing_started_time_s",
        "observed_transport_before_landing_s", "validation_completed",
        "all_vehicles_zero", "final_vehicle_z_m", "final_vehicle_vz_mps",
        "final_time_s", "minimum_clearance_m", "minimum_vehicle_distance_m",
        "maximum_body_rate_rps", "rate_saturation_fraction", "phases")}
    stage_first = {}
    for row in control:
        stage = row.get("flight_validation_landing_stage", "")
        if stage:
            stage_first.setdefault(stage, float(row["control_time_s"]))
        for key in ("payload_landing_touchdown_confirmed", "payload_landing_unloading"):
            if row.get(key, "").lower() in ("true", "1", "1.0"):
                stage_first.setdefault(key, float(row["control_time_s"]))
    result["first_stage_time_s"] = stage_first
    landing_start = summary["validation_landing_started_time_s"]
    landing_trace = [row for row in trace if landing_start is not None
                     and float(row["control_time_s"]) >= landing_start]
    result["peak_actual_body_rate_rps_entire_mission"] = max(
        (abs(value) for axis in "xyz" for value in finite_values(trace, "body_rate_" + axis)),
        default=None)
    result["peak_actual_body_rate_rps_during_landing"] = max(
        (abs(value) for axis in "xyz" for value in finite_values(landing_trace, "body_rate_" + axis)),
        default=None)
    landing_tilts = []
    for row in control:
        if landing_start is None or float(row["control_time_s"]) < landing_start:
            continue
        try:
            cosine = np.cos(float(row["roll_rad"])) * np.cos(float(row["pitch_rad"]))
            landing_tilts.append(float(np.rad2deg(np.arccos(np.clip(cosine, -1., 1.)))))
        except (KeyError, ValueError):
            pass
    result["peak_landing_tilt_deg"] = max(landing_tilts, default=None)
    descent_start = stage_first.get("vehicle_descent")
    descent_vertical_speeds = []
    powered_descent_vertical_speeds = []
    for vehicle_id in (3, 4, 5):
        rows = [row for row in trace if int(row["vehicle_id"]) == vehicle_id
                and descent_start is not None and float(row["control_time_s"]) >= descent_start]
        if len(rows) > 1:
            times = np.array([float(row["control_time_s"]) for row in rows])
            heights = np.array([float(row["position_z"]) for row in rows])
            thrusts = np.array([float(row["command_thrust_n"]) for row in rows])
            speeds = np.diff(heights) / np.diff(times)
            descent_vertical_speeds.extend(speeds.tolist())
            powered_descent_vertical_speeds.extend(speeds[(thrusts[:-1] > 0.) & (thrusts[1:] > 0.)].tolist())
    result["vehicle_descent_truth_height_difference_vz_min_mps"] = min(descent_vertical_speeds, default=None)
    result["vehicle_descent_truth_height_difference_vz_max_mps"] = max(descent_vertical_speeds, default=None)
    result["powered_vehicle_descent_truth_height_difference_vz_min_mps"] = min(powered_descent_vertical_speeds, default=None)
    result["landing_tail_by_vehicle"] = {}
    if trace:
        end = float(trace[-1]["control_time_s"])
        for vehicle_id in (3, 4, 5):
            rows = [row for row in trace if int(row["vehicle_id"]) == vehicle_id
                    and float(row["control_time_s"]) >= end - 5.]
            result["landing_tail_by_vehicle"][str(vehicle_id)] = {
                name + "_median": float(np.median(finite_values(rows, name)))
                for name in ("position_z", "command_thrust_n", "actual_thrust_n")
                if finite_values(rows, name)}
            crows = [row for row in control if int(row["vehicle_id"]) == vehicle_id
                     and float(row["control_time_s"]) >= end - 5.]
            for name in ("target_z", "position_error_z", "position_integral_z",
                         "target_velocity_z", "transport_support_trim_n"):
                values = finite_values(crows, name)
                if values:
                    result["landing_tail_by_vehicle"][str(vehicle_id)][name + "_median"] = float(np.median(values))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("frozen", "current"), required=True)
    parser.add_argument("--gain", type=float, default=1., help="Actual per-rotor thrust efficiency")
    parser.add_argument("--gain-onset", choices=("from_start", "vehicle_descent"),
                        default="from_start", help="Time when physical efficiency error begins")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--time-limit", type=float, default=90.)
    parser.add_argument("--candidate", type=Path, default=HERE / "frozen_parameters.yaml")
    args = parser.parse_args()
    source = HERE / "frozen_sources" if args.source == "frozen" else SCRIPTS
    sys.path.insert(0, str(WORKSPACE / "simulate_uav" / "udaan"))
    sys.path.insert(0, str(source))
    import test_ctbr_controller_v2 as helper
    helper.CONTROLLER_CONFIG_PATH = SCRIPTS.parent / "config" / "ctbr_controller.yaml"
    import simulate_slung_handoff as replay
    overrides = yaml.safe_load(args.candidate.read_text())
    simulation = overrides["slung_payload"]["simulation"]
    simulation["motor_thrust_efficiency"] = [
        args.gain if args.gain_onset == "from_start" else 1.] * 3
    simulation["time_limit_s"] = args.time_limit
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "candidate.yaml").write_text(yaml.safe_dump(overrides, sort_keys=False))
    hashes = {name: hashlib.sha256((source / name).read_bytes()).hexdigest()
              for name in ("ctbr_controller.py", "ctbr_trajectory.py", "simulate_slung_handoff.py")}
    snapshot = args.output / "source_snapshot"
    snapshot.mkdir(exist_ok=True)
    for name in hashes:
        shutil.copy2(source / name, snapshot / name)
    metadata = dict(source=args.source, source_directory=str(source), source_hashes=hashes,
                    actual_motor_thrust_efficiency=args.gain, gain_onset=args.gain_onset,
                    physical_state_or_completion_forcing=False,
                    source_parameters=str(args.candidate), command=sys.argv)
    manager_holder = {}
    if args.gain_onset == "vehicle_descent":
        # Change only the physical rotor coefficient once measured payload
        # contact and the full unload have actually finished. This controlled
        # disturbance isolates the reported post-release aircraft stall; it is
        # explicitly distinct from a constant full-flight calibration error.
        from udaan.models.mujoco.crazyswarm_slung import CrazySwarmSlungModel
        original_manager = replay._slung_manager
        original_apply = CrazySwarmSlungModel._apply_commands

        def capture_manager(*positional, **named):
            manager, anchors = original_manager(*positional, **named)
            manager_holder["manager"] = manager
            return manager, anchors

        def apply_with_mapping_change(model, commands, dt):
            manager = manager_holder.get("manager")
            if manager is not None and manager.payload_validation_landing_stage == "vehicle_descent":
                metadata.setdefault("actual_gain_onset_time_s", float(model.data.time))
                for actuator in model._onboard_actuators:
                    actuator.efficiency = args.gain
            return original_apply(model, commands, dt)

        replay._slung_manager = capture_manager
        CrazySwarmSlungModel._apply_commands = apply_with_mapping_change
    (args.output / "run_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    summary = replay.run_simulation(args.output, parameter_overrides=overrides,
                                    post_handoff_s=10., time_limit_s=args.time_limit)
    (args.output / "run_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    result = analyze(args.output, summary)
    (args.output / "landing_metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
