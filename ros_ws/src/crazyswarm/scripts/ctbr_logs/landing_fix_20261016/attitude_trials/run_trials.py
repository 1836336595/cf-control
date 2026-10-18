#!/usr/bin/env python3
"""Freeze full candidates and compare payload bandwidths with optional pose noise."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parents[2]
CONFIG = SCRIPTS.parent / "config"
ROOT = SCRIPTS.parents[4]
CASES = (("bandwidth_2hz", 2.0), ("bandwidth_1hz", 1.0), ("bandwidth_1p2hz", 1.2))
NOISE_CASES = (("noise_bw2", 2.0), ("noise_bw1", 1.0), ("noise_bw12", 1.2))
CONDITIONING_CASES = (("baseline_noise_delay", 2.0), ("observer_rate_010", 2.0),
                      ("observer_rate_015", 2.0), ("damping_06", 2.0))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes():
    paths = [SCRIPTS / filename for filename in (
        "ctbr_controller.py", "slung_load_controller.py", "simulate_slung_handoff.py",
        "slung_simulated_telemetry.py", "test_ctbr_controller_v2.py")]
    paths.extend(ROOT / "simulate_uav" / "udaan" / "udaan" / path for path in (
        "control/crazyflie_pid.py", "control/crazyflie_brushless.py",
        "models/mujoco/crazyswarm_slung.py", "crazyswarm_slung_parameters.py"))
    return {str(path): sha256(path) for path in paths}


def prepare(cases=CASES, noise=False, conditioning=False):
    roots = {}
    config_hashes = {}
    for filename in ("ctbr_controller.yaml", "ctbr_vehicle.yaml", "slung_payload.yaml"):
        path = CONFIG / filename
        roots.update(yaml.safe_load(path.read_text()))
        config_hashes[str(path)] = sha256(path)
    roots["ctbr_controller"]["target_confirmed"] = True
    roots["slung_payload"]["flight_validation_enabled"] = False
    roots["slung_payload"]["post_handoff_s"] = 60.0
    for name, bandwidth in cases:
        values = copy.deepcopy(roots)
        values["slung_payload"]["attitude_bandwidth_hz"] = [bandwidth, bandwidth, .45]
        if noise:
            values["slung_payload"]["simulation"]["payload_rotation_noise_std_rad"] = [.005, .004, .004]
            values["slung_payload"]["simulation"]["pose_noise_seed"] = 20261016
        if conditioning:
            payload = values["slung_payload"]
            payload["simulation"]["pose_delay_s"] = .02
            payload["simulation"]["command_delay_s"] = .01
            payload["observer_angular_rate_gain"] = {"observer_rate_010": .10,
                                                     "observer_rate_015": .15}.get(name, .30)
            payload["attitude_damping_ratio"] = .60 if name == "damping_06" else .90
        directory = HERE / name
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "candidate.yaml").write_text(yaml.safe_dump(values, sort_keys=False))
    preparation_name = "conditioning_preparation.json" if conditioning else "noise_preparation.json" if noise else "preparation.json"
    (HERE / preparation_name).write_text(json.dumps({
        "configuration_sha256": config_hashes,
        "source_sha256": source_hashes(),
        "noise_hypothesis": noise,
        "note": "Complete parameter snapshots; validation landing disabled, full60 required. Optional rotation noise is a sensitivity hypothesis, not measured NOKOV accuracy.",
    }, indent=2))
    print("Prepared complete candidates:", ", ".join(name for name, _ in cases), flush=True)


def run_case(case):
    name, bandwidth = case
    directory = HERE / name
    sources = source_hashes()
    with (directory / "console.txt").open("w") as stream:
        result = subprocess.run([
            sys.executable, str(SCRIPTS / "simulate_slung_handoff.py"),
            "--candidate", str(directory / "candidate.yaml"), "--output", str(directory),
        ], cwd=SCRIPTS, stdout=stream, stderr=subprocess.STDOUT)
    (directory / "run_metadata.json").write_text(json.dumps({
        "python": sys.executable, "returncode": result.returncode,
        "source_sha256_before": sources, "source_sha256_after": source_hashes(),
    }, indent=2))
    summary_path = directory / "summary.json"
    if not summary_path.exists():
        return {"case": name, "returncode": result.returncode, "missing_summary": True}
    summary = json.loads(summary_path.read_text())
    with (directory / "simulation_trace.csv").open() as stream:
        trace_rows = list(csv.DictReader(stream))
    rows = [row for row in trace_rows if row["vehicle_id"] == "3"]
    time = np.array([float(row["control_time_s"]) for row in rows])
    handoff = summary["handoff_time_s"]
    active = time >= handoff if handoff is not None else np.ones(len(time), dtype=bool)
    recent = active & (time >= time[-1] - 20)
    vector = lambda prefix: np.array([[float(row[prefix + "_" + axis]) for axis in "xyz"] for row in rows])
    attitude = np.array([[float(row["payload_" + angle + "_rad"]) for angle in ("roll", "pitch", "yaw")] for row in rows])
    rates = vector("payload_body_rate")
    force = np.array([float(row["command_thrust_n"]) for row in rows])
    with Path(summary["controller_csv"]).open() as stream:
        control_rows = [row for row in csv.DictReader(stream) if row["vehicle_id"] == "3"]
    control_time = np.array([float(row["control_time_s"]) for row in control_rows])
    observed_rate = np.array([[float(row["payload_body_rate_" + axis] or "nan") for axis in "xyz"] for row in control_rows])
    observed_mask = control_time >= max(time[-1] - 20, handoff or 0)
    force_variability = {}
    for ident in (3, 4, 5):
        vehicle_rows = [row for row in trace_rows if row["vehicle_id"] == str(ident)]
        command_force = np.array([float(row["command_thrust_n"]) for row in vehicle_rows])
        actual_force = np.array([float(row["actual_thrust_n"]) for row in vehicle_rows])
        force_variability[str(ident)] = {"command_std_n": float(np.std(command_force[recent])),
                                        "actual_std_n": float(np.std(actual_force[recent])),
                                        "command_mean_n": float(np.mean(command_force[recent]))}
    result = dict(case=name, bandwidth_hz=bandwidth, success=summary["success"], failure=summary["failure"],
                observed_post_handoff_s=summary["observed_post_handoff_s"],
                minimum_vehicle_distance_m=summary["minimum_vehicle_distance_m"],
                minimum_clearance_m=summary["minimum_clearance_m"],
                maximum_body_rate_rps=summary["maximum_body_rate_rps"],
                rate_saturation_fraction=summary["rate_saturation_fraction"],
                final_payload_height_m=summary["final_payload_height_m"],
                final_payload_position_error_m=summary["final_payload_position_error_m"],
                final_20s_payload_attitude_mean_deg=np.rad2deg(np.mean(attitude[recent], axis=0)).tolist(),
                final_20s_payload_attitude_std_rad=np.std(attitude[recent], axis=0).tolist(),
                final_20s_payload_rate_std_rps=np.std(rates[recent], axis=0).tolist(),
                final_20s_cf3_command_thrust_std_n=float(np.std(force[recent])),
                final_20s_observed_payload_rate_std_rps=np.std(observed_rate[observed_mask], axis=0).tolist(),
                final_20s_per_vehicle_force_variability=force_variability)
    candidate = yaml.safe_load((directory / "candidate.yaml").read_text())["slung_payload"]
    result.update(observer_angular_rate_gain=candidate["observer_angular_rate_gain"],
                  attitude_damping_ratio=candidate["attitude_damping_ratio"],
                  pose_delay_s=summary["pose_delay_s"], command_delay_s=summary["command_delay_s"])
    (directory / "trial_metrics.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", action="store_true", help="Refresh full snapshots from current parameter files")
    parser.add_argument("--run", action="store_true", help="Run all three nominal candidates")
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--noise", action="store_true", help="Use repeatable payload orientation perturbation hypothesis")
    parser.add_argument("--conditioning", action="store_true", help="Compare observer gains/damping at 2 Hz with pose/command delays .02/.01 s")
    args = parser.parse_args()
    cases = CONDITIONING_CASES if args.conditioning else NOISE_CASES if args.noise else CASES
    if args.prepare or not all((HERE / name / "candidate.yaml").exists() for name, _ in cases):
        prepare(cases, noise=args.noise or args.conditioning, conditioning=args.conditioning)
    if args.run:
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            results = list(pool.map(run_case, cases))
        metrics_name = "conditioning_comparison_metrics.json" if args.conditioning else "noise_comparison_metrics.json" if args.noise else "comparison_metrics.json"
        (HERE / metrics_name).write_text(json.dumps(results, indent=2))
        print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
