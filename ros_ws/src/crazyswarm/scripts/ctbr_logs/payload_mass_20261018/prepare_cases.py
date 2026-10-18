#!/usr/bin/env python3
"""Freeze mass-comparison inputs; no changes to ROS flight parameter files."""
import copy
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parents[1]
sys.path.insert(0, str(SCRIPTS))
from test_ctbr_controller_v2 import _controller_parameter_roots


def main():
    roots = _controller_parameter_roots()
    roots.update(yaml.safe_load((SCRIPTS.parent / "config/slung_payload.yaml").read_text()))
    (HERE / "baseline_parameters.yaml").write_text(yaml.safe_dump(roots, sort_keys=False))
    source = HERE.parent / "multi_ctbr_20261018_160849.csv"
    rows = [r for r in csv.DictReader(source.open()) if r["vehicle_id"] == "3"]
    handoff = next(float(r["control_time_s"]) for r in rows if r["flight_phase"] == "tension_ramp")
    times = np.array([float(r["control_time_s"]) for r in rows if
                      handoff <= float(r["control_time_s"]) < handoff + 10])
    intervals = np.diff(times).tolist()
    entries = []
    for bandwidth in (1., 2.):
        for damping_name, damping in (("configured_damping", .001), ("zero_damping_bound", 0.)):
            for mass_g in (54, 40, 27):
                name = f"bw{int(bandwidth)}_{damping_name}_{mass_g}g"
                cfg = copy.deepcopy(roots)
                payload = cfg["slung_payload"]
                payload.update(mass_kg=mass_g/1000., post_handoff_s=60.,
                               attitude_bandwidth_hz=[bandwidth, bandwidth, .45],
                               attitude_damping_ratio=.9, flight_validation_enabled=False)
                payload["simulation"].update(control_intervals_s=intervals,
                                              payload_rotational_damping=damping)
                directory = HERE / name
                directory.mkdir(exist_ok=True)
                candidate = directory / "candidate.yaml"
                candidate.write_text(yaml.safe_dump(cfg, sort_keys=False))
                entries.append(dict(name=name, mass_g=mass_g, bandwidth_xy_hz=bandwidth,
                                    rotational_damping=damping, scenario=damping_name,
                                    candidate=str(candidate), output=str(directory)))
    metadata = dict(
        cases=entries, control_interval_source=str(source),
        interval_pattern_count=len(intervals), interval_min_s=min(intervals), interval_max_s=max(intervals),
        pose_noise_std_rad=roots["slung_payload"]["simulation"].get("payload_rotation_noise_std_rad", [0, 0, 0]),
        shape_m=roots["slung_payload"]["size_m"],
        note="Zero damping is an uncalibrated sensitivity bound, not an estimate of the real aircraft/payload."
             " Mass changes both physical box inertia and controller inertia-derived gains."
             " Identical geometry assumes the same center of mass, attachment points and homogeneous inertia scaling.",
        source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (
            SCRIPTS/"slung_load_controller.py", SCRIPTS/"ctbr_controller.py", SCRIPTS/"simulate_slung_handoff.py",
            SCRIPTS.parent/"config/slung_payload.yaml", SCRIPTS.parent/"config/ctbr_vehicle.yaml")},
    )
    (HERE / "cases.json").write_text(json.dumps(metadata, indent=2))
    print(json.dumps(dict(cases=len(entries), baseline_bandwidth=roots["slung_payload"]["attitude_bandwidth_hz"],
                          control_interval_count=len(intervals), interval_bounds_s=[min(intervals),max(intervals)])))


if __name__ == "__main__":
    main()
