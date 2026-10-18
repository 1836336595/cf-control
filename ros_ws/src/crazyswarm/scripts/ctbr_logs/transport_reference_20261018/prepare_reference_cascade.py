#!/usr/bin/env python3
"""Freeze matched reference-cascade candidates without changing flight parameters."""

from copy import deepcopy
import json
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent


def main():
    base = yaml.safe_load((HERE / "equilibrium_observed_q_damping000/candidate.yaml").read_text())
    intervals = json.loads((HERE / "control_intervals_latest.json").read_text())["control_intervals_s"]
    for damping, suffix in ((.001, "001"), (0., "000")):
        for stress in (False, True):
            name = "cascade_observed_q_damping" + suffix + ("_timing_noise" if stress else "")
            directory = HERE / name
            directory.mkdir(exist_ok=True)
            candidate = deepcopy(base)
            load = candidate["slung_payload"]
            load.update(
                transport_force_mode="reference_cascade",
                transport_attitude_reference_enabled=True,
                transport_equilibrium_reference_enabled=True,
                transport_attitude_reference_s=12.,
                ramp_position_hold_gain_scale=2.,
                transport_position_hold_gain_scale=1.5,
            )
            simulation = load["simulation"]
            simulation.update(
                payload_rotational_damping=damping,
                ekf_feedback_mode="sampled_filtered",
                ekf_sample_rate_hz=50. if stress else 70.,
                ekf_delay_s=.02,
                pose_sample_rate_hz=100.,
                pose_delay_s=.02 if stress else .01,
                command_delay_s=.01 if stress else .005,
                payload_rotation_noise_std_rad=[.005, .004, .004] if stress else [0., 0., 0.],
                pose_noise_seed=20261016,
                control_intervals_s=intervals if stress else [],
            )
            (directory / "candidate.yaml").write_text(yaml.safe_dump(candidate, sort_keys=False))
            manifest = {
                "source_candidate": "equilibrium_observed_q_damping000/candidate.yaml",
                "physical_payload_mass_kg": .023,
                "physical_link_lengths_m": [.85, .85, .85],
                "transport_force_mode": "reference_cascade",
                "requested_post_handoff_s": 60.,
                "load_rotational_damping": damping,
                "control_schedule": "measured_intervals" if stress else "fixed_70_hz",
                "control_interval_count": len(intervals) if stress else 0,
                "ekf_sample_rate_hz": simulation["ekf_sample_rate_hz"],
                "pose_delay_s": simulation["pose_delay_s"],
                "command_delay_s": simulation["command_delay_s"],
                "payload_rotation_noise_std_rad": simulation["payload_rotation_noise_std_rad"],
                "status": "prepared; requires integrated reference_cascade implementation before execution",
                "caveat": "A physically valid tilted geometry stress case, not an exact real-flight replay.",
            }
            (directory / "case_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(name)


if __name__ == "__main__":
    main()
