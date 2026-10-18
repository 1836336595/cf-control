#!/usr/bin/env python3
"""Render completed physical leveling diagnostics and tabulate exact hold errors."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent


def main():
    names = ["nominal", "unequal_from_start", "unequal_at_handoff"]
    labels = ["Native efficiency [1, 1, 1]", "[1, 1.05, 1.10] from start",
              "[1, 1.05, 1.10] at handoff"]
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True, constrained_layout=True)
    combined = {}
    for name, label in zip(names, labels):
        directory = HERE / name
        metrics = json.loads((directory / "leveling_metrics.json").read_text())
        metadata = json.loads((directory / "run_metadata.json").read_text())
        combined[name] = dict(metrics=metrics, metadata=metadata)
        handoff = metrics["handoff_time_s"]
        rows = [r for r in csv.DictReader((directory / "simulation_trace.csv").open())
                if r["vehicle_id"] == "3" and float(r["control_time_s"]) >= handoff]
        time = np.array([float(r["control_time_s"]) - handoff for r in rows])
        for axis, angle in zip(axes[:3], ("roll", "pitch", "yaw")):
            angle_values = np.rad2deg([float(r["payload_" + angle + "_rad"]) for r in rows])
            axis.plot(time, angle_values, label=label, lw=1.)
            axis.axhline(0., color="black", lw=.7, ls="--")
            axis.set_ylabel(angle.title() + " (deg)")
        axes[3].plot(time, [float(r["payload_position_z"]) for r in rows], label=label, lw=1.)
    axes[0].legend(fontsize=9)
    axes[3].axhline(.6, color="black", lw=.7, ls="--")
    axes[3].set_ylabel("Load center height (m)")
    axes[3].set_xlabel("Time since physical handoff (s)")
    for axis in axes:
        axis.grid(alpha=.25)
        axis.set_xlim(0, 60)
        axis.axvline(12., color="grey", lw=.8, ls=":")
    fig.suptitle("Frozen production reference cascade; load level target completes at 12 s\n"
                 "Physical rotor lift efficiency only; zero load damping; measured timing, 50 Hz EKF, rotation noise")
    fig.savefig(HERE / "physical_leveling_comparison.png", dpi=160)
    (HERE / "comparison.json").write_text(json.dumps(combined, indent=2) + "\n")
    lines = ["# Offline physical leveling diagnostic", "",
        "These are hypothesis tests, not identification of any real aircraft's motor coefficients. "
        "No production source was edited, and no physical state, pose bias, phase or completion flag was forced.", "",
        "All runs use a frozen snapshot of the current reference cascade, 12 s consistent full R/cable reference, "
        "hold scales 2.0 ramp and 1.5 steady, load attitude bandwidth [1,1,.45] Hz and damping ratio .9. "
        "The reference is level from 12 s onward. The real control interval sequence, 50 Hz sampled/filtered EKF, "
        "20 ms pose/EKF delay, 10 ms command delay, rotation noise [.005,.004,.004] rad and zero physical load "
        "rotational damping are retained. All runs cover the complete physical 60 s post-handoff window.", "",
        "The disturbance is the native per-vehicle motor_thrust_efficiency. It multiplies all four rotor lift "
        "forces and resulting roll/pitch moments, while retaining rotor dynamics, reactive yaw torque, "
        "rope mechanics and ground contacts. The configured bridge raw scales remain "
        "[1.0213949773,1.0174765859,1.0266666884] in all runs; native efficiency 1 is the model baseline, "
        "not a claim that actual thrust equals commanded thrust.", "",
        "Physical assumptions: vehicle masses [43.4,44.0,42.4] g for CF3/4/5, centered uniform 23 g box "
        "[.08,.06,.05] m, identical true/nominal cables .85 m, no rope calibration. Top attachment points "
        "are [+.04,0,+.025],[-.04,+.03,+.025],[-.04,-.03,+.025] m relative to box center. "
        "ROS world is z-up and R=Rz(yaw) Ry(pitch) Rx(roll). Trace vehicle positions are true body centers. "
        "Vehicle pose feedback uses those same body-center coordinates with the configured sampling delay. "
        "The replay starts at airborne TAKEUP anchors; it is not a replay of aircraft ground takeoff. "
        "Reported hold errors are synchronized true body-center XYZ minus each vehicle's actual "
        "position_hold_position, not ordinary target XYZ.", "",
        "| Case | 50–60 s true mean R/P/Y (deg) | R/P std (deg) | Final load position error (m) | Outcome |",
        "|---|---:|---:|---:|---|"]
    for name, label in zip(names, labels):
        m = combined[name]["metrics"]; w=m["windows"][-1]
        lines.append("| " + label + " | " + ", ".join(f"{x:+.3f}" for x in w["truth_rpy_mean_deg"]) +
            " | " + ", ".join(f"{x:.3f}" for x in w["truth_rpy_std_deg"][:2]) +
            f" | {m['final_payload_position_error_m']:.4f} | {m['failure'] or 'pass'} |")
    lines += ["", "| Case | Vehicle | 50–60 s true XYZ minus hold reference (m) | Command / actual thrust mean (N) |",
              "|---|---|---:|---:|"]
    for name, label in zip(names, labels):
        w=combined[name]["metrics"]["windows"][-1]
        for vehicle, data in w["vehicles"].items():
            lines.append("| " + label + " | CF" + vehicle + " | " +
                ", ".join(f"{x:+.4f}" for x in data["truth_minus_hold_mean_m"]) +
                f" | {data['command_thrust_mean_n']:.4f} / {data['actual_thrust_mean_n']:.4f} |")
    lines += ["", "A [0,+5%,+10%] actual rotor lift mismatch produces a steady ~20.6 degree pitch "
        "error after a level full-attitude reference has completed. Applying it from simulation start or exactly "
        "at handoff gives the same settled result, isolating the mechanism from the prelift history. "
        "There is no rate saturation; all original safety/quality checks run. The two biased cases are "
        "correctly marked hover_position_error at the end (about .161 m), rather than treated as successful flights.", "",
        "This supports testing bounded per-vehicle hold integral/disturbance compensation and improving "
        "the per-aircraft thrust mapping. It does not prove that the real log's tilt is caused by thrust "
        "mapping: load rigid-body frame tilt, center-of-mass offset, rope/attachment geometry and tension "
        "identification remain distinct hypotheses.", "",
        "Artifacts: source_snapshot/ and config_snapshot/ contain frozen controller/config dependencies; "
        "physics_source_snapshot/ preserves native rotor and MuJoCo implementations. Every case has "
        "candidate.json, candidate.yaml, run_metadata.json, input_parameters.yaml, summary.json, "
        "leveling_metrics.json, controller CSV and truth trace CSV. Source hashes are recorded at the root.", ""]
    (HERE / "report.md").write_text("\n".join(lines))
    print(HERE / "report.md")
    print(HERE / "physical_leveling_comparison.png")


if __name__ == "__main__":
    main()
