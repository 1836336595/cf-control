#!/usr/bin/env python3
"""Compare physical zero-sum hold-integral probes with unchanged production."""
import csv
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
CASES = ["nominal", "nominal_diffI002", "unequal_from_start", "unequal_diffI002",
         "unequal_diffI005", "unequal_bw2_noI"]


def main():
    combined = {}
    for name in CASES:
        directory = HERE / name
        metrics = json.loads((directory / "leveling_metrics.json").read_text())
        metadata = json.loads((directory / "run_metadata.json").read_text())
        window = metrics["windows"][-1]
        errors = np.array([v["truth_minus_hold_mean_m"] for v in window["vehicles"].values()])
        relative_errors = errors - errors.mean(axis=0)
        combined[name] = dict(success=metrics["success"], failure=metrics["failure"],
            observed_post_handoff_s=metrics["observed_post_handoff_s"],
            mean_rpy_50_60_deg=window["truth_rpy_mean_deg"],
            std_rpy_50_60_deg=window["truth_rpy_std_deg"],
            differential_mean_hold_error_rms_m=float(np.sqrt(np.mean(relative_errors ** 2))),
            differential_mean_hold_error_m=relative_errors.tolist(),
            mean_hold_error_m=errors.mean(axis=0).tolist(),
            final_payload_position_error_m=metrics["final_payload_position_error_m"],
            maximum_body_rate_rps=metrics["maximum_body_rate_rps"],
            rate_saturation_fraction=metrics["rate_saturation_fraction"],
            minimum_vehicle_distance_m=metrics["minimum_vehicle_distance_m"],
            bias=metadata.get("differential_hold_integral", {}))
        if "diffI" in name:
            shutil.copy2(HERE / "run_probe.py", directory / "candidate_probe.py")
            manifest = dict(source_snapshot=str(HERE / "source_snapshot"),
                config_snapshot=str(HERE / "config_snapshot"),
                source_hashes=json.loads((HERE / "source_hashes.json").read_text()),
                candidate_probe_sha256=hashlib.sha256((directory / "candidate_probe.py").read_bytes()).hexdigest(),
                runtime_parameters=metadata["differential_hold_integral"],
                production_files_modified=False)
            (directory / "candidate_code.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (HERE / "integral_comparison.json").write_text(json.dumps(combined, indent=2) + "\n")
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True, constrained_layout=True)
    for column, names in enumerate((["nominal", "nominal_diffI002"],
        ["unequal_from_start", "unequal_diffI002", "unequal_diffI005", "unequal_bw2_noI"])):
        for name in names:
            directory = HERE / name
            summary = json.loads((directory / "summary.json").read_text())
            rows = [r for r in csv.DictReader((directory / "simulation_trace.csv").open())
                    if r["vehicle_id"] == "3" and float(r["control_time_s"]) >= summary["handoff_time_s"]]
            time = np.array([float(r["control_time_s"]) - summary["handoff_time_s"] for r in rows])
            for row, angle in enumerate(("roll", "pitch")):
                values = np.rad2deg([float(r["payload_" + angle + "_rad"]) for r in rows])
                axes[row, column].plot(time, values, lw=1.1, label=name)
        axes[0, column].set_title("Native efficiency " + ("[1,1,1]" if column == 0 else "[1,1.05,1.10]"))
        axes[0, column].legend(fontsize=8)
    for axis in axes.ravel():
        axis.grid(alpha=.25)
        axis.axhline(0., color="black", lw=.7, ls="--")
        axis.axvline(17., color="grey", lw=.8, ls=":")
        axis.set_xlim(0, 60)
    axes[0, 0].set_ylabel("True load roll (deg)")
    axes[1, 0].set_ylabel("True load pitch (deg)")
    for axis in axes[1]:
        axis.set_xlabel("Time since physical handoff (s)")
    fig.suptitle("Offline differential hold integral vs attitude-gain control\n"
                 "Zero-sum bias, 0.03 N / 0.002 N/s joint caps, learn after references finish at 17 s")
    fig.savefig(HERE / "integral_comparison.png", dpi=160)
    lines = ["# Offline bounded differential hold-integral comparison", "",
        "All cases retain the frozen production controller, common load support trim, actual physical "
        "rotor model, measured control timing, sampled EKF, delays, zero load rotational damping "
        "and rotation noise from report.md. Production files were not edited. The experimental control "
        "is implemented only by wrapping the manager's _transport_overrides in the offline runner.", "",
        "For each aircraft e_i = measured position minus actual hold reference. The probe removes "
        "the fleet mean independently on XYZ and integrates db_i/dt = -Ki (e_i - mean(e)). "
        "A common scale applied to the complete fleet enforces per-aircraft norm caps of .03 N and "
        ".002 N/s while retaining sum(b_i)=sum(db_i/dt)=0. The force and its actual bounded derivative "
        "are added to desired_force_override and desired_force_dot_override. The existing transport "
        "blend remains responsible for blending them into the applied aircraft force.", "",
        "Learning starts only after max(attitude reference duration, tension ramp + load lift duration) "
        "= max(12,5+12)=17 s. The wrapper checks valid position/source-derivative flags, prior command "
        "force/tilt/rate/collective limits, and landing/fault state. Those failure paths were not injected "
        "in this study; production adoption requires explicit reset/landing/invalid-EKF handling and "
        "behavioral tests. Bias trace and command-source manifests are saved per integral case.", "",
        "| Case | Mean true R/P 50–60 s (deg) | R/P std (deg) | Differential hold mean-error RMS (mm) | Final load XYZ error norm (m) | Outcome |",
        "|---|---:|---:|---:|---:|---|"]
    for name, result in combined.items():
        lines.append(f"| {name} | " + ", ".join(f"{x:+.3f}" for x in result["mean_rpy_50_60_deg"][:2]) +
            " | " + ", ".join(f"{x:.3f}" for x in result["std_rpy_50_60_deg"][:2]) +
            f" | {1000*result['differential_mean_hold_error_rms_m']:.2f} | "
            f"{result['final_payload_position_error_m']:.4f} | {result['failure'] or 'pass'} |")
    lines += ["", "The differential RMS above measures shape error: remove the fleet mean from "
        "the three per-aircraft mean XYZ hold errors, square all nine components, average and take "
        "the square root. It deliberately excludes the common translation error.", "",
        "| Integral case | First learning after handoff (s) | Max bias norm (N) | Max bias rate (N/s) | Max norm of sum(b) (N) |",
        "|---|---:|---:|---:|---:|"]
    for name, result in combined.items():
        bias = result["bias"]
        if "first_learning_elapsed_s" in bias:
            lines.append(f"| {name} | {bias['first_learning_elapsed_s']:.5f} | "
                f"{bias['maximum_bias_norm_n']:.6f} | {bias['maximum_bias_rate_norm_nps']:.6f} | "
                f"{bias['maximum_sum_bias_norm_n']:.3g} |")
    lines += ["", "No integral case changes the production common load support trim or injects "
        "net fleet force. Thus a reduction in shape/attitude error does not establish correction of "
        "common thrust-mapping error or common load height/position. The 60 s success/failure result "
        "and final load position norm are reported separately.", "",
        "The 2 Hz case changes only attitude_bandwidth_hz from [1,1,.45] to [2,2,.45] under the same "
        "[1,1.05,1.10] rotor-efficiency mismatch. It is a diagnostic comparison in one model/noise "
        "configuration, not a recommendation to deploy 2 Hz gains.", ""]
    settling_path = HERE / "integral_settling_time.json"
    if settling_path.exists():
        settling = json.loads(settling_path.read_text())
        lines += ["The current 20 s transport window permits only about 3 s of learning after "
            "the 17 s reference-completion gate. In the unequal-efficiency hypothesis test, true pitch "
            f"at 20 s is {settling['unequal_diffI002']['20'][1]:.2f} deg with Ki=.02 and "
            f"{settling['unequal_diffI005']['20'][1]:.2f} deg with Ki=.05, versus "
            f"{settling['unequal_from_start']['20'][1]:.2f} deg without integral. "
            "Neither integral therefore establishes leveling within a 20 s flight. "
            f"Ki=.02 first holds both |roll| and |pitch| below 5 deg for 2 s starting at "
            f"{settling['unequal_diffI002']['first_2sec_below_5.0deg']:.2f} s after handoff; "
            f"Ki=.05 does so from {settling['unequal_diffI005']['first_2sec_below_5.0deg']:.2f} s "
            f"and below 2 deg from {settling['unequal_diffI005']['first_2sec_below_2.0deg']:.2f} s. "
            "See integral_settling_time.json for sampled trajectories and the precise criterion.", ""]
    (HERE / "integral_report.md").write_text("\n".join(lines))
    print(json.dumps(combined, indent=2))


if __name__ == "__main__":
    main()
