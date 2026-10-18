#!/usr/bin/env python3
"""Summarize the reference cascade across physical damping and timing/noise cases."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from analyze_case import analyze
from compare_cases import trace_data


HERE = Path(__file__).resolve().parent


def main():
    fig, axes = plt.subplots(5, 2, figsize=(13, 12), sharex="col", constrained_layout=True)
    results = {}
    for column, damping in enumerate(("001", "000")):
        baseline_name = "baseline_observed_q_damping" + damping
        baseline_directory = HERE / baseline_name
        baseline_time, baseline_values, baseline_summary = trace_data(baseline_directory)
        baseline_rows = [row for row in csv.DictReader((baseline_directory / "simulation_trace.csv").open())
                         if row["vehicle_id"] == "3"]
        baseline_height = np.array([float(row["payload_position_z"]) for row in baseline_rows])
        baseline_plot = np.column_stack([baseline_values[:, :3], baseline_height, baseline_values[:, 3]])
        baseline_mask = baseline_time >= 0.
        for row in range(5):
            axes[row, column].plot(baseline_time[baseline_mask], baseline_plot[baseline_mask, row],
                                   color="#777777", ls="--", lw=1.1, label="Coupled baseline (hold 0.5)")
            if not baseline_summary["success"]:
                axes[row, column].plot(baseline_time[baseline_mask][-1], baseline_plot[baseline_mask, row][-1],
                                       "x", color="#777777", ms=7)
        baseline_metrics = json.loads((baseline_directory / "physical_attitude_metrics.json").read_text())
        baseline_metrics.update(mode="coupled", payload_final_height_m=baseline_summary["final_payload_height_m"],
                                maximum_body_rate_rps=baseline_summary["maximum_body_rate_rps"])
        results[baseline_name] = baseline_metrics
        for stress, color, label in ((False, "#246db0", "Fixed 70 Hz"),
                                     (True, "#bc4b35", "Measured timing + delay/noise")):
            name = "cascade_observed_q_damping" + damping + ("_timing_noise" if stress else "")
            directory = HERE / name
            if not (directory / "summary.json").exists():
                continue
            metrics = analyze(directory)
            parameters = yaml.safe_load((directory / "input_parameters.yaml").read_text())
            mode = parameters["slung_payload"]["transport_force_mode"]
            assert mode == "reference_cascade", "Result must use the integrated production mode"
            time, values, summary = trace_data(directory)
            rows = [row for row in csv.DictReader((directory / "simulation_trace.csv").open())
                    if row["vehicle_id"] == "3"]
            height = np.array([float(row["payload_position_z"]) for row in rows])
            mask = time >= 0.
            plotted = np.column_stack([values[:, :3], height, values[:, 3]])
            for row in range(5):
                axes[row, column].plot(time[mask], plotted[mask, row], color=color, lw=1.25, label=label)
                if not summary["success"]:
                    axes[row, column].plot(time[mask][-1], plotted[mask, row][-1], "x", color=color, ms=7)
            metrics["mode"] = mode
            metrics["payload_final_height_m"] = summary["final_payload_height_m"]
            metrics["maximum_body_rate_rps"] = summary["maximum_body_rate_rps"]
            metrics["effective_timing"] = {key: parameters["simulation_runtime"][key] for key in (
                "ekf_sample_rate_hz", "pose_delay_s", "command_delay_s", "payload_rotation_noise_std_rad")}
            metrics["effective_timing"]["control_interval_count"] = len(
                parameters["simulation_runtime"]["control_intervals_s"])
            results[name] = metrics
            manifest_path = directory / "case_manifest.json"
            if manifest_path.exists():
                manifest = json.loads(manifest_path.read_text())
                manifest.update(status="executed", success=summary["success"],
                                observed_post_handoff_s=summary["observed_post_handoff_s"])
                manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        for row in range(5):
            axes[row, column].grid(alpha=.25)
            axes[row, column].axvline(5., color="grey", ls=":", lw=1.)
            axes[row, column].set_xlim(0, 60)
        axes[0, column].set_title("Load rotational damping = " + ("0.001" if damping == "001" else "0"))
        axes[0, column].legend(fontsize=8)
        axes[3, column].axhline(.60, color="black", ls="--", lw=1.)
        axes[4, column].axhline(.30, color="black", ls="--", lw=1.)
        axes[4, column].set_xlabel("Time since physical handoff (s)")
    for axis, label in zip(axes[:, 0], ("Load pitch (deg)", "Load roll (deg)", "Load yaw (deg)",
                                      "Load center height (m)", "Minimum aircraft distance (m)")):
        axis.set_ylabel(label)
    fig.suptitle("Reference-cascade validation: actual tilted start, 23 g load, three 0.850 m cables\n"
                 "hold gains 2.0 ramp / 1.5 steady; physical truth; existing guards retained", fontsize=13)
    fig.savefig(HERE / "cascade_comparison.png", dpi=180)
    plt.close(fig)
    (HERE / "cascade_comparison.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps({name: {key: metrics[key] for key in ("success", "observed_post_handoff_s",
        "final_payload_position_error_m", "payload_final_height_m", "maximum_body_rate_rps", "terminal_reason")}
        for name, metrics in results.items()}, indent=2))


if __name__ == "__main__":
    main()
