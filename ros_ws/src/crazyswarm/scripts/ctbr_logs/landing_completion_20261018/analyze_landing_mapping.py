#!/usr/bin/env python3
"""Rebuild comparison metrics and plots from completed physical replay traces."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from run_landing_mapping import analyze


HERE = Path(__file__).resolve().parent


def main():
    cases = {}
    for summary_path in sorted(HERE.glob("*/summary.json")):
        summary = json.loads(summary_path.read_text())
        directory = summary_path.parent
        result = analyze(directory, summary)
        metadata = json.loads((directory / "run_metadata.json").read_text())
        result["source"] = metadata["source"]
        result["actual_motor_thrust_efficiency"] = metadata["actual_motor_thrust_efficiency"]
        result["gain_onset"] = metadata.get("gain_onset", "from_start")
        result["actual_gain_onset_time_s"] = metadata.get("actual_gain_onset_time_s", 0.)
        (directory / "landing_metrics.json").write_text(json.dumps(result, indent=2) + "\n")
        cases[directory.name] = result
    (HERE / "comparison.json").write_text(json.dumps(cases, indent=2) + "\n")
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    selected = ("baseline_nominal", "baseline_release_gain1075", "current_nominal",
                "current_release_gain1075", "current_release_gain110")
    labels = dict(baseline_nominal="Before, nominal",
                  baseline_release_gain1075="Before, +7.5% after unload",
                  current_nominal="After, nominal",
                  current_release_gain1075="After, +7.5% after unload",
                  current_release_gain110="After, +10% after unload")
    for name in selected:
        if name not in cases:
            continue
        with (HERE / name / "simulation_trace.csv").open() as stream:
            trace = list(csv.DictReader(stream))
        by_vehicle = [[row for row in trace if int(row["vehicle_id"]) == i] for i in (3, 4, 5)]
        count = min(map(len, by_vehicle))
        time = np.array([float(row["control_time_s"]) for row in by_vehicle[0][:count]])
        z = np.array([[float(row["position_z"]) for row in rows[:count]] for rows in by_vehicle])
        force = np.array([[float(row["command_thrust_n"]) for row in rows[:count]] for rows in by_vehicle])
        clearance = np.array([float(row["payload_clearance_m"]) for row in by_vehicle[0][:count]])
        label = labels[name]
        line, = axes[0].plot(time, clearance, label=label, linewidth=1.6)
        axes[1].plot(time, z.mean(axis=0), color=line.get_color(), linewidth=1.6)
        axes[1].fill_between(time, z.min(axis=0), z.max(axis=0), color=line.get_color(), alpha=.1)
        axes[2].plot(time, force.mean(axis=0), color=line.get_color(), linewidth=1.6)
    axes[0].set_ylabel("Payload clearance [m]")
    axes[0].legend(fontsize=8, loc="upper right")
    axes[1].set_ylabel("Aircraft height [m]\nmean and min/max across 3")
    axes[1].axhline(.007, color="k", linestyle=":", linewidth=1., label="physical body-center ground")
    axes[2].set_ylabel("Mean commanded thrust [N]")
    axes[2].set_xlabel("Physical simulation time [s]")
    axes[2].set_xlim(left=30.)
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle("10 s transport validation, 23 g payload, 0.850 m cables, reference cascade hold gain 1.5\n"
                 "Mapping disturbances begin only after measured payload touchdown and complete 2 s unload")
    fig.savefig(HERE / "landing_mapping_comparison.png", dpi=170)
    plt.close(fig)
    print(json.dumps({name: dict(success=row["success"], failure=row["failure"],
                                final_vehicle_z_m=row["final_vehicle_z_m"],
                                final_time_s=row["final_time_s"], all_vehicles_zero=row["all_vehicles_zero"])
                      for name, row in cases.items()}, indent=2))


if __name__ == "__main__":
    main()
