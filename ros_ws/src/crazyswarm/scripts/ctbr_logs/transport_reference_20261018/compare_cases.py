#!/usr/bin/env python3
"""Plot physical truth for paired geometry-reference and load-damping cases."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_case import analyze


HERE = Path(__file__).resolve().parent


def trace_data(directory):
    summary = json.loads((directory / "summary.json").read_text())
    fleet = {str(vehicle): [] for vehicle in (3, 4, 5)}
    for row in csv.DictReader((directory / "simulation_trace.csv").open()):
        fleet[row["vehicle_id"]].append(row)
    time = np.array([float(row["control_time_s"]) for row in fleet["3"]]) - summary["handoff_time_s"]
    angles = np.rad2deg([[float(row["payload_" + name + "_rad"])
                        for name in ("pitch", "roll", "yaw")] for row in fleet["3"]])
    position = np.array([[[float(row["position_" + axis]) for axis in "xyz"]
                         for row in fleet[str(vehicle)]] for vehicle in (3, 4, 5)])
    distance = np.min([np.linalg.norm(position[a] - position[b], axis=1)
                       for a,b in ((0,1), (0,2), (1,2))], axis=0)
    return time, np.column_stack([angles, distance]), summary


def main():
    fig, axes = plt.subplots(4, 2, figsize=(13, 10), sharex="col", constrained_layout=True)
    results = {}
    for column, damping in enumerate(("001", "000")):
        for kind, color, label in (("baseline", "#bc4b35", "Reference disabled"),
                                   ("consistent", "#246db0", "Reference enabled")):
            name = kind + "_observed_q_damping" + damping
            directory = HERE / name
            metrics = analyze(directory)
            results[name] = metrics
            time, values, summary = trace_data(directory)
            mask = time >= 0.
            for row in range(4):
                axes[row, column].plot(time[mask], values[mask, row], color=color, lw=1.3, label=label)
                if not summary["success"]:
                    axes[row, column].plot(time[mask][-1], values[mask, row][-1], "x", color=color, ms=7)
            terminal = "60 s complete" if summary["success"] else "guard at %.2f s" % summary["observed_post_handoff_s"]
            axes[0, column].plot([], [], color=color, label=terminal)
        for row in range(4):
            axes[row, column].grid(alpha=.25)
            axes[row, column].axvline(5., color="grey", ls=":", lw=1.)
            axes[row, column].set_xlim(0, 60 if damping == "001" else 13)
        axes[0, column].set_title("Load rotational damping = " + ("0.001" if damping == "001" else "0"))
        axes[0, column].legend(fontsize=9, ncol=2)
        axes[3, column].axhline(.30, color="black", ls="--", lw=1., label="0.30 m guard")
        axes[3, column].set_xlabel("Time since physical handoff (s)")
    for axis, label in zip(axes[:, 0], ("Load pitch (deg)", "Load roll (deg)",
                                      "Load yaw (deg)", "Minimum aircraft distance (m)")):
        axis.set_ylabel(label)
    fig.suptitle("23 g load, three physical 0.850 m cables, actual tilted start\n"
                 "Fixed 70 Hz control / sampled 70 Hz EKF; physical truth, no sensor attitude bias", fontsize=13)
    fig.savefig(HERE / "comparison.png", dpi=180)
    plt.close(fig)
    (HERE / "comparison.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps({name: {key: metrics[key] for key in ("success", "observed_post_handoff_s",
        "handoff_physical_payload_rpy_deg", "final_physical_payload_rpy_deg", "terminal_reason")}
        for name, metrics in results.items()}, indent=2))


if __name__ == "__main__":
    main()
