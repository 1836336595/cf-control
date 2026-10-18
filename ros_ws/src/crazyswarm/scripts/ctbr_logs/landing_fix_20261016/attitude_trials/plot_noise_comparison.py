#!/usr/bin/env python3
"""Compare the same final 5 s of all completed 60 s orientation-noise trials."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent


def read_window(path, start):
    with path.open() as stream:
        return [row for row in csv.DictReader(stream) if float(row["control_time_s"]) >= start]


def field(rows, key):
    return np.array([float(row[key]) for row in rows])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conditioning", action="store_true")
    args = parser.parse_args()
    cases = (("baseline_noise_delay", "2 Hz, damping 0.9"), ("damping_06", "2 Hz, damping 0.6")) if args.conditioning else (
        ("noise_bw2", "2 Hz"), ("noise_bw12", "1.2 Hz"), ("noise_bw1", "1 Hz"))
    fig, axes = plt.subplots(4, len(cases), figsize=(5 * len(cases), 11), constrained_layout=True, sharex=True, sharey="row")
    for column, (name, label) in enumerate(cases):
        directory = HERE / name
        summary = json.loads((directory / "summary.json").read_text())
        start = summary["final_time_s"] - 5
        handoff = summary["handoff_time_s"]
        trace = read_window(directory / "simulation_trace.csv", start)
        control = [row for row in read_window(Path(summary["controller_csv"]), start) if row["vehicle_id"] == "3"]
        cf3 = [row for row in trace if row["vehicle_id"] == "3"]
        cf4 = [row for row in trace if row["vehicle_id"] == "4"]
        t = field(cf3, "control_time_s") - handoff
        axes[0, column].plot(t, np.rad2deg(field(cf3, "payload_roll_rad")), label="Payload roll")
        axes[0, column].plot(t, np.rad2deg(field(cf3, "payload_pitch_rad")), label="Payload pitch")
        axes[0, column].set_title("Payload attitude bandwidth " + label)
        axes[1, column].plot(t, field(cf3, "command_thrust_n"), label="CF3 command")
        axes[1, column].plot(t, field(cf3, "actual_thrust_n"), label="CF3 actual force")
        tc4 = field(cf4, "control_time_s") - handoff
        axes[2, column].plot(tc4, field(cf4, "body_rate_x"), label="CF4 p truth")
        axes[2, column].plot(tc4, field(cf4, "body_rate_y"), label="CF4 q truth")
        axes[3, column].plot(field(control, "control_time_s") - handoff,
                             field(control, "payload_body_rate_y"), color="gray", alpha=.65, label="Observed payload q")
        axes[3, column].plot(t, field(cf3, "payload_body_rate_y"), label="True payload q")
        for row, ylabel in enumerate(("Payload angle [deg]", "Thrust [N]", "Aircraft body rate [rad/s]", "Payload body rate [rad/s]")):
            axes[row, column].set_ylabel(ylabel)
            axes[row, column].grid(True, alpha=.25)
            axes[row, column].legend(fontsize=8, loc="best")
        axes[3, column].set_xlabel("Time since handoff [s]")
    title = "Payload orientation perturbation hypothesis: final 5 s of full 60 s trials\n"
    if args.conditioning:
        title += "Matched pose/command delays 20/10 ms; observer gain 0.30; same attitude stiffness\n"
    title += "Rotation noise sigma [0.005, 0.004, 0.004] rad; not measured NOKOV accuracy"
    fig.suptitle(title, fontsize=13)
    output = HERE / ("conditioning_comparison.png" if args.conditioning else "noise_comparison.png")
    fig.savefig(output, dpi=160)
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    main()
