#!/usr/bin/env python3
"""Plot simulation_trace.csv produced by simulate_slung_handoff.py.

This is a diagnostic plotter, not a second controller or a replay engine.
"""

import argparse
import csv
from pathlib import Path

import matplotlib


def read_trace(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"trace is empty: {path}")
    required = {"control_time_s", "vehicle_id", "payload_height_m",
                "payload_clearance_m", "position_x", "position_y",
                "position_z", "body_rate_x", "body_rate_y", "body_rate_z",
                "command_rate_x", "command_rate_y", "command_rate_z",
                "payload_roll_rad", "payload_pitch_rad", "payload_yaw_rad",
                "command_thrust_n"}
    missing = required.difference(rows[0])
    if missing:
        raise ValueError(f"not a simulation_trace.csv; missing columns: {sorted(missing)}")
    return rows


def values(rows, field):
    return [float(row[field]) for row in rows]


def plot_trace(path, save=None, show=True):
    import numpy as np
    import matplotlib.pyplot as plt

    rows = read_trace(path)
    by_vehicle = {vehicle: [r for r in rows if r["vehicle_id"] == vehicle]
                  for vehicle in sorted({r["vehicle_id"] for r in rows})}
    if set(by_vehicle) != {"3", "4", "5"}:
        raise ValueError(f"expected vehicle IDs 3, 4, 5; got {sorted(by_vehicle)}")
    # Use one row per control tick for payload and pair diagnostics.
    ticks = {}
    for row in rows:
        ticks.setdefault(row["control_time_s"], {})[row["vehicle_id"]] = row
    complete = [(float(time), frame) for time, frame in ticks.items()
                if set(frame) == {"3", "4", "5"}]
    complete.sort(key=lambda item: item[0])
    t = np.asarray([item[0] for item in complete])

    def position(frame, vehicle):
        return np.asarray([float(frame[vehicle][f"position_{axis}"]) for axis in "xyz"])

    distance = {}
    for first, second in (("3", "4"), ("3", "5"), ("4", "5")):
        distance[f"CF{first}-CF{second}"] = np.asarray([
            np.linalg.norm(position(frame, first) - position(frame, second))
            for _, frame in complete
        ])

    fig, axes = plt.subplots(3, 2, figsize=(13, 11), sharex=True)
    ax = axes[0, 0]
    ax.plot(t, [float(frame["3"]["payload_height_m"]) for _, frame in complete], label="payload actual")
    # The trace keeps target height in the real controller CSV; simulation_trace
    # may omit it in older runs, so show actual height in that case.
    target_field = "payload_target_height_m"
    target = [float(frame["3"].get(target_field, "nan")) for _, frame in complete]
    if np.isfinite(target).any():
        ax.plot(t, target, "--", label="payload target")
    ax.set_ylabel("Payload height (m)")
    ax.grid(True, alpha=.25)
    ax.legend()

    ax = axes[0, 1]
    load_roll = np.rad2deg([float(frame["3"]["payload_roll_rad"]) for _, frame in complete])
    load_pitch = np.rad2deg([float(frame["3"]["payload_pitch_rad"]) for _, frame in complete])
    load_yaw = np.rad2deg([float(frame["3"]["payload_yaw_rad"]) for _, frame in complete])
    ax.plot(t, load_roll, label="load roll")
    ax.plot(t, load_pitch, label="load pitch")
    ax.plot(t, load_yaw, label="load yaw")
    ax.set_ylabel("Load attitude (deg)")
    ax.grid(True, alpha=.25)
    ax.legend()

    ax = axes[1, 0]
    for label, series in distance.items():
        ax.plot(t, series, label=label)
    ax.axhline(.30, color="black", linestyle=":", label="guard boundary")
    ax.set_ylabel("Vehicle center distance (m)")
    ax.grid(True, alpha=.25)
    ax.legend()

    ax = axes[1, 1]
    for vehicle, vehicle_rows in by_vehicle.items():
        times = np.asarray(values(vehicle_rows, "control_time_s"))
        rates = np.asarray([[float(row[f"body_rate_{axis}"]) for axis in "xyz"]
                            for row in vehicle_rows])
        commands = np.asarray([[float(row[f"command_rate_{axis}"]) for axis in "xyz"]
                               for row in vehicle_rows])
        ax.plot(times, np.linalg.norm(rates, axis=1), label=f"CF{vehicle} actual")
        ax.plot(times, np.linalg.norm(commands, axis=1), "--", label=f"CF{vehicle} command")
    ax.axhline(3.0, color="black", linestyle=":", label="rate limit")
    ax.set_ylabel("Body-rate norm (rad/s)")
    ax.grid(True, alpha=.25)
    ax.legend(fontsize=8, ncol=2)

    ax = axes[2, 0]
    for vehicle, vehicle_rows in by_vehicle.items():
        ax.plot(values(vehicle_rows, "control_time_s"), values(vehicle_rows, "command_thrust_n"),
                label=f"CF{vehicle}")
    ax.set_ylabel("Command thrust (N)")
    ax.set_xlabel("Control time (s)")
    ax.grid(True, alpha=.25)
    ax.legend()

    ax = axes[2, 1]
    ax.plot(t, [float(frame["3"]["payload_clearance_m"]) for _, frame in complete],
            label="payload clearance")
    ax.axhline(.20, color="black", linestyle=":", label="prelift boundary")
    ax.set_ylabel("Payload clearance (m)")
    ax.set_xlabel("Control time (s)")
    ax.grid(True, alpha=.25)
    ax.legend()

    for ax in axes[0, :]:
        ax.set_xlabel("Control time (s)")
    fig.suptitle("Real CTBR controller replay: handoff and transport diagnostics")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=180)
    if show:
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="simulation_trace.csv")
    parser.add_argument("--save", type=Path, help="PNG/PDF output path")
    parser.add_argument("--no-show", action="store_true", help="Do not open an interactive window")
    args = parser.parse_args()
    matplotlib.use("Agg" if args.no_show else "TkAgg")
    plot_trace(args.trace, save=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
