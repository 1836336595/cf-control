#!/usr/bin/env python3
"""Read-only diagnosis of synchronized pose gaps and independent takeoff."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "multi_ctbr_20261016_222247.csv"


def values(rows, field):
    return np.array([float(r[field] or "nan") for r in rows])


def vector(rows, prefix):
    return np.column_stack([values(rows, prefix + "_" + a) for a in "xyz"])


def main():
    rows = list(csv.DictReader(SOURCE.open(newline="")))
    fleet = {ident: [r for r in rows if r["vehicle_id"] == str(ident)] for ident in (3, 4, 5)}
    first = fleet[3]
    time = values(first, "control_time_s")
    fault = next(float(r["control_time_s"]) for r in first if r["flight_phase"] == "emergency_landing")
    takeoff = next(float(r["control_time_s"]) for r in first if r["flight_phase"] == "takeoff")
    ages = np.column_stack([values(fleet[i], "mocap_sample_age_s") for i in fleet])
    payload_age = values(first, "payload_state_age_s")
    valid = np.all(np.isfinite(ages), axis=1)
    flight = (time >= takeoff) & (time < fault)
    metrics = dict(source=str(SOURCE), rows=len(rows), takeoff_s=takeoff, fault_s=fault,
                   fault_reason=next(r["emergency_landing_reason"] for r in first if r["flight_phase"] == "emergency_landing"),
                   reached_transport=any(r["flight_phase"] == "tension_ramp" for r in first),
                   pose_age_cross_vehicle_correlation=np.corrcoef(ages[valid].T).tolist(),
                   vehicles={}, pose_gap_blocks=[], comparison={})
    for j, ident in enumerate(fleet):
        rs = fleet[ident]
        metrics["vehicles"][str(ident)] = {}
        for name, mask in (("full_record", np.ones(len(time), dtype=bool)), ("takeoff_to_fault", flight)):
            selected = [r for r, keep in zip(rs, mask) if keep]
            sample = {}
            for field in ("mocap_sample_age_s", "state_age_s", "ekf_state_age_s", "bridge_latest_command_age_s", "bridge_latest_send_call_wall_s", "previous_fleet_cycle_wall_s"):
                x = values(rs, field)[mask]
                x = x[np.isfinite(x)]
                sample[field + "_p50_p95_max"] = np.percentile(x, [50, 95, 100]).tolist()
            sample.update(rows=len(selected), mocap_derivatives_invalid=sum(r["derivatives_valid"] == "0" for r in selected),
                          local_mocap_filter_invalid=sum(r["filter_derivatives_valid"] == "0" for r in selected),
                          ekf_status_fresh=sum(r["ekf_status"] == "fresh" for r in selected),
                          pose_age_over50ms_fraction=float(np.mean(ages[mask, j] > .05)))
            metrics["vehicles"][str(ident)][name] = sample
    indices = np.flatnonzero(ages[:, 0] > .1)
    for block in np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1):
        if len(block):
            metrics["pose_gap_blocks"].append(dict(start_s=float(time[block[0]]), end_s=float(time[block[-1]]),
                                                  maximum_sample_age_s=float(ages[block, 0].max())))
    position = np.stack([vector(fleet[i], "position") for i in fleet], axis=1)
    distances = np.column_stack([np.linalg.norm(position[:, a] - position[:, b], axis=1) for a, b in ((0, 1), (0, 2), (1, 2))])
    frame, pair = np.unravel_index(np.nanargmin(distances), distances.shape)
    metrics["minimum_pair_distance"] = dict(m=float(distances[frame, pair]), time_s=float(time[frame]),
                                            pair=("CF3-CF4", "CF3-CF5", "CF4-CF5")[pair],
                                            caveat="Logged positions during emergency; contact/rigid-body quality cannot be established from CSV alone.")
    for name in ("multi_ctbr_20261016_213804.csv", SOURCE.name):
        other = [r for r in csv.DictReader((HERE.parent / name).open()) if r["vehicle_id"] == "3"]
        other = [r for r in other if 3.3 <= float(r["control_time_s"]) < 10.39]
        x = values(other, "mocap_sample_age_s")
        metrics["comparison"][name] = dict(window_s=[3.3, 10.39], rows=len(other), pose_age_p95_s=float(np.percentile(x, 95)),
                                          pose_age_max_s=float(x.max()), over50ms_fraction=float(np.mean(x > .05)),
                                          source_derivatives_invalid=sum(r["derivatives_valid"] == "0" for r in other))
    (HERE / "metrics.json").write_text(json.dumps(metrics, indent=2))
    fig, axes = plt.subplots(4, 2, figsize=(15, 14), constrained_layout=True, sharex=True)
    for index, ident in enumerate(fleet):
        rs = fleet[ident]
        axes[0, 0].plot(time, 1e3 * ages[:, index], label=f"CF{ident}", alpha=.7)
        axes[0, 1].plot(time, 1e3 * values(rs, "ekf_state_age_s"), label=f"CF{ident}", alpha=.7)
        axes[1, 0].plot(time, values(rs, "position_z"), label=f"CF{ident} actual")
        axes[1, 0].plot(time, values(rs, "target_z"), "--", label=f"CF{ident} target")
        axes[2, 0].plot(time, values(rs, "command_thrust_newton"), label=f"CF{ident}")
        axes[2, 1].plot(time, np.linalg.norm(vector(rs, "command_rate"), axis=1), label=f"CF{ident}")
    axes[0, 0].plot(time, 1e3 * payload_age, "--", label="load", alpha=.7)
    axes[0, 0].axhline(50, color="gray", linestyle=":")
    for prefix, label in (("control_velocity", "valid EKF velocity"), ("target_velocity", "target velocity"), ("velocity_error", "logged velocity error")):
        axes[1, 1].plot(time, values(first, prefix + "_z"), label=label)
    bad = values(first, "filter_derivatives_valid") == 0
    axes[1, 1].fill_between(time, -1, 1, where=bad, color="red", alpha=.1, label="Mocap filter unready")
    for pair, name in enumerate(("CF3-CF4", "CF3-CF5", "CF4-CF5")):
        axes[3, 0].plot(time, distances[:, pair], label=name)
    axes[3, 0].axhline(.3, color="red", linestyle=":")
    for field, label in (("fleet_tick_interval_s", "fleet tick interval"), ("previous_fleet_cycle_wall_s", "fleet compute"), ("bridge_latest_command_age_s", "CF3 bridge command age")):
        axes[3, 1].plot(time, 1e3 * values(first, field), label=label)
    titles = ("Mocap publication age: synchronized stalls", "Fresh EKF telemetry during pose stalls",
              "Takeoff height oscillation before transport", "CF3: fresh velocity exists while feedback is gated",
              "Collective thrust commands", "Body-rate command norm",
              "Distances from logged poses", "Host control and command timing")
    units = ("Age [ms]", "Age [ms]", "Height [m]", "Velocity [m/s]", "Thrust [N]", "Command [rad/s]", "Distance [m]", "Time [ms]")
    for ax, title, unit in zip(axes.flat, titles, units):
        ax.set(title=title, ylabel=unit, xlabel="Controller elapsed time [s]", xlim=(0, time[-1]))
        for start, color in ((takeoff, "black"), (8.239545784, "gray"), (fault, "red")):
            ax.axvline(start, color=color, linestyle="--", linewidth=.7)
        ax.grid(alpha=.25)
        ax.legend(fontsize=7, ncol=2)
    fig.suptitle("2026-10-16 22:22:47 real flight — pose-stream stalls before slung handoff\n"
                 "Black: takeoff; gray: TAKEUP; red: separation fault. Network utilization was not recorded.")
    fig.savefig(HERE / "analysis.png", dpi=150)
    plt.close(fig)
    print(json.dumps({k: metrics[k] for k in ("takeoff_s", "fault_s", "reached_transport", "comparison", "minimum_pair_distance")}, indent=2))


if __name__ == "__main__":
    main()
