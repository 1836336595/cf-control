#!/usr/bin/env python3
"""Compare the two real flights without modifying runtime or configuration."""

import csv
import itertools
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = Path(__file__).resolve().parent
IDS = (3, 4, 5)
MASSES = np.array([0.0434, 0.0440, 0.0424])
PAYLOAD_MASS = 0.054
GRAVITY = 9.80665


def numbers(rows, name):
    return np.array([float(row.get(name) or "nan") for row in rows])


def vectors(rows, name, axes="xyz"):
    return np.column_stack([numbers(rows, name + "_" + axis) for axis in axes])


def longest_interval(time, predicate):
    indices = np.flatnonzero(predicate)
    intervals = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    intervals = [part for part in intervals if len(part)]
    if not intervals:
        return {"start_s": None, "end_s": None, "duration_s": 0.0, "frames": 0}
    best = max(intervals, key=lambda part: time[part[-1]] - time[part[0]])
    return dict(start_s=float(time[best[0]]), end_s=float(time[best[-1]]),
                duration_s=float(time[best[-1]] - time[best[0]]), frames=len(best))


def minimum(time, values, mask):
    indices = np.flatnonzero(mask & np.isfinite(values))
    if not len(indices):
        return None
    index = indices[np.argmin(values[indices])]
    return {"distance_m": float(values[index]), "time_s": float(time[index])}


def analyze(suffix):
    path = ROOT / ("multi_ctbr_20261016_" + suffix + ".csv")
    with path.open(newline="", encoding="utf-8") as stream:
        all_rows = list(csv.DictReader(stream))
    rows = {ident: [row for row in all_rows if row["vehicle_id"] == str(ident)] for ident in IDS}
    first = rows[3]
    time = numbers(first, "control_time_s")
    for ident in IDS:
        assert np.array_equal(time, numbers(rows[ident], "control_time_s")), "CSV fleet ticks must align"
    handoff_index = next(index for index, row in enumerate(first) if row["flight_phase"] == "tension_ramp")
    fault_index = next(index for index, row in enumerate(first) if row["flight_phase"] == "emergency_landing")
    handoff, fault = time[handoff_index], time[fault_index]
    planned_indices = [index for index, row in enumerate(first)
                       if row["flight_validation_landing_stage"] == "payload_lowering"]
    planned = time[planned_indices[0]] if planned_indices else None
    height_fault_onset = None
    if "height_drop" in first[fault_index]["emergency_landing_reason"]:
        height_error = numbers(first, "payload_target_z") - numbers(first, "payload_position_z")
        onset = fault_index - 1
        while onset > handoff_index and height_error[onset - 1] > .08:
            onset -= 1
        height_fault_onset = float(time[onset])
    stages = []
    for index, row in enumerate(first):
        stage = (row["flight_phase"], row["flight_validation_landing_stage"])
        if not stages or stage != (stages[-1]["flight_phase"], stages[-1]["validation_landing_stage"]):
            stages.append({"time_s": float(time[index]), "flight_phase": stage[0], "validation_landing_stage": stage[1]})

    positions = {ident: vectors(rows[ident], "position") for ident in IDS}
    pair_distances = {f"CF{a}-CF{b}": np.linalg.norm(positions[a] - positions[b], axis=1)
                      for a, b in itertools.combinations(IDS, 2)}
    distances = np.min(np.array(list(pair_distances.values())), axis=0)
    thrust = np.column_stack([numbers(rows[ident], "command_thrust_newton") for ident in IDS])
    rates = np.stack([vectors(rows[ident], "body_rate") for ident in IDS], axis=1)
    rate_commands = np.stack([vectors(rows[ident], "command_rate") for ident in IDS], axis=1)
    trim = np.column_stack([numbers(rows[ident], "transport_support_trim_n") for ident in IDS])
    clearance, vertical_velocity = numbers(first, "payload_ground_clearance_m"), numbers(first, "payload_velocity_z")
    quiet = (time >= handoff - 2) & (time < handoff)
    before_fault = (time >= handoff) & (time < fault)
    airborne = (time >= handoff) & (time < fault)
    center_accelerations = np.column_stack([numbers(rows[ident], "control_acceleration_z") for ident in IDS])
    payload_acceleration = numbers(first, "payload_acceleration_z")
    required_force = ((MASSES.sum() + PAYLOAD_MASS) * GRAVITY
                      + center_accelerations @ MASSES + PAYLOAD_MASS * payload_acceleration)
    vertical_command = np.zeros(len(time))
    for column, ident in enumerate(IDS):
        vertical_command += thrust[:, column] * np.cos(numbers(rows[ident], "roll_rad")) * np.cos(numbers(rows[ident], "pitch_rad"))
    mass = float(MASSES.sum() + PAYLOAD_MASS)
    release_mask = ((time >= planned) & (time < fault) & (clearance <= .015) &
                    (np.abs(vertical_velocity) <= .05) & (numbers(first, "payload_state_valid") == 1)) if planned is not None else np.zeros(len(time), dtype=bool)
    metrics = {
        "source_csv": str(path), "handoff_s": float(handoff), "fault_s": float(fault),
        "fault_elapsed_s": float(fault - handoff), "planned_landing_s": None if planned is None else float(planned),
        "continuous_height_error_over_0p08m_onset_s": height_fault_onset,
        "final_time_s": float(time[-1]), "fault_reason": first[fault_index]["emergency_landing_reason"], "stages": stages,
        "whole_log_minimum_true_pair_distance": minimum(time, distances, np.ones(len(time), dtype=bool)),
        "post_handoff_before_fault_minimum_true_pair_distance": minimum(time, distances, before_fault),
        "per_pair_whole_log_minimum": {name: minimum(time, values, np.ones(len(time), dtype=bool)) for name, values in pair_distances.items()},
        "handoff_payload_center_m": float(numbers(first, "payload_position_z")[handoff_index]),
        "handoff_payload_clearance_m": float(clearance[handoff_index]),
        "fault_payload_clearance_m": float(clearance[fault_index]),
        "command_step_at_fault_n": {str(ident): float(thrust[fault_index, column] - thrust[fault_index - 1, column]) for column, ident in enumerate(IDS)},
        "before_fault_command_n": {str(ident): float(thrust[fault_index - 1, column]) for column, ident in enumerate(IDS)},
        "fault_command_n": {str(ident): float(thrust[fault_index, column]) for column, ident in enumerate(IDS)},
        "quiet_force_balance": {
            "window_s": [float(handoff - 2), float(handoff)], "system_mass_kg": mass,
            "static_required_force_n": mass * GRAVITY,
            "required_force_from_logged_acceleration_n": float(np.nanmean(required_force[quiet])),
            "commanded_vertical_force_n": float(np.nanmean(vertical_command[quiet])),
            "required_to_command_ratio": float(np.nanmean(required_force[quiet]) / np.nanmean(vertical_command[quiet])),
            "per_vehicle_mean_command_n": {str(ident): float(np.nanmean(thrust[quiet, column])) for column, ident in enumerate(IDS)},
            "note": "Host command units are not measured motor force; acceleration estimates also have filtering errors.",
        },
        "post_handoff_before_fault": {
            "rate_command_any_xy_saturation_fraction": float(np.mean(np.any(np.abs(rate_commands[airborne, :, :2]) >= 3 - 1e-6, axis=(1, 2)))),
            "thrust_command_saturation_fraction_by_vehicle": {str(ident): float(np.mean(thrust[airborne, column] >= 1 - 1e-6)) for column, ident in enumerate(IDS)},
            "rate_command_any_xy_saturation_fraction_by_vehicle": {str(ident): float(np.mean(np.any(np.abs(rate_commands[airborne, column, :2]) >= 3 - 1e-6, axis=1))) for column, ident in enumerate(IDS)},
            "maximum_abs_measured_body_rate_rps": float(np.max(np.abs(rates[airborne]))),
        },
        "planned_payload_landing_release_gate": {
            "clearance_limit_m": .015, "abs_vertical_velocity_limit_mps": .05, "required_continuous_s": .30,
            "longest_continuous_ready_before_fault": longest_interval(time, release_mask),
            "vehicle_descent_logged": any(row["flight_validation_landing_stage"] == "vehicle_descent" for row in first),
        },
    }
    return metrics, dict(time=time, handoff=handoff, fault=fault, planned=planned, height_fault_onset=height_fault_onset,
                         rows=rows, distances=pair_distances, thrust=thrust, rates=rates, trim=trim)


def main():
    results = [analyze(suffix) for suffix in ("145240", "145831")]
    metrics = {suffix: result[0] for suffix, result in zip(("145240", "145831"), results)}
    (OUTPUT / "metrics.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
    fig, axes = plt.subplots(5, 2, figsize=(15, 15), constrained_layout=True)
    colors = ("tab:blue", "tab:orange", "tab:green")
    for column, (suffix, (_, data)) in enumerate(zip(("145240", "145831"), results)):
        x = data["time"] - data["handoff"]
        mask = x >= -3
        first = data["rows"][3]
        axes[0, column].plot(x[mask], numbers(first, "payload_position_z")[mask], label="Measured center")
        axes[0, column].plot(x[mask], numbers(first, "payload_target_z")[mask], "--", label="Commanded center")
        axes[0, column].plot(x[mask], numbers(first, "payload_ground_clearance_m")[mask], alpha=.65, label="Bottom clearance")
        axes[0, column].set_title("Real flight 2026-10-16 " + suffix[:2] + ":" + suffix[2:4] + ":" + suffix[4:])
        for name, values in data["distances"].items():
            axes[1, column].plot(x[mask], values[mask], label=name)
        axes[1, column].axhline(.30, color="red", linestyle=":", label="Distance threshold")
        for index, (ident, color) in enumerate(zip(IDS, colors)):
            axes[2, column].plot(x[mask], data["thrust"][mask, index], color=color, label=f"CF{ident}")
            axes[3, column].plot(x[mask], data["rates"][mask, index, 0], color=color, label=f"CF{ident} p")
            axes[3, column].plot(x[mask], data["rates"][mask, index, 1], color=color, linestyle="--", alpha=.8, label=f"CF{ident} q")
            axes[4, column].plot(x[mask], data["trim"][mask, index], color=color, label=f"CF{ident}")
        axes[2, column].axhline(1.0, color="red", linestyle=":", label="Command cap")
        for row, label in enumerate(("Payload height [m]", "True pair distance [m]", "Commanded thrust [N]", "Measured body rate [rad/s]", "Support trim [N]")):
            ax = axes[row, column]
            ax.set_ylabel(label)
            ax.set_xlim(-3, float(x[-1]))
            ax.axvline(0, color="black", linestyle=":", linewidth=1, label="Handoff" if row == 0 else None)
            ax.axvline(data["fault"] - data["handoff"], color="red", linestyle="--", linewidth=1, label="Safety landing" if row == 0 else None)
            if data["height_fault_onset"] is not None:
                ax.axvspan(data["height_fault_onset"] - data["handoff"], data["fault"] - data["handoff"], color="red", alpha=.06,
                           label="Height-drop confirmation" if row == 0 else None)
            if data["planned"] is not None:
                ax.axvline(data["planned"] - data["handoff"], color="purple", linestyle="--", linewidth=1, label="10 s planned landing" if row == 0 else None)
            ax.grid(True, alpha=.25)
            ax.legend(fontsize=7, ncol=2, loc="best")
        axes[4, column].set_xlabel("Time since handoff [s]")
    fig.suptitle("Real flights: actual measured separation and transition commands", fontsize=15)
    fig.savefig(OUTPUT / "comparison.png", dpi=160)
    plt.close(fig)
    print(json.dumps({suffix: {key: metrics[suffix][key] for key in (
        "handoff_s", "fault_s", "whole_log_minimum_true_pair_distance", "command_step_at_fault_n", "planned_payload_landing_release_gate")}
        for suffix in metrics}, indent=2))


if __name__ == "__main__":
    main()
