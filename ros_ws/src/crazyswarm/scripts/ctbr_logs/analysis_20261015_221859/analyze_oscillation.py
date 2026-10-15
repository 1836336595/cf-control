"""Compare recorded real/sim handoffs without ROS or aircraft connections."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent
LOGS = OUT.parent


def read(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def values(rows, names):
    if isinstance(names, str):
        return np.array([float(row[names] or "nan") for row in rows])
    return np.array([[float(row[name] or "nan") for name in names] for row in rows])


def handoff(rows):
    return min(float(row["control_time_s"]) for row in rows
               if row["flight_phase"] == "tension_ramp")


def peak_frequency(rows, field):
    time = values(rows, "control_time_s")
    uniform = np.arange(time[0], time[-1], 1 / 70)
    signal = np.interp(uniform, time, values(rows, field))
    signal -= np.polyval(np.polyfit(uniform, signal, 1), uniform)
    power = np.abs(np.fft.rfft(signal * np.hanning(len(signal)))) ** 2
    frequency = np.fft.rfftfreq(len(signal), 1 / 70)
    selected = np.flatnonzero((frequency > 1) & (frequency < 25))
    return float(frequency[selected[np.argmax(power[selected])]])


def main():
    real = read(LOGS / "multi_ctbr_20261015_221859.csv")
    sim = read(LOGS / "final_stable_60" / "multi_ctbr_20261015_171610.csv")
    start, sim_start = handoff(real), handoff(sim)
    result = {"real_handoff_s": start, "sim_handoff_s": sim_start, "vehicles": {}}
    colors = {"3": "#9467bd", "4": "#1f77b4", "5": "#2ca02c"}
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True, layout="constrained")
    for cf, color in colors.items():
        rr = [r for r in real if r["vehicle_id"] == cf
              and start - 5 <= float(r["control_time_s"]) <= start + 7.5]
        ss = [r for r in sim if r["vehicle_id"] == cf
              and sim_start - 5 <= float(r["control_time_s"]) <= sim_start + 7.5]
        t, st = values(rr, "control_time_s") - start, values(ss, "control_time_s") - sim_start
        w = values(rr, ["body_rate_" + a for a in "xyz"])
        command = values(rr, ["command_rate_" + a for a in "xyz"])
        blend = np.nan_to_num(values(rr, "transport_blend"))
        er = values(rr, ["attitude_error_" + a for a in "xyz"])
        p = -((1 - blend[:, None]) * [12, 12, 6] + blend[:, None] * [6, 6, 3]) * er
        d = -blend[:, None] * [2, 2, 1] * w
        error = np.max(np.abs(np.clip(p + d, [-3, -3, -2], [3, 3, 2]) - command))
        metrics = {"command_formula_max_error": float(error), "windows": {}}
        for label, lo, hi in (("prelift_quiet", -5, 0), ("ramp_early", 0, 2),
                              ("ramp_middle", 2, 3), ("ramp_late", 3, 5),
                              ("after_ramp", 5, 7.5)):
            mask = (t >= lo) & (t < hi)
            metrics["windows"][label] = {
                "samples": int(mask.sum()),
                "measured_xy_rate_rms": np.sqrt(np.mean(w[mask, :2] ** 2, axis=0)).tolist(),
                "command_xy_rate_rms": np.sqrt(np.mean(command[mask, :2] ** 2, axis=0)).tolist(),
                "any_xy_command_saturation_fraction": float(np.mean(np.any(
                    np.abs(command[mask, :2]) >= 2.99, axis=1))),
            }
        spectral = [r for r in rr if 51 < float(r["control_time_s"]) < 56.3]
        metrics["rate_spectrum_peak_hz"] = {a: peak_frequency(spectral, "body_rate_" + a)
                                             for a in "xy"}
        result["vehicles"][cf] = metrics
        axes[1, 0].plot(t, np.max(np.abs(w[:, :2]), axis=1), c=color, label="CF" + cf)
        axes[2, 0].plot(t, values(rr, "command_thrust_newton"), c=color, label="CF" + cf)
        axes[2, 1].plot(t, values(rr, "battery_voltage_v"), c=color, label="CF" + cf)
        axes[2, 1].axhline(float(rr[0]["preflight_voltage_v"]), c=color, ls=":", alpha=.6)
        if cf == "3":
            axes[0, 0].plot(t, values(rr, "payload_ground_clearance_m"), label="Real", c="#d62728")
            axes[0, 0].plot(st, values(ss, "payload_ground_clearance_m"), label="Stable sim", c="black", ls="--")
            axes[0, 1].plot(t, blend, c="black", label="Transport blend")
            sim_w = values(ss, ["body_rate_" + a for a in "xy"])
            axes[1, 0].plot(st, np.max(np.abs(sim_w), axis=1), c="black", ls="--", label="Sim CF3")
            axes[2, 0].plot(st, values(ss, "command_thrust_newton"), c="black", ls="--", label="Sim CF3")
        if cf == "5":
            axes[1, 1].plot(t, command[:, 0], label="Command", c="black", lw=1)
            axes[1, 1].plot(t, p[:, 0], label="Attitude term", alpha=.8)
            axes[1, 1].plot(t, d[:, 0], label="Rate feedback term", alpha=.8)
    labels = (("Payload ground clearance (m)", "Handoff blend"),
              ("Maximum |measured x/y rate| (rad/s)", "CF5 roll-rate command terms (rad/s)"),
              ("Commanded thrust (N)", "Battery voltage (V); dotted: preflight"))
    for row in range(3):
        for col in range(2):
            ax = axes[row, col]
            ax.set_ylabel(labels[row][col])
            ax.axvline(0, c="gray", ls=":")
            ax.axvline(5, c="gray", ls=":")
            ax.grid(alpha=.25)
            ax.legend(loc="best", fontsize=8)
    axes[1, 1].set_ylim(-8, 8)
    for ax in axes[-1]:
        ax.set_xlabel("Time relative to handoff (s)")
    fig.suptitle("Real flight 22:18 vs stable simulation: oscillation develops during TAUT_RAMP")
    fig.savefig(OUT / "handoff_comparison.png", dpi=150)
    (OUT / "metrics.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
