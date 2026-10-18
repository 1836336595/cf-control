"""Offline analysis of the 2026-10-16 11:01 real flight; no ROS connection."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent
SOURCE = OUT.parent / "multi_ctbr_20261016_110152.csv"
MASS = {"3": 0.0434, "4": 0.0440, "5": 0.0424}
PAYLOAD_MASS = 0.054


def array(rows, field):
    return np.array([float(row[field] or "nan") for row in rows])


def quantiles(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return dict(zip(("min", "p50", "p95", "p99", "max"),
                    np.quantile(values, [0, .5, .95, .99, 1]).tolist())) if len(values) else {}


def main():
    with SOURCE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    vehicles = {cf: [row for row in rows if row["vehicle_id"] == cf] for cf in MASS}
    primary = vehicles["3"]
    start = next(float(r["control_time_s"]) for r in primary if r["flight_phase"] == "tension_ramp")
    emergency = next(r for r in primary if r["flight_phase"] == "emergency_landing")
    end = float(emergency["control_time_s"])
    result = {
        "source": str(SOURCE), "configured_payload_mass_kg": PAYLOAD_MASS,
        "configured_total_mass_kg": sum(MASS.values()) + PAYLOAD_MASS,
        "nominal_total_weight_n": (sum(MASS.values()) + PAYLOAD_MASS) * 9.80665,
        "handoff_s": start, "emergency_s": end,
        "emergency_reason": emergency["emergency_landing_reason"],
        "maximum_payload_clearance_m": float(np.nanmax(array(primary, "payload_ground_clearance_m"))),
        "vehicles": {},
    }
    for cf, rr in vehicles.items():
        t = array(rr, "control_time_s")
        rate = np.column_stack([array(rr, "body_rate_" + a) for a in "xyz"])
        cmd = np.column_stack([array(rr, "command_rate_" + a) for a in "xyz"])
        tilt = np.arccos(np.clip(np.cos(array(rr, "roll_rad")) * np.cos(array(rr, "pitch_rad")), -1, 1))
        force = array(rr, "command_thrust_newton")
        raw = array(rr, "command_raw_thrust")
        item = {"nominal_equal_share_hover_n": (MASS[cf] + PAYLOAD_MASS / 3) * 9.80665,
                "windows": {}}
        for label, lo, hi in (("prelift_loaded", start - 2, start),
                              ("ramp_0_1", start, start + 1),
                              ("ramp_1_2", start + 1, start + 2),
                              ("ramp_2_3", start + 2, start + 3),
                              ("ramp_3_5", start + 3, start + 5),
                              ("after_ramp", start + 5, end)):
            selected = (t >= lo) & (t < hi)
            item["windows"][label] = {
                "samples": int(selected.sum()),
                "thrust_n": quantiles(force[selected]),
                "raw_thrust": quantiles(raw[selected]),
                "battery_v": quantiles(array(rr, "battery_voltage_v")[selected]),
                "tilt_deg": quantiles(np.degrees(tilt[selected])),
                "xy_measured_rate_rms": np.sqrt(np.mean(rate[selected, :2] ** 2, axis=0)).tolist(),
                "xy_command_rate_rms": np.sqrt(np.mean(cmd[selected, :2] ** 2, axis=0)).tolist(),
                "any_xy_command_saturation_fraction": float(np.mean(np.any(np.abs(cmd[selected, :2]) >= 2.99, axis=1))),
                "thrust_command_limit_fraction": float(np.mean(force[selected] >= .999)),
                "position_error_z_m": quantiles(array(rr, "position_error_z")[selected]),
                "support_trim_n": quantiles(array(rr, "transport_support_trim_n")[selected]),
            }
        ramp = (t >= start) & (t < end)
        for field, condition in (("first_thrust_0_99_s", force >= .99),
                                 ("first_xy_rate_0_5_s", np.any(np.abs(rate[:, :2]) >= .5, axis=1)),
                                 ("first_xy_command_saturation_s", np.any(np.abs(cmd[:, :2]) >= 2.99, axis=1))):
            selected = t[ramp & condition]
            item[field] = float(selected[0]) if len(selected) else None
        b = np.nan_to_num(array(rr, "transport_blend"))[:, None]
        er = np.column_stack([array(rr, "attitude_error_" + a) for a in "xyz"])
        attitude_term = -((1 - b) * [12, 12, 6] + b * [6, 6, 3]) * er
        measured_rate_term = -b * [2, 2, 1] * rate
        item["rate_formula_max_error"] = float(np.max(np.abs(np.clip(
            attitude_term[ramp] + measured_rate_term[ramp], [-3, -3, -2], [3, 3, 2]) - cmd[ramp])))
        spectral = (t >= start + 2) & (t < end)
        uniform = np.arange(t[spectral][0], t[spectral][-1], 1 / 70)
        peaks = {}
        for index, a in enumerate("xy"):
            signal = np.interp(uniform, t[spectral], rate[spectral, index])
            signal -= np.polyval(np.polyfit(uniform, signal, 1), uniform)
            power = np.abs(np.fft.rfft(signal * np.hanning(len(signal)))) ** 2
            frequency = np.fft.rfftfreq(len(signal), 1 / 70)
            selected = np.flatnonzero((frequency > 1) & (frequency < 25))
            peaks[a] = float(frequency[selected[np.argmax(power[selected])]])
        item["measured_rate_spectrum_peak_hz"] = peaks
        reports = {}
        for r in rr:
            if r["bridge_latest_command_stamp_ns"] not in ("", "nan"):
                reports[(r["bridge_latest_command_stamp_ns"], r["bridge_latest_command_seq"])] = r
        item["unique_bridge_reports"] = len(reports)
        item["timing"] = {}
        for label, lo, hi in (("whole_flight", 0, float("inf")), ("ramp", start, end)):
            ss = [r for r in reports.values() if lo <= float(r["control_time_s"]) < hi]
            item["timing"][label] = {
                field: quantiles(array(ss, field) * 1000)
                for field in ("bridge_latest_command_age_s", "bridge_latest_send_call_wall_s")}
        result["vehicles"][cf] = item

    # A system force balance removes internal cable forces. The command Newton
    # values are not measured thrust; the ratio below is conditional on masses.
    t = array(primary, "control_time_s")
    z_com = sum(MASS[cf] * array(vehicles[cf], "position_z") for cf in MASS)
    z_com += PAYLOAD_MASS * array(primary, "payload_position_z")
    z_com /= sum(MASS.values()) + PAYLOAD_MASS
    vertical_command = sum(array(rr, "command_thrust_newton")
                           * np.cos(array(rr, "roll_rad")) * np.cos(array(rr, "pitch_rad"))
                           for rr in vehicles.values())
    selected = (t >= 50) & (t < start)
    acceleration = 2 * np.polyfit(t[selected] - t[selected][0], z_com[selected], 2)[0]
    required = (sum(MASS.values()) + PAYLOAD_MASS) * (9.80665 + acceleration)
    result["loaded_force_balance_50s_to_handoff"] = {
        "quadratic_fit_com_acceleration_mps2": float(acceleration),
        "required_total_vertical_force_n": float(required),
        "mean_total_vertical_command_n": float(np.mean(vertical_command[selected])),
        "required_over_command_ratio": float(required / np.mean(vertical_command[selected])),
        "note": "Conditional on configured masses; not a measured motor calibration.",
    }
    fig, axes = plt.subplots(5, 2, figsize=(15, 15), layout="constrained")
    colors = {"3": "#9467bd", "4": "#1f77b4", "5": "#2ca02c"}
    for col, limits in enumerate(((0, end + 3), (start - 3, end + .5))):
        axes[0, col].plot(t, array(primary, "payload_ground_clearance_m"), c="black", label="Payload bottom clearance")
        axes[0, col].axhline(.2, c="gray", ls="--", label="Prelift threshold")
        for cf, rr in vehicles.items():
            rate = np.column_stack([array(rr, "body_rate_" + a) for a in "xy"])
            series = (array(rr, "command_thrust_newton"), array(rr, "command_raw_thrust"),
                      np.max(np.abs(rate), axis=1), array(rr, "battery_voltage_v"))
            for row, values in enumerate(series, 1):
                axes[row, col].plot(t, values, color=colors[cf], label="CF" + cf, lw=.9)
        axes[1, col].axhline(1, c="gray", ls="--")
        axes[2, col].axhline(60000, c="gray", ls="--")
        for row, ylabel in enumerate(("Payload clearance (m)", "Command thrust (N)",
                                      "Collective raw command", "Max |measured xy rate| (rad/s)", "Battery (V)")):
            ax = axes[row, col]
            ax.set_xlim(*limits)
            ax.set_ylabel(ylabel)
            ax.axvline(start, c="#e67e22", ls=":", label="Handoff")
            ax.axvline(end, c="#d62728", ls=":", label="Height-drop landing")
            ax.grid(alpha=.25)
            ax.legend(fontsize=7, loc="best")
        axes[-1, col].set_xlabel("Control time (s)")
    fig.suptitle("Real flight 2026-10-16 11:01: loaded prelift and controller handoff")
    fig.savefig(OUT / "payload_margin.png", dpi=140)
    (OUT / "metrics.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
