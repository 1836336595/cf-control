#!/usr/bin/env python3
"""Summarize actual physical handoff attitude and subsequent oscillation windows."""

import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.signal import detrend, welch


def analyze(directory):
    directory = Path(directory)
    summary = json.loads((directory / "summary.json").read_text())
    rows = [row for row in csv.DictReader((directory / "simulation_trace.csv").open())
            if row["vehicle_id"] == "3"]
    time = np.array([float(row["control_time_s"]) for row in rows])
    rpy = np.rad2deg([[float(row["payload_" + name + "_rad"]) for name in ("roll", "pitch", "yaw")]
                    for row in rows])
    handoff = summary["handoff_time_s"]
    metrics = {name: summary[name] for name in ("success", "failure", "handoff_time_s", "final_time_s",
        "observed_post_handoff_s", "initial_physical_cable_errors_m", "minimum_clearance_m",
        "minimum_vehicle_distance_m", "maximum_tilt_deg", "rate_saturation_fraction",
        "final_payload_position_error_m", "terminal_reason")}
    metrics["initial_physical_payload_rpy_deg"] = rpy[0].tolist()
    metrics["final_physical_payload_rpy_deg"] = rpy[-1].tolist()
    metrics["handoff_physical_payload_rpy_deg"] = None
    metrics["windows"] = []
    if handoff is not None:
        index = int(np.argmin(np.abs(time - handoff)))
        metrics["handoff_physical_payload_rpy_deg"] = rpy[index].tolist()
        for lower, upper in ((0, 5), (5, 15), (15, 30), (30, 60)):
            mask = (time >= handoff + lower) & (time < handoff + upper)
            if np.count_nonzero(mask) < 10:
                continue
            selected_time, selected_rpy = time[mask], rpy[mask]
            sampling_period = float(np.median(np.diff(selected_time)))
            regular_time = np.arange(selected_time[0], selected_time[-1], sampling_period)
            regular_pitch = np.interp(regular_time, selected_time, selected_rpy[:, 1])
            frequency, power = welch(detrend(regular_pitch), fs=1. / sampling_period,
                nperseg=min(512, len(regular_time)))
            band = (frequency >= 1.) & (frequency <= 3.)
            metrics["windows"].append({"start_after_handoff_s": lower, "end_after_handoff_s": upper,
                "actual_duration_s": float(selected_time[-1]-selected_time[0]),
                "physical_payload_mean_rpy_deg": np.mean(selected_rpy, axis=0).tolist(),
                "physical_payload_std_rpy_deg": np.std(selected_rpy, axis=0).tolist(),
                "physical_payload_peak_to_peak_rpy_deg": np.ptp(selected_rpy, axis=0).tolist(),
                "pitch_dominant_1_3_hz": float(frequency[band][np.argmax(power[band])]) if np.any(band) else None,
                "pitch_1_3_hz_rms_deg": float(np.sqrt(np.sum(power[band]) * (frequency[1]-frequency[0])))
                    if len(frequency) > 1 else None})
    (directory / "physical_attitude_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    return metrics


if __name__ == "__main__":
    print(json.dumps(analyze(Path(sys.argv[1])), indent=2))
