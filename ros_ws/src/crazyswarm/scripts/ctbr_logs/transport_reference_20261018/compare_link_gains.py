#!/usr/bin/env python3
"""Compare only link-loop gains in the same zero-damping physical boundary case."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import detrend, welch

from analyze_case import analyze
from compare_cases import trace_data


HERE = Path(__file__).resolve().parent


def main():
    cases = (("consistent_observed_q_damping000", "kq=40, kw=20", "#bc4b35"),
             ("consistent_observed_q_damping000_link20_10", "kq=20, kw=10", "#246db0"),
             ("consistent_observed_q_damping000_link10_8", "kq=10, kw=8", "#26945d"))
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    results = {}
    final_times = []
    for name, label, color in cases:
        directory = HERE / name
        metrics = analyze(directory)
        results[name] = metrics
        time, values, summary = trace_data(directory)
        # All three cases survive this same window, avoiding comparisons
        # between a full ten-second spectrum and a truncated failure window.
        common = (time >= 5.) & (time < 10.)
        common_time = time[common]
        dt = float(np.median(np.diff(common_time)))
        regular_time = np.arange(common_time[0], common_time[-1], dt)
        regular_pitch = np.interp(regular_time, common_time, values[common, 0])
        frequency, power = welch(detrend(regular_pitch), fs=1./dt,
                                  nperseg=len(regular_pitch))
        band = (frequency >= 1.) & (frequency <= 3.)
        metrics["common_5_10_s"] = {
            "pitch_mean_deg": float(np.mean(values[common, 0])),
            "pitch_std_deg": float(np.std(values[common, 0])),
            "yaw_std_deg": float(np.std(values[common, 2])),
            "pitch_dominant_1_3_hz": float(frequency[band][np.argmax(power[band])]),
            "pitch_1_3_hz_rms_deg": float(np.sqrt(np.sum(power[band]) * (frequency[1]-frequency[0])))}
        mask = time >= 0.
        final_times.append(float(time[mask][-1]))
        terminal = "60 s complete" if summary["success"] else "guard %.2f s" % summary["observed_post_handoff_s"]
        for axis, index in zip(axes, (0, 1, 2, 3)):
            axis.plot(time[mask], values[mask, index], color=color, lw=1.15,
                      label=label + ": " + terminal)
            if not summary["success"]:
                axis.plot(time[mask][-1], values[mask, index][-1], "x", color=color, ms=7)
    for axis, label in zip(axes, ("Load pitch (deg)", "Load roll (deg)",
                                  "Load yaw (deg)", "Minimum aircraft distance (m)")):
        axis.set_ylabel(label)
        axis.grid(alpha=.25)
        axis.axvline(5., color="grey", ls=":", lw=1.)
        axis.set_xlim(0, max(final_times) + .5)
    axes[0].legend(loc="best", fontsize=9)
    axes[-1].axhline(.30, color="black", ls="--", lw=1.)
    axes[-1].set_xlabel("Time since physical handoff (s)")
    fig.suptitle("Link-loop gain sensitivity: zero load damping boundary\n"
                 "23 g / three 0.850 m cables / reference enabled / 70 Hz control and EKF", fontsize=13)
    fig.savefig(HERE / "link_gain_comparison.png", dpi=180)
    plt.close(fig)
    (HERE / "link_gain_comparison.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps({name: {key: metrics[key] for key in ("success", "observed_post_handoff_s",
        "handoff_physical_payload_rpy_deg", "final_physical_payload_rpy_deg", "terminal_reason")}
        for name, metrics in results.items()}, indent=2))


if __name__ == "__main__":
    main()
