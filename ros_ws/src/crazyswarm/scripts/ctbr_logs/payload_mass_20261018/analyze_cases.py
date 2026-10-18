#!/usr/bin/env python3
"""Summarize identical-parameter mass comparisons, including failed cases."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch

HERE = Path(__file__).resolve().parent


def column(rows, key):
    return np.array([float(r[key] or "nan") for r in rows])


def vector(rows, prefix):
    return np.column_stack([column(rows, prefix + "_" + a) for a in "xyz"])


def weighted_stats(t, x):
    if len(t) < 2:
        return None
    # Half the adjacent spacing weights each sample; exact for a constant.
    weights = np.r_[t[1] - t[0], t[2:] - t[:-2], t[-1] - t[-2]] / 2
    weights /= weights.sum()
    mean = np.sum(x * weights.reshape((-1,) + (1,) * (x.ndim - 1)), axis=0)
    var = np.sum((x - mean)**2 * weights.reshape((-1,) + (1,) * (x.ndim - 1)), axis=0)
    rms = np.sqrt(np.sum(x**2 * weights.reshape((-1,) + (1,) * (x.ndim - 1)), axis=0))
    return dict(mean=np.asarray(mean).tolist(), std=np.sqrt(var).tolist(), rms=np.asarray(rms).tolist())


def analyze(case):
    directory = Path(case["output"])
    summary_path = directory / "summary.json"
    if not summary_path.exists():
        return None, None
    summary = json.loads(summary_path.read_text())
    trace = list(csv.DictReader((directory / "simulation_trace.csv").open()))
    fleet = {ident: [r for r in trace if r["vehicle_id"] == str(ident)] for ident in (3, 4, 5)}
    rows = fleet[3]
    t = column(rows, "control_time_s")
    x = column(rows, "handoff_elapsed_s")
    result = {k: v for k, v in case.items() if k not in ("candidate", "output")}
    result.update({k: summary[k] for k in ("success", "failure", "terminal_reason", "observed_post_handoff_s",
                                          "minimum_vehicle_distance_m", "maximum_body_rate_rps", "rate_saturation_fraction",
                                          "final_payload_position_error_m", "handoff_time_s", "final_time_s")})
    result["windows"] = {}
    payload_w = vector(rows, "payload_body_rate")
    attitudes = np.rad2deg(np.column_stack([column(rows, "payload_" + a + "_rad") for a in ("roll", "pitch", "yaw")]))
    for start, end in ((0, 2), (2, 5), (5, 10), (10, 30), (40, 60)):
        mask = (x >= start) & (x < end)
        if mask.sum() < 2:
            continue
        window = dict(actual_span_s=float(t[mask][-1] - t[mask][0]),
                      payload_rates_rps=weighted_stats(t[mask], payload_w[mask]),
                      payload_rpy_deg=weighted_stats(t[mask], attitudes[mask]),
                      vehicles={})
        actual_total = np.zeros(mask.sum())
        for ident, records in fleet.items():
            command = column(records, "command_thrust_n")[mask]
            actual = column(records, "actual_thrust_n")[mask]
            actual_total += actual
            window["vehicles"][str(ident)] = dict(
                command_thrust_n=weighted_stats(t[mask], command),
                actual_thrust_n=weighted_stats(t[mask], actual),
                body_rates_rps=weighted_stats(t[mask], vector(records, "body_rate")[mask]),
                upper_motor_limit_sample_fraction=float(np.mean(column(records, "motor_at_upper_limit_count")[mask] > 0)),
            )
        window["total_actual_thrust_n"] = weighted_stats(t[mask], actual_total)
        result["windows"][f"{start}-{end}s"] = window
    finite = x >= 0
    if finite.sum() > 80:
        uniform = np.arange(t[finite][0], t[finite][-1], 1/70)
        spectra = {}
        for axis, label in enumerate("xyz"):
            signal = np.interp(uniform, t[finite], payload_w[finite, axis])
            frequencies, power = welch(signal, fs=70, nperseg=min(256, len(signal)))
            band = (frequencies > .5) & (frequencies < 10)
            spectra[label] = float(frequencies[band][np.argmax(power[band])])
        result["payload_rate_psd_peak_hz"] = spectra
    snapshot = __import__("yaml").safe_load((directory / "input_parameters.yaml").read_text())
    assert np.isclose(snapshot["slung_payload"]["mass_kg"], case["mass_g"] / 1000.)
    assert snapshot["slung_payload"]["attitude_bandwidth_hz"][:2] == [case["bandwidth_xy_hz"]] * 2
    assert snapshot["simulation_runtime"]["effective_plant"]["payload_rotational_damping"] == case["rotational_damping"]
    return result, (x, attitudes, payload_w, column(fleet[4], "command_thrust_n"))


def main():
    cases = json.loads((HERE / "cases.json").read_text())["cases"]
    groups = [(1., "configured_damping"), (2., "configured_damping"),
              (1., "zero_damping_bound"), (2., "zero_damping_bound")]
    fig, axes = plt.subplots(4, 3, figsize=(17, 14), constrained_layout=True, sharex="row")
    results = []
    colors = {54: "#a33c31", 40: "#d7911d", 27: "#2166ac"}
    for case in cases:
        result, data = analyze(case)
        if result is None:
            continue
        results.append(result)
        x, angles, w, thrust = data
        row = groups.index((case["bandwidth_xy_hz"], case["scenario"]))
        select = x >= 0
        label = f"{case['mass_g']} g" + (" (guard exit)" if not result["success"] else "")
        color = colors[case["mass_g"]]
        axes[row, 0].plot(x[select], angles[select, 1], color=color, label=label)
        axes[row, 1].plot(x[select], w[select, 2], color=color, label=label, alpha=.8)
        axes[row, 2].plot(x[select], thrust[select], color=color, label=label, alpha=.8)
        end = np.max(x)
        if not result["success"]:
            for ax in axes[row]:
                ax.axvline(end, color=color, linestyle=":", linewidth=.8)
    for row, (bandwidth, scenario) in enumerate(groups):
        name = "configured c=0.001" if scenario == "configured_damping" else "sensitivity bound c=0"
        axes[row, 0].set_title(f"{bandwidth:g} Hz, {name}: payload pitch")
        axes[row, 1].set_title("Physical payload yaw rate")
        axes[row, 2].set_title("CF4 collective command")
        axes[row, 0].set_ylabel("Angle [deg]")
        axes[row, 1].set_ylabel("Rate [rad/s]")
        axes[row, 2].set_ylabel("Command [N]")
        row_end = (60 if scenario == "configured_damping" else
                   max(result["observed_post_handoff_s"] for result in results
                       if result["scenario"] == scenario and result["bandwidth_xy_hz"] == bandwidth) + .5)
        for ax in axes[row]:
            ax.set(xlabel="Time after handoff [s]", xlim=(0, row_end))
            ax.axvline(5, color="gray", linestyle=":", linewidth=.7)
            ax.grid(alpha=.25)
            ax.legend(fontsize=7)
    fig.suptitle("Payload mass comparison — same geometry, gains, delays and measured control-interval pattern\n"
                 "Each row is a separate modeling assumption; zero damping is not a measured real-flight value.")
    fig.savefig(HERE / "comparison.png", dpi=140)
    plt.close(fig)
    (HERE / "comparison.json").write_text(json.dumps(results, indent=2))
    fields = ["name", "mass_g", "bandwidth_xy_hz", "rotational_damping", "success", "failure",
              "observed_post_handoff_s", "maximum_body_rate_rps", "minimum_vehicle_distance_m"]
    with (HERE / "comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k: result[k] for k in fields} for result in results)
    for result in results:
        late = result["windows"].get("40-60s")
        short = result["windows"].get("5-10s")
        print(json.dumps(dict(name=result["name"], success=result["success"],
                             observed_s=result["observed_post_handoff_s"],
                             late_pitch_deg=late["payload_rpy_deg"]["mean"][1] if late else None,
                             late_CF4_mean_command_n=late["vehicles"]["4"]["command_thrust_n"]["mean"] if late else None,
                             late_total_actual_thrust_n=late["total_actual_thrust_n"]["mean"] if late else None,
                             short_yaw_std_rps=short["payload_rates_rps"]["std"][2] if short else None,
                             psd_hz=result.get("payload_rate_psd_peak_hz"))))


if __name__ == "__main__":
    main()
