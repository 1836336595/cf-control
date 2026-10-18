"""Read-only flight telemetry comparison; does not import ROS or alter parameters."""
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
LOGS = ROOT.parent


def a(rows, key):
    return np.array([float(r.get(key, "") or "nan") for r in rows])


def stats(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(n=len(values), mean=float(values.mean()), min=float(values.min()),
                p50=float(np.median(values)), p95=float(np.percentile(values, 95)),
                p99=float(np.percentile(values, 99)), max=float(values.max()))


def analyze(name):
    with (LOGS / name).open() as stream:
        rows = list(csv.DictReader(stream))
    groups = {i: [r for r in rows if r["vehicle_id"] == str(i)] for i in (3, 4, 5)}
    active_phases = {"tension_ramp", "height_correction", "transport", "hover"}
    result = dict(file=name, rows=len(rows), vehicles={}, phases=[])
    last = None
    for r in groups[3]:
        if r["flight_phase"] != last:
            last = r["flight_phase"]
            result["phases"].append([float(r["control_time_s"]), last])
    active3 = [r for r in groups[3] if r["flight_phase"] in active_phases]
    handoff = float(active3[0]["control_time_s"]) if active3 else np.nan
    result["handoff_s"] = handoff
    if active3:
        result["transport_span_s"] = float(active3[-1]["control_time_s"]) - handoff
        result["controller_mass_fit"] = []
        size = [.08, .06, .05]
        for j, axis in enumerate("xyz"):
            design = -np.column_stack([a(active3, "payload_attitude_error_" + axis),
                                       a(active3, "payload_body_rate_" + axis)])
            moment = a(active3, "payload_desired_moment_" + axis)
            good = np.isfinite(design).all(axis=1) & np.isfinite(moment)
            fit, *_ = np.linalg.lstsq(design[good], moment[good], rcond=None)
            bandwidth = [1., 1., .45][j]
            moment_per_mass = sum(size[k] ** 2 for k in range(3) if k != j) / 12
            result["controller_mass_fit"].append(dict(axis=axis, attitude_gain=float(fit[0]),
                rate_gain=float(fit[1]), estimated_mass_from_attitude_kg=float(fit[0] / (moment_per_mass * (2*np.pi*bandwidth)**2)),
                residual_rms=float(np.sqrt(np.mean((design[good] @ fit - moment[good])**2)))))
    for i, rr in groups.items():
        tr = [r for r in rr if r["flight_phase"] in active_phases]
        end = rr[-1]
        d = dict(rows=len(rr), final_time=float(end["control_time_s"]),
                 final_phase=end["flight_phase"], final_z=float(end["position_z"]),
                 final_target_z=float(end["target_z"]), final_thrust=float(end["command_thrust_newton"]),
                 preflight_voltage=stats(a(rr, "preflight_voltage_v")), transport={})
        for key in ["fleet_tick_interval_s", "previous_fleet_cycle_wall_s", "controller_compute_wall_s",
                    "state_age_s", "mocap_sample_age_s", "ekf_state_age_s", "payload_state_age_s",
                    "bridge_latest_command_age_s", "bridge_latest_send_call_wall_s", "battery_voltage_v",
                    "command_thrust_newton", "command_raw_thrust", "bridge_latest_raw_thrust",
                    "transport_support_trim_n", "desired_tension_n", "link_length_nominal_m",
                    "link_length_effective_m", "link_length_measured_m", "transport_max_tilt_rad"]:
            d["transport"][key] = stats(a(tr, key))
        for key in ["ekf_status", "ekf_kinematics_held", "derivatives_valid", "link_calibration_applied",
                    "link_calibration_status", "link_calibration_reason", "link_force_rate_limited",
                    "link_force_magnitude_limited", "payload_landing_rope_preserving"]:
            d["transport"][key] = dict(Counter(r[key] for r in tr))
        d["whole_flight_rope_preserving_counts"] = dict(Counter(r["payload_landing_rope_preserving"] for r in rr))
        d["whole_flight_thrust_max"] = float(np.max(a(rr, "command_thrust_newton")))
        d["windows"] = []
        for lo, hi in [(0, 2), (2, 5), (5, 7), (7, 9.5)]:
            wr = [r for r in tr if lo <= float(r["control_time_s"]) - handoff < hi]
            if not wr:
                continue
            pq = np.column_stack([a(wr, "body_rate_x"), a(wr, "body_rate_y")])
            d["windows"].append(dict(start=lo, end=hi, n=len(wr),
                xy_body_rate_rms=float(np.sqrt(np.mean(pq**2))),
                thrust_mean=float(np.mean(a(wr, "command_thrust_newton"))),
                thrust_std=float(np.std(a(wr, "command_thrust_newton"))),
                payload_yaw_rate_std=float(np.std(a(wr, "payload_body_rate_z")))))
        result["vehicles"][str(i)] = d
    unique = {float(r["control_time_s"]): {} for r in rows}
    for r in rows:
        unique[float(r["control_time_s"])][int(r["vehicle_id"])] = r
    pairs = []
    for time, states in unique.items():
        if len(states) != 3:
            continue
        for i, j in [(3, 4), (3, 5), (4, 5)]:
            dist = float(np.linalg.norm(np.array([float(states[i]["position_"+k]) for k in "xyz"]) -
                                        np.array([float(states[j]["position_"+k]) for k in "xyz"])))
            pairs.append((dist, time, i, j))
    result["minimum_pair"] = min(pairs)
    result["minimum_pair_transport"] = min(x for x in pairs if handoff <= x[1] <= float(active3[-1]["control_time_s"]))
    result["first_emergency"] = next(({k:r[k] for k in ["control_time_s", "emergency_landing_reason", "payload_ground_clearance_m", "position_z", "command_thrust_newton"]}
                                      for r in groups[3] if r["flight_phase"] == "emergency_landing"), None)
    result["handoff_pair_distances"] = [x for x in pairs if x[1] == handoff]
    return result


if __name__ == "__main__":
    result = [analyze("multi_ctbr_20261018_" + stamp + ".csv") for stamp in ["211815", "210320", "203211"]]
    (ROOT / "telemetry.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    for run in result:
        print(run["file"], "handoff", run["handoff_s"], "span", run["transport_span_s"], "minpair", run["minimum_pair"])
        print("mass fit", run["controller_mass_fit"])
        for i, d in run["vehicles"].items():
            print(i, "end", d["final_z"], d["final_thrust"], "windows", d["windows"])
            print("telemetry", {k:v for k,v in d["transport"].items() if k in ["fleet_tick_interval_s", "state_age_s", "ekf_status", "ekf_kinematics_held", "bridge_latest_command_age_s", "bridge_latest_send_call_wall_s", "battery_voltage_v", "command_thrust_newton", "command_raw_thrust", "link_length_measured_m"]})
