#!/usr/bin/env python3
"""Reproduce the 22:24:18 flight audit; no ROS or hardware access.

Euler angles in the latest log are direct logged state/target values. Cable
wrench projection is a commanded model quantity, not a tension measurement.
"""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
LATEST = HERE.parent / "multi_ctbr_20261018_222418.csv"
PRIOR = HERE.parent / "multi_ctbr_20261018_211815.csv"
IDS = (3, 4, 5)
RHO = np.array([[.04, 0., .025], [-.04, .03, .025], [-.04, -.03, .025]])


def number(rows, name):
    return np.array([float(row.get(name) or "nan") for row in rows])


def vector(rows, prefix):
    return np.column_stack([number(rows, prefix + "_" + axis) for axis in "xyz"])


def euler(rows, prefix):
    return np.column_stack([number(rows, prefix + axis + "_rad")
                            for axis in ("roll", "pitch", "yaw")])


def stats(values):
    return dict(mean=np.nanmean(values, axis=0).tolist(),
                std=np.nanstd(values, axis=0).tolist(),
                min=np.nanmin(values, axis=0).tolist(),
                max=np.nanmax(values, axis=0).tolist())


def transitions(rows, name):
    return [dict(time_s=float(row["control_time_s"]), value=row.get(name, ""))
            for i, row in enumerate(rows)
            if not i or row.get(name, "") != rows[i-1].get(name, "")]


def load(path):
    with path.open(newline="") as source:
        raw = list(csv.DictReader(source))
    fleet = {i: [row for row in raw if row["vehicle_id"] == str(i)] for i in IDS}
    # Each fleet tick has the same time; one last CF3 row may lack CF4/CF5.
    common = min(map(len, fleet.values()))
    aligned = {i: rows[:common] for i, rows in fleet.items()}
    rows = aligned[3]
    time = number(rows, "control_time_s")
    assert all(np.array_equal(time, number(aligned[i], "control_time_s")) for i in IDS)
    handoff = next(float(row["control_time_s"]) for row in rows
                   if row["flight_phase"] == "tension_ramp")
    exit_time = next(float(row["control_time_s"]) for row in rows
                     if row["flight_phase"] in ("landing", "emergency_landing"))
    positions = np.stack([vector(aligned[i], "position") for i in IDS], axis=1)
    distances = np.column_stack([np.linalg.norm(positions[:, a] - positions[:, b], axis=1)
                                 for a, b in ((0, 1), (0, 2), (1, 2))])
    return dict(path=str(path), raw=raw, fleet=fleet, aligned=aligned, rows=rows,
                time=time, relative_time=time-handoff, handoff=handoff,
                exit_time=exit_time, positions=positions, distances=distances,
                thrust=np.column_stack([number(aligned[i], "command_thrust_newton") for i in IDS]),
                rates=np.stack([vector(aligned[i], "body_rate") for i in IDS], axis=1),
                payload_rates=vector(rows, "payload_body_rate"))


def matched_windows(data):
    result = {}
    for start, end in ((0, 2), (2, 5), (5, 7), (7, 9.5)):
        mask = (data["relative_time"] >= start) & (data["relative_time"] < end)
        result[f"{start}-{end}s"] = dict(
            samples_per_vehicle=int(mask.sum()),
            # Same definition as the 21:18:15 analysis.
            transverse_body_rate_rms_rps=np.sqrt(np.mean(
                np.sum(data["rates"][mask, :, :2]**2, axis=2)/2, axis=0)).tolist(),
            command_thrust_mean_n=np.mean(data["thrust"][mask], axis=0).tolist(),
            command_thrust_std_n=np.std(data["thrust"][mask], axis=0).tolist(),
            payload_body_rate_std_rps=np.std(data["payload_rates"][mask], axis=0).tolist(),
            pair_distance_min_m=np.min(data["distances"][mask], axis=0).tolist())
    return result


def main():
    latest, prior = load(LATEST), load(PRIOR)
    rows, time, rel = latest["rows"], latest["time"], latest["relative_time"]
    attitude = euler(rows, "payload_")
    target = euler(rows, "payload_target_")
    valid = np.all(np.isfinite(attitude), axis=1)
    rotations = np.full((len(rows), 3, 3), np.nan)
    rotations[valid] = Rotation.from_euler("xyz", attitude[valid]).as_matrix()
    # Angle of measured/desired payload +z from world +z, independent of yaw.
    tilt = np.rad2deg(np.arccos(np.clip(np.cos(attitude[:, 0])*np.cos(attitude[:, 1]), -1, 1)))
    target_tilt = np.rad2deg(np.arccos(np.clip(np.cos(target[:, 0])*np.cos(target[:, 1]), -1, 1)))
    active = (time >= latest["handoff"]) & (time < latest["exit_time"])
    final = active & (rel >= 9.)
    gain = number(rows, "effective_transport_link_gain_scale")
    progress = number(rows, "payload_attitude_reference_progress")
    q = np.stack([vector(latest["aligned"][i], "link_direction") for i in IDS], axis=1)
    qd = np.stack([vector(latest["aligned"][i], "desired_link_direction") for i in IDS], axis=1)
    tensions = np.column_stack([number(latest["aligned"][i], "desired_tension_n") for i in IDS])
    modeled_forces = -tensions[:, :, None]*q
    modeled_body_forces = np.einsum("nvi,nij->nvj", modeled_forces, rotations)
    projected_moments = np.cross(RHO[None, :, :], modeled_body_forces).sum(axis=1)
    requested_moments = vector(rows, "payload_desired_moment")
    hold = np.stack([vector(latest["aligned"][i], "position_hold_position") for i in IDS], axis=1)
    cable_angles = np.rad2deg(np.arccos(np.clip(np.sum(q*qd, axis=2), -1, 1)))
    handoff_index = np.flatnonzero(active)[0]
    payload_positions = vector(rows, "payload_position")
    attachments = payload_positions[:, None, :] + np.einsum("nij,vj->nvi", rotations, RHO)
    geometric_distances = np.linalg.norm(attachments-latest["positions"], axis=2)
    # At handoff the position-hold anchors are the aircraft measured anchors.
    # These are the d0 distances frozen by the equilibrium reference generator;
    # unlike the model's nominal length, they preserve the handoff geometry.
    frozen_reference_d0 = np.linalg.norm(attachments[handoff_index]-hold[handoff_index], axis=1)
    modeled_wrench_windows = {}
    for a, b in ((0, 2), (7, 10.00001)):
        mask = active & (rel >= a) & (rel < b)
        modeled_wrench_windows[f"{a}-{int(round(b))}s"] = dict(
            requested_body_moment_mnm=stats(requested_moments[mask]*1e3),
            projected_desired_tension_body_moment_mnm=stats(projected_moments[mask]*1e3),
            projected_minus_requested_moment_mnm=stats((projected_moments-requested_moments)[mask]*1e3),
            caveat="sum rho_i cross (R.T @ (-T_i q_i)); desired tensions, not measured cable forces; does not measure the physical tension changes due to aircraft PD hold or support trim.")
    pre_exit_index = np.flatnonzero(active)[-1]
    snapshots = []
    for elapsed in (0, 2, 5, 7, 9, 10, 10.01425200898666, 12, 16.53478167398134, 18.53481964900857):
        index = int(np.argmin(np.abs(rel-elapsed)))
        snapshots.append(dict(time_s=float(time[index]), elapsed_s=float(rel[index]),
                              actual_rpy_deg=np.rad2deg(attitude[index]).tolist(),
                              target_rpy_deg=np.rad2deg(target[index]).tolist(),
                              actual_tilt_deg=float(tilt[index]),
                              reference_progress=float(progress[index]),
                              actual_z_m=float(number(rows, "payload_position_z")[index]),
                              target_z_m=float(number(rows, "payload_target_z")[index]),
                              landing_stage=rows[index]["flight_validation_landing_stage"]))
    metrics = dict(
        source=str(LATEST), row_count=len(latest["raw"]),
        rows_by_vehicle={i: len(latest["fleet"][i]) for i in IDS},
        handoff_s=latest["handoff"], landing_s=latest["exit_time"],
        transport_duration_s=latest["exit_time"]-latest["handoff"],
        planned_attitude_reference_duration_s=12.,
        last_transport=dict(time_s=float(time[pre_exit_index]),
                            elapsed_s=float(rel[pre_exit_index]),
                            reference_progress=float(progress[pre_exit_index]),
                            actual_rpy_deg=np.rad2deg(attitude[pre_exit_index]).tolist(),
                            target_rpy_deg=np.rad2deg(target[pre_exit_index]).tolist(),
                            actual_tilt_deg=float(tilt[pre_exit_index])),
        force_modes=sorted({row["payload_transport_force_mode"] for row in rows if row["payload_transport_force_mode"]}),
        active_effective_link_gain=stats(gain[active]),
        active_raw_link_force_max_abs_n=max(float(np.nanmax(np.abs(vector(latest["aligned"][i], "raw_link_force")[active]))) for i in IDS),
        frozen_reference_geometry_d0_m=frozen_reference_d0.tolist(),
        model_nominal_link_lengths_m=[float(latest["aligned"][i][handoff_index]["link_length_nominal_m"]) for i in IDS],
        last_second_instantaneous_geometry_link_length_m=stats(geometric_distances[final]),
        modeled_wrench_windows=modeled_wrench_windows,
        final_second=dict(actual_rpy_deg=stats(np.rad2deg(attitude[final])),
                          target_rpy_deg=stats(np.rad2deg(target[final])),
                          actual_tilt_deg=stats(tilt[final]), target_tilt_deg=stats(target_tilt[final]),
                          vehicle_hold_error_mean_m=np.mean(latest["positions"][final]-hold[final], axis=0).tolist(),
                          vehicle_hold_error_rms_m=np.sqrt(np.mean((latest["positions"][final]-hold[final])**2, axis=0)).tolist(),
                          cable_angle_to_allocated_direction_deg=stats(cable_angles[final]),
                          requested_payload_moment_mnm=stats(requested_moments[final]*1e3),
                          projected_desired_tension_moment_mnm=stats(projected_moments[final]*1e3)),
        transitions={name: transitions(rows, name) for name in (
            "flight_phase", "payload_prelift_stage", "flight_validation_landing_stage",
            "payload_landing_touchdown_confirmed", "payload_landing_unloading")},
        snapshots=snapshots, matched_windows={"latest": matched_windows(latest), "prior": matched_windows(prior)},
        active_pair_distance_min_m=np.min(latest["distances"][active], axis=0).tolist(),
        active_payload_state_age_s=stats(number(rows, "payload_state_age_s")[active]),
        active_flight_safety_reasons=sorted({row["payload_safety_reason"] for row, ok in zip(rows, active) if ok}),
        ending_states={i: {key: latest["fleet"][i][-1][key] for key in (
            "control_time_s", "flight_phase", "flight_validation_landing_stage", "position_z", "target_z",
            "command_thrust_newton", "command_raw_thrust", "payload_position_z", "payload_ground_clearance_m")} for i in IDS},
        caveat="Angles are logged mocap-derived state, not an independent physical leveling check. Projected desired tensions are model commands, not measured tensions; their wrench does not include the unknown physical effect of aircraft hold feedback. No claim identifying COM/attachment/calibration as the cause is possible from this CSV alone.",
    )
    metrics["prelift_attitudes_deg"] = {}
    for a, b in ((2, 3), (20.4, 21.4), (35.4, 36.4)):
        mask = (time >= a) & (time < b)
        metrics["prelift_attitudes_deg"][f"{a}-{b}s"] = stats(np.rad2deg(attitude[mask]))
    (HERE/"metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=True)+"\n")
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), constrained_layout=True)
    shown = (rel >= -3) & (rel <= 19)
    for col, name in enumerate(("roll", "pitch", "yaw")):
        axes[0, 0].plot(rel[shown], np.rad2deg(attitude[shown, col]), label=name)
        axes[0, 0].plot(rel[shown], np.rad2deg(target[shown, col]), "--", label=name+" target")
    axes[0, 1].plot(rel[shown], tilt[shown], label="actual tilt")
    axes[0, 1].plot(rel[shown], target_tilt[shown], "--", label="target tilt")
    axes[0, 1].plot(rel[shown], progress[shown]*20, ":", label="reference progress x20")
    for index, ident in enumerate(IDS):
        axes[1, 0].plot(rel[shown], np.sqrt(np.sum(latest["rates"][shown, index, :2]**2, axis=1)/2), label=f"CF{ident}")
        axes[1, 1].plot(rel[shown], latest["thrust"][shown, index], label=f"CF{ident}")
    axes[2, 0].plot(rel[shown], number(rows, "payload_position_z")[shown], label="payload z")
    axes[2, 0].plot(rel[shown], number(rows, "payload_target_z")[shown], "--", label="target z")
    for index, pair in enumerate(("3-4", "3-5", "4-5")):
        axes[2, 1].plot(rel[shown], latest["distances"][shown, index], label=pair)
    titles = ("Logged load attitude and target", "Payload z-axis tilt and transition progress", "Aircraft transverse body rate magnitude", "Host thrust command", "Payload height", "Aircraft pair distances")
    units = ("deg", "deg / scaled progress", "rad/s", "N", "m", "m")
    for ax, title, unit in zip(axes.flat, titles, units):
        ax.set(title=title, ylabel=unit, xlabel="Time since handoff [s]", xlim=(-3, 19))
        for at, style in ((0, ":"), (5, ":"), (10.01425200898666, "--"), (12, ":"), (16.53478167398134, "--"), (18.53481964900857, "--")):
            ax.axvline(at, color="gray", linestyle=style, linewidth=.7)
        ax.grid(True, alpha=.2); ax.legend(fontsize=8, ncol=2)
    fig.suptitle("Real flight 2026-10-18 22:24:18 — reference cascade\nHandoff 0s; force ramp 5s; planned lowering 10.014s; intended attitude endpoint 12s; touchdown 16.535s; aircraft descent 18.535s")
    fig.savefig(HERE/"analysis.png", dpi=160)
    plt.close(fig)
    print(json.dumps({key: metrics[key] for key in ("handoff_s", "landing_s", "last_transport", "final_second", "matched_windows")}, indent=2))


if __name__ == "__main__":
    main()
