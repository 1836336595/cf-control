#!/usr/bin/env python3
"""Audit logged geometry and command growth; inferred forces are not measurements."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "multi_ctbr_20261016_213804.csv"
# Known configured top-face attachment coordinates, relative to geometric center.
RHO = np.array([[.04, 0, .025], [-.04, .03, .025], [-.04, -.03, .025]])
SIZE = np.array([.08, .06, .05])


def number(rows, name):
    return np.array([float(row.get(name) or "nan") for row in rows])


def vector(rows, prefix):
    return np.column_stack([number(rows, prefix + "_" + axis) for axis in "xyz"])


def hat(a):
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])


def main():
    with SOURCE.open(newline="") as stream:
        raw = list(csv.DictReader(stream))
    fleet = {i: [row for row in raw if row["vehicle_id"] == str(i)] for i in (3, 4, 5)}
    rows = fleet[3]
    time = number(rows, "control_time_s")
    handoff_index = next(i for i, row in enumerate(rows) if row["flight_phase"] == "tension_ramp")
    fault_index = next(i for i, row in enumerate(rows) if row["flight_phase"] == "emergency_landing")
    handoff, fault = time[handoff_index], time[fault_index]
    x = time - handoff
    attitude_error = vector(rows, "payload_attitude_error")
    error_norm = np.linalg.norm(attitude_error, axis=1)
    rotvec = attitude_error * np.divide(np.arcsin(np.clip(error_norm, 0, 1)), error_norm,
                                        out=np.ones_like(error_norm), where=error_norm > 1e-10)[:, None]
    # Only principal relative error rotation: does not recover absolute payload yaw.
    relative_euler = np.full_like(rotvec, np.nan)
    valid = np.all(np.isfinite(rotvec), axis=1)
    relative_euler[valid] = Rotation.from_rotvec(rotvec[valid]).as_euler("xyz", degrees=True)
    payload_w = vector(rows, "payload_body_rate")
    vehicle_w = np.stack([vector(fleet[i], "body_rate") for i in (3, 4, 5)], axis=1)
    thrust = np.column_stack([number(fleet[i], "command_thrust_newton") for i in (3, 4, 5)])
    tensions = np.column_stack([number(fleet[i], "desired_tension_n") for i in (3, 4, 5)])
    positions = np.stack([vector(fleet[i], "position") for i in (3, 4, 5)], axis=1)
    pair_distances = np.column_stack([np.linalg.norm(positions[:, a] - positions[:, b], axis=1)
                                     for a, b in ((0, 1), (0, 2), (1, 2))])
    blend = number(fleet[4], "transport_blend")
    cf4_attitude = vector(fleet[4], "attitude_error")
    cf4_w = vector(fleet[4], "body_rate")
    attitude_term = -np.array([12., 12., 6.]) * cf4_attitude
    rate_term = -np.array([.25, .25, .125]) * blend[:, None] * cf4_w
    command = vector(fleet[4], "command_rate")

    matrix = np.zeros((6, 9))
    for i in range(3):
        matrix[:3, 3*i:3*i+3] = np.eye(3)
        matrix[3:, 3*i:3*i+3] = hat(RHO[i])
    null = np.linalg.svd(matrix)[2][6:].T
    radial = RHO.copy(); radial[:, 2] = 0
    radial /= np.linalg.norm(radial, axis=1)[:, None]
    bias = null @ (null.T @ (.25 * .054 * 9.80665 / np.sqrt(3) * radial).ravel())
    bias *= (.25 * .054 * 9.80665 / np.sqrt(3)) / np.sqrt(np.mean(np.sum(bias.reshape(3, 3)**2, axis=0)))
    samples = np.unique(np.r_[handoff_index, np.linspace(handoff_index, fault_index-1, 30).astype(int)])
    inferred = []
    previous = np.zeros(3)
    for index in samples:
        p0 = vector(rows[index:index+1], "payload_position")[0]
        q = np.array([vector(fleet[i][index:index+1], "link_direction")[0] for i in (3, 4, 5)])
        pv = positions[index]
        def residual(value):
            rotation = Rotation.from_rotvec(value).as_matrix()
            links = p0 + RHO @ rotation.T - pv
            links /= np.linalg.norm(links, axis=1)[:, None]
            return (links - q).ravel()
        solution = least_squares(residual, previous, xtol=1e-12, ftol=1e-12, gtol=1e-12)
        previous = solution.x
        rotation = Rotation.from_rotvec(previous).as_matrix()
        projected = -tensions[index, :, None] * q
        modeled_force = projected.sum(axis=0)
        modeled_moment = np.sum(np.cross(RHO, projected @ rotation), axis=0)
        requested_force = vector(rows[index:index+1], "payload_desired_force")[0]
        requested_moment = vector(rows[index:index+1], "payload_desired_moment")[0]
        bias_world = bias.reshape(3, 3) @ rotation.T
        projected_bias = np.sum(q * bias_world, axis=1)[:, None] * q
        inferred.append(dict(time_s=float(time[index]), rpy_deg=Rotation.from_matrix(rotation).as_euler("xyz", degrees=True).tolist(),
                             direction_fit_residual=float(np.linalg.norm(solution.fun)),
                             projected_force_residual_n=(modeled_force-requested_force).tolist(),
                             projected_moment_residual_nm=(modeled_moment-requested_moment).tolist(),
                             requested_moment_nm=requested_moment.tolist(), projected_moment_nm=modeled_moment.tolist(),
                             projected_bias_force_n=projected_bias.sum(axis=0).tolist(),
                             projected_bias_moment_nm=np.sum(np.cross(RHO, projected_bias @ rotation), axis=0).tolist()))
    metrics = {"source": str(SOURCE), "handoff_s": float(handoff), "fault_s": float(fault),
               "fault_reason": rows[fault_index]["emergency_landing_reason"],
               "inference_caveat": "Geometry inversion recovers orientation used in logged cable model, not independently measured physical orientation. Projected desired tensions are commanded model quantities, not measured cable tensions.",
               "relative_attitude_caveat": "asin(norm(eR)) assumes principal error angle below90deg; relative Euler angles are not absolute RPY.",
               "window_metrics": {}, "geometry_samples": inferred,
               "bias_assumptions": {"outward_bias_fraction": .25, "outward_bias_max_n": .15, "payload_mass_kg": .054,
                                    "virtual_nullspace_wrench_norm": float(np.linalg.norm(matrix @ bias))}}
    for start, end in ((0, 2), (2, 5), (5, 7), (7, 10), (10, float(fault-handoff))):
        mask = (x >= start) & (x < end)
        metrics["window_metrics"][f"{start}-{end}s"] = dict(
            payload_body_rate_std_rps=np.std(payload_w[mask], axis=0).tolist(),
            relative_error_euler_mean_deg=np.mean(relative_euler[mask], axis=0).tolist(),
            per_vehicle_body_rate_rms_rps=np.sqrt(np.mean(np.sum(vehicle_w[mask]**2, axis=2), axis=0)).tolist(),
            command_thrust_std_n=np.std(thrust[mask], axis=0).tolist(),
            command_thrust_mean_n=np.mean(thrust[mask], axis=0).tolist(),
            desired_tension_std_n=np.std(tensions[mask], axis=0).tolist(),
            desired_tension_mean_n=np.mean(tensions[mask], axis=0).tolist(),
            relative_error_euler_std_deg=np.std(relative_euler[mask], axis=0).tolist(),
            minimum_true_pair_distance_m=float(np.min(pair_distances[mask])),
            cf4_attitude_term_std_rps=np.std(attitude_term[mask, :2], axis=0).tolist(),
            cf4_rate_term_std_rps=np.std(rate_term[mask, :2], axis=0).tolist())
    ground = (time >= 2) & (time < 3)
    airborne = (x >= 0) & (x < 5)
    half_extent = number(rows, "payload_position_z") - number(rows, "payload_ground_clearance_m")
    metrics["logged_projected_payload_half_height_m"] = dict(ground_mean=float(np.mean(half_extent[ground])),
                                                              first5s_transport_mean=float(np.mean(half_extent[airborne])))
    metrics["cf4_command_reconstruction_max_abs_error_rps"] = float(np.nanmax(np.abs(
        np.clip(attitude_term+rate_term, [-3., -3., -2.], [3., 3., 2.]) - command)))
    (HERE / "metrics.json").write_text(json.dumps(metrics, indent=2))
    fig, axes = plt.subplots(4, 2, figsize=(15, 14), constrained_layout=True)
    mask = (x >= -2) & (x <= fault-handoff+1)
    labels = ("x", "y", "z")
    for axis in range(3):
        axes[0, 0].plot(x[mask], relative_euler[mask, axis], label=f"Relative error {labels[axis]}")
        axes[0, 1].plot(x[mask], payload_w[mask, axis], label=labels[axis])
    fits = np.array([item["rpy_deg"] for item in inferred])
    fit_x = np.array([item["time_s"] for item in inferred])-handoff
    for axis in range(3): axes[0, 0].plot(fit_x, fits[:, axis], ".", markersize=3, label=f"Geometry fit {labels[axis]}")
    for index, ident in enumerate((3, 4, 5)):
        axes[1, 0].plot(x[mask], np.max(np.abs(vehicle_w[mask, index, :2]), axis=1), label=f"CF{ident}")
        axes[2, 0].plot(x[mask], thrust[mask, index], label=f"CF{ident}")
        axes[2, 1].plot(x[mask], tensions[mask, index], label=f"CF{ident}")
    for axis, label in enumerate(("p", "q")):
        axes[1, 1].plot(x[mask], attitude_term[mask, axis], label=f"CF4 attitude {label}")
        axes[1, 1].plot(x[mask], rate_term[mask, axis], "--", label=f"CF4 rate {label}")
    for index, name in enumerate(("CF3-CF4", "CF3-CF5", "CF4-CF5")):
        axes[3, 0].plot(x[mask], pair_distances[mask, index], label=name)
    axes[3, 0].axhline(.30, color="red", linestyle=":")
    residuals = np.array([item["projected_moment_residual_nm"] for item in inferred])
    for axis in range(3): axes[3, 1].plot(fit_x, residuals[:, axis]*1e3, ".-", label=f"Moment residual {labels[axis]}")
    titles = ("Relative attitude error and cable-geometry inversion", "Observed payload body rates",
              "Aircraft maximum |p| or |q|", "CF4 host rate-command components",
              "Host thrust commands", "Projected desired cable tensions (model)",
              "True aircraft pair distances", "Projected wrench residual: modeled minus requested")
    units = ("Angle [deg]", "Rate [rad/s]", "Rate [rad/s]", "Command [rad/s]", "Thrust [N]", "Tension [N]", "Distance [m]", "Moment residual [mN m]")
    for ax, title, unit in zip(axes.flat, titles, units):
        ax.set(title=title, ylabel=unit, xlabel="Time since handoff [s]", xlim=(-2, fault-handoff+1))
        for location, color, style in ((0, "black", ":"), (5, "gray", ":"), (10, "purple", "--"), (fault-handoff, "red", "--")):
            ax.axvline(location, color=color, linestyle=style, linewidth=.8)
        ax.legend(fontsize=7, ncol=2); ax.grid(True, alpha=.25)
    fig.suptitle("Real flight 2026-10-16 21:38:04 — logged geometry and growing oscillation\n"
                 "Black: handoff; gray: ramp end; purple: planned landing; red: distance protection. Geometry/wrench quantities are inferred.", fontsize=13)
    fig.savefig(HERE / "analysis.png", dpi=160)
    plt.close(fig)
    print(json.dumps({key: metrics[key] for key in ("handoff_s", "fault_s", "window_metrics", "logged_projected_payload_half_height_m")}, indent=2))


if __name__ == "__main__":
    main()
