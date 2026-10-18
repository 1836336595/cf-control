#!/usr/bin/env python3
"""Audit measured CSV diagnostics; reconstructed allocator terms are estimates."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "multi_ctbr_20261018_162455.csv"


def col(rows, key):
    return np.array([float(r[key] or "nan") for r in rows])


def vec(rows, prefix):
    return np.column_stack([col(rows, prefix + "_" + a) for a in "xyz"])


def main():
    raw = list(csv.DictReader(SOURCE.open(newline="")))
    fleet = {i: [r for r in raw if r["vehicle_id"] == str(i)] for i in (3, 4, 5)}
    count = min(map(len, fleet.values()))
    fleet = {i: r[:count] for i, r in fleet.items()}
    rows = fleet[3]
    t = col(rows, "control_time_s")
    active = np.isfinite(col(rows, "payload_desired_moment_x"))
    h = t[active][0]
    tt = t[active]
    elapsed = tt - h
    error = vec(rows, "payload_attitude_error")[active]
    norm = np.linalg.norm(error, axis=1)
    rotation = Rotation.from_rotvec(error * (np.arcsin(np.clip(norm, 0, 1)) / norm)[:, None]).as_matrix()
    inertia = .054 / 12 * np.array([.06**2 + .05**2, .08**2 + .05**2, .08**2 + .06**2])
    w = vec(rows, "payload_body_rate")[active]
    moment = vec(rows, "payload_desired_moment")[active]
    omega_dot_cmd = (moment - np.cross(w, w * inertia)) / inertia
    translation = vec(rows, "payload_desired_force")[active] / .054
    payload_velocity = vec(rows, "payload_velocity")[active]
    end_window = (elapsed >= 7) & (elapsed < 10)
    report = dict(source=str(SOURCE), handoff_s=float(h), shared_ticks=count, ignored_partial_last_tick=len(raw) - 3 * count,
                  reconstruction_caveat="Attitude recovered from principal SO(3) error assuming horizontal identity reference. Logged payload states and vehicle data are asynchronous. Reconstructed force terms are model estimates, not cable/motor measurements or closed-loop counterfactuals.",
                  vehicles={}, frequencies_hz={})
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), constrained_layout=True, sharex=True)
    for index, (ident, mass, rho) in enumerate(zip((3, 4, 5), (.0434, .0440, .0424),
                                                  ([.04, 0, .025], [-.04, .03, .025], [-.04, -.03, .025]))):
        rs = fleet[ident]
        q = vec(rs, "link_direction")[active]
        limited = vec(rs, "limited_link_force")[active]
        raw_link = vec(rs, "raw_link_force")[active]
        kin = vec(rs, "kinematic_perpendicular_force")[active]
        tension = col(rs, "desired_tension_n")[active]
        length = col(rs, "link_length_effective_m")[active]
        measured_length = col(rs, "link_length_measured_m")[active]
        rho_world = np.einsum("nij,j->ni", rotation, rho)
        omega_world = np.einsum("nij,nj->ni", rotation, w)
        relative_velocity = payload_velocity + np.cross(omega_world, rho_world) - vec(rs, "control_velocity")[active]
        qdot = (relative_velocity - np.sum(q * relative_velocity, axis=1)[:, None] * q) / measured_length[:, None]
        omega_link = np.cross(q, qdot)
        terms = dict(tension=-tension[:, None] * q, link_correction=limited,
                     load_translation=mass * translation,
                     load_angular_acceleration=mass * np.einsum("nij,nj->ni", rotation, np.cross(omega_dot_cmd, rho)),
                     load_centripetal=mass * np.einsum("nij,nj->ni", rotation, np.cross(w, np.cross(w, rho))),
                     cable_centripetal=mass * length[:, None] * np.sum(omega_link**2, axis=1)[:, None] * q)
        # q q^T ai plus (I-q q^T)ai equals ai; use the full angular
        # acceleration term here to avoid mistaking a perpendicular split for
        # its actual contribution to the total aircraft command.
        terms_sum = sum(terms.values())
        report["vehicles"][str(ident)] = dict(
            active_ticks=len(tt),
            rate_limiter_flags=int(np.sum(col(rs, "link_force_rate_limited")[active] == 1)),
            magnitude_limiter_flags=int(np.sum(col(rs, "link_force_magnitude_limited")[active] == 1)),
            maximum_raw_vs_limited_difference_n=float(np.max(np.abs(raw_link - limited))),
            late_force_std_n={name: np.std(value[end_window], axis=0).tolist() for name, value in terms.items()},
            logged_kinematic_perpendicular_std_n=np.std(kin[end_window], axis=0).tolist(),
            late_correction_vs_angular_acceleration_y_correlation=float(np.corrcoef(
                limited[end_window, 1], terms["load_angular_acceleration"][end_window, 1])[0, 1]),
            normal_collective_max_n=float(np.max(col(rs, "command_thrust_newton")[(t >= h) & (t < h + 10)])),
        )
        for axis, label in enumerate("xyz"):
            axes[index, 0].plot(elapsed, limited[:, axis], label=f"link correction {label}")
        axes[index, 1].plot(elapsed, terms["link_correction"][:, 1], label="link correction y (logged)")
        axes[index, 1].plot(elapsed, terms["load_angular_acceleration"][:, 1], label="angular-acceleration y (estimated)")
        axes[index, 1].plot(elapsed, terms["load_translation"][:, 1], label="payload translation y (estimated)")
        axes[index, 0].set_title(f"CF{ident}: raw equals limited correction, both flags zero")
        axes[index, 1].set_title(f"CF{ident}: separate force contributions, before host hold/blend")
        window = (elapsed >= 2) & (elapsed < 10)
        uniform_t = np.arange(tt[window][0], tt[window][-1], 1/70)
        for name, data in (("correction_y", limited[:, 1]), ("kinematic_perpendicular_y", kin[:, 1]),
                           ("aircraft_body_rate_x", col(rs, "body_rate_x")[active]),
                           ("collective", col(rs, "command_thrust_newton")[active])):
            interpolated = np.interp(uniform_t, tt[window], data[window])
            frequency, psd = welch(interpolated, fs=70, nperseg=min(256, len(interpolated)))
            band = (frequency > .5) & (frequency < 10)
            report["frequencies_hz"][f"CF{ident}_{name}"] = float(frequency[band][np.argmax(psd[band])])
    for ax in axes.flat:
        ax.set(xlabel="Time since handoff [s]", ylabel="Force [N]", xlim=(0, elapsed[-1]))
        ax.axvline(5, color="gray", linestyle=":")
        ax.axvline(10, color="purple", linestyle="--")
        ax.grid(alpha=.25)
        ax.legend(fontsize=7)
    fig.suptitle("2026-10-18 16:24:55 — force-path growth with inactive limiters\n"
                 "Reconstruction uses logged model states; these are not measured cable tensions.")
    fig.savefig(HERE / "force_terms.png", dpi=150)
    plt.close(fig)
    (HERE / "force_terms.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
