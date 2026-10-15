"""Rebuild distance, handoff and state-health diagnostics for the real flight."""

import csv
from collections import Counter, defaultdict
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / "multi_ctbr_20261014_152409.csv"
IDS = ("3", "4", "5")
PAIRS = (("3", "4"), ("3", "5"), ("4", "5"))


def number(row, field):
    return float(row[field]) if row.get(field) else float("nan")


def vector(row, prefix):
    return np.array([number(row, f"{prefix}_{axis}") for axis in "xyz"])


def main():
    with SOURCE.open() as stream:
        rows = list(csv.DictReader(stream))
    grouped = defaultdict(dict)
    for row in rows:
        grouped[float(row["control_time_s"])][row["vehicle_id"]] = row
    frames = [(t, frame) for t, frame in sorted(grouped.items()) if set(frame) == set(IDS)]
    handoff = next(t for t, frame in frames if frame["3"]["flight_phase"] == "tension_ramp")
    before_abort = [(t, f) for t, f in frames if all(f[cf]["flight_phase"] != "aborted" for cf in IDS)]
    summary = {"source": str(SOURCE), "handoff_time_s": handoff, "pairs": {}, "vehicles": {}}
    for a, b in PAIRS:
        valid = [(t, f) for t, f in before_abort if np.all(np.isfinite(vector(f[a], "position")))
                 and np.all(np.isfinite(vector(f[b], "position")))]
        distance = np.array([np.linalg.norm(vector(f[a], "position")-vector(f[b], "position")) for _, f in valid])
        index = int(np.argmin(distance))
        t, f = valid[index]
        d = vector(f[a], "position")-vector(f[b], "position")
        after_handoff = [(ti, fi) for ti, fi in valid if ti >= handoff]
        target_distances = [np.linalg.norm(vector(fi[a], "target")-vector(fi[b], "target"))
                            for _, fi in after_handoff]
        crossings = {}
        for boundary in (0.30, 0.25, 0.20, 0.15, 0.12, 0.10):
            hit = np.flatnonzero(distance < boundary)
            crossings[str(boundary)] = valid[int(hit[0])][0] if len(hit) else None
        summary["pairs"][a+"-"+b] = dict(
            minimum_distance_m=float(distance[index]), time_s=t,
            horizontal_distance_m=float(np.linalg.norm(d[:2])), vertical_difference_m=float(d[2]),
            minimum_target_distance_after_handoff_m=float(np.min(target_distances)),
            first_below_boundary_s=crossings,
        )

    for cf in IDS:
        records = [r for r in rows if r["vehicle_id"] == cf]
        controlled = [r for r in records if r["flight_phase"] in ("tension_ramp", "height_correction")]
        first = next(i for i, r in enumerate(records) if r["flight_phase"] == "tension_ramp")
        clipped = []
        reconstruction_error = []
        for r in controlled:
            b = number(r, "transport_blend")
            predicted = np.clip(-np.array([12, 12, 6])*vector(r, "attitude_error")
                                -b*np.array([1, 1, .5])*vector(r, "body_rate"), [-3, -3, -2], [3, 3, 2])
            reconstruction_error.extend(predicted-vector(r, "command_rate"))
            if np.any(np.abs(vector(r, "command_rate")[:2]) >= 3-1e-8):
                clipped.append(r)
        def handoff_snapshot(r):
            force = vector(r, "desired_force")
            return dict(time_s=number(r, "control_time_s"), force_n=force.tolist(),
                        tilt_deg=float(np.rad2deg(np.arctan2(np.linalg.norm(force[:2]), force[2]))),
                        thrust_n=number(r, "command_thrust_newton"))
        snapshot = dict(before=handoff_snapshot(records[first-1]), after=handoff_snapshot(records[first]))
        summary["vehicles"][cf] = dict(
            handoff=snapshot,
            first_rate_saturation_s=number(clipped[0], "control_time_s") if clipped else None,
            first_rate_saturation_blend=number(clipped[0], "transport_blend") if clipped else None,
            command_reconstruction_max_error=float(np.nanmax(np.abs(reconstruction_error))),
            saturation_fraction_after_5s=float(np.mean([
                np.any(np.abs(vector(r, "command_rate")[:2]) >= 3-1e-8)
                for r in controlled if 5 <= number(r, "control_time_s")-handoff < 7])),
            pre_collision_state_valid=Counter(r["state_valid"] for r in records
                                             if handoff <= number(r, "control_time_s") < 55.9),
        )
    sample_times = (handoff, handoff+1, handoff+2, handoff+3, handoff+4, handoff+5, 54.4616, 54.8040, 55.9040)
    summary["timeline"] = []
    for requested in sample_times:
        t, f = min(frames, key=lambda frame: abs(frame[0]-requested))
        r = f["3"]
        summary["timeline"].append(dict(
            time_s=t, handoff_elapsed_s=t-handoff,
            distance_cf3_cf4_m=float(np.linalg.norm(vector(f["3"], "position")-vector(f["4"], "position"))),
            payload_height_m=number(r, "payload_position_z"),
            payload_target_height_m=number(r, "payload_target_z"),
            logged_clearance_m=number(r, "payload_ground_clearance_m"),
            hold_gain_scale=number(r, "transport_position_hold_gain_scale"),
        ))
    (ROOT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    show = [(t, f) for t, f in frames if t >= handoff-1.0]
    t = np.array([ti for ti, _ in show])
    for a, b in PAIRS:
        distance = [np.linalg.norm(vector(f[a], "position")-vector(f[b], "position")) for _, f in show]
        axes[0, 0].plot(t, distance, label=f"CF{a}-CF{b}")
    axes[0, 0].plot(t, [np.linalg.norm(vector(f["3"], "target")-vector(f["4"], "target")) for _, f in show],
                    "--", label="CF3-CF4 target")
    axes[0, 0].axhline(.30, color="grey", linestyle=":", label="0.30 m reference")
    axes[0, 0].set_ylabel("Vehicle center distance (m)")
    axes[0, 1].plot(t, [number(f["3"], "payload_position_z") for _, f in show], label="Measured center height")
    axes[0, 1].plot(t, [number(f["3"], "payload_target_z") for _, f in show], label="Target height")
    axes[0, 1].plot(t, [number(f["3"], "payload_ground_clearance_m") for _, f in show], ":", label="Logged clearance (stale)")
    axes[0, 1].set_ylabel("Payload height (m)")
    for cf in IDS:
        r = [f[cf] for _, f in show]
        axes[1, 0].plot(t, [vector(ri, "body_rate")[1] for ri in r], label="CF"+cf)
        axes[1, 1].plot(t, [vector(ri, "command_rate")[1] for ri in r], label="CF"+cf)
        axes[2, 0].plot(t, [np.rad2deg(np.arctan2(np.linalg.norm(vector(ri, "desired_force")[:2]),
                                                vector(ri, "desired_force")[2])) for ri in r], label="CF"+cf)
        axes[2, 1].plot(t, [number(ri, "command_thrust_newton") for ri in r], label="CF"+cf)
        axes[3, 0].plot(t, [vector(ri, "position")[0] for ri in r], label="CF"+cf)
        axes[3, 1].plot(t, [number(ri, "mocap_sample_age_s")*1000 for ri in r], label="CF"+cf)
    for ax, label in zip(axes[1:].ravel(), ("Pitch rate (rad/s)", "Pitch command (rad/s)",
                                         "Desired force tilt (deg)", "Command thrust (N)",
                                         "Vehicle X (m)", "Mocap sample age (ms)")):
        ax.set_ylabel(label)
    for ax in axes.ravel():
        ax.axvline(handoff, color="grey", linestyle="--", linewidth=.8)
        ax.axvline(handoff+5, color="grey", linestyle="--", linewidth=.8)
        ax.grid(True, alpha=.25)
        ax.legend(fontsize=7)
        ax.set_xlim(handoff-.5, 56.5)
        ax.set_xlabel("CSV control time (s)")
    axes[1, 0].set_ylim(-7, 7)
    fig.suptitle("Real flight 2026-10-14 15:24:09: handoff, loss of altitude, rate oscillation and CF3/CF4 approach")
    fig.tight_layout()
    fig.savefig(ROOT / "collision_diagnostics.png", dpi=170)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
