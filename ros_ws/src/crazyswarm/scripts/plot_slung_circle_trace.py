#!/usr/bin/env python3
"""Plot measured payload motion against circle, figure-eight or attitude-cone references.

Usage: plot_slung_circle_trace.py trace.csv --save circle.png --no-show
The reference trajectory alone does not establish successful tracking.
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib


ACTUAL_FIELDS = (
    "control_time_s",
    "payload_position_x", "payload_position_y", "payload_position_z",
    "payload_roll_rad", "payload_pitch_rad", "payload_yaw_rad",
)
REFERENCE_FIELDS = (
    "payload_target_position_x", "payload_target_position_y", "payload_target_position_z",
    "payload_target_roll_rad", "payload_target_pitch_rad", "payload_target_yaw_rad",
    "payload_circle_progress", "payload_circle_center_x", "payload_circle_center_y",
    "payload_circle_center_z", "payload_circle_radius_m",
)
LOBE_CENTER_FIELDS = tuple("payload_lobe_center_{}_{}".format(lobe, axis)
                           for lobe in (1, 2) for axis in "xyz")
BODY_Z_FIELDS = tuple("payload_body_z_" + axis for axis in "xyz")
TARGET_BODY_Z_FIELDS = tuple("payload_target_body_z_" + axis for axis in "xyz")
CONE_FIELDS = (
    "payload_cone_half_angle_deg", "payload_target_cone_half_angle_deg",
    "payload_cone_axis_error_deg", "payload_cone_half_angle_error_deg",
    "payload_cone_config_half_angle_deg", "payload_cone_period_s",
)
OPTIONAL_REFERENCE_FIELDS = (("payload_trajectory_progress",) + LOBE_CENTER_FIELDS
                             + BODY_Z_FIELDS + TARGET_BODY_Z_FIELDS + CONE_FIELDS)
REQUIRED_FIELDS = set(ACTUAL_FIELDS + REFERENCE_FIELDS + (
    "vehicle_id", "payload_trajectory_mode", "payload_trajectory_phase",
))


def read_trace(path):
    """Validate payload samples, keep the first aircraft row per tick, and sort.

    Unavailable reference cells may be NaN or empty. Measured payload state and
    control time must always be finite; reference values are never synthesized.
    """
    ticks = {}
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or ())
        missing = REQUIRED_FIELDS.difference(columns)
        if missing:
            raise ValueError("trajectory trace missing columns: {}; generate a new "
                             "simulation trace with payload pose and reference logging"
                             .format(", ".join(sorted(missing))))
        reference_fields = REFERENCE_FIELDS + tuple(
            field for field in OPTIONAL_REFERENCE_FIELDS if field in columns)
        for row in reader:
            cone = row["payload_trajectory_mode"] == "attitude_cone"
            if row["payload_trajectory_mode"] == "figure_eight":
                missing_lobes = set(LOBE_CENTER_FIELDS).difference(columns)
                if missing_lobes:
                    raise ValueError("figure_eight trace missing columns: {}"
                                     .format(", ".join(sorted(missing_lobes))))
            if cone:
                missing_axes = set(BODY_Z_FIELDS + TARGET_BODY_Z_FIELDS).difference(columns)
                if missing_axes:
                    raise ValueError("attitude_cone trace missing columns: {}"
                                     .format(", ".join(sorted(missing_axes))))
            actual_fields = ACTUAL_FIELDS + (BODY_Z_FIELDS if cone else ())
            for field in ACTUAL_FIELDS + reference_fields:
                raw = row[field]
                unavailable = field not in actual_fields
                if unavailable and raw is not None and not raw.strip():
                    value = float("nan")
                else:
                    try:
                        value = float(raw)
                    except (TypeError, ValueError):
                        raise ValueError("CSV line {}: invalid number for {}: {!r}"
                                         .format(reader.line_num, field, raw)) from None
                if math.isinf(value) or (not unavailable and not math.isfinite(value)):
                    requirement = "finite or NaN" if unavailable else "finite"
                    raise ValueError("CSV line {}: {} must be {}; got {!r}"
                                     .format(reader.line_num, field, requirement, raw))
                row[field] = value
            ticks.setdefault(row["control_time_s"], row)
    if not ticks:
        raise ValueError("trace is empty: {}".format(path))
    return [ticks[time] for time in sorted(ticks)]


def plot_trace(path, save=None, show=True):
    """Draw actual/reference pose and tracking errors; return the figure."""
    import numpy as np
    import matplotlib.pyplot as plt

    rows = read_trace(path)
    figure_eight = any(row["payload_trajectory_mode"] == "figure_eight" for row in rows)
    cone = any(row["payload_trajectory_mode"] == "attitude_cone" for row in rows)
    time = np.asarray([row["control_time_s"] for row in rows])
    actual_position = np.asarray([[row["payload_position_" + axis] for axis in "xyz"]
                                  for row in rows])
    desired_position = np.asarray([[row["payload_target_position_" + axis] for axis in "xyz"]
                                   for row in rows])
    actual_attitude = np.asarray([[row["payload_" + angle + "_rad"]
                                  for angle in ("roll", "pitch", "yaw")] for row in rows])
    desired_attitude = np.asarray([[row["payload_target_" + angle + "_rad"]
                                   for angle in ("roll", "pitch", "yaw")] for row in rows])
    position_error = np.linalg.norm(actual_position - desired_position, axis=1)
    yaw_delta = actual_attitude[:, 2] - desired_attitude[:, 2]
    yaw_error_deg = np.rad2deg(np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta)))
    if cone:
        actual_axis = np.asarray([[row[field] for field in BODY_Z_FIELDS] for row in rows])
        desired_axis = np.asarray([[row[field] for field in TARGET_BODY_Z_FIELDS] for row in rows])
        desired_axis[~np.isfinite(desired_axis).all(axis=1)] = np.nan
        actual_half_angle = np.rad2deg(np.arccos(np.clip(actual_axis[:, 2], -1., 1.)))
        desired_half_angle = np.rad2deg(np.arccos(np.clip(desired_axis[:, 2], -1., 1.)))
        axis_dot = np.sum(actual_axis * desired_axis, axis=1)
        axis_error = np.rad2deg(np.arccos(np.clip(axis_dot, -1., 1.)))

    figure, axes = plt.subplots(2, 2, figsize=(13, 9))
    xy = axes[0, 0]
    actual_xy, desired_xy = (actual_axis, desired_axis) if cone else (actual_position, desired_position)
    xy.plot(actual_xy[:, 0], actual_xy[:, 1], color="tab:blue",
            label="Body +Z actual" if cone else "Payload actual")
    xy.plot(desired_xy[:, 0], desired_xy[:, 1], "--", color="tab:orange",
            label="Body +Z desired" if cone else "Payload desired")
    if cone:
        xy.scatter([0.], [0.], marker="+", color="black", s=70, label="World +Z", zorder=5)
        configured_angles = np.asarray([row.get("payload_cone_config_half_angle_deg", np.nan)
                                        for row in rows])
        for angle in np.unique(configured_angles[np.isfinite(configured_angles)]):
            phase = np.linspace(0., 2.*np.pi, 181)
            radius = np.sin(np.deg2rad(angle))
            xy.plot(radius*np.cos(phase), radius*np.sin(phase), ":", color="0.5",
                    label="Configured cone ({:g} deg)".format(angle))
    center_styles = [] if cone else [("payload_circle_center_",
                      "Figure eight crossing" if figure_eight else "Circle center", "x", "black")]
    if figure_eight:
        center_styles.extend(("payload_lobe_center_{}_".format(lobe),
                              "Lobe center {}".format(lobe), "+", "tab:purple")
                             for lobe in (1, 2))
    for prefix, label, marker, color in center_styles:
        centers = np.asarray([[row[prefix + axis] for axis in "xy"] for row in rows])
        finite_centers = centers[np.isfinite(centers).all(axis=1)]
        if len(finite_centers):
            unique_centers = np.unique(finite_centers, axis=0)
            xy.scatter(unique_centers[:, 0], unique_centers[:, 1], marker=marker,
                       color=color, s=70, label=label, zorder=5)

    radii = np.asarray([row["payload_circle_radius_m"] for row in rows])
    usable_radii = radii[np.isfinite(radii) & (radii > 0)]
    arrow_length = .18 * float(np.median(usable_radii)) if len(usable_radii) else .08
    # R[:, 0] = [cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch)].
    # Keep its XY projection rather than implying a tilted +X axis is level.
    heading_series = () if cone else ((actual_position, actual_attitude, "tab:blue"),
                                     (desired_position, desired_attitude, "tab:orange"))
    for position, attitude, color in heading_series:
        candidates = np.linspace(0, len(time)-1, min(16, len(time)), dtype=int)
        valid = np.isfinite(position[candidates, :2]).all(axis=1)
        valid &= np.isfinite(attitude[candidates, 1:]).all(axis=1)
        selected = candidates[valid]
        if len(selected):
            pitch, yaw = attitude[selected, 1], attitude[selected, 2]
            xy.quiver(position[selected, 0], position[selected, 1],
                      np.cos(pitch)*np.cos(yaw), np.cos(pitch)*np.sin(yaw),
                      color=color, angles="xy", scale_units="xy", scale=1./arrow_length,
                      width=.005, zorder=4)
    if cone:
        xy.set(title="Body +Z tip projection", xlabel="X direction cosine", ylabel="Y direction cosine")
    else:
        xy_title = "Payload figure eight XY" if figure_eight else "Payload XY"
        xy.set(title=xy_title + " (arrows: body +X)", xlabel="X (m)", ylabel="Y (m)")
    xy.set_aspect("equal", adjustable="datalim")
    xy.legend(fontsize=9, **({"loc": "lower left"} if cone else {}))

    xyz = axes[0, 1]
    attitude_axis = axes[1, 0]
    for index, (axis_name, angle, color) in enumerate(zip(
            "XYZ", ("Roll", "Pitch", "Yaw"), ("tab:blue", "tab:orange", "tab:green"))):
        xyz.plot(time, actual_position[:, index], color=color, label=axis_name + " actual")
        xyz.plot(time, desired_position[:, index], "--", color=color,
                 label=axis_name + " desired")
        attitude_axis.plot(time, np.rad2deg(actual_attitude[:, index]), color=color,
                           label=angle + " actual")
        attitude_axis.plot(time, np.rad2deg(desired_attitude[:, index]), "--", color=color,
                           label=angle + " desired")
    xyz.set(title="Payload centroid position" if cone else "Payload position",
            xlabel="Control time (s)", ylabel="Position (m)")
    attitude_axis.set(title="Payload attitude", xlabel="Control time (s)", ylabel="Angle (deg)")
    xyz.legend(fontsize=8, ncol=2)
    attitude_axis.legend(fontsize=8, ncol=2)

    error_axis = axes[1, 1]
    if cone:
        axis_line, = error_axis.plot(time, axis_error, color="tab:red", label="Body +Z axis error")
        half_error_line, = error_axis.plot(time, actual_half_angle - desired_half_angle,
            ":", color="tab:purple", label="Half-angle error (actual - desired)")
        error_axis.set(title="Attitude cone tracking", xlabel="Control time (s)",
                       ylabel="Tracking error (deg)")
        # Keep constant tracking offsets legible rather than magnifying the
        # floating-point noise of acos into a nearly zero-width vertical axis.
        errors = np.concatenate((axis_error, actual_half_angle - desired_half_angle))
        errors = errors[np.isfinite(errors)]
        low = min(0., float(errors.min())) if len(errors) else 0.
        high = max(0., float(errors.max())) if len(errors) else 0.
        padding = max(.5, .05*(high-low))
        error_axis.set_ylim(low-padding, high+padding)
        half_angle_axis = error_axis.twinx()
        actual_line, = half_angle_axis.plot(time, actual_half_angle, color="tab:blue",
                                            label="Cone half-angle actual")
        desired_line, = half_angle_axis.plot(time, desired_half_angle, "--", color="tab:orange",
                                             label="Cone half-angle desired")
        half_angle_axis.set_ylabel("Half-angle from world +Z (deg)")
        half_angle_axis.set_ylim(bottom=0)
        error_axis.legend(handles=[axis_line, half_error_line, actual_line, desired_line],
                          fontsize=8, loc="lower left")
    else:
        position_line, = error_axis.plot(time, position_error, color="tab:blue", label="Position error")
        error_axis.set(title="Tracking errors (actual - desired)", xlabel="Control time (s)",
                       ylabel="Position error norm (m)")
        error_axis.set_ylim(bottom=0)
        yaw_axis = error_axis.twinx()
        yaw_line, = yaw_axis.plot(time, yaw_error_deg, "--", color="tab:red", label="Yaw error")
        yaw_axis.set_ylabel("Wrapped yaw error (deg)")
        error_axis.legend(handles=[position_line, yaw_line], fontsize=9)
    for axis in axes.flat:
        axis.grid(True, alpha=.25)
    trajectory_name = "attitude cone" if cone else "figure eight" if figure_eight else "circle"
    title = "Payload {}: measured motion and desired reference".format(trajectory_name)
    if cone:
        periods = np.asarray([row.get("payload_cone_period_s", np.nan) for row in rows])
        periods = np.unique(periods[np.isfinite(periods)])
        if len(periods) == 1:
            title += " (period {:g} s)".format(periods[0])
    figure.suptitle(title)
    figure.tight_layout()
    if save:
        figure.savefig(save, dpi=180)
    if show:
        plt.show()
    else:
        plt.close(figure)
    return figure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="simulation_trace.csv with payload trajectory fields")
    parser.add_argument("--save", type=Path, help="PNG/PDF output path")
    parser.add_argument("--no-show", action="store_true", help="Do not open an interactive window")
    args = parser.parse_args()
    if args.no_show:
        matplotlib.use("Agg")
    try:
        plot_trace(args.trace, save=args.save, show=not args.no_show)
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
