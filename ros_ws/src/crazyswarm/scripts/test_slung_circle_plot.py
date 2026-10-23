"""Verify payload motion diagnostics use measured motion and references."""

import csv
import importlib
import math
from pathlib import Path
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import numpy as np
import pytest


PLOT_SCRIPT = Path(__file__).with_name("plot_slung_circle_trace.py")


def sample(time=0., vehicle="17", **changes):
    row = dict(
        control_time_s=time, vehicle_id=vehicle,
        payload_position_x=.1 + .1*time, payload_position_y=0.,
        payload_position_z=.6, payload_roll_rad=.02,
        payload_pitch_rad=.03, payload_yaw_rad=math.pi-.05,
        payload_target_position_x=.4 + .1*time, payload_target_position_y=.4,
        payload_target_position_z=.6, payload_target_roll_rad=0.,
        payload_target_pitch_rad=0., payload_target_yaw_rad=-math.pi+.05,
        payload_trajectory_mode="circle", payload_trajectory_phase="cruise",
        payload_circle_progress=.1*time, payload_circle_center_x=.5,
        payload_circle_center_y=.4, payload_circle_center_z=.6,
        payload_circle_radius_m=.5,
    )
    row.update(changes)
    return row


def figure_eight_sample(time=0., **changes):
    row = sample(time, payload_trajectory_mode="figure_eight",
                 payload_trajectory_phase="figure_eight",
                 payload_trajectory_progress=.1*time,
                 payload_lobe_center_1_x=1., payload_lobe_center_1_y=.4,
                 payload_lobe_center_1_z=.6,
                 payload_lobe_center_2_x=0., payload_lobe_center_2_y=.4,
                 payload_lobe_center_2_z=.6)
    row.update(changes)
    return row


def cone_sample(time=25., azimuth=0., **changes):
    """A 12 degree actual tilt tracks a 10 degree target at a fixed centroid."""
    actual, desired = math.radians(12.), math.radians(10.)
    row = sample(time, payload_trajectory_mode="attitude_cone",
        payload_trajectory_phase="attitude_cone",
        payload_position_x=.01, payload_position_y=0., payload_position_z=.61,
        payload_target_position_x=0., payload_target_position_y=0., payload_target_position_z=.6,
        payload_roll_rad=0., payload_pitch_rad=actual, payload_yaw_rad=azimuth,
        payload_target_roll_rad=0., payload_target_pitch_rad=desired, payload_target_yaw_rad=azimuth,
        payload_body_z_x=math.sin(actual)*math.cos(azimuth),
        payload_body_z_y=math.sin(actual)*math.sin(azimuth), payload_body_z_z=math.cos(actual),
        payload_target_body_z_x=math.sin(desired)*math.cos(azimuth),
        payload_target_body_z_y=math.sin(desired)*math.sin(azimuth),
        payload_target_body_z_z=math.cos(desired),
        payload_cone_half_angle_deg=12., payload_target_cone_half_angle_deg=10.,
        payload_cone_axis_error_deg=2., payload_cone_half_angle_error_deg=2.,
        payload_cone_config_half_angle_deg=10., payload_cone_period_s=30.)
    row.update(changes)
    return row


def write_trace(tmp_path, rows, omit=()):
    path = tmp_path / "trace.csv"
    fields = [field for field in rows[0] if field not in omit]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return path


def plot_module():
    return importlib.import_module("plot_slung_circle_trace")


def lines_by_label(axis):
    return {line.get_label(): line for line in axis.lines}


def test_cli_writes_real_plot_without_display(tmp_path):
    trace = write_trace(tmp_path, [sample(), sample(1.)])
    destination = tmp_path / "circle.png"
    result = subprocess.run(
        [sys.executable, str(PLOT_SCRIPT), str(trace), "--save",
         str(destination), "--no-show"], capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert destination.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    import matplotlib.image as mpimg
    pixels = mpimg.imread(destination)
    assert pixels.shape[0] > 600 and pixels.shape[1] > 600
    assert np.std(pixels[:, :, :3]) > .05


def test_plot_shows_actual_desired_and_wrapped_tracking_error(tmp_path):
    trace = write_trace(tmp_path, [sample(1.), sample(),
        sample(1., vehicle="99", payload_position_x=90.)])
    figure = plot_module().plot_trace(trace, show=False)
    xy, xyz, attitude, error, yaw_error = figure.axes
    xy_lines = lines_by_label(xy)
    np.testing.assert_allclose(xy_lines["Payload actual"].get_xdata(), [.1, .2])
    np.testing.assert_allclose(xy_lines["Payload desired"].get_xdata(), [.4, .5])
    assert (xy_lines["Payload actual"].get_linestyle()
            != xy_lines["Payload desired"].get_linestyle())
    assert xy_lines["Payload actual"].get_color() != xy_lines["Payload desired"].get_color()
    xyz_lines = lines_by_label(xyz)
    np.testing.assert_allclose(xyz_lines["X actual"].get_xdata(), [0., 1.])
    np.testing.assert_allclose(xyz_lines["Y desired"].get_ydata(), [.4, .4])
    np.testing.assert_allclose(lines_by_label(attitude)["Roll actual"].get_ydata(),
                               [math.degrees(.02)]*2)
    np.testing.assert_allclose(lines_by_label(error)["Position error"].get_ydata(), [.5, .5])
    np.testing.assert_allclose(lines_by_label(yaw_error)["Yaw error"].get_ydata(),
                               [math.degrees(-.1)]*2, atol=1e-10)
    from matplotlib.quiver import Quiver
    arrows = [item for item in xy.collections if isinstance(item, Quiver)]
    assert len(arrows) == 2
    # Body +X projected into XY includes pitch and the measured yaw.
    np.testing.assert_allclose(arrows[0].U,
                               [math.cos(.03)*math.cos(math.pi-.05)]*2)
    np.testing.assert_allclose(arrows[0].V,
                               [math.cos(.03)*math.sin(math.pi-.05)]*2)
    centers = [item for item in xy.collections if item.get_label() == "Circle center"]
    assert len(centers) == 1
    np.testing.assert_allclose(centers[0].get_offsets(), [[.5, .4]])


def test_nan_references_remain_missing_instead_of_becoming_motion(tmp_path):
    unavailable = {name: "nan" for name in sample()
                   if name.startswith("payload_target_") or name.startswith("payload_circle_")}
    trace = write_trace(tmp_path, [sample(**unavailable), sample(1.)])
    figure = plot_module().plot_trace(trace, show=False)
    desired = lines_by_label(figure.axes[0])["Payload desired"].get_xdata()
    assert np.isnan(desired[0]) and desired[1] == pytest.approx(.5)
    position_error = lines_by_label(figure.axes[3])["Position error"].get_ydata()
    assert np.isnan(position_error[0]) and position_error[1] == pytest.approx(.5)
    yaw_error = lines_by_label(figure.axes[4])["Yaw error"].get_ydata()
    assert np.isnan(yaw_error[0])


def test_legacy_trace_missing_pose_fields_has_clear_cli_error(tmp_path):
    trace = write_trace(tmp_path, [sample()], omit=("payload_position_x", "payload_target_yaw_rad"))
    result = subprocess.run([sys.executable, str(PLOT_SCRIPT), str(trace), "--no-show"],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert "missing columns" in result.stderr.lower()
    assert "payload_position_x" in result.stderr
    assert "payload_target_yaw_rad" in result.stderr
    assert "Traceback" not in result.stderr


@pytest.mark.parametrize("field,value", [
    ("control_time_s", "nan"), ("control_time_s", "inf"),
    ("payload_position_x", "nan"), ("payload_position_z", "inf"),
    ("payload_yaw_rad", "not-a-number"), ("payload_roll_rad", ""),
    ("payload_target_position_x", "broken"), ("payload_target_yaw_rad", "inf"),
])
def test_malformed_or_invalid_numeric_values_are_rejected(tmp_path, field, value):
    trace = write_trace(tmp_path, [sample(**{field: value})])
    with pytest.raises(ValueError, match=field):
        plot_module().read_trace(trace)


def test_single_row_trace_needs_no_specific_fleet_ids(tmp_path):
    trace = write_trace(tmp_path, [sample(vehicle="42")])
    figure = plot_module().plot_trace(trace, show=False)
    np.testing.assert_allclose(lines_by_label(figure.axes[0])["Payload actual"].get_xdata(), [.1])


def test_figure_eight_labels_recorded_crossing_and_both_lobe_centers(tmp_path):
    trace = write_trace(tmp_path, [figure_eight_sample(), figure_eight_sample(1.)])
    destination = tmp_path / "figure_eight.png"
    figure = plot_module().plot_trace(trace, save=destination, show=False)
    xy = figure.axes[0]
    labels = {item.get_label(): item for item in xy.collections}
    assert "Circle center" not in labels
    np.testing.assert_allclose(labels["Figure eight crossing"].get_offsets(), [[.5, .4]])
    np.testing.assert_allclose(labels["Lobe center 1"].get_offsets(), [[1., .4]])
    np.testing.assert_allclose(labels["Lobe center 2"].get_offsets(), [[0., .4]])
    assert "figure eight" in figure._suptitle.get_text().lower()
    assert "figure eight" in xy.get_title().lower()
    np.testing.assert_allclose(lines_by_label(xy)["Payload actual"].get_xdata(), [.1, .2])
    np.testing.assert_allclose(lines_by_label(figure.axes[3])["Position error"].get_ydata(), [.5, .5])
    assert destination.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_figure_eight_missing_lobe_coordinate_has_clear_cli_error(tmp_path):
    trace = write_trace(tmp_path, [figure_eight_sample()], omit=("payload_lobe_center_2_z",))
    result = subprocess.run([sys.executable, str(PLOT_SCRIPT), str(trace), "--no-show"],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert "figure_eight" in result.stderr.lower()
    assert "missing columns" in result.stderr.lower()
    assert "payload_lobe_center_2_z" in result.stderr
    assert "Traceback" not in result.stderr


def test_figure_eight_skips_unavailable_early_lobe_data(tmp_path):
    unavailable = {name: "nan" for name in figure_eight_sample()
                   if name.startswith("payload_lobe_center_")
                   or name.startswith("payload_target_")
                   or name.startswith("payload_circle_")}
    trace = write_trace(tmp_path, [figure_eight_sample(**unavailable), figure_eight_sample(1.)])
    figure = plot_module().plot_trace(trace, show=False)
    labels = {item.get_label(): item for item in figure.axes[0].collections}
    np.testing.assert_allclose(labels["Lobe center 1"].get_offsets(), [[1., .4]])
    np.testing.assert_allclose(labels["Lobe center 2"].get_offsets(), [[0., .4]])
    assert np.isnan(lines_by_label(figure.axes[3])["Position error"].get_ydata()[0])


@pytest.mark.parametrize("field,value", [
    ("payload_lobe_center_1_x", "bad-coordinate"),
    ("payload_lobe_center_2_z", "inf"),
    ("payload_trajectory_progress", "bad-progress"),
])
def test_figure_eight_rejects_malformed_optional_reference_fields(tmp_path, field, value):
    trace = write_trace(tmp_path, [figure_eight_sample(**{field: value})])
    with pytest.raises(ValueError, match=field):
        plot_module().read_trace(trace)


def test_cone_plot_shows_recorded_body_z_motion_with_fixed_centroid(tmp_path):
    trace = write_trace(tmp_path, [cone_sample(), cone_sample(32.5, math.pi/2)])
    destination = tmp_path / "cone.png"
    figure = plot_module().plot_trace(trace, save=destination, show=False)
    projection, xyz, attitude, error, half_angle = figure.axes
    lines = lines_by_label(projection)
    np.testing.assert_allclose(lines["Body +Z actual"].get_xdata(), [.20791169081775931, 0.], atol=1e-12)
    np.testing.assert_allclose(lines["Body +Z actual"].get_ydata(), [0., .20791169081775931], atol=1e-12)
    np.testing.assert_allclose(lines["Body +Z desired"].get_ydata(), [0., .17364817766693033], atol=1e-12)
    assert lines["Body +Z actual"].get_color() != lines["Body +Z desired"].get_color()
    assert lines["Body +Z actual"].get_linestyle() != lines["Body +Z desired"].get_linestyle()
    outline = lines["Configured cone (10 deg)"].get_xydata()
    np.testing.assert_allclose(np.linalg.norm(outline, axis=1), .17364817766693033)
    labels = {item.get_label(): item for item in projection.collections}
    np.testing.assert_allclose(labels["World +Z"].get_offsets(), [[0., 0.]])
    assert "Circle center" not in labels
    assert "direction cosine" in projection.get_xlabel().lower()
    assert "centroid" in xyz.get_title().lower()
    np.testing.assert_allclose(lines_by_label(xyz)["X actual"].get_ydata(), [.01, .01])
    np.testing.assert_allclose(lines_by_label(xyz)["X desired"].get_ydata(), [0., 0.])
    np.testing.assert_allclose(lines_by_label(attitude)["Pitch actual"].get_ydata(), [12., 12.])
    np.testing.assert_allclose(lines_by_label(error)["Body +Z axis error"].get_ydata(), [2., 2.], atol=1e-10)
    np.testing.assert_allclose(lines_by_label(error)["Half-angle error (actual - desired)"].get_ydata(), [2., 2.], atol=1e-10)
    assert error.get_ylim()[0] <= 0. and error.get_ylim()[1] >= 2.
    np.testing.assert_allclose(lines_by_label(half_angle)["Cone half-angle actual"].get_ydata(), [12., 12.], atol=1e-10)
    np.testing.assert_allclose(lines_by_label(half_angle)["Cone half-angle desired"].get_ydata(), [10., 10.], atol=1e-10)
    title = figure._suptitle.get_text().lower()
    assert "attitude cone" in title and "30 s" in title
    assert destination.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_cone_errors_follow_body_z_when_logged_error_columns_are_absent(tmp_path):
    optional = [name for name in cone_sample() if "cone_" in name]
    trace = write_trace(tmp_path, [cone_sample()], omit=optional)
    figure = plot_module().plot_trace(trace, show=False)
    np.testing.assert_allclose(lines_by_label(figure.axes[3])["Body +Z axis error"].get_ydata(), [2.], atol=1e-10)
    assert "Configured cone (10 deg)" not in lines_by_label(figure.axes[0])


def test_cone_half_angle_error_uses_moving_desired_angle_during_tilt_in(tmp_path):
    row = cone_sample(payload_trajectory_phase="cone_tilt_in",
        payload_target_body_z_x=0., payload_target_body_z_y=0., payload_target_body_z_z=1.,
        payload_target_cone_half_angle_deg=0., payload_cone_axis_error_deg=12.,
        payload_cone_half_angle_error_deg=12.)
    figure = plot_module().plot_trace(write_trace(tmp_path, [row]), show=False)
    np.testing.assert_allclose(lines_by_label(figure.axes[3])["Half-angle error (actual - desired)"].get_ydata(), [12.], atol=1e-10)


def test_cone_unavailable_target_axis_is_masked_in_projection_and_error(tmp_path):
    unavailable = {name: "nan" for name in cone_sample()
                   if name.startswith("payload_target_") or name in
                   ("payload_cone_axis_error_deg", "payload_cone_half_angle_error_deg")}
    trace = write_trace(tmp_path, [cone_sample(**unavailable), cone_sample(32.5, math.pi/2)])
    figure = plot_module().plot_trace(trace, show=False)
    assert np.isnan(lines_by_label(figure.axes[0])["Body +Z desired"].get_xdata()[0])
    assert np.isnan(lines_by_label(figure.axes[3])["Body +Z axis error"].get_ydata()[0])
    assert np.isnan(lines_by_label(figure.axes[4])["Cone half-angle desired"].get_ydata()[0])


def test_cone_trace_missing_body_z_has_clear_cli_error(tmp_path):
    trace = write_trace(tmp_path, [cone_sample()], omit=("payload_body_z_y", "payload_target_body_z_z"))
    result = subprocess.run([sys.executable, str(PLOT_SCRIPT), str(trace), "--no-show"],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert "attitude_cone" in result.stderr and "missing columns" in result.stderr.lower()
    assert "payload_body_z_y" in result.stderr and "payload_target_body_z_z" in result.stderr
    assert "Traceback" not in result.stderr


@pytest.mark.parametrize("field,value", [
    ("payload_body_z_x", "nan"), ("payload_body_z_z", "inf"),
    ("payload_target_body_z_x", "bad-axis"),
    ("payload_cone_config_half_angle_deg", "not-an-angle"),
    ("payload_cone_period_s", "inf"),
])
def test_cone_rejects_invalid_axis_or_metadata(tmp_path, field, value):
    trace = write_trace(tmp_path, [cone_sample(**{field: value})])
    with pytest.raises(ValueError, match=field):
        plot_module().read_trace(trace)
