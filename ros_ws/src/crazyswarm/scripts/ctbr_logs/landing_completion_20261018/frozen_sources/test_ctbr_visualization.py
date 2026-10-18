#!/usr/bin/env python3
"""Tests for CTBR CSV visualization calculations."""

import math
import csv
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ctbr_visualization import attitude_trace_error, rpy_to_rotation  # noqa: E402
from ctbr_visualization import second_order_velocity_filter  # noqa: E402
from ctbr_visualization import load_plot_data  # noqa: E402


def test_attitude_trace_error_is_zero_for_matching_attitude():
    rpy = np.array([0.2, -0.15, 0.7])
    assert math.isclose(attitude_trace_error(rpy, rpy), 0.0, abs_tol=1e-12)


def test_attitude_trace_error_matches_rotation_trace_definition():
    actual_rpy = np.array([0.1, -0.2, 0.3])
    desired_rpy = np.array([-0.25, 0.15, -0.4])
    actual_rotation = rpy_to_rotation(actual_rpy)
    desired_rotation = rpy_to_rotation(desired_rpy)
    expected = np.trace(np.eye(3) - desired_rotation.T @ actual_rotation)

    assert math.isclose(
        attitude_trace_error(actual_rpy, desired_rpy), expected, rel_tol=1e-12, abs_tol=1e-12
    )


def test_attitude_trace_error_returns_nan_for_invalid_rpy():
    actual_rpy = np.array([np.nan, 0.0, 0.0])
    desired_rpy = np.zeros(3)
    assert math.isnan(attitude_trace_error(actual_rpy, desired_rpy))


def test_load_plot_data_exposes_nokov_velocity_and_acceleration():
    fields = [
        "mode", "control_time_s",
        "position_x", "position_y", "position_z",
        "target_x", "target_y", "target_z",
        "position_error_x", "position_error_y", "position_error_z",
        "velocity_x", "velocity_y", "velocity_z",
        "acceleration_x", "acceleration_y", "acceleration_z",
        "roll_rad", "pitch_rad", "yaw_rad",
        "desired_roll_rad", "desired_pitch_rad", "desired_yaw_rad",
        "command_rate_x", "command_rate_y", "command_rate_z",
        "command_thrust_newton",
    ]
    row = {
        "mode": "control",
        "control_time_s": "1.0",
        "position_x": "0.0", "position_y": "0.0", "position_z": "1.0",
        "target_x": "0.0", "target_y": "0.0", "target_z": "1.0",
        "position_error_x": "0.0", "position_error_y": "0.0", "position_error_z": "0.0",
        "velocity_x": "0.1", "velocity_y": "-0.2", "velocity_z": "0.3",
        "acceleration_x": "1.1", "acceleration_y": "-1.2", "acceleration_z": "1.3",
        "roll_rad": "0.0", "pitch_rad": "0.0", "yaw_rad": "0.0",
        "desired_roll_rad": "0.0", "desired_pitch_rad": "0.0", "desired_yaw_rad": "0.0",
        "command_rate_x": "0.0", "command_rate_y": "0.0", "command_rate_z": "0.0",
        "command_thrust_newton": "0.4",
    }
    with tempfile.NamedTemporaryFile(mode="w", newline="", suffix=".csv") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerow(row)
        csv_file.flush()
        data = load_plot_data(csv_file.name, 10.0)

    assert np.allclose(data["velocity"][:, 0], [0.1, -0.2, 0.3])
    assert np.allclose(data["acceleration"][:, 0], [1.1, -1.2, 1.3])


def test_load_plot_data_prefers_logged_controller_filtered_derivatives():
    fields = [
        "mode", "control_time_s",
        "position_x", "position_y", "position_z",
        "target_x", "target_y", "target_z",
        "position_error_x", "position_error_y", "position_error_z",
        "velocity_x", "velocity_y", "velocity_z",
        "acceleration_x", "acceleration_y", "acceleration_z",
        "filtered_velocity_x", "filtered_velocity_y", "filtered_velocity_z",
        "filtered_acceleration_x", "filtered_acceleration_y", "filtered_acceleration_z",
        "roll_rad", "pitch_rad", "yaw_rad",
        "desired_roll_rad", "desired_pitch_rad", "desired_yaw_rad",
        "command_rate_x", "command_rate_y", "command_rate_z",
        "command_thrust_newton",
    ]
    row = dict.fromkeys(fields, "0.0")
    row.update({
        "mode": "control", "control_time_s": "0.0", "position_z": "1.0",
        "target_z": "1.0", "filtered_velocity_x": "0.4",
        "filtered_velocity_y": "0.5", "filtered_velocity_z": "0.6",
        "filtered_acceleration_x": "1.4", "filtered_acceleration_y": "1.5",
        "filtered_acceleration_z": "1.6",
    })
    with tempfile.NamedTemporaryFile(mode="w", newline="", suffix=".csv") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerow(row)
        csv_file.flush()
        data = load_plot_data(csv_file.name, 10.0)

    assert np.allclose(data["velocity_filtered"][:, 0], [0.4, 0.5, 0.6])
    assert np.allclose(data["acceleration_filtered"][:, 0], [1.4, 1.5, 1.6])


def test_load_plot_data_marks_invalid_controller_filter_samples_as_gaps():
    fields = [
        "mode", "control_time_s",
        "position_x", "position_y", "position_z",
        "target_x", "target_y", "target_z",
        "position_error_x", "position_error_y", "position_error_z",
        "velocity_x", "velocity_y", "velocity_z",
        "acceleration_x", "acceleration_y", "acceleration_z",
        "filtered_velocity_x", "filtered_velocity_y", "filtered_velocity_z",
        "filtered_acceleration_x", "filtered_acceleration_y", "filtered_acceleration_z",
        "filter_derivatives_valid",
        "roll_rad", "pitch_rad", "yaw_rad",
        "desired_roll_rad", "desired_pitch_rad", "desired_yaw_rad",
        "command_rate_x", "command_rate_y", "command_rate_z",
        "command_thrust_newton",
    ]
    invalid_row = dict.fromkeys(fields, "0.0")
    invalid_row.update({
        "mode": "control", "control_time_s": "0.0", "position_z": "1.0",
        "target_z": "1.0", "filtered_velocity_x": "0.4",
        "filtered_acceleration_x": "1.4", "filter_derivatives_valid": "0",
    })
    valid_row = dict(invalid_row)
    valid_row.update({
        "control_time_s": "0.01", "filtered_velocity_x": "0.5",
        "filtered_acceleration_x": "1.5", "filter_derivatives_valid": "1",
    })
    with tempfile.NamedTemporaryFile(mode="w", newline="", suffix=".csv") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows((invalid_row, valid_row))
        csv_file.flush()
        data = load_plot_data(csv_file.name, 10.0)

    assert data["filtered_source"] == "controller"
    assert np.all(np.isnan(data["velocity_filtered"][:, 0]))
    assert np.all(np.isnan(data["acceleration_filtered"][:, 0]))
    assert np.isclose(data["velocity_filtered"][0, 1], 0.5)
    assert np.isclose(data["acceleration_filtered"][0, 1], 1.5)


def test_velocity_and_acceleration_use_separate_axes():
    import matplotlib

    matplotlib.use("Agg")
    import ctbr_visualization

    pyplot = ctbr_visualization.require_matplotlib()
    handles = ctbr_visualization.create_figure(pyplot)
    assert handles["velocity_axes"] is not handles["acceleration_axes"]
    assert len(handles["velocity_lines"]) == 3
    assert len(handles["acceleration_lines"]) == 3
    pyplot.close(handles["figure"])


def test_second_order_velocity_filter_returns_smoothed_velocity_and_derived_acceleration():
    time_s = np.arange(120, dtype=float) * 0.01
    velocity = np.vstack((
        np.sin(2.0 * np.pi * 1.0 * time_s) + 0.25 * np.sin(2.0 * np.pi * 20.0 * time_s),
        np.zeros(time_s.size),
        np.zeros(time_s.size),
    ))

    filtered_velocity, filtered_acceleration = second_order_velocity_filter(
        velocity, time_s, cutoff_hz=5.0
    )

    assert filtered_velocity.shape == velocity.shape
    assert filtered_acceleration.shape == velocity.shape
    raw_high_frequency = np.std(velocity[0] - np.sin(2.0 * np.pi * 1.0 * time_s))
    filtered_high_frequency = np.std(
        filtered_velocity[0, 20:-20] - np.sin(2.0 * np.pi * 1.0 * time_s[20:-20])
    )
    assert filtered_high_frequency < raw_high_frequency
    expected_acceleration = np.gradient(filtered_velocity[0], time_s)
    assert np.allclose(filtered_acceleration[0], expected_acceleration)


def test_second_order_velocity_filter_does_not_bridge_invalid_samples():
    time_s = np.arange(20, dtype=float) * 0.01
    velocity = np.ones((3, time_s.size))
    velocity[:, 10] = np.nan

    filtered_velocity, filtered_acceleration = second_order_velocity_filter(
        velocity, time_s, cutoff_hz=5.0
    )

    assert np.all(np.isnan(filtered_velocity[:, 10]))
    assert np.all(np.isnan(filtered_acceleration[:, 10]))
    assert np.all(np.isfinite(filtered_velocity[:, :10]))
    assert np.all(np.isfinite(filtered_velocity[:, 11:]))


def _payload_rows(vehicle_ids=(3,), attitude=True, payload=True):
    rows = []
    for time_s in (0.0, 0.01):
        for vehicle_id in vehicle_ids:
            row = {
                "mode": "control", "control_time_s": time_s,
                "mission_time_s": time_s, "vehicle_id": vehicle_id,
                "position_x": 0.1, "position_y": 0.2, "position_z": 1.0,
                "target_x": 0.0, "target_y": 0.0, "target_z": 1.0,
            }
            if payload:
                row.update({
                    "payload_state_valid": 1,
                    "payload_position_x": 0.2 + time_s,
                    "payload_position_y": -0.1, "payload_position_z": 0.4,
                    "payload_target_x": 0.3, "payload_target_y": 0.0,
                    "payload_target_z": 0.5,
                    "payload_velocity_x": 0.0, "payload_velocity_y": 0.0,
                    "payload_velocity_z": 0.0,
                    "payload_acceleration_x": 0.0, "payload_acceleration_y": 0.0,
                    "payload_acceleration_z": 0.0,
                })
                if attitude:
                    row.update({
                        "payload_roll_rad": math.pi / 6,
                        "payload_pitch_rad": -math.pi / 12,
                        "payload_yaw_rad": math.pi / 2,
                        "payload_target_roll_rad": 0.0,
                        "payload_target_pitch_rad": math.pi / 18,
                        "payload_target_yaw_rad": math.pi / 3,
                    })
            rows.append(row)
    return rows


def _write_rows(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return path


def test_payload_loader_converts_attitude_and_deduplicates_merged_ticks(tmp_path):
    from ctbr_visualization import load_payload_plot_data

    path = _write_rows(tmp_path / "merged.csv", _payload_rows((3, 4, 5)))
    data = load_payload_plot_data(path)

    assert np.allclose(data["time_s"], [0.0, 0.01])
    assert np.allclose(data["rpy"][:, 0], [30.0, -15.0, 90.0])
    assert np.allclose(data["desired_rpy"][:, 0], [0.0, 10.0, 60.0])


def test_payload_loader_keeps_invalid_state_gaps_and_independent_reference(tmp_path):
    from ctbr_visualization import load_payload_plot_data

    rows = _payload_rows()
    rows[1]["payload_state_valid"] = 0
    missing_position = dict(rows[0], mission_time_s=0.02, payload_position_x="")
    rows.append(missing_position)
    data = load_payload_plot_data(_write_rows(tmp_path / "gaps.csv", rows))

    assert np.allclose(data["time_s"], [0.0, 0.01, 0.02])
    assert np.all(np.isnan(data["position"][:, 1:]))
    assert np.all(np.isnan(data["rpy"][:, 1:]))
    assert np.allclose(data["target"][0], [0.3, 0.3, 0.3])
    assert np.allclose(data["desired_rpy"][1], [10.0, 10.0, 10.0])


def test_legacy_payload_position_only_keeps_missing_attitude_unknown(tmp_path):
    from ctbr_visualization import load_payload_plot_data

    rows = _payload_rows(attitude=False)
    for row in rows:
        for field in list(row):
            if field.startswith(("payload_velocity", "payload_acceleration")):
                del row[field]
    data = load_payload_plot_data(_write_rows(tmp_path / "legacy.csv", rows))

    assert data is not None
    assert np.allclose(data["position"][0], [0.2, 0.21])
    assert np.all(np.isnan(data["rpy"]))
    assert np.all(np.isnan(data["desired_rpy"]))


@pytest.mark.parametrize("vehicle_ids,attitude", [((3,), True), ((3, 4), True), ((3,), False)])
def test_static_viewer_renders_and_exports_payload_pose(tmp_path, monkeypatch, vehicle_ids, attitude):
    import matplotlib
    matplotlib.use("Agg")
    import ctbr_visualization as viewer

    pyplot = viewer.require_matplotlib()
    path = _write_rows(tmp_path / "flight.csv", _payload_rows(vehicle_ids, attitude))
    output = tmp_path / "flight.png"
    monkeypatch.setattr(pyplot, "show", lambda: None)
    try:
        viewer.plot(path, output, True, 10.0)
        payload_figure = pyplot.figure("Crazyflie Payload Pose")
        assert len(payload_figure.axes) == 6
        payload_figure.canvas.draw()
        position_line, position_reference = payload_figure.axes[0].lines
        assert np.allclose(position_line.get_ydata(), [0.2, 0.21])
        assert np.allclose(position_reference.get_ydata(), [0.3, 0.3])
        roll_line, roll_reference = payload_figure.axes[3].lines
        if attitude:
            assert np.allclose(roll_line.get_ydata(), [30.0, 30.0])
            assert np.allclose(roll_reference.get_ydata(), [0.0, 0.0])
        else:
            assert np.all(np.isnan(roll_line.get_ydata()))
            assert np.all(np.isnan(roll_reference.get_ydata()))
        assert output.is_file()
        assert (tmp_path / "flight_payload.png").is_file()
    finally:
        pyplot.close("all")


def test_vehicle_only_log_does_not_open_or_export_payload_figure(tmp_path, monkeypatch):
    import matplotlib
    matplotlib.use("Agg")
    import ctbr_visualization as viewer

    pyplot = viewer.require_matplotlib()
    path = _write_rows(tmp_path / "vehicle.csv", _payload_rows(payload=False))
    monkeypatch.setattr(pyplot, "show", lambda: None)
    try:
        viewer.plot(path, tmp_path / "vehicle.png", True, 10.0)
        assert "Crazyflie Payload Pose" not in pyplot.get_figlabels()
        assert not (tmp_path / "vehicle_payload.png").exists()
    finally:
        pyplot.close("all")


def test_realtime_viewer_adds_payload_figure_when_state_first_arrives(tmp_path, monkeypatch):
    import matplotlib
    matplotlib.use("Agg")
    import ctbr_visualization as viewer

    pyplot = viewer.require_matplotlib()
    rows = _payload_rows((3, 4))
    initial_rows = [dict(row, payload_position_x="", payload_roll_rad="") for row in rows]
    path = _write_rows(tmp_path / "growing.csv", initial_rows)

    def advance_log():
        assert "Crazyflie Payload Pose" not in pyplot.get_figlabels()
        _write_rows(path, rows)
        main_figure = pyplot.figure("Crazyflie Multi-CTBR Visualization")
        main_figure.canvas.draw()
        main_figure._ctbr_animation._func(1)
        payload_figure = pyplot.figure("Crazyflie Payload Pose")
        assert len(payload_figure.axes) == 6
        payload_figure.canvas.draw()
        assert np.allclose(payload_figure.axes[3].lines[0].get_ydata(), [30.0, 30.0])
        for row in rows:
            row["payload_roll_rad"] = math.pi / 4
        _write_rows(path, rows)
        main_figure._ctbr_animation._func(2)
        assert np.allclose(payload_figure.axes[3].lines[0].get_ydata(), [45.0, 45.0])

    monkeypatch.setattr(pyplot, "show", advance_log)
    try:
        viewer.realtime_plot(path, tmp_path / "live.png", 0.2, 10.0, str(tmp_path))
        assert (tmp_path / "live_payload.png").is_file()
    finally:
        pyplot.close("all")
