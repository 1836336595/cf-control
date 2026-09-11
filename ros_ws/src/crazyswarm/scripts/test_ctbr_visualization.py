#!/usr/bin/env python3
"""Tests for CTBR CSV visualization calculations."""

import math
import csv
import sys
import tempfile
from pathlib import Path

import numpy as np


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
