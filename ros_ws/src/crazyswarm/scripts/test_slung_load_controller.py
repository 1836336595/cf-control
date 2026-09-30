#!/usr/bin/env python3
"""Regression tests for the MATLAB slung-load boundary conversion."""

import math
from pathlib import Path
import sys

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from slung_load_controller import (  # noqa: E402
    SlungLoadConfig,
    attachment_points_ros_z_up,
    mocap_top_surface_to_center,
    ordered_vehicle_ids,
    SlungLoadController,
    SecondOrderVelocityFilter,
    cable_distances,
    smoothstep5_profile,
    takeup_vehicle_targets,
)


def test_load_origin_on_top_surface_is_shifted_to_geometric_center():
    rotation = np.eye(3)
    center = mocap_top_surface_to_center(
        np.array([1.0, 2.0, 3.0]), rotation, height_m=0.06
    )
    assert np.allclose(center, [1.0, 2.0, 2.97])


def test_load_center_shift_follows_body_z_axis_after_yaw_rotation():
    yaw = math.pi / 2.0
    rotation = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    center = mocap_top_surface_to_center(
        np.array([0.0, 0.0, 1.0]), rotation, height_m=0.08
    )
    assert np.allclose(center, [0.0, 0.0, 0.96])


def test_matlab_attachment_order_maps_cf3_cf4_cf5_in_ros_z_up():
    points = attachment_points_ros_z_up([0.08, 0.06, 0.05])
    assert np.allclose(points[:, 0], [0.04, 0.0, 0.025])
    assert np.allclose(points[:, 1], [-0.04, 0.03, 0.025])
    assert np.allclose(points[:, 2], [-0.04, -0.03, 0.025])


def test_vehicle_order_is_explicit_and_does_not_depend_on_yaml_order():
    entries = [{"id": 5}, {"id": 3}, {"id": 4}]
    assert ordered_vehicle_ids(entries) == [3, 4, 5]


def test_invalid_payload_state_does_not_produce_transport_command():
    config = SlungLoadConfig(
        payload_mass_kg=0.08,
        gravity_mps2=9.80665,
        payload_size_m=np.array([0.08, 0.06, 0.05]),
        attachment_points_m=attachment_points_ros_z_up([0.08, 0.06, 0.05]),
        link_length_m=0.65,
        position_gain=np.array([3.0, 3.0, 3.75]),
        velocity_gain=np.array([3.12, 3.12, 3.12]),
        integral_gain=np.array([1.6, 1.6, 1.6]),
        integral_limit=np.array([0.5, 0.5, 0.5]),
        c1=0.5,
        force_norm_epsilon=1.0e-9,
        tension_pinv_tolerance=1.0e-9,
        link_kq=55.0,
        link_komega=20.0,
        link_integral_gain=0.0,
        link_integral_limit=np.array([0.3, 0.3, 0.3]),
    )
    controller = SlungLoadController(config)
    result = controller.compute(
        payload_state={"valid": False},
        vehicle_states=[],
        target={"position": np.zeros(3), "velocity": np.zeros(3),
                "acceleration": np.zeros(3), "rotation": np.eye(3),
                "body_rate": np.zeros(3), "body_rate_dot": np.zeros(3)},
        dt=0.01,
    )
    assert result is None


def test_payload_velocity_filter_derives_acceleration_from_filtered_velocity():
    filter_state = SecondOrderVelocityFilter(cutoff_hz=5.0, max_dt=0.05)
    assert filter_state.update(np.zeros(3), 0.0)[2] is False
    filtered, acceleration, ready = filter_state.update(np.ones(3), 0.01)
    assert ready is True
    assert np.all(np.isfinite(filtered))
    assert np.allclose(acceleration, (filtered - np.zeros(3)) / 0.01)


def test_smoothstep5_profile_has_zero_endpoint_velocity_and_acceleration():
    start = smoothstep5_profile(0.0, 2.0)
    end = smoothstep5_profile(2.0, 2.0)
    assert np.allclose(start, [0.0, 0.0, 0.0])
    assert np.allclose(end, [1.0, 0.0, 0.0])


def test_takeup_targets_are_cable_length_away_and_keep_aircraft_spread():
    attachments = attachment_points_ros_z_up([0.08, 0.06, 0.05])
    payload_position = np.array([0.3, -0.2, 0.025])
    targets = takeup_vehicle_targets(
        payload_position, np.eye(3), attachments, 0.65, 0.12
    ).T
    distances = cable_distances(
        payload_position, np.eye(3), attachments, targets
    )
    assert np.allclose(distances, 0.65, atol=1e-12)
    assert np.linalg.norm(targets[0, :2] - targets[1, :2]) > 0.15
    assert np.linalg.norm(targets[1, :2] - targets[2, :2]) > 0.15
