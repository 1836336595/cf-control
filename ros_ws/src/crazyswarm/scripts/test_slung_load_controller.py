#!/usr/bin/env python3
"""Regression tests for the MATLAB slung-load boundary conversion."""

import math
from pathlib import Path
import sys
import yaml

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
    PayloadStateObserver,
    SecondOrderVelocityFilter,
    cable_distances,
    link_lengths_vector,
    takeup_tracking_ready,
    translate_vehicle_references,
    smoothstep5_profile,
    takeup_distance_ready,
    takeup_target_distance,
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


def test_payload_velocity_filter_rejects_implausible_spike_without_blowup():
    filter_state = SecondOrderVelocityFilter(
        cutoff_hz=5.0,
        max_dt=0.05,
        max_speed_mps=[1.0, 1.0, 1.0],
        max_jump_mps=[0.8, 0.8, 0.8],
    )
    filter_state.update(np.zeros(3), 0.0)
    accepted, _, ready = filter_state.update(np.array([0.05, 0.0, 0.0]), 0.01)
    assert ready is True
    rejected, rejected_acceleration, rejected_ready = filter_state.update(
        np.array([8.0, 0.0, 0.0]), 0.02
    )
    assert rejected_ready is False
    assert np.allclose(rejected, accepted)
    assert np.allclose(rejected_acceleration, 0.0)

    recovered, _, recovered_ready = filter_state.update(np.zeros(3), 0.03)
    assert recovered_ready is True
    assert np.all(np.abs(recovered) < 0.2)


def test_payload_state_observer_uses_pose_only_and_estimates_velocity():
    observer = PayloadStateObserver(
        position_gain=0.35,
        velocity_gain=0.20,
        acceleration_gain=0.02,
        attitude_gain=0.35,
        angular_rate_gain=0.08,
        max_dt=0.05,
        max_velocity_mps=[2.0, 2.0, 2.0],
        max_acceleration_mps2=[8.0, 8.0, 8.0],
        max_body_rate_rps=[8.0, 8.0, 8.0],
        min_samples=3,
    )
    for index in range(30):
        state = observer.update(
            np.array([0.1 * index * 0.01, 0.0, 0.0]),
            np.eye(3),
            index * 0.01,
        )
    assert state["derivatives_valid"] is True
    assert 0.03 < state["velocity"][0] < 0.15
    assert np.all(np.isfinite(state["acceleration"]))
    assert np.allclose(state["body_rate"], np.zeros(3))


def test_payload_state_observer_resets_after_timestamp_gap():
    observer = PayloadStateObserver(max_dt=0.05, min_samples=3)
    observer.update(np.zeros(3), np.eye(3), 0.0)
    observer.update(np.array([0.01, 0.0, 0.0]), np.eye(3), 0.01)
    state = observer.update(np.array([0.02, 0.0, 0.0]), np.eye(3), 0.20)
    assert state["derivatives_valid"] is False
    assert np.allclose(state["velocity"], np.zeros(3))


def test_payload_state_observer_estimates_body_rate_from_attitude():
    observer = PayloadStateObserver(
        attitude_gain=0.35, angular_rate_gain=0.20, min_samples=3
    )
    for index in range(80):
        angle = 0.5 * index * 0.01
        rotation = np.array([
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        state = observer.update(np.zeros(3), rotation, index * 0.01)
    assert state["derivatives_valid"] is True
    assert abs(state["body_rate"][2]) < 1.0
    assert abs(state["body_rate"][2]) > 0.05


def test_takeup_requires_position_and_link_direction_alignment():
    attachments = attachment_points_ros_z_up([0.08, 0.06, 0.05])
    payload_position = np.zeros(3)
    target_positions = takeup_vehicle_targets(
        payload_position, np.eye(3), attachments, 0.65, 0.12
    ).T
    assert takeup_tracking_ready(
        target_positions, target_positions, payload_position, np.eye(3),
        attachments, 0.03, math.radians(10.0)
    )

    position_error = target_positions.copy()
    position_error[1, 0] += 0.04
    assert not takeup_tracking_ready(
        position_error, target_positions, payload_position, np.eye(3),
        attachments, 0.03, math.radians(10.0)
    )
    assert takeup_tracking_ready(
        position_error, target_positions, payload_position, np.eye(3),
        attachments, None, math.radians(10.0)
    )

    direction_error = target_positions.copy()
    direction_error[0, 1] += 0.15
    assert not takeup_tracking_ready(
        direction_error, target_positions, payload_position, np.eye(3),
        attachments, 0.03, math.radians(10.0)
    )


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


def test_takeup_targets_stop_inside_cable_length_and_readiness_uses_that_target():
    cable_length = 0.65
    target_distance = takeup_target_distance(cable_length, 0.05)
    assert math.isclose(target_distance, 0.60, abs_tol=1e-12)
    assert takeup_distance_ready([0.61, 0.60, 0.59], cable_length,
                                 target_distance, 0.03)
    assert not takeup_distance_ready([0.66, 0.60, 0.60], cable_length,
                                     target_distance, 0.03)


def test_measured_link_lengths_are_applied_per_vehicle():
    lengths = np.array([0.692, 0.693, 0.666])
    assert np.allclose(link_lengths_vector(lengths), lengths)
    attachments = attachment_points_ros_z_up([0.08, 0.06, 0.05])
    targets = takeup_vehicle_targets(
        np.zeros(3), np.eye(3), attachments, lengths, 0.0
    ).T
    distances = cable_distances(np.zeros(3), np.eye(3), attachments, targets)
    assert np.allclose(distances, lengths, atol=1.0e-12)
    assert takeup_distance_ready(distances, lengths, lengths, 0.01)


def test_vehicle_transport_reference_follows_payload_trajectory_without_step():
    anchors = np.array([
        [0.30, 0.00, 0.70],
        [-0.25, 0.25, 0.65],
        [-0.25, -0.20, 0.66],
    ])
    payload_start = np.array([0.0, 0.0, 0.034])
    position = payload_start.copy()
    positions, velocities, accelerations = translate_vehicle_references(
        anchors, payload_start, position, np.zeros(3), np.zeros(3)
    )
    assert np.allclose(positions, anchors)
    assert np.allclose(velocities, 0.0)
    assert np.allclose(accelerations, 0.0)

    position = payload_start + np.array([0.02, -0.01, 0.25])
    positions, velocities, accelerations = translate_vehicle_references(
        anchors, payload_start, position,
        np.array([0.1, -0.05, 0.4]), np.array([0.2, -0.1, 0.3])
    )
    assert np.allclose(positions, anchors + [0.02, -0.01, 0.25])
    assert np.allclose(velocities, [0.1, -0.05, 0.4])
    assert np.allclose(accelerations, [0.2, -0.1, 0.3])


def test_takeup_accepts_small_tracking_overrun_within_distance_tolerance():
    lengths = np.array([0.692, 0.693, 0.666])
    observed = np.array([0.696, 0.695, 0.669])
    assert takeup_distance_ready(observed, lengths, lengths, 0.03)


def test_real_payload_takeoff_hover_height_is_above_low_slack_stage():
    config_path = SCRIPT_DIR.parent / "config" / "slung_payload.yaml"
    with config_path.open(encoding="utf-8") as stream:
        payload = yaml.safe_load(stream)["slung_payload"]
    assert math.isclose(payload["independent_hover_height_m"], 0.50, abs_tol=1e-12)
    assert payload["independent_hover_height_m"] < payload["link_length_m"]
    link_lengths = np.asarray(payload["link_lengths_m"], dtype=float)
    assert link_lengths.shape == (3,)
    assert np.all(np.isfinite(link_lengths))
    assert np.all(link_lengths > 0.0)


def test_equilibrium_takeup_targets_follow_matlab_allocation_directions():
    """TAKEUP must use MATLAB's static allocation directions, not one radial offset."""
    config = SlungLoadConfig(
        payload_mass_kg=0.053,
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
    link_units = controller.equilibrium_link_units(np.eye(3))
    targets = takeup_vehicle_targets(
        np.zeros(3), np.eye(3), config.attachment_points_m,
        config.link_length_m, 0.12, link_units=link_units
    ).T
    distances = cable_distances(
        np.zeros(3), np.eye(3), config.attachment_points_m, targets
    )
    assert np.allclose(distances, config.link_length_m, atol=1e-12)
    assert np.all(link_units[2, :] < 0.0)  # ROS z-up: vehicle -> load points down.
    assert not np.isclose(
        np.linalg.norm(link_units[:2, 0]),
        np.linalg.norm(link_units[:2, 1]),
        atol=1.0e-6,
    )


def test_equilibrium_takeup_targets_keep_collision_margin_between_rear_vehicles():
    config = SlungLoadConfig(
        payload_mass_kg=0.053,
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
    targets = takeup_vehicle_targets(
        np.zeros(3), np.eye(3), config.attachment_points_m,
        0.60, 0.12, link_units=controller.equilibrium_link_units(np.eye(3))
    ).T
    assert np.linalg.norm(targets[1, :2] - targets[2, :2]) >= 0.50
