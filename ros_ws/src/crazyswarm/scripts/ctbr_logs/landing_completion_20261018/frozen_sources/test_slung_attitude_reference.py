"""The real manager must use one attitude path for the load and its aircraft."""

import csv

import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _slung_manager


def tilted_manager(directory, enabled=True, equilibrium=False):
    module = _controller_module()
    manager, anchors = _slung_manager(module, directory, {
        "slung_payload": {"transport_force_mode": "coupled",
                          "transport_attitude_reference_enabled": enabled,
                          "transport_equilibrium_reference_enabled": equilibrium,
                          "transport_attitude_reference_s": 12.0}})
    angle = np.deg2rad(24.0)
    rotation = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]])
    rho = manager.slung_controller.config.attachment_points_m
    anchors = anchors + ((rotation - np.eye(3)) @ rho).T
    manager.latest_payload_state["rotation"] = rotation.copy()
    manager.payload_transport_active = True
    manager.payload_transport_start_time = 0.0
    manager.payload_transport_start_position = manager.latest_payload_state["position"].copy()
    manager.payload_transport_vehicle_start_positions = anchors.copy()
    manager.payload_handoff_reference_positions = anchors + [.01, -.008, .015]
    manager.payload_handoff_forces = np.tile([.08, 0, .8], (3, 1))
    for vehicle, position in zip(manager.vehicles, anchors):
        vehicle.latest_state["position"] = position.copy()
    return manager, anchors, rotation


def sample(manager, now):
    manager.latest_payload_state["received_time"] = now
    for vehicle in manager.vehicles:
        vehicle.latest_state["received_time"] = now
    return manager._transport_overrides(now)


def test_handoff_load_attitude_starts_at_measured_rotation(tmp_path):
    manager, anchors, initial = tilted_manager(tmp_path)
    try:
        overrides = sample(manager, 0.0)
        for i, vehicle in enumerate(manager.vehicles):
            target = overrides[id(vehicle)]
            assert "payload_target_rotation" in target
            np.testing.assert_allclose(target["payload_target_rotation"], initial, atol=1e-12)
            np.testing.assert_allclose(target["position_hold_position"], anchors[i])
            np.testing.assert_allclose(target["payload_attitude_error"], 0, atol=1e-12)
    finally:
        manager.logger.close()


def test_final_hold_anchors_preserve_cables_when_load_becomes_level(tmp_path):
    manager, anchors, initial = tilted_manager(tmp_path)
    try:
        sample(manager, 0.0)
        overrides = sample(manager, 12.0)
        rho = manager.slung_controller.config.attachment_points_m
        for i, vehicle in enumerate(manager.vehicles):
            target = overrides[id(vehicle)]
            translation = target["payload_target_position"] - manager.payload_transport_start_position
            expected = anchors[i] + translation + (np.eye(3) - initial) @ rho[:, i]
            np.testing.assert_allclose(target["position_hold_position"], expected, atol=1e-12)
            np.testing.assert_allclose(target["payload_target_rotation"], np.eye(3), atol=1e-12)
            initial_cable = manager.payload_transport_start_position + initial @ rho[:, i] - anchors[i]
            final_cable = target["payload_target_position"] + rho[:, i] - expected
            np.testing.assert_allclose(final_cable, initial_cable, atol=1e-12)
    finally:
        manager.logger.close()


def test_equilibrium_reference_aligns_final_hold_cables_with_tension_allocation(tmp_path):
    manager, anchors, initial = tilted_manager(tmp_path, equilibrium=True)
    try:
        rho = manager.slung_controller.config.attachment_points_m
        original_cables = (manager.payload_transport_start_position[:, None]
                           + initial @ rho - anchors.T)
        lengths = np.linalg.norm(original_cables, axis=0)
        sample(manager, 0.0)
        overrides = sample(manager, 12.0)
        q_goal = manager.slung_controller.equilibrium_link_units(np.eye(3))
        for i, vehicle in enumerate(manager.vehicles):
            target = overrides[id(vehicle)]
            reference_cable = target["payload_target_position"] + rho[:, i] - target["position_hold_position"]
            np.testing.assert_allclose(reference_cable, lengths[i]*q_goal[:, i], atol=1e-12)
    finally:
        manager.logger.close()


def test_equilibrium_reference_preserves_lengths_and_has_consistent_derivatives(tmp_path):
    manager, anchors, initial = tilted_manager(tmp_path, equilibrium=True)
    try:
        sample(manager, 0.0)
        rho = manager.slung_controller.config.attachment_points_m
        original_lengths = np.linalg.norm(manager.payload_transport_start_position[:, None]
                                           + initial @ rho - anchors.T, axis=0)
        dt, now = 1e-3, 3.0
        left, middle, right = [sample(manager, t) for t in (now-dt, now, now+dt)]
        for i, vehicle in enumerate(manager.vehicles):
            a, b, c = [item[id(vehicle)] for item in (left, middle, right)]
            cable = (b["payload_target_position"] + b["payload_target_rotation"] @ rho[:, i]
                     - b["position_hold_position"])
            assert np.isclose(np.linalg.norm(cable), original_lengths[i], atol=1e-12)
            for p, v, acc in (("position", "velocity", "acceleration"),
                              ("position_hold_position", "position_hold_velocity", "position_hold_acceleration")):
                np.testing.assert_allclose((c[p]-a[p])/(2*dt), b[v], atol=1e-8)
                np.testing.assert_allclose((c[p]-2*b[p]+a[p])/dt**2, b[acc], atol=1e-8)
    finally:
        manager.logger.close()


def test_equilibrium_endpoint_realizes_hover_wrench_without_competing_link_error(tmp_path):
    manager, _, _ = tilted_manager(tmp_path, equilibrium=True)
    try:
        sample(manager, 0.0)
        overrides = sample(manager, 12.0)
        targets = [overrides[id(v)] for v in manager.vehicles]
        load_target = dict(position=targets[0]["payload_target_position"], velocity=np.zeros(3),
                           acceleration=np.zeros(3), rotation=np.eye(3),
                           payload_attitude_gain=manager.payload_attitude_gain,
                           payload_rate_gain=manager.payload_rate_gain)
        payload = dict(load_target, valid=True, body_rate=np.zeros(3))
        vehicles = [dict(position=t["position_hold_position"], velocity=np.zeros(3), mass=.043)
                    for t in targets]
        manager.slung_controller.reset()
        result = manager.slung_controller.compute(payload, vehicles, load_target, 1/70)
        forces = np.array([-t["desired_tension"] * t["link_direction"] for t in result["vehicles"]])
        np.testing.assert_allclose(np.sum(forces, axis=0), result["desired_force"], atol=1e-12)
        moment = np.sum(np.cross(manager.slung_controller.config.attachment_points_m.T, forces), axis=0)
        np.testing.assert_allclose(moment, result["desired_moment"], atol=1e-12)
        for output in result["vehicles"]:
            np.testing.assert_allclose(output["link_direction_error"], 0, atol=1e-12)
    finally:
        manager.logger.close()


def test_quintic_roundoff_near_end_does_not_kill_control_callback(tmp_path):
    manager, _, _ = tilted_manager(tmp_path)
    try:
        sample(manager, 0.0)
        overrides = sample(manager, np.nextafter(12.0, 0.0))
        assert len(overrides) == 3
        for target in overrides.values():
            assert 0 <= target["payload_attitude_reference_progress"] <= 1
    finally:
        manager.logger.close()


def test_rotating_handoff_position_and_hold_derivatives_match_finite_difference(tmp_path):
    manager, _, _ = tilted_manager(tmp_path)
    try:
        sample(manager, 0.0)
        dt, now = 1e-3, 3.0
        left, middle, right = [sample(manager, t) for t in (now-dt, now, now+dt)]
        for vehicle in manager.vehicles:
            a, b, c = [item[id(vehicle)] for item in (left, middle, right)]
            for p, v, acc in (("position", "velocity", "acceleration"),
                              ("position_hold_position", "position_hold_velocity", "position_hold_acceleration")):
                np.testing.assert_allclose((c[p]-a[p])/(2*dt), b[v], atol=1e-8)
                np.testing.assert_allclose((c[p]-2*b[p]+a[p])/dt**2, b[acc], atol=1e-8)
            assert np.linalg.norm(b["position_hold_velocity"]) > 1e-5
    finally:
        manager.logger.close()


def test_measured_rotation_is_frozen_instead_of_following_measurement_noise(tmp_path):
    manager, _, initial = tilted_manager(tmp_path)
    try:
        sample(manager, 0.0)
        manager.latest_payload_state["rotation"] = np.eye(3)
        overrides = sample(manager, 0.0)
        np.testing.assert_allclose(next(iter(overrides.values()))["payload_target_rotation"], initial)
        manager._reset_payload_handoff()
        assert manager.payload_transport_start_rotation is None
    finally:
        manager.logger.close()


def test_reference_switch_disabled_preserves_original_translated_anchors(tmp_path):
    manager, anchors, _ = tilted_manager(tmp_path, enabled=False)
    try:
        sample(manager, 0.0)
        overrides = sample(manager, 12.0)
        for i, vehicle in enumerate(manager.vehicles):
            target = overrides[id(vehicle)]
            translation = target["payload_target_position"] - manager.payload_transport_start_position
            np.testing.assert_allclose(target["position_hold_position"], anchors[i] + translation)
    finally:
        manager.logger.close()


def test_actual_hold_reference_and_payload_attitude_reach_flight_csv(tmp_path):
    manager, _, initial = tilted_manager(tmp_path)
    try:
        sample(manager, 0.0)
        now = 3.0
        overrides = sample(manager, now)
        vehicle = manager.vehicles[0]
        state = dict(vehicle.latest_state, filtered_velocity=np.zeros(3),
                     filtered_acceleration=np.zeros(3))
        target = vehicle._target_for(state, now)
        target.update(overrides[id(vehicle)])
        vehicle.payload_diagnostics = manager.latest_payload_state
        command = vehicle.controller.compute(state, target, 1/70)
        vehicle._write_log(now, state, target, command, control_state=state)
        manager.logger.flush()
        with open(manager.logger.path, newline="") as stream:
            row = next(csv.DictReader(stream))
        for prefix in ("position_hold_position", "position_hold_velocity", "position_hold_acceleration",
                       "payload_target_body_rate", "payload_target_body_rate_dot"):
            np.testing.assert_allclose([float(row[prefix + "_" + axis]) for axis in "xyz"], target[prefix])
        assert np.isclose(float(row["payload_pitch_rad"]), np.deg2rad(24))
        assert 0 < float(row["payload_target_pitch_rad"]) < np.deg2rad(24)
        assert 0 < float(row["payload_attitude_reference_progress"]) < 1
        np.testing.assert_allclose(
            [float(row["transport_position_hold_force_" + axis]) for axis in "xyz"],
            command["transport_position_hold_force"],
        )
    finally:
        manager.logger.close()


@pytest.mark.parametrize("enabled,equilibrium,hold", [(False, True, 1.5), (True, False, 1.5), (True, True, 0.)])
def test_reference_cascade_requires_coherent_geometry_and_positive_hold(tmp_path, enabled, equilibrium, hold):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="reference_cascade"):
        _slung_manager(module, tmp_path, {"slung_payload": {
            "transport_force_mode": "reference_cascade",
            "transport_attitude_reference_enabled": enabled,
            "transport_equilibrium_reference_enabled": equilibrium,
            "transport_position_hold_gain_scale": hold}})


def test_cascade_effective_link_gain_log_matches_actual_bypass_in_all_overrides(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path, {"slung_payload": {
        "transport_force_mode": "reference_cascade",
        "transport_attitude_reference_enabled": True,
        "transport_equilibrium_reference_enabled": True}})
    try:
        manager.payload_transport_active = True
        manager.payload_transport_start_time = 0.
        manager.payload_transport_start_position = manager.latest_payload_state["position"].copy()
        manager.payload_transport_vehicle_start_positions = anchors.copy()
        manager.payload_handoff_reference_positions = anchors.copy()
        sample(manager, 0.)
        # At late times the ordinary aircraft trajectory may enter landing;
        # its gain snapshot must not misreport a direction loop still bypassed.
        for now in (3., 12., 60.):
            targets = sample(manager, now)
            assert targets
            for target in targets.values():
                assert target["effective_transport_link_gain_scale"] == 0.
                np.testing.assert_array_equal(target["raw_link_force"], np.zeros(3))
    finally:
        manager.logger.close()


@pytest.mark.parametrize("duration", [0, -1, float("nan"), float("inf")])
def test_invalid_attitude_reference_duration_is_rejected(tmp_path, duration):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="transport_attitude_reference"):
        _slung_manager(module, tmp_path, {"slung_payload": {"transport_attitude_reference_s": duration}})
