"""Exercise transport overrides and physical guards in the real manager."""

import numpy as np

from test_ctbr_controller_v2 import _controller_module, _slung_manager


def prepare(manager, anchors, now):
    manager.payload_transport_active = True
    manager.payload_transport_start_time = 0.0
    manager.payload_transport_start_position = manager.latest_payload_state["position"].copy()
    manager.payload_transport_vehicle_start_positions = anchors.copy()
    manager.payload_handoff_reference_positions = anchors.copy()
    manager.payload_handoff_forces = np.tile([.08, 0, .8], (3, 1))
    manager.latest_payload_state["received_time"] = now
    for vehicle in manager.vehicles:
        vehicle.latest_state["received_time"] = now


def test_rope_preserving_landing_references_keep_length_and_pair_spacing():
    module = _controller_module()
    payload_position = np.array([0.0, 0.0, 0.50])
    target_position = np.array([0.0, 0.0, 0.08])
    rotation = np.eye(3)
    attachment = np.array([
        [0.04, -0.04, -0.04],
        [0.00, 0.03, -0.03],
        [0.025, 0.025, 0.025],
    ])
    directions = np.array([
        [0.02, -0.35, -0.93],
        [0.12, 0.30, -0.95],
        [-0.15, 0.28, -0.95],
    ]).T
    directions /= np.linalg.norm(directions, axis=0, keepdims=True)
    lengths = np.array([0.70, 0.71, 0.68])
    positions, velocities, accelerations = module.rope_preserving_vehicle_references(
        target_position, np.zeros(3), np.zeros(3), rotation, attachment,
        directions, lengths,
    )
    target_attachment = target_position[:, None] + attachment
    np.testing.assert_allclose(
        np.linalg.norm(target_attachment.T - positions, axis=1), lengths, atol=1e-12
    )
    initial_attachment = payload_position[:, None] + attachment
    initial_positions = initial_attachment - directions * lengths[None, :]
    initial_pair = np.linalg.norm(initial_positions[:, 1] - initial_positions[:, 2])
    final_pair = np.linalg.norm(positions[1] - positions[2])
    assert np.isclose(initial_pair, final_pair, atol=1e-12)
    np.testing.assert_allclose(velocities, np.zeros((3, 3)))
    np.testing.assert_allclose(accelerations, np.zeros((3, 3)))


def test_transport_has_explicit_tilt_and_preserves_horizontal_force(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        prepare(manager, anchors, 0.0)
        overrides = manager._transport_overrides(0.0)
        for vehicle in manager.vehicles:
            target = vehicle._target_for(vehicle.latest_state, 0.0)
            target.update(overrides[id(vehicle)])
            command = vehicle.controller.compute(vehicle.latest_state, target, 1/70)
            np.testing.assert_allclose(command["desired_force"], [.08, 0, .8], atol=1e-12)
        prepare(manager, anchors, 6.0)
        targets = manager._transport_overrides(6.0)
        assert all(target["max_tilt_rad"] > np.deg2rad(10) for target in targets.values())
    finally:
        manager.logger.close()


def test_vertical_support_residual_survives_full_handoff(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        prepare(manager, anchors, 0.0)
        manager.payload_control_dt = 1/70
        initial = manager._transport_overrides(0.0)
        vehicle = manager.vehicles[0]
        assert initial[id(vehicle)]["transport_support_trim_n"] > 0
        prepare(manager, anchors, 5.0)
        target = manager._transport_overrides(5.0)[id(vehicle)]
        assert target["transport_blend"] == 1
        assert np.isclose(target["transport_support_trim_n"],
                          initial[id(vehicle)]["transport_support_trim_n"])
        # The residual remains exposed to the per-vehicle command at blend=1.
        assert target["transport_support_trim_n"] > 0.0
    finally:
        manager.logger.close()


def test_physical_failure_routes_every_vehicle_to_landing_and_clears_overrides(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        prepare(manager, anchors, 1.0)
        manager.vehicles[0].latest_state["position"] = np.array([0, 0, 1])
        manager.vehicles[1].latest_state["position"] = np.array([.2, 0, 1])
        manager.vehicles[2].latest_state["position"] = np.array([0, 1, 1])
        manager.last_transport_overrides = {123: {"transport_mode": True}}
        assert manager._check_payload_physical_safety(1.0)
        assert not manager.payload_transport_active
        assert not manager.last_transport_overrides
        assert all(v.trajectory.phase == "emergency_landing" for v in manager.vehicles)
        assert "distance" in manager.payload_safety_reason
    finally:
        manager.logger.close()


def test_payload_lowering_distance_fault_keeps_payload_aware_landing(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        prepare(manager, anchors, 1.0)
        manager.payload_validation_landing_stage = "payload_lowering"
        manager.latest_payload_state["received_time"] = 1.0
        manager._begin_payload_emergency_landing(1.0, "distance: distance=0.300 m predicted=0.280 m")
        assert manager.payload_transport_active
        assert manager.payload_payload_aware_emergency
        assert all(vehicle.trajectory.phase != "emergency_landing"
                   for vehicle in manager.vehicles)
    finally:
        manager.logger.close()


def test_payload_landing_override_preserves_each_rope_distance(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        prepare(manager, anchors, 1.0)
        manager.payload_validation_landing_stage = "payload_lowering"
        manager.latest_payload_state["received_time"] = 1.0
        for vehicle in manager.vehicles:
            vehicle.trajectory.begin_controlled_landing(
                1.0, vehicle.latest_state["position"], 0.0, "test"
            )
        overrides = manager._transport_overrides(1.1)
        payload = manager.latest_payload_state
        measured = np.asarray([vehicle.latest_state["position"] for vehicle in manager.vehicles])
        attachment = (
            payload["position"][:, None]
            + payload["rotation"] @ manager.slung_controller.config.attachment_points_m
        )
        target = np.asarray([overrides[id(vehicle)]["position"]
                             for vehicle in manager.vehicles])
        distances = np.linalg.norm(attachment.T - target, axis=1)
        assert np.all(distances >= manager.slung_controller.config.link_lengths_m - 1.0e-9)
        np.testing.assert_allclose(
            np.linalg.norm(target[1] - target[2]),
            np.linalg.norm(measured[1] - measured[2]),
            atol=1.0e-9,
        )
    finally:
        manager.logger.close()


def test_clearance_refreshes_during_transport_and_prelift_ready_clears(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        prepare(manager, anchors, 0.0)
        # Keep all pairs well spaced to isolate height diagnostics.
        for vehicle, position in zip(manager.vehicles, [[.5,0,1],[0,0,1],[0,1,1]]):
            vehicle.latest_state["position"] = np.array(position, dtype=float)
        manager.payload_prelift_ready = True
        manager.payload_ground_clearance_m = .546
        manager.latest_payload_state["position"] = np.array([0,0,.035])
        manager._check_payload_physical_safety(0.0)
        assert np.isclose(manager.payload_ground_clearance_m, .010)
        assert not manager.payload_prelift_ready
    finally:
        manager.logger.close()


def test_stale_payload_is_not_reported_as_current_clearance_or_ready(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        prepare(manager, anchors, 1.0)
        manager.latest_payload_state["received_time"] = 0.0
        for vehicle, position in zip(manager.vehicles, [[.5,0,1],[0,0,1],[0,1,1]]):
            vehicle.latest_state["position"] = np.array(position, dtype=float)
        manager.payload_ground_clearance_m = .546
        manager.payload_prelift_ready = True
        manager._check_payload_physical_safety(1.0)
        assert np.isnan(manager.payload_ground_clearance_m)
        assert not manager.payload_prelift_ready
    finally:
        manager.logger.close()


def test_timer_cannot_publish_cached_transport_after_predicted_distance_fault(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    original_time = module.time.monotonic
    published = []
    try:
        prepare(manager, anchors, 1.0)
        manager.payload_transport_guard.confirm_s = 0.20
        manager.fleet_gate_open = True
        manager.last_tick = 0.99
        for i, vehicle in enumerate(manager.vehicles):
            vehicle.preflight_voltage_ready = True
            vehicle._publish_path = lambda _state: None
            vehicle._publish_command = lambda command: published.append(command)
            vehicle._publish_zero = lambda: None
            vehicle.latest_state.update(
                position=np.array([[.40,0,1],[0,0,1],[0,1,1]][i], dtype=float),
                filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
                filter_derivatives_valid=True,
            )
            vehicle.latest_ekf_state = dict(
                position=vehicle.latest_state["position"].copy(),
                filtered_velocity=np.array([-.4,0,0] if i == 0 else [0,0,0]),
                filtered_acceleration=np.zeros(3), filter_derivatives_valid=True,
                received_time=1.0,
            )
        for now in (1.0, 1.21):
            module.time.monotonic = lambda: now
            manager.latest_payload_state["received_time"] = now
            for vehicle in manager.vehicles:
                vehicle.latest_state["received_time"] = now
                vehicle.latest_ekf_state["received_time"] = now
            manager._timer_callback(None)
        assert manager.payload_safety_reason == "distance"
        assert all(v.flight_phase == "emergency_landing" for v in manager.vehicles)
        assert all(not command["transport_mode"] for command in published[-3:])
        assert all(command["collective_thrust"] > 0 for command in published[-3:])
    finally:
        module.time.monotonic = original_time
        manager.logger.close()
