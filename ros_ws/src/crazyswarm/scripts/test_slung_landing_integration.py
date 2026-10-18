"""Loaded landing must preserve references, force and separation protection."""

import numpy as np

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_handoff_safety import prepare


def ready_manager(tmp_path, now=10):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    prepare(manager, anchors, now)
    for vehicle in manager.vehicles:
        vehicle.trajectory.start_position[2] = .05
    return module, manager


def test_landing_inherits_reference_gains_and_support_at_first_tick(tmp_path):
    _, manager = ready_manager(tmp_path)
    try:
        manager.payload_transport_start_position[2] = .35
        manager.latest_payload_state['position'][2] = .45
        manager.payload_ground_center_z = .034
        before = manager._transport_overrides(10)
        manager.last_transport_overrides = before
        for vehicle in manager.vehicles:
            vehicle.trajectory.begin_controlled_landing(10, vehicle.latest_state['position'], 0, 'test')
        after = manager._transport_overrides(10)
        for vehicle in manager.vehicles:
            a, b = before[id(vehicle)], after[id(vehicle)]
            # Landing begins from the current measured payload state. The old
            # transport reference may still be above it after the validation
            # timer, so equality with ``before`` would reintroduce an upward
            # step at the start of descent.
            assert b['payload_target_position'][2] <= .45 + 1.0e-12
            assert a['transport_position_hold_gain_scale'] == b['transport_position_hold_gain_scale']
            assert a['effective_transport_link_gain_scale'] == b['effective_transport_link_gain_scale']
            assert abs(a['transport_support_trim_n'] - b['transport_support_trim_n']) < .0002
        np.testing.assert_allclose(manager.payload_landing_start_position[2], .45, atol=1e-12)
        assert manager.payload_landing_profile.duration >= 1.875*(.45-.034)/.12
    finally:
        manager.logger.close()


def test_distance_fault_seeds_last_supported_force_for_independent_descent(tmp_path):
    module, manager = ready_manager(tmp_path)
    published = []
    original_time = module.time.monotonic
    try:
        now = 10.
        module.time.monotonic = lambda: now
        manager.fleet_gate_open = True
        for vehicle, position in zip(manager.vehicles, ([.6,0,1],[0,.6,1],[-.6,0,1])):
            vehicle.latest_state.update(position=np.array(position), received_time=now,
                                        filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
                                        filter_derivatives_valid=True)
            vehicle.latest_ekf_state = dict(position=np.array(position), received_time=now,
                filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3), filter_derivatives_valid=True)
            vehicle.last_control_command = dict(desired_force=np.array([0,0,.85]), collective_thrust=.85)
            vehicle.preflight_voltage_ready = True
            vehicle._publish_path = lambda _state: None
            vehicle._publish_command = lambda command: published.append(command)
            vehicle._publish_zero = lambda: None
        manager._begin_payload_emergency_landing(now, 'distance: test')
        manager._reset_payload_handoff()
        manager._timer_callback(None)
        assert len(published) == 3
        for command in published:
            assert abs(command['collective_thrust'] - .85) < .001
        assert all(v.trajectory.phase == 'emergency_landing' for v in manager.vehicles)
    finally:
        module.time.monotonic = original_time
        manager.logger.close()


def test_height_fault_uses_supported_cooperative_descent_and_preserves_fault_reason(tmp_path):
    _, manager = ready_manager(tmp_path)
    try:
        manager.latest_payload_state['position'][2] = .4
        manager.last_transport_overrides = manager._transport_overrides(10)
        manager._begin_payload_emergency_landing(10, 'height_drop: test')
        assert manager.payload_transport_active
        assert manager.payload_validation_landing_stage == 'payload_lowering'
        assert manager.payload_validation_landing_started_time is None
        assert all(v.trajectory.phase == 'landing' for v in manager.vehicles)
        assert all('height_drop' in v.trajectory.emergency_landing_reason for v in manager.vehicles)
    finally:
        manager.logger.close()


def test_support_residual_has_total_force_headroom_not_arbitrary_initial_clip(tmp_path):
    _, manager = ready_manager(tmp_path, now=0)
    try:
        manager.payload_handoff_forces[:,2] = [.85,.85,.82]
        manager._transport_overrides(0)
        assert np.all(manager.payload_support_trim.bias <= manager.payload_support_trim.limits)
        assert not np.any(manager.payload_support_trim.initialization_clipped)
    finally:
        manager.logger.close()


def test_ground_reference_captures_initial_measured_surface_before_takeup(tmp_path):
    _, manager = ready_manager(tmp_path)
    try:
        manager.payload_transport_active = False
        manager.payload_takeup_start_time = None
        manager.payload_ground_reference_samples.clear()
        manager.payload_ground_center_z = float('nan')
        for vehicle in manager.vehicles:
            vehicle.trajectory.phase = 'pre_takeoff_hold'
        manager.latest_payload_state['position'][2] = .034
        manager._update_payload_ground_reference(10)
        assert np.isclose(manager.payload_ground_center_z, .034)
        for vehicle in manager.vehicles:
            vehicle.trajectory.phase = 'height_correction'
        manager.payload_takeup_start_time = 11
        manager.latest_payload_state['position'][2] = .5
        manager._update_payload_ground_reference(11)
        assert np.isclose(manager.payload_ground_center_z, .034)
    finally:
        manager.logger.close()


def test_emergency_descent_keeps_live_spacing_diagnostics(tmp_path):
    _, manager = ready_manager(tmp_path)
    try:
        manager.payload_transport_active = False
        manager.payload_takeup_start_time = None
        manager.payload_emergency_support = {id(v): (10, .3) for v in manager.vehicles}
        for vehicle, position in zip(manager.vehicles, ([0,0,1],[.2,0,1],[0,1,1])):
            vehicle.latest_state.update(position=np.array(position), received_time=10)
            vehicle.trajectory.phase = 'emergency_landing'
        assert not manager._check_payload_physical_safety(10)
        assert np.isclose(manager.payload_minimum_vehicle_distance_m, .2)
    finally:
        manager.logger.close()


def test_touchdown_timeout_is_a_fault_and_retains_force_handoff(tmp_path):
    _, manager = ready_manager(tmp_path)
    try:
        manager.latest_payload_state['position'][2] = .5
        manager._transport_overrides(10)
        for vehicle in manager.vehicles:
            vehicle.last_control_command = dict(desired_force=np.array([0,0,.85]), collective_thrust=.85)
        manager._begin_payload_emergency_landing(10, 'height_drop: test')
        manager._transport_overrides(10)
        deadline = manager.payload_landing_profile.start_time + manager.payload_landing_profile.duration + 4.01
        manager.latest_payload_state['received_time'] = deadline
        assert not manager._release_validation_payload(deadline)
        assert not manager.payload_touchdown_detector.detected
        assert not manager.global_abort_reason
        assert len(manager.payload_emergency_support) == 3
        assert all(v.trajectory.phase == 'emergency_landing' for v in manager.vehicles)
        assert all('touchdown_timeout' in v.trajectory.emergency_landing_reason for v in manager.vehicles)
    finally:
        manager.logger.close()
