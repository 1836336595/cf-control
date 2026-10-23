"""Cooperative feedback and fleet diagnostics must describe the same sample."""

from copy import deepcopy

import numpy as np
import pytest

from test_slung_landing_failure_containment import flight_case
from test_slung_transport_integral_integration import manager_case, fresh, ERRORS


def test_transport_carries_an_independent_snapshot_of_its_actual_feedback(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        manager.latest_payload_state["body_rate"] = np.array([.01, -.02, .03])
        overrides = manager._transport_overrides(18.)
        manager.latest_payload_state["position"][:] = 20.
        manager.latest_payload_state["body_rate"][:] = 8.
        for target in overrides.values():
            snapshot = target.get("_payload_control_state")
            assert snapshot is not None
            np.testing.assert_array_equal(snapshot["position"], [0., 0., .60])
            np.testing.assert_array_equal(snapshot["body_rate"], [.01, -.02, .03])
            assert target["_payload_control_time"] == 18.
    finally:
        manager.logger.close()


def test_fleet_uses_control_snapshot_and_one_source_snapshot_for_all_vehicles(tmp_path, monkeypatch):
    _, manager, _, _, tick = flight_case(tmp_path, monkeypatch)
    try:
        manager.payload_flight_validation_enabled = False
        first = manager.vehicles[0]
        original_callback = first._timer_callback

        def receive_new_payload_between_vehicle_callbacks(*args, **kwargs):
            result = original_callback(*args, **kwargs)
            manager.latest_payload_state["position"][:] = [9., 8., 7.]
            manager.latest_payload_state["body_rate"][:] = [8., 8., 8.]
            return result

        monkeypatch.setattr(first, "_timer_callback", receive_new_payload_between_vehicle_callbacks)
        tick(18.02)
        for vehicle in manager.vehicles:
            np.testing.assert_array_equal(vehicle.payload_diagnostics["position"], [0., 0., .60])
            np.testing.assert_array_equal(vehicle.payload_source_diagnostics["position"], [0., 0., .60])
            assert vehicle.payload_control_time == 18.02
            assert not vehicle.payload_control_reused
    finally:
        manager.logger.close()


def test_cached_hold_keeps_original_control_feedback_and_original_compute_time(tmp_path, monkeypatch):
    module, manager, anchors, published, _ = flight_case(tmp_path, monkeypatch)
    try:
        manager.payload_flight_validation_enabled = False
        manager.payload_hold_s = .20
        manager.payload_emergency_land_after_s = .50
        fresh(module, manager, anchors, 18.02, ERRORS)
        manager.latest_payload_state.update(derivatives_valid=False, body_rate=np.full(3, 8.))
        monkeypatch.setattr(module.time, "monotonic", lambda: 18.02)
        manager._timer_callback(None)
        for vehicle in manager.vehicles:
            np.testing.assert_array_equal(vehicle.payload_diagnostics["body_rate"], np.zeros(3))
            np.testing.assert_array_equal(vehicle.payload_source_diagnostics["body_rate"], [8., 8., 8.])
            assert vehicle.payload_control_time == 18.
            assert vehicle.payload_control_reused
            assert published[id(vehicle)][-1]["collective_thrust"] > 0.
        assert manager.payload_fault_since == 18.02
    finally:
        manager.logger.close()


@pytest.mark.parametrize("gate", ["settle_release", "touchdown_release"])
def test_rejected_observer_feedback_cannot_release_payload_support(tmp_path, gate):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        manager.latest_payload_state.update(position=np.array([0., 0., .025]), velocity=np.zeros(3),
            observer_updated=False, observer_quality_valid=False, source_valid=True)
        for vehicle in manager.vehicles:
            vehicle.trajectory.phase = "landing_settle"
        if gate == "settle_release":
            released = manager._payload_landing_release_ready(18.)
        else:
            manager.payload_validation_landing_stage = "payload_lowering"
            manager.payload_unload_start_time = 0.
            detector = manager.payload_touchdown_detector
            for stamp in (17.4, 17.55, 17.7):
                detector.update(stamp, detector.ground_clearance_m, 0.)
            assert detector.detected
            released = manager._release_validation_payload(18.)
        assert not released
        assert manager.payload_transport_active
    finally:
        manager.logger.close()


def test_transport_learning_checks_its_selected_feedback_instead_of_newer_latest(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        overrides = manager._transport_overrides(18.)
        selected = deepcopy(manager.latest_payload_state)
        control_states = [v._build_ekf_control_state(v.latest_state, 18.) for v in manager.vehicles]
        manager.latest_payload_state = dict(selected, derivatives_valid=False)
        reason = manager._transport_learning_reason(18., selected, control_states, overrides,
                                                    False, True, True)
        assert reason == ""
    finally:
        manager.logger.close()


def test_transport_learning_freezes_rejected_observer_feedback(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        overrides = manager._transport_overrides(18.)
        selected = dict(manager.latest_payload_state, observer_updated=False,
                        observer_quality_valid=False, source_valid=True)
        control_states = [v._build_ekf_control_state(v.latest_state, 18.) for v in manager.vehicles]
        reason = manager._transport_learning_reason(18., selected, control_states, overrides,
                                                    False, True, True)
        assert reason == "payload_feedback"
    finally:
        manager.logger.close()


@pytest.mark.parametrize("cache", ["expired", "absent", "incomplete"])
def test_payload_fault_exits_to_supported_descent_without_an_unsupported_gap(tmp_path, monkeypatch, cache):
    module, manager, anchors, published, _ = flight_case(tmp_path, monkeypatch)
    try:
        manager.payload_flight_validation_enabled = False
        manager.payload_hold_s = .20
        manager.payload_emergency_land_after_s = .50
        if cache == "absent":
            manager.last_transport_overrides = {}
        elif cache == "incomplete":
            manager.last_transport_overrides.pop(id(manager.vehicles[-1]))
        integral = manager.payload_transport_integral.bias.copy()
        core_integral = manager.slung_controller.position_integral.copy()
        for now in ((18.02, 18.23) if cache == "expired" else (18.02,)):
            previous_force_z = np.asarray([v.last_control_command["desired_force"][2]
                                           for v in manager.vehicles])
            fresh(module, manager, anchors, now, ERRORS)
            manager.latest_payload_state["derivatives_valid"] = False
            monkeypatch.setattr(module.time, "monotonic", lambda current=now: current)
            manager._timer_callback(None)
        assert all(v.trajectory.phase == "emergency_landing" for v in manager.vehicles)
        assert len(manager.payload_emergency_support) == 3
        assert not manager.last_transport_overrides
        np.testing.assert_array_equal(manager.payload_transport_integral.bias, integral)
        np.testing.assert_array_equal(manager.slung_controller.position_integral, core_integral)
        for vehicle, force_z in zip(manager.vehicles, previous_force_z):
            command = published[id(vehicle)][-1]
            assert command is not None and command["collective_thrust"] > 0.
            assert np.isclose(command["desired_force"][2], force_z, atol=1.e-12)
            assert not command["transport_mode"]
            assert not command.get("transport_full_pid_learning", False)
        assert not manager.global_abort_reason
    finally:
        manager.logger.close()


@pytest.mark.parametrize("selected_valid", [False, True])
def test_physical_guard_validates_the_geometry_snapshot_it_checks(tmp_path, monkeypatch, selected_valid):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        good = deepcopy(manager.latest_payload_state)
        bad = dict(deepcopy(good), position=np.array([0., 0., .10]), derivatives_valid=False)
        manager.latest_payload_state = good if selected_valid else bad
        manager.payload_transport_start_position = np.array([0., 0., .60])
        for vehicle, position in zip(manager.vehicles, ([.5, 0., 1.], [0., 0., 1.], [0., 1., 1.])):
            vehicle.latest_state["position"] = np.asarray(position)
        original_ready = manager._payload_ready

        def receive_other_payload_before_latest_readiness_check(now):
            manager.latest_payload_state = bad if selected_valid else good
            return original_ready(now)

        monkeypatch.setattr(manager, "_payload_ready", receive_other_payload_before_latest_readiness_check)
        assert not manager._check_payload_physical_safety(18.)
        if selected_valid:
            assert manager.payload_ground_clearance_m == pytest.approx(.575)
        else:
            assert np.isnan(manager.payload_ground_clearance_m)
            assert not manager.payload_prelift_ready
        assert "height_drop" not in manager.payload_transport_guard._since
    finally:
        manager.logger.close()


def test_handoff_start_pose_is_the_same_snapshot_that_passed_readiness(tmp_path, monkeypatch):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        manager.fleet_gate_open = True
        manager.payload_transport_active = False
        manager.payload_takeup_start_time = 17.
        manager.payload_ready_since = 17.
        manager.payload_activation_hold_s = 0.
        manager.payload_last_takeup_reference = {"positions": anchors.copy(), "reason": "", "stage": "holding"}
        for vehicle in manager.vehicles:
            vehicle.last_control_command = {"desired_force": np.array([0., 0., .5])}
        monkeypatch.setattr(manager, "_payload_takeup_overrides", lambda _now: {})
        monkeypatch.setattr(manager, "_payload_takeup_ready", lambda _now: True)
        original_ready = manager._payload_ready
        ready_calls = [0]

        class HandoffCaptured(Exception):
            pass

        def receive_rejected_payload_after_successful_readiness(now):
            if manager.payload_transport_active:
                raise HandoffCaptured()
            ready = original_ready(now)
            ready_calls[0] += 1
            if ready_calls[0] == 2:
                manager.latest_payload_state = dict(manager.latest_payload_state,
                    position=np.array([7., 8., 9.]), derivatives_valid=False)
            return ready

        monkeypatch.setattr(manager, "_payload_ready", receive_rejected_payload_after_successful_readiness)
        with pytest.raises(HandoffCaptured):
            manager._run_fleet_tick(None, 18.)
        assert manager.payload_transport_active
        np.testing.assert_array_equal(manager.payload_transport_start_position, [0., 0., .60])
        np.testing.assert_array_equal(manager.payload_transport_start_rotation, np.eye(3))
    finally:
        manager.logger.close()
