"""Native transport PID selection, manager gates and lifecycle diagnostics."""
import csv

import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_transport_integral_integration import ERRORS, fresh, manager_case


def native_case(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    manager.payload_transport_feedback_controller = "native_position_pid"
    return module, manager, anchors


def test_native_cannot_silently_run_with_coupled_force_mode(tmp_path):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="reference_cascade"):
        _slung_manager(module, tmp_path, {"slung_payload": {
            "transport_feedback_controller": "native_position_pid",
            "transport_force_mode": "coupled"}})


def test_native_targets_disable_differential_learning_and_identify_tick(tmp_path):
    module, manager, anchors = native_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        overrides = manager._transport_overrides(18.)
        for target in overrides.values():
            assert target["transport_full_pid_learning"]
            assert target["transport_full_pid_tick"] == 18.
            assert not target["transport_hold_integral_learning"]
            assert not target["transport_hold_integral_enabled"]
            np.testing.assert_array_equal(target["transport_hold_integral_force"], np.zeros(3))
    finally:
        manager.logger.close()


def test_native_unload_gates_learning_and_fades_only_integral_bias(tmp_path):
    module, manager, anchors = native_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        manager._transport_overrides(18.)
        manager.payload_validation_landing_stage = "payload_lowering"
        manager._begin_validation_vehicle_descent(18.)
        lowering = manager._transport_overrides(18.)
        for target in lowering.values():
            assert not target["transport_full_pid_learning"]
            assert target["transport_full_pid_weight"] == 1.
        manager.payload_unload_start_time = 18.
        manager.payload_landing_trim_at_unload = manager.payload_support_trim.bias.copy()
        fresh(module, manager, anchors, 19., ERRORS)
        for target in manager._transport_overrides(19.).values():
            assert target["transport_full_pid_weight"] == pytest.approx(.5)
            assert target["transport_full_pid_weight_dot"] == pytest.approx(-.9375)
        fresh(module, manager, anchors, 20., ERRORS)
        for target in manager._transport_overrides(20.).values():
            assert target["transport_full_pid_weight"] == 0.
            assert target["transport_full_pid_weight_dot"] == 0.
    finally:
        manager.logger.close()


@pytest.mark.parametrize("method", ["_reset_payload_handoff", "_set_abort"])
def test_native_lifecycle_reset_preserves_ordinary_hover_integral(tmp_path, method):
    module, manager, anchors = native_case(tmp_path)
    try:
        for vehicle in manager.vehicles:
            vehicle.controller.transport_position_integral[:] = [.1, -.2, .3]
            vehicle.controller.transport_full_pid_force_previous[:] = [.01, -.02, .03]
            vehicle.controller.position_integral[:] = [.03, .02, .01]
        getattr(manager, method)(*(["test"] if method == "_set_abort" else []))
        for vehicle in manager.vehicles:
            np.testing.assert_array_equal(vehicle.controller.transport_position_integral, np.zeros(3))
            np.testing.assert_array_equal(vehicle.controller.transport_full_pid_force_previous, np.zeros(3))
            np.testing.assert_array_equal(vehicle.controller.position_integral, [.03, .02, .01])
    finally:
        manager.logger.close()


def test_native_short_payload_gap_freezes_learning_with_current_tick(tmp_path, monkeypatch):
    module, manager, anchors = native_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        previous = manager._transport_overrides(18.)
        for target in previous.values():
            target["transport_full_pid_weight"] = .5
            target["transport_full_pid_weight_dot"] = -.9375
        manager.last_transport_overrides = previous
        manager.fleet_gate_open = True
        manager.last_tick = 18.
        fresh(module, manager, anchors, 18.02, ERRORS)
        manager.latest_payload_state["received_time"] = 17.
        monkeypatch.setattr(manager, "_check_payload_physical_safety", lambda _now: False)
        received = {}
        for vehicle in manager.vehicles:
            def capture(_event, now=None, transport_override=None, key=id(vehicle)):
                received[key] = transport_override
            monkeypatch.setattr(vehicle, "_timer_callback", capture)
        manager._run_fleet_tick(None, 18.02)
        for key, target in received.items():
            assert not target["transport_full_pid_learning"]
            assert target["transport_full_pid_freeze_reason"] == "payload_cached_hold"
            assert target["transport_full_pid_tick"] == 18.02
            assert target["transport_full_pid_force_hold"]
            assert target["transport_full_pid_weight"] == .5
            assert target["transport_full_pid_weight_dot"] == 0.
            assert previous[key]["transport_full_pid_learning"]
            assert previous[key]["transport_full_pid_weight_dot"] == -.9375
    finally:
        manager.logger.close()


def test_csv_records_active_pid_state_and_actual_blended_integral_force(tmp_path):
    module, manager, anchors = native_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        overrides = manager._transport_overrides(18.)
        vehicle = manager.vehicles[0]
        state = vehicle._build_ekf_control_state(vehicle.latest_state, 18.)
        target = vehicle._target_for(state, 18.)
        target.update(overrides[id(vehicle)], transport_blend=.4)
        command = vehicle.controller.compute(state, target, .02)
        vehicle._write_log(18., state, target, command)
        manager.logger.flush()
        with open(manager.logger.path, newline="") as stream:
            row = next(csv.DictReader(stream))
        assert row["transport_feedback_controller"] == "native_position_pid"
        assert row["transport_full_pid_learning"] == "1"
        assert row["transport_hold_integral_learning"] == "0"
        for prefix in ("transport_full_pid_integral_state", "transport_full_pid_force",
                       "transport_full_pid_force_dot", "transport_full_pid_pd_force",
                       "transport_full_pid_pd_force_dot", "transport_full_pid_feedback_force_dot"):
            for index, axis in enumerate("xyz"):
                assert float(row[prefix + "_" + axis]) == pytest.approx(command[prefix][index])
    finally:
        manager.logger.close()
