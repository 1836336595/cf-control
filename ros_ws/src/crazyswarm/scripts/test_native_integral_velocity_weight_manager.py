"""The transport integral velocity weight must reach the real core and CSV."""
import csv

import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_handoff_safety import prepare
from test_slung_transport_integral_integration import ERRORS, fresh
import test_ctbr_controller_v2 as fixtures


def weighted_manager(tmp_path, weight):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path, {"slung_payload": {
        "flight_validation_enabled": False,
        "transport_feedback_controller": "native_position_pid",
        "transport_position_pid_integral_velocity_weight": weight},
        "ctbr_trajectory": {"hover_duration_s": 120.}})
    prepare(manager, anchors, 0.)
    return module, manager, anchors


@pytest.mark.parametrize("weight", [0., .5, 1., 2.])
def test_parameter_controls_native_integral_and_is_recorded_in_csv(tmp_path, weight):
    module, manager, anchors = weighted_manager(tmp_path, weight)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        velocity = np.array([.018, -.011, .003])
        for vehicle in manager.vehicles:
            vehicle.latest_ekf_state["filtered_velocity"] = velocity.copy()
        overrides = manager._transport_overrides(18.)
        vehicle = manager.vehicles[0]
        state = vehicle._build_ekf_control_state(vehicle.latest_state, 18.)
        target = vehicle._target_for(state, 18.)
        target.update(overrides[id(vehicle)])
        assert target["transport_full_pid_integral_velocity_weight"] == weight
        command = vehicle.controller.compute(state, target, .02)
        ep = state["position"] - target["position_hold_position"]
        ev = state["control_velocity"] - target["position_hold_velocity"]
        np.testing.assert_allclose(command["transport_full_pid_integral_state_delta"],
                                   .02*(weight*ev + .5*ep), atol=1e-14)
        vehicle._write_log(18., state, target, command)
        manager.logger.flush()
        with open(manager.logger.path, newline="") as stream:
            row = next(csv.DictReader(stream))
        assert float(row["transport_full_pid_integral_velocity_weight"]) == weight
    finally:
        manager.logger.close()


@pytest.mark.parametrize("weight", [-.1, float("nan"), float("inf"), True, False, [0.], None])
def test_invalid_velocity_weight_fails_at_startup(tmp_path, weight):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="integral_velocity_weight"):
        _slung_manager(module, tmp_path, {"slung_payload": {
            "transport_position_pid_integral_velocity_weight": weight}})


def test_omitted_parameter_restores_legacy_velocity_weight(tmp_path, monkeypatch):
    configure = fixtures._configure_controller_node_ros

    def old_configuration(module, roots):
        roots["slung_payload"].pop("transport_position_pid_integral_velocity_weight", None)
        configure(module, roots)

    monkeypatch.setattr(fixtures, "_configure_controller_node_ros", old_configuration)
    module, manager, anchors = weighted_manager(tmp_path, 0.)
    try:
        assert manager.payload_transport_pid_integral_velocity_weight == 1.
        fresh(module, manager, anchors, 18., ERRORS)
        targets = manager._transport_overrides(18.)
        assert all(target["transport_full_pid_integral_velocity_weight"] == 1.
                   for target in targets.values())
    finally:
        manager.logger.close()
