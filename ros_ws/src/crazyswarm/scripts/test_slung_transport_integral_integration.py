"""Exercise differential hold adaptation through the actual manager and CTBR law."""

import csv
from types import SimpleNamespace

import numpy as np
import pytest

from test_ctbr_controller_v2 import _config, _controller_module, _slung_manager, _state, _target
from test_slung_handoff_safety import prepare


def manager_case(directory, enabled=True, mode="reference_cascade"):
    module = _controller_module()
    module.rospy.logerr = lambda *_args: None
    manager, anchors = _slung_manager(module, directory, {
        "slung_payload": {"flight_validation_enabled": False,
            "transport_attitude_trim_enabled": False,
            "transport_feedback_controller": "differential_integral",
            "transport_force_mode": mode, "transport_hold_integral_enabled": enabled,
            "transport_hold_integral_gain_npm_s": .02,
            "transport_hold_integral_force_limit_n": .03,
            "transport_hold_integral_rate_limit_nps": .002},
        "ctbr_trajectory": {"hover_duration_s": 120.}})
    prepare(manager, anchors, 0.)
    manager.payload_control_dt = .02
    return module, manager, anchors


def fresh(module, manager, anchors, now, offsets=None):
    module.rospy.Time.now = lambda: SimpleNamespace(to_sec=lambda: 100. + now)
    manager.latest_payload_state.update(received_time=now, sample_time=100. + now-.01,
                                       position=np.array([0., 0., .60]), derivatives_valid=True)
    offsets = np.zeros((3, 3)) if offsets is None else np.asarray(offsets)
    for vehicle, anchor, offset in zip(manager.vehicles, anchors, offsets):
        position = anchor + [0., 0., .575] + offset
        vehicle.latest_state.update(position=position, received_time=now,
            mocap_sample_time_s=100.+now, filtered_velocity=np.zeros(3),
            filtered_acceleration=np.zeros(3), filter_derivatives_valid=True)
        vehicle.latest_ekf_state = dict(position=position.copy(), received_time=now,
            filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
            filter_derivatives_valid=True)


def targets(manager, overrides, field):
    return np.asarray([overrides[id(v)][field] for v in manager.vehicles])


ERRORS = np.array([[.10, -.02, -.04], [-.04, .05, .02], [-.06, -.03, .02]])


def test_learning_waits_for_both_references_and_updates_once_per_fleet_tick(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 16., ERRORS)
        before = manager._transport_overrides(16.)
        assert not before[id(manager.vehicles[0])].get("transport_hold_integral_learning", False)
        fresh(module, manager, anchors, 18., ERRORS)
        first = manager._transport_overrides(18.)
        assert first[id(manager.vehicles[0])].get("transport_hold_integral_learning", False)
        bias = targets(manager, first, "transport_hold_integral_bias")
        assert np.linalg.norm(bias) > 0
        np.testing.assert_allclose(np.sum(bias, axis=0), 0., atol=1e-16)
        second = manager._transport_overrides(18.)
        np.testing.assert_array_equal(targets(manager, second, "transport_hold_integral_bias"), bias)
    finally:
        manager.logger.close()


@pytest.mark.parametrize("fault", ["held_ekf", "disabled_ekf", "stale_payload_source", "host_force_limit", "host_rate_limit", "hold_limit", "collective_limit", "source_derivatives", "filtered_derivatives"])
def test_unreliable_or_limited_feedback_freezes_learned_bias(tmp_path, fault):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        first = manager._transport_overrides(18.)
        bias = targets(manager, first, "transport_hold_integral_bias")
        fresh(module, manager, anchors, 18.02, ERRORS)
        vehicle = manager.vehicles[0]
        if fault == "held_ekf":
            vehicle.latest_ekf_state["received_time"] = 17.
        elif fault == "disabled_ekf":
            vehicle.ekf_kinematics_weight = 0.
        elif fault == "stale_payload_source":
            manager.latest_payload_state["sample_time"] = 100.
        elif fault == "source_derivatives":
            vehicle.latest_state["derivatives_valid"] = False
        elif fault == "filtered_derivatives":
            vehicle.latest_state["filter_derivatives_valid"] = False
        else:
            key = {"host_force_limit": "force_limit_active", "host_rate_limit": "body_rate_limit_active",
                   "hold_limit": "hold_feedback_limit_active", "collective_limit": "collective_limit_active"}[fault]
            vehicle.last_control_command = {key: True, "collective_thrust": .5}
        second = manager._transport_overrides(18.02)
        np.testing.assert_array_equal(targets(manager, second, "transport_hold_integral_bias"), bias)
        assert all(not t["transport_hold_integral_learning"] for t in second.values())
        np.testing.assert_array_equal(targets(manager, second, "transport_hold_integral_force_dot"), np.zeros((3, 3)))
    finally:
        manager.logger.close()


@pytest.mark.parametrize("enabled,mode", [(False, "reference_cascade"), (True, "coupled")])
def test_disabled_and_legacy_mode_never_add_bias(tmp_path, enabled, mode):
    module, manager, anchors = manager_case(tmp_path, enabled, mode)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        result = manager._transport_overrides(18.)
        np.testing.assert_array_equal(targets(manager, result, "transport_hold_integral_force"), np.zeros((3, 3)))
        assert not any(t["transport_hold_integral_learning"] for t in result.values())
    finally:
        manager.logger.close()


def test_native_position_pid_transport_feedback_is_selectable_from_payload_config(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        manager.payload_transport_feedback_controller = "native_position_pid"
        manager.payload_transport_pid_integral_gain_scale = 3.0
        manager.payload_transport_pid_integral_limit_scale = 10.0 / 3.0
        manager.payload_transport_pid_integral_gate_m = .23
        fresh(module, manager, anchors, 18., ERRORS)
        result = manager._transport_overrides(18.)
        for target in result.values():
            assert target["transport_feedback_controller"] == "native_position_pid"
            assert target["transport_full_pid_learning"]
            np.testing.assert_array_equal(target["transport_hold_integral_force"], np.zeros(3))
            np.testing.assert_array_equal(target["transport_hold_integral_force_dot"], np.zeros(3))
        vehicle = manager.vehicles[0]
        state = vehicle._build_ekf_control_state(vehicle.latest_state, 18.)
        target = vehicle._target_for(state, 18.)
        target.update(result[id(vehicle)])
        command = vehicle.controller.compute(state, target, .02)
        assert command["transport_feedback_controller"] == "native_position_pid"
        assert np.linalg.norm(command["transport_full_pid_force"]) > 0.0
    finally:
        manager.logger.close()


def test_lowering_holds_bias_then_unloading_fades_without_duplicate_learning(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        first = manager._transport_overrides(18.)
        bias = targets(manager, first, "transport_hold_integral_bias")
        manager.payload_validation_landing_stage = "payload_lowering"
        manager._begin_validation_vehicle_descent(18.)
        lowering = manager._transport_overrides(18.)
        np.testing.assert_array_equal(targets(manager, lowering, "transport_hold_integral_force"), bias)
        assert not any(t["transport_hold_integral_learning"] for t in lowering.values())
        manager.payload_unload_start_time = 18.
        manager.payload_landing_trim_at_unload = manager.payload_support_trim.bias.copy()
        fresh(module, manager, anchors, 19., ERRORS)
        middle = manager._transport_overrides(19.)
        np.testing.assert_allclose(targets(manager, middle, "transport_hold_integral_force"), .5*bias, atol=1e-15)
        fresh(module, manager, anchors, 20., ERRORS)
        end = manager._transport_overrides(20.)
        np.testing.assert_array_equal(targets(manager, end, "transport_hold_integral_force"), np.zeros((3, 3)))
        manager._reset_payload_handoff()
        np.testing.assert_array_equal(manager.payload_transport_integral.bias, np.zeros((3, 3)))
    finally:
        manager.logger.close()


def test_global_abort_clears_new_state(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        manager._transport_overrides(18.)
        manager._set_abort("test")
        np.testing.assert_array_equal(manager.payload_transport_integral.bias, np.zeros((3, 3)))
    finally:
        manager.logger.close()


def test_ctbr_bias_force_and_derivative_follow_transport_blend_and_ignore_independent_mode():
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))
    target = _target()
    target.update(transport_mode=True, transport_blend=.4, transport_blend_dot=.1,
        desired_force_override=np.array([0., 0., .5]), desired_force_dot_override=np.zeros(3))
    base = controller.compute(_state(), target, .02)
    controller.reset()
    bias = np.array([.01, -.015, .02]); rate = np.array([.001, .002, -.001])
    target.update(transport_hold_integral_force=bias, transport_hold_integral_force_dot=rate)
    changed = controller.compute(_state(), target, .02)
    np.testing.assert_allclose(changed["raw_desired_force"]-base["raw_desired_force"], .4*bias, atol=1e-15)
    np.testing.assert_allclose(changed["desired_force_dot"]-base["desired_force_dot"], .4*rate+.1*bias, atol=1e-15)
    np.testing.assert_allclose(changed["transport_hold_integral_force"], .4*bias)
    target.update(independent_mode=True, transport_mode=False)
    independent = controller.compute(_state(), target, .02)
    np.testing.assert_array_equal(independent["transport_hold_integral_force"], np.zeros(3))


@pytest.mark.parametrize("name,value", [("transport_hold_integral_enabled", "true"),
    ("transport_hold_integral_gain_npm_s", 0.), ("transport_hold_integral_force_limit_n", float("nan")),
    ("transport_hold_integral_rate_limit_nps", -1.)])
def test_invalid_integral_parameters_rejected_at_startup(tmp_path, name, value):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="integral"):
        _slung_manager(module, tmp_path, {"slung_payload": {name: value}})


def test_validation_deadline_freezes_before_same_tick_changes_to_landing(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        first = manager._transport_overrides(18.)
        bias = targets(manager, first, "transport_hold_integral_bias")
        manager.payload_flight_validation_enabled = True
        manager.payload_flight_validation_transport_s = 18.02
        fresh(module, manager, anchors, 18.02, ERRORS)
        stopped = manager._transport_overrides(18.02)
        np.testing.assert_array_equal(targets(manager, stopped, "transport_hold_integral_bias"), bias)
        assert all(t["transport_hold_integral_freeze_reason"] == "validation_due" for t in stopped.values())
    finally:
        manager.logger.close()


def test_nonpositive_requested_tension_freezes_the_whole_fleet(tmp_path, monkeypatch):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        first = manager._transport_overrides(18.)
        bias = targets(manager, first, "transport_hold_integral_bias")
        original_compute = manager.slung_controller.compute

        def no_tension(*args):
            result = original_compute(*args)
            result["vehicles"][1]["desired_tension_signed"] = 0.
            return result

        monkeypatch.setattr(manager.slung_controller, "compute", no_tension)
        fresh(module, manager, anchors, 18.02, ERRORS)
        stopped = manager._transport_overrides(18.02)
        np.testing.assert_array_equal(targets(manager, stopped, "transport_hold_integral_bias"), bias)
        assert all(t["transport_hold_integral_freeze_reason"] == "nonpositive_tension" for t in stopped.values())
    finally:
        manager.logger.close()


def test_force_filter_lag_is_not_reported_as_saturation_but_real_limits_are():
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module, transport_force_filter_tau_s=.05,
                                                      max_body_rate=np.full(3, .05)))
    target = _target()
    target.update(transport_mode=True, transport_blend=1., desired_force_override=np.array([0., 0., .5]),
                  desired_force_dot_override=np.zeros(3), max_tilt_rad=np.deg2rad(20.))
    controller.compute(_state(), target, .02)
    target["desired_force_override"] = np.array([.02, 0., .6])
    filtered = controller.compute(_state(), target, .02)
    assert np.linalg.norm(filtered["raw_desired_force"] - filtered["desired_force"]) > .01
    assert not filtered["force_limit_active"]
    target["desired_force_override"] = np.array([1., 0., .6])
    limited = controller.compute(_state(), target, .02)
    assert limited["force_limit_active"]
    assert limited["body_rate_limit_active"]


def test_csv_distinguishes_learned_state_from_blended_force_and_logs_independent_zero(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        overrides = manager._transport_overrides(18.)
        vehicle = manager.vehicles[0]
        state = vehicle.latest_state
        target = vehicle._target_for(state, 18.)
        target.update(overrides[id(vehicle)])
        target.update(transport_blend=.4, transport_blend_dot=0.)
        command = vehicle.controller.compute(state, target, .02)
        vehicle._write_log(18., state, target, command)
        target.update(independent_mode=True, transport_mode=False)
        independent = vehicle.controller.compute(state, target, .02)
        vehicle._write_log(18.02, state, target, independent)
        manager.logger.flush()
        with open(manager.logger.path, newline="") as stream:
            first, second = list(csv.DictReader(stream))
        for axis, value in zip("xyz", target["transport_hold_integral_bias"]):
            assert float(first["transport_hold_integral_bias_" + axis]) == pytest.approx(value)
            assert float(first["transport_hold_integral_force_" + axis]) == pytest.approx(.4*value)
            assert float(second["transport_hold_integral_force_" + axis]) == 0.
        assert first["transport_hold_integral_learning"] == "1"
        assert first["transport_mode"] == "1"
        assert second["independent_mode"] == "1"
    finally:
        manager.logger.close()


def test_short_payload_gap_holds_bias_without_relearning_or_mutating_cached_target(tmp_path, monkeypatch):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18., ERRORS)
        previous = manager._transport_overrides(18.)
        bias = manager.payload_transport_integral.bias.copy()
        manager.last_transport_overrides = previous
        manager.fleet_gate_open = True
        manager.last_tick = 18.
        fresh(module, manager, anchors, 18.02, ERRORS)
        manager.latest_payload_state["received_time"] = 17.
        # Isolate the cached-output lifecycle; the normal phase/health and
        # short-gap branches run, while ROS actuation and unrelated guards do not.
        monkeypatch.setattr(manager, "_check_payload_physical_safety", lambda _now: False)
        received = {}
        for vehicle in manager.vehicles:
            def capture(_event, now=None, transport_override=None, key=id(vehicle)):
                received[key] = transport_override
            monkeypatch.setattr(vehicle, "_timer_callback", capture)
        manager._run_fleet_tick(None, 18.02)
        np.testing.assert_array_equal(manager.payload_transport_integral.bias, bias)
        for vehicle in manager.vehicles:
            target = received[id(vehicle)]
            assert target["transport_hold_integral_freeze_reason"] == "payload_cached_hold"
            assert not target["transport_hold_integral_learning"]
            np.testing.assert_array_equal(target["transport_hold_integral_force"], previous[id(vehicle)]["transport_hold_integral_force"])
            np.testing.assert_array_equal(target["transport_hold_integral_force_dot"], np.zeros(3))
            assert previous[id(vehicle)]["transport_hold_integral_learning"]
    finally:
        manager.logger.close()
