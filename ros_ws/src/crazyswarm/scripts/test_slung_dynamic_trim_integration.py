"""Moving payload references must use bounded trim without learning the path."""

import csv

import numpy as np
import pytest

from slung_reference import _rotation_exp
from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_transport_integral_integration import fresh, targets


def dynamic_case(directory, mode="attitude_cone", **changes):
    module = _controller_module()
    payload = dict(
        trajectory=dict(mode=mode, cone_half_angle_deg=15., revolutions=1.,
                        center_xy=[0., 0.], speed_mps=.1, settle_s=3.),
        flight_validation_enabled=True, flight_validation_transport_s=100.,
        transport_force_mode="reference_cascade", transport_attitude_trim_enabled=True,
        transport_attitude_trim_dynamic_enabled=True,
        transport_attitude_trim_feedback_max_rate_rps=.5,
        transport_attitude_trim_feedback_pose_jump_deg=1.,
        transport_attitude_trim_feedback_recovery_s=.5)
    payload.update(changes)
    manager, anchors = _slung_manager(module, directory, {
        "ctbr_trajectory": {"hover_duration_s": 100.}, "slung_payload": payload})
    manager.payload_transport_active = True
    manager.payload_transport_start_time = 0.
    manager.payload_transport_start_position = manager.latest_payload_state["position"].copy()
    manager.payload_transport_vehicle_start_positions = anchors.copy()
    manager.payload_handoff_reference_positions = anchors.copy()
    fresh(module, manager, anchors, 0.)
    manager.latest_payload_state["position"] = manager.payload_transport_start_position.copy()
    manager._transport_overrides(0.)
    return module, manager, anchors


def tick(module, manager, anchors, now, bias=0.):
    fresh(module, manager, anchors, now)
    # Evaluate the nominal full reference without consuming a trim update.
    manager.payload_attitude_trim_enabled = False
    baseline = manager._transport_overrides(now)
    manager.payload_attitude_trim_enabled = True
    item = next(iter(baseline.values()))
    manager.latest_payload_state.update(
        position=item["payload_target_position"].copy(),
        rotation=item["payload_target_rotation"] @ _rotation_exp([0., bias, 0.]),
        body_rate=item["payload_target_body_rate"].copy(),
        velocity=item["payload_target_velocity"].copy(),
        acceleration=item["payload_target_acceleration"].copy())
    for vehicle in manager.vehicles:
        vehicle.latest_state["position"] = baseline[id(vehicle)]["position_hold_position"].copy()
        # Keep the synthetic EKF at the same moving position: before h17 the
        # load is still lifting, not already at fresh()'s steady hover height.
        vehicle.latest_ekf_state["position"] = vehicle.latest_state["position"].copy()
    return manager._transport_overrides(now), baseline


@pytest.mark.parametrize("mode", ["circle", "figure_eight", "attitude_cone"])
def test_moving_trim_uses_instantaneous_full_goal_without_learning_exact_tracking(tmp_path, mode):
    from slung_attitude_trim import SlungDynamicAttitudeReferenceTrim
    module, manager, anchors = dynamic_case(tmp_path, mode)
    try:
        assert isinstance(manager.payload_attitude_trim, SlungDynamicAttitudeReferenceTrim)
        for now in np.arange(18., 40., .05):
            result, _ = tick(module, manager, anchors, float(now))
            item = next(iter(result.values()))
            assert item["transport_attitude_trim_dynamic_enabled"]
            assert item["transport_attitude_trim_freeze_reason"] != "reference_moving"
            np.testing.assert_allclose(item["transport_attitude_trim_error"], 0., atol=1e-12)
            np.testing.assert_allclose(targets(manager, result, "transport_attitude_trim_position"), 0., atol=1e-12)
    finally:manager.logger.close()


def test_moving_bias_updates_during_scan_and_both_reference_sets_keep_pva(tmp_path):
    module, manager, anchors = dynamic_case(tmp_path)
    try:
        for now in np.arange(18., 30., .05):
            tick(module, manager, anchors, float(now), bias=.08)
        now, step = 30.2, 1e-4
        pairs = [tick(module, manager, anchors, t, bias=.08) for t in (now-step, now, now+step)]
        middle = pairs[1][0]
        offset = targets(manager, middle, "transport_attitude_trim_position")
        assert np.linalg.norm(offset) > 1e-5
        np.testing.assert_allclose(offset.sum(axis=0), 0., atol=1e-15)
        for vehicle in manager.vehicles:
            a,b,c = [pair[0][id(vehicle)] for pair in pairs]
            baseline = pairs[1][1][id(vehicle)]
            for p,v,acc in [("position", "velocity", "acceleration"),
                           ("position_hold_position", "position_hold_velocity", "position_hold_acceleration")]:
                np.testing.assert_allclose(b[p]-baseline[p], b["transport_attitude_trim_position"], atol=1e-12)
                np.testing.assert_allclose((c[p]-a[p])/(2*step), b[v], atol=2e-7)
                np.testing.assert_allclose((c[v]-a[v])/(2*step), b[acc], atol=2e-7)
        frozen = manager.payload_attitude_trim.integral.copy()
        duplicate = manager._transport_overrides(now+step)
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral, frozen)
        np.testing.assert_array_equal(targets(manager, duplicate, "transport_attitude_trim_position"),
                                      targets(manager, pairs[2][0], "transport_attitude_trim_position"))
    finally:manager.logger.close()


@pytest.mark.parametrize("fault,reason", [("rate", "payload_rate_outlier"),
    ("pose", "payload_pose_jump"), ("source_order", "payload_sample_order")])
def test_bad_dynamic_feedback_freezes_then_recovers_without_offset_jump(tmp_path, fault, reason):
    module, manager, anchors = dynamic_case(tmp_path)
    try:
        for now in (18., 19., 20.):tick(module, manager, anchors, now, bias=.05)
        before = manager.payload_attitude_trim.integral.copy()
        fresh(module, manager, anchors, 20.02)
        if fault == "rate":manager.latest_payload_state["body_rate"] = np.array([0., 8., 0.])
        elif fault == "pose":manager.latest_payload_state["rotation"] = _rotation_exp([0., .7, 0.])
        else:manager.latest_payload_state["sample_time"] = 119.9
        state = manager.latest_payload_state
        assert manager._dynamic_attitude_trim_feedback_reason(20.02, state) == reason
        assert not manager.payload_attitude_trim_feedback_healthy
        # Same source/time may be checked repeatedly without losing its result.
        assert manager._dynamic_attitude_trim_feedback_reason(20.02, state) == reason
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral, before)
        result,_ = tick(module, manager, anchors, 20.04, bias=.05)
        assert next(iter(result.values()))["transport_attitude_trim_freeze_reason"] == "payload_feedback_recovery"
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral, before)
        result,_ = tick(module, manager, anchors, 20.8, bias=.05)
        assert next(iter(result.values()))["transport_attitude_trim_feedback_healthy"]
    finally:manager.logger.close()


def test_dynamic_world_offsets_stay_world_fixed_during_cached_circle_gap(tmp_path):
    module, manager, anchors = dynamic_case(tmp_path, "circle")
    try:
        for now in np.arange(18., 31., .1):tick(module, manager, anchors, float(now), bias=.05)
        trim = manager.payload_attitude_trim
        old = {key: dict(value) for key,value in manager.last_transport_overrides.items()}
        # Cached hold takes the last command; sample helper itself need not save it.
        result,_ = tick(module, manager, anchors, 31.2, bias=.05)
        old = {key: dict(value) for key,value in result.items()}
        frozen = trim.integral.copy()
        manager._refresh_cached_attitude_trim(31.3, old, "payload_cached_hold")
        expected = trim.evaluate(31.3)
        np.testing.assert_allclose(targets(manager, old, "transport_attitude_trim_position"), expected[0], atol=1e-12)
        np.testing.assert_array_equal(trim.integral, frozen)
        assert all(not item["transport_attitude_trim_feedback_healthy"] for item in old.values())
        assert all(item["transport_attitude_trim_feedback_quality_reason"] == "payload_cached_hold"
                   for item in old.values())
    finally:manager.logger.close()


def test_flight_csv_records_dynamic_mode_and_feedback_quality(tmp_path):
    module, manager, anchors = dynamic_case(tmp_path)
    try:
        result,_ = tick(module, manager, anchors, 30., bias=.05)
        vehicle = manager.vehicles[0]
        vehicle.payload_diagnostics = dict(manager.latest_payload_state)
        state = dict(vehicle.latest_state, filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3))
        target = vehicle._target_for(state, 30.)
        target.update(result[id(vehicle)])
        command = vehicle.controller.compute(state, target, 1./70.)
        vehicle._write_log(30., state, target, command, control_state=state)
        manager.logger.flush()
        with open(manager.logger.path) as stream:row = next(csv.DictReader(stream))
        assert row["transport_attitude_trim_dynamic_enabled"] == "1"
        assert row["transport_attitude_trim_feedback_healthy"] == "1"
        assert row["transport_attitude_trim_feedback_quality_reason"] == ""
        assert float(row["transport_attitude_trim_body_rate_norm_rps"]) < .5
    finally:manager.logger.close()


@pytest.mark.parametrize("mode", ["circle", "attitude_cone"])
def test_dynamic_trim_interrupted_landing_bridges_previous_complete_pva(tmp_path, mode):
    module, manager, anchors = dynamic_case(tmp_path, mode)
    try:
        for now in np.arange(18., 31., .1):tick(module, manager, anchors, float(now), bias=.05)
        now = 31.2
        old,_ = tick(module, manager, anchors, now, bias=.05)
        manager.last_transport_overrides = old
        item = next(iter(old.values()))
        manager.latest_payload_state.update(position=item["payload_target_position"].copy(),
            rotation=item["payload_target_rotation"].copy(),
            velocity=item["payload_target_velocity"].copy(),
            acceleration=item["payload_target_acceleration"].copy())
        for vehicle in manager.vehicles:
            vehicle.latest_state["position"] = old[id(vehicle)]["position"].copy()
            vehicle.trajectory.begin_controlled_landing(now, vehicle.latest_state["position"], 0., "dynamic_trim_exit")
        new = manager._transport_overrides(now)
        for vehicle in manager.vehicles:
            for field in ("position", "velocity", "acceleration", "position_hold_position",
                          "position_hold_velocity", "position_hold_acceleration"):
                np.testing.assert_allclose(new[id(vehicle)][field], old[id(vehicle)][field], atol=1e-10)
            assert not new[id(vehicle)]["transport_attitude_trim_learning"]
    finally:manager.logger.close()


def test_duplicate_source_sample_and_handoff_reset_keep_quality_history_consistent(tmp_path):
    module, manager, anchors = dynamic_case(tmp_path)
    try:
        tick(module, manager, anchors, 19., bias=.04)
        state = dict(manager.latest_payload_state)
        assert manager._dynamic_attitude_trim_feedback_reason(19.1, state) == ""
        assert manager._dynamic_attitude_trim_feedback_reason(19.2, state) == ""
        # A pose jump with that same timestamp remains an outlier on future
        # control ticks; receiving a fresh timestamp is needed for recovery.
        state["rotation"] = _rotation_exp([0., .8, 0.])
        assert manager._dynamic_attitude_trim_feedback_reason(19.3, state) == "payload_pose_jump"
        assert manager._dynamic_attitude_trim_feedback_reason(19.4, state) == "payload_pose_jump"
        manager._reset_payload_handoff()
        assert manager._attitude_trim_feedback_sample is None
        assert not manager.payload_attitude_trim_feedback_healthy
        np.testing.assert_array_equal(manager.payload_attitude_trim.evaluate(20.), np.zeros((3,3,3)))
    finally:manager.logger.close()


def test_dynamic_trim_can_update_during_lift_after_tension_ramp(tmp_path):
    module, manager, anchors = dynamic_case(tmp_path,
        transport_attitude_trim_during_lift_enabled=True)
    try:
        for now in (0.1, 2., 4.99):
            result,_ = tick(module, manager, anchors, now, bias=.08)
            assert all(not item["transport_attitude_trim_learning"] for item in result.values())
            np.testing.assert_array_equal(manager.payload_attitude_trim.integral, np.zeros(3))
        result,_ = tick(module, manager, anchors, 5., bias=.08)
        assert all(item["transport_attitude_trim_during_lift_enabled"] for item in result.values())
        assert all(item["transport_attitude_trim_learning"] for item in result.values())
        assert np.linalg.norm(manager.payload_attitude_trim.integral) > 0.
        # This option must not enable the separate position-force integrators
        # before their ordinary reference-establishment deadline.
        assert all(not item["transport_full_pid_learning"] for item in result.values())
        assert all(not item["transport_hold_integral_learning"] for item in result.values())
        result,_ = tick(module, manager, anchors, 5.5, bias=.08)
        assert np.linalg.norm(targets(manager,result,"transport_attitude_trim_position")) > 0.
    finally:manager.logger.close()


def test_early_dynamic_trim_never_updates_with_incomplete_transport_blend(tmp_path):
    module, manager, anchors = dynamic_case(tmp_path,
        transport_attitude_trim_during_lift_enabled=True)
    try:
        result,_ = tick(module, manager, anchors, 6., bias=.08)
        for item in result.values():item["transport_blend"] = .9
        state = manager.latest_payload_state
        controls = [vehicle._build_ekf_control_state(vehicle.latest_state,6.) for vehicle in manager.vehicles]
        reason = manager._transport_learning_reason(6.,state,controls,result,False,True,True,
            allow_dynamic_reference=True,allow_reference_lift=True)
        assert reason == "tension_ramp"
    finally:manager.logger.close()


def test_early_update_switch_can_restore_previous_reference_wait(tmp_path):
    module, manager, anchors = dynamic_case(tmp_path,
        transport_attitude_trim_during_lift_enabled=False)
    try:
        result,_ = tick(module, manager, anchors, 8., bias=.08)
        assert all(item["transport_attitude_trim_freeze_reason"] == "reference_moving" for item in result.values())
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral,0.)
    finally:manager.logger.close()


def test_early_switch_alone_does_not_change_legacy_static_trim_wait(tmp_path):
    module, manager, anchors = dynamic_case(tmp_path,
        transport_attitude_trim_dynamic_enabled=False,
        transport_attitude_trim_during_lift_enabled=True)
    try:
        result,_ = tick(module, manager, anchors, 8., bias=.08)
        assert all(item["transport_attitude_trim_freeze_reason"] == "reference_moving" for item in result.values())
        assert all(not item["transport_attitude_trim_during_lift_enabled"] for item in result.values())
    finally:manager.logger.close()


def test_during_lift_parameter_requires_a_boolean(tmp_path):
    with pytest.raises(RuntimeError,match="trim"):
        dynamic_case(tmp_path,transport_attitude_trim_during_lift_enabled="yes")


def test_early_trim_keeps_health_gates_and_starts_with_zero_pva(tmp_path):
    module, manager, anchors = dynamic_case(tmp_path,
        transport_attitude_trim_during_lift_enabled=True)
    try:
        result,_ = tick(module, manager, anchors, 5., bias=.08)
        for field in ("position","velocity","acceleration"):
            np.testing.assert_array_equal(targets(manager,result,"transport_attitude_trim_"+field),np.zeros((3,3)))
        before=manager.payload_attitude_trim.integral.copy()
        fresh(module,manager,anchors,5.1)
        manager.latest_payload_state["body_rate"]=np.array([0.,8.,0.])
        result=manager._transport_overrides(5.1)
        assert all(item["transport_attitude_trim_freeze_reason"]=="payload_rate_outlier" for item in result.values())
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral,before)
        assert np.linalg.norm(targets(manager,result,"transport_attitude_trim_velocity"))>0.
    finally:manager.logger.close()


def test_exact_tracking_early_trim_does_not_learn_the_initial_leveling_path(tmp_path):
    module,manager,anchors=dynamic_case(tmp_path,
        transport_attitude_trim_during_lift_enabled=True)
    try:
        manager.payload_transport_start_rotation=_rotation_exp([0.,.2,0.])
        for now in np.arange(5.,17.,.1):
            result,_=tick(module,manager,anchors,float(now))
            np.testing.assert_allclose(targets(manager,result,"transport_attitude_trim_position"),0.,atol=1e-12)
            np.testing.assert_allclose(manager.payload_attitude_trim.integral,0.,atol=1e-12)
    finally:manager.logger.close()


@pytest.mark.parametrize("parameter,value", [
    ("transport_attitude_trim_dynamic_enabled", "yes"),
    ("transport_attitude_trim_feedback_max_rate_rps", 0.),
    ("transport_attitude_trim_feedback_pose_jump_deg", float("nan")),
    ("transport_attitude_trim_feedback_recovery_s", -1.)])
def test_dynamic_trim_configuration_rejects_invalid_values(tmp_path, parameter, value):
    with pytest.raises(RuntimeError, match="trim|构型"):
        dynamic_case(tmp_path, **{parameter:value})
