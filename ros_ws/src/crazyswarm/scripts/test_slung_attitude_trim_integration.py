"""The shared attitude outer loop must preserve references and handovers."""

import numpy as np
import pytest

from test_slung_transport_integral_integration import manager_case, fresh, targets


def pitch_rotation(angle=.12):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def trim_case(tmp_path, feedback="differential_integral", enabled=True):
    module, manager, anchors = manager_case(tmp_path)
    from slung_attitude_trim import SlungAttitudeReferenceTrim
    manager.payload_attitude_trim_enabled = enabled
    manager.payload_attitude_trim_parameters = dict(gain_per_s=np.array([.05, .05, .03]),
        max_offset_m=.01, max_rate_mps=.001, max_acceleration_mps2=.001,
        adaptation_interval_s=1.)
    manager.payload_attitude_trim = SlungAttitudeReferenceTrim(
        manager.slung_controller.config.attachment_points_m,
        enabled=enabled, **manager.payload_attitude_trim_parameters)
    manager.payload_transport_feedback_controller = feedback
    fresh(module, manager, anchors, 0.)
    manager._transport_overrides(0.)
    return module, manager, anchors


def steady(module, manager, anchors, now, rotation=None):
    fresh(module, manager, anchors, now)
    manager.latest_payload_state["rotation"] = pitch_rotation() if rotation is None else rotation
    return manager._transport_overrides(now)


def test_legacy_manager_emits_disabled_trim_without_changing_references(tmp_path):
    module, manager, anchors = manager_case(tmp_path)
    try:
        fresh(module, manager, anchors, 18.)
        result = manager._transport_overrides(18.)
        for item in result.values():
            assert item.get("transport_attitude_trim_enabled") is False
            np.testing.assert_array_equal(item["transport_attitude_trim_position"], np.zeros(3))
    finally:
        manager.logger.close()


@pytest.mark.parametrize("feedback", ["differential_integral", "native_position_pid"])
def test_shared_trim_waits_for_steady_reference_and_adjusts_both_reference_sets(tmp_path, feedback):
    module, manager, anchors = trim_case(tmp_path, feedback)
    try:
        before = steady(module, manager, anchors, 16.)
        assert all(t["transport_attitude_trim_freeze_reason"] == "reference_moving"
                   for t in before.values())
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral, np.zeros(3))
        baseline = steady(module, manager, anchors, 17.)
        steady(module, manager, anchors, 18.)
        result = steady(module, manager, anchors, 18.25)
        trim = targets(manager, result, "transport_attitude_trim_position")
        assert trim[0, 2] > 0. and np.all(trim[1:, 2] < 0.)
        np.testing.assert_allclose(trim.sum(axis=0), 0., atol=1e-16)
        for field in ("position", "position_hold_position"):
            np.testing.assert_allclose(targets(manager, result, field),
                targets(manager, baseline, field) + trim, atol=1e-12)
        for field, diagnostic in (("velocity", "velocity"),
                ("position_hold_velocity", "velocity"), ("acceleration", "acceleration"),
                ("position_hold_acceleration", "acceleration")):
            np.testing.assert_allclose(targets(manager, result, field),
                targets(manager, result, "transport_attitude_trim_" + diagnostic), atol=1e-12)
        for item in result.values():
            np.testing.assert_array_equal(item["payload_target_rotation"], np.eye(3))
        if feedback == "differential_integral":
            errors = np.asarray([v.latest_state["position"]-result[id(v)]["position_hold_position"]
                                 for v in manager.vehicles])
            np.testing.assert_allclose(targets(manager, result, "transport_hold_integral_error"),
                errors-errors.mean(axis=0), atol=1e-12)
        integral = manager.payload_attitude_trim.integral.copy()
        repeated = manager._transport_overrides(18.25)
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral, integral)
        np.testing.assert_array_equal(targets(manager, repeated, "position"), targets(manager, result, "position"))
    finally:
        manager.logger.close()


@pytest.mark.parametrize("fault", ["held_ekf", "payload_source", "force_limit", "rate_limit", "nonpositive_tension", "nonfinite_tension", "infinite_tension"])
def test_feedback_or_limit_fault_freezes_learning_and_finishes_existing_segment(tmp_path, monkeypatch, fault):
    module, manager, anchors = trim_case(tmp_path)
    try:
        steady(module, manager, anchors, 18.)
        first = steady(module, manager, anchors, 18.2)
        integral = manager.payload_attitude_trim.integral.copy()
        fresh(module, manager, anchors, 18.3)
        if fault == "held_ekf":
            manager.vehicles[0].latest_ekf_state["received_time"] = 17.
        elif fault == "payload_source":
            manager.latest_payload_state["sample_time"] = 100.
        elif fault in ("nonpositive_tension", "nonfinite_tension", "infinite_tension"):
            original = manager.slung_controller.compute
            def no_tension(*args):
                result = original(*args)
                result["vehicles"][1]["desired_tension_signed"] = {
                    "nonpositive_tension": 0., "nonfinite_tension": float("nan"),
                    "infinite_tension": float("inf")}[fault]
                return result
            monkeypatch.setattr(manager.slung_controller, "compute", no_tension)
        else:
            key = "force_limit_active" if fault == "force_limit" else "body_rate_limit_active"
            manager.vehicles[0].last_control_command = {key: True, "collective_thrust": .5}
        second = manager._transport_overrides(18.3)
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral, integral)
        assert all(not t["transport_attitude_trim_learning"] for t in second.values())
        assert np.linalg.norm(targets(manager, second, "transport_attitude_trim_velocity")) > 0.
        assert not np.array_equal(targets(manager, first, "transport_attitude_trim_position"),
                                  targets(manager, second, "transport_attitude_trim_position"))
    finally:
        manager.logger.close()


def test_short_payload_gap_advances_only_cached_trim_and_preserves_cached_arrays(tmp_path, monkeypatch):
    module, manager, anchors = trim_case(tmp_path)
    try:
        steady(module, manager, anchors, 18.)
        previous = steady(module, manager, anchors, 18.2)
        frozen = {key: {name: value.copy() if isinstance(value, np.ndarray) else value
                        for name, value in item.items()} for key, item in previous.items()}
        manager.last_transport_overrides = previous
        integral = manager.payload_attitude_trim.integral.copy()
        manager.fleet_gate_open = True
        manager.last_tick = 18.2
        fresh(module, manager, anchors, 18.3)
        manager.latest_payload_state["received_time"] = 17.
        monkeypatch.setattr(manager, "_check_payload_physical_safety", lambda _now: False)
        captured = {}
        for vehicle in manager.vehicles:
            monkeypatch.setattr(vehicle, "_timer_callback", lambda _event, now=None,
                transport_override=None, key=id(vehicle): captured.update({key: transport_override}))
        manager._run_fleet_tick(None, 18.3)
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral, integral)
        expected = manager.payload_attitude_trim.evaluate(18.3)
        for index, vehicle in enumerate(manager.vehicles):
            old, new = frozen[id(vehicle)], captured[id(vehicle)]
            assert new["transport_attitude_trim_freeze_reason"] == "payload_cached_hold"
            for field, trim_field, derivative in (("position", "position", 0),
                    ("position_hold_position", "position", 0), ("velocity", "velocity", 1),
                    ("position_hold_velocity", "velocity", 1), ("acceleration", "acceleration", 2),
                    ("position_hold_acceleration", "acceleration", 2)):
                np.testing.assert_allclose(new[field], old[field] + expected[derivative][index]
                    - old["transport_attitude_trim_" + trim_field], atol=1e-12)
                np.testing.assert_array_equal(previous[id(vehicle)][field], old[field])
    finally:
        manager.logger.close()


def test_loaded_landing_bridges_last_pva_without_adding_trim_twice(tmp_path):
    module, manager, anchors = trim_case(tmp_path)
    try:
        steady(module, manager, anchors, 18.)
        before = steady(module, manager, anchors, 18.25)
        assert np.linalg.norm(targets(manager, before, "velocity")) > 0.
        assert np.linalg.norm(targets(manager, before, "acceleration")) > 0.
        manager.last_transport_overrides = before
        integral = manager.payload_attitude_trim.integral.copy()
        for vehicle in manager.vehicles:
            vehicle.trajectory.begin_controlled_landing(18.25, vehicle.latest_state["position"], 0., "test")
        after = manager._transport_overrides(18.25)
        for field in ("position", "velocity", "acceleration", "position_hold_position",
                      "position_hold_velocity", "position_hold_acceleration"):
            np.testing.assert_allclose(targets(manager, after, field), targets(manager, before, field), atol=1e-11)
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral, integral)
        assert all(t["transport_attitude_trim_freeze_reason"] == "landing" for t in after.values())
        dt = 1e-4
        middle = steady(module, manager, anchors, 18.25+dt)
        np.testing.assert_allclose((targets(manager, middle, "position")-targets(manager, after, "position"))/dt,
                                  targets(manager, after, "velocity"), atol=2e-6)
        # At the end the measured rope snapshot owns the learned geometry;
        # an additional stored trim here would count the correction twice.
        end_time = manager.payload_attitude_trim_landing_bridge["start_time"] + manager.payload_attitude_trim_landing_bridge["duration"]
        end = steady(module, manager, anchors, end_time)
        load = next(iter(end.values()))
        expected = module.rope_preserving_vehicle_references(load["payload_target_position"],
            load["payload_target_velocity"], load["payload_target_acceleration"],
            load["payload_target_rotation"], manager.slung_controller.config.attachment_points_m,
            manager.payload_landing_rope_directions, manager.payload_landing_rope_distances)[0]
        np.testing.assert_allclose(targets(manager, end, "position_hold_position"), expected, atol=1e-10)
    finally:
        manager.logger.close()


@pytest.mark.parametrize("reset", ["handoff", "abort"])
def test_terminal_or_handoff_reset_clears_trim_and_landing_bridge(tmp_path, reset):
    module, manager, anchors = trim_case(tmp_path)
    try:
        steady(module, manager, anchors, 18.)
        assert np.linalg.norm(manager.payload_attitude_trim.integral) > 0.
        manager.payload_attitude_trim_landing_bridge = {"sentinel": True}
        if reset == "handoff":
            manager._reset_payload_handoff()
        else:
            manager._set_abort("test")
        np.testing.assert_array_equal(manager.payload_attitude_trim.integral, np.zeros(3))
        assert manager.payload_attitude_trim_landing_bridge is None
    finally:
        manager.logger.close()


def test_landing_bridge_rejects_outward_acceleration_at_inherited_speed_ceiling(tmp_path):
    module, manager, anchors = trim_case(tmp_path)
    try:
        steady(module, manager, anchors, 18.)
        previous = steady(module, manager, anchors, 19.)
        manager.last_transport_overrides = previous
        base = {key: dict(item) for key, item in previous.items()}
        for item in previous.values():
            for field in ("velocity", "position_hold_velocity"):
                item[field] = np.array([.12, 0., 0.])
            for field in ("acceleration", "position_hold_acceleration"):
                item[field] = np.array([.15, 0., 0.])
        for item in base.values():
            for field in ("velocity", "position_hold_velocity", "acceleration", "position_hold_acceleration"):
                item[field] = np.zeros(3)
        with pytest.raises(ValueError, match="speed ceiling"):
            manager._make_attitude_trim_landing_bridge(19., base)
    finally:
        manager.logger.close()


def test_landing_bridge_failure_uses_supported_retry_and_keeps_trim_motion(tmp_path, monkeypatch):
    module, manager, anchors = trim_case(tmp_path)
    try:
        steady(module, manager, anchors, 18.)
        before = steady(module, manager, anchors, 18.2)
        manager.last_transport_overrides = before
        for vehicle in manager.vehicles:
            vehicle.trajectory.begin_controlled_landing(18.2, vehicle.latest_state["position"], 0., "test")
        def infeasible_bridge(*_args):
            raise ValueError("outward acceleration at speed ceiling")
        monkeypatch.setattr(manager, "_make_attitude_trim_landing_bridge", infeasible_bridge)
        fresh(module, manager, anchors, 18.3)
        result = manager._transport_overrides(18.3)
        assert manager.payload_landing_failure_count == 1
        assert manager.payload_transport_active and not manager.global_abort_reason
        for vehicle in manager.vehicles:
            old, new = before[id(vehicle)], result[id(vehicle)]
            np.testing.assert_array_equal(new["desired_force_override"], old["desired_force_override"])
            assert new["transport_attitude_trim_freeze_reason"] == "landing_planner_retry"
            assert not new["transport_attitude_trim_learning"]
            np.testing.assert_allclose(new["position_hold_velocity"], new["transport_attitude_trim_velocity"], atol=1e-12)
        assert np.linalg.norm(targets(manager, result, "position_hold_velocity")) > 0.
    finally:
        manager.logger.close()
