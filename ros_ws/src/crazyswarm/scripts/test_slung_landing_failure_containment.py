"""Planner failures must not terminate the shared CTBR control timer."""

import csv

import numpy as np
import pytest

from test_slung_transport_integral_integration import manager_case, fresh, ERRORS


def flight_case(tmp_path, monkeypatch):
    module, manager, anchors = manager_case(tmp_path)
    # The shared low-level fixture uses a close bench formation. Spread it
    # before exercising the real flight distance guard and vehicle callbacks.
    anchors[:, :2] *= 4.
    manager.payload_transport_vehicle_start_positions = anchors.copy()
    manager.payload_handoff_reference_positions = anchors.copy()
    clock = [18.]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    module.rospy.logerr = lambda *_args: None
    manager.fleet_gate_open = True
    manager.payload_flight_validation_enabled = True
    manager.payload_flight_validation_transport_s = 18.02
    published = {id(v): [] for v in manager.vehicles}
    for vehicle in manager.vehicles:
        vehicle.set_fleet_gate(True)
        vehicle.preflight_voltage_ready = True
        vehicle.last_tick = 18. - 1. / 70.
        monkeypatch.setattr(vehicle, "_publish_path", lambda _state: None)
        monkeypatch.setattr(vehicle, "_publish_command",
            lambda command, key=id(vehicle): published[key].append(command))
        monkeypatch.setattr(vehicle, "_publish_zero",
            lambda key=id(vehicle): published[key].append(None))
    manager.last_tick = 18. - 1. / 70.
    fresh(module, manager, anchors, 18., ERRORS)
    manager._timer_callback(None)
    assert all(commands[-1]["collective_thrust"] > 0 for commands in published.values())

    def tick(now, velocity=(0., 0., 0.), acceleration=(0., 0., 0.), at_ground=False):
        clock[0] = now
        fresh(module, manager, anchors, now, ERRORS)
        if at_ground:
            manager.latest_payload_state["position"][2] = manager.payload_ground_center_z
        manager.latest_payload_state.update(velocity=np.asarray(velocity),
                                            acceleration=np.asarray(acceleration))
        manager._timer_callback(None)

    return module, manager, anchors, published, tick


def test_actual_planner_error_at_validation_keeps_all_three_commands_and_recovers(tmp_path, monkeypatch):
    module, manager, _, published, tick = flight_case(tmp_path, monkeypatch)
    try:
        # Any continuous descent starting at the ground with downward velocity
        # violates the ground constraint, including the bounded piecewise plan.
        manager.payload_ground_center_z = .025
        bias = manager.payload_transport_integral.bias.copy()
        tick(18.02, velocity=(0., 0., -.2), acceleration=(0., 0., -.2), at_ground=True)
        assert all(len(commands) == 2 for commands in published.values())
        assert all(commands[-1]["collective_thrust"] > 0 for commands in published.values())
        assert manager.payload_landing_profile is None
        assert manager.payload_landing_start_time is None
        assert manager.payload_landing_failure_count == 1
        assert manager.payload_landing_failure_snapshot["start_velocity"] == [0., 0., -.2]
        assert manager.payload_landing_failure_reason
        np.testing.assert_array_equal(manager.payload_transport_integral.bias, bias)
        started = manager.payload_validation_landing_started_time
        tick(18.04, velocity=(0., 0., -.2), acceleration=(0., 0., -.2), at_ground=True)
        assert manager.payload_landing_failure_count == 2
        assert all(len(commands) == 3 for commands in published.values())
        tick(18.06)
        assert manager.payload_landing_profile is not None
        assert manager.payload_landing_start_time == 18.06
        assert manager.payload_validation_landing_started_time == started
        assert manager.payload_validation_landing_stage == "payload_lowering"
        assert manager.payload_landing_failure_snapshot["start_velocity"] == [0., 0., -.2]
        np.testing.assert_array_equal(manager.payload_transport_integral.bias, bias)
        assert all(len(commands) == 4 for commands in published.values())
        assert all(np.all(np.isfinite(commands[-1]["desired_force"])) for commands in published.values())
    finally:
        manager.logger.close()


def test_repeated_planning_failure_exits_bounded_hold_with_supported_descent(tmp_path, monkeypatch):
    _, manager, _, published, tick = flight_case(tmp_path, monkeypatch)
    try:
        manager.payload_ground_center_z = .025
        for now in (18.02, 18.04, 18.06, 18.08):
            tick(now, velocity=(0., 0., -.2), at_ground=True)
        assert all(len(commands) == 5 for commands in published.values())
        assert all(commands[-1]["collective_thrust"] > 0 for commands in published.values())
        assert all(v.trajectory.phase == "emergency_landing" for v in manager.vehicles)
        assert manager.payload_validation_landing_stage == "planning_fault_descent"
        assert len(manager.payload_emergency_support) == 3
        assert not manager.payload_transport_active
        assert "landing_planner" in manager.payload_controlled_exit_reason
        assert manager.payload_validation_landing_started_time == 18.02
    finally:
        manager.logger.close()


@pytest.mark.parametrize("cache", ["absent", "nonfinite"])
def test_planning_hold_uses_finite_previous_commands_if_cache_is_unusable(tmp_path, monkeypatch, cache):
    _, manager, _, published, tick = flight_case(tmp_path, monkeypatch)
    try:
        assert manager._check_payload_flight_validation(18.02)
        if cache == "absent":
            manager.last_transport_overrides = {}
        else:
            for target in manager.last_transport_overrides.values():
                target["desired_force_override"] = np.full(3, np.nan)
        manager.payload_ground_center_z = .025
        tick(18.02, velocity=(0., 0., -.2), at_ground=True)
        assert manager.payload_landing_failure_count == 1
        assert manager.payload_transport_active
        for commands in published.values():
            assert len(commands) == 2
            assert commands[-1]["collective_thrust"] > 0
            assert np.all(np.isfinite(commands[-1]["desired_force"]))
    finally:
        manager.logger.close()


def test_unexpected_fleet_numerical_error_keeps_timer_and_supported_descent(tmp_path, monkeypatch):
    _, manager, _, published, tick = flight_case(tmp_path, monkeypatch)
    try:
        def fail(*_args, **_kwargs):
            raise FloatingPointError("allocation overflow")
        monkeypatch.setattr(manager.slung_controller, "compute", fail)
        tick(18.02)
        assert all(len(commands) == 2 for commands in published.values())
        assert all(commands[-1]["collective_thrust"] > 0 for commands in published.values())
        assert all(v.trajectory.phase == "emergency_landing" for v in manager.vehicles)
        assert "allocation overflow" in manager.fleet_numerical_fault_reason
        tick(18.04)
        assert all(len(commands) == 3 for commands in published.values())
    finally:
        manager.logger.close()


@pytest.mark.parametrize("guard", ["gate_closed", "stale_state"])
def test_numerical_recovery_respects_preflight_and_state_health(tmp_path, monkeypatch, guard):
    _, manager, _, published, _ = flight_case(tmp_path, monkeypatch)
    try:
        if guard == "gate_closed":
            manager.fleet_gate_open = False
        else:
            manager.vehicles[0].latest_state["received_time"] = 0.

        def fail(*_args, **_kwargs):
            raise ValueError("fleet preparation failed")
        monkeypatch.setattr(type(manager), "_run_fleet_tick", fail)
        manager._timer_callback(None)
        assert all(len(commands) == 2 and commands[-1] is None for commands in published.values())
    finally:
        manager.logger.close()


@pytest.mark.parametrize("evaluation", ["error", "nan"])
def test_failed_profile_evaluation_cannot_restart_the_retry_budget(tmp_path, monkeypatch, evaluation):
    module, manager, _, published, tick = flight_case(tmp_path, monkeypatch)
    try:
        class BrokenEvaluation:
            def __init__(self, *_args, **_kwargs):
                self.duration = 5.

            def evaluate(self, _now):
                if evaluation == "error":
                    raise ValueError("landing evaluation failed")
                return np.full(3, np.nan), np.zeros(3), np.zeros(3)

        monkeypatch.setattr(module, "SlungLandingProfile", BrokenEvaluation)
        for now in (18.02, 18.04, 18.06):
            tick(now)
        assert all(len(commands) == 4 for commands in published.values())
        assert manager.payload_validation_landing_stage == "planning_fault_descent"
        assert not manager.payload_transport_active
        assert manager.payload_landing_failure_count == 3
        assert all(v.trajectory.phase == "emergency_landing" for v in manager.vehicles)
    finally:
        manager.logger.close()


def test_planning_failure_and_recovery_are_visible_in_actual_flight_csv(tmp_path, monkeypatch):
    _, manager, _, _, tick = flight_case(tmp_path, monkeypatch)
    try:
        manager.payload_ground_center_z = .025
        tick(18.02, velocity=(0., 0., -.2), at_ground=True)
        tick(18.04)
        with open(manager.logger.path, newline="") as stream:
            rows = list(csv.DictReader(stream))
        failed, recovered = rows[-6:-3], rows[-3:]
        assert len(failed) == len(recovered) == 3
        for row in failed:
            assert row["payload_landing_planning_failure_count"] == "1"
            assert row["payload_landing_planning_retry_count"] == "1"
            assert "ValueError" in row["payload_landing_planning_failure_reason"]
            assert row["payload_landing_planning_fallback"] == "1"
            assert float(row["command_thrust_newton"]) > 0
        for row in recovered:
            assert row["payload_landing_planning_failure_count"] == "1"
            assert row["payload_landing_planning_retry_count"] == "0"
            assert row["payload_landing_planning_fallback"] == "0"
    finally:
        manager.logger.close()


def test_vehicle_numerical_fault_does_not_duplicate_completed_publish_or_hold_forever(tmp_path, monkeypatch):
    _, manager, _, published, tick = flight_case(tmp_path, monkeypatch)
    try:
        failing_vehicle = manager.vehicles[1]

        def fail(*_args, **_kwargs):
            raise FloatingPointError("vehicle force overflow")

        monkeypatch.setattr(failing_vehicle.controller, "compute", fail)
        tick(18.02)
        assert all(len(commands) == 2 for commands in published.values())
        assert all(commands[-1]["collective_thrust"] > 0 for commands in published.values())
        assert manager.payload_validation_landing_stage == "planning_fault_descent"
        tick(18.04)
        assert all(len(commands) == 3 for commands in published.values())
        tick(18.35)
        assert manager.global_abort_reason
        assert published[id(failing_vehicle)][-1] is None
        tick(18.37)
        assert all(commands[-1] is None for commands in published.values())
    finally:
        manager.logger.close()


def test_numerical_log_error_after_publication_does_not_republish_that_vehicle(tmp_path, monkeypatch):
    _, manager, _, published, tick = flight_case(tmp_path, monkeypatch)
    try:
        def fail(*_args, **_kwargs):
            raise ValueError("CSV conversion failed after actuation")

        monkeypatch.setattr(manager.vehicles[1], "_write_log", fail)
        tick(18.02)
        assert all(len(commands) == 2 for commands in published.values())
        assert all(commands[-1]["collective_thrust"] > 0 for commands in published.values())
        assert manager.payload_validation_landing_stage == "planning_fault_descent"
    finally:
        manager.logger.close()


@pytest.mark.parametrize("fault", ["flush", "write_and_flush"])
def test_failed_logger_cannot_stop_repeated_fleet_publication(tmp_path, monkeypatch, fault):
    _, manager, _, published, tick = flight_case(tmp_path, monkeypatch)
    try:
        def fail(*_args, **_kwargs):
            raise ValueError("I/O operation on closed file")

        monkeypatch.setattr(manager.logger, "flush", fail)
        if fault == "write_and_flush":
            monkeypatch.setattr(manager.logger, "write", fail)
        tick(18.02)
        assert all(len(commands) == 2 for commands in published.values())
        tick(18.04)
        assert all(len(commands) == 3 for commands in published.values())
        assert all(commands[-1]["collective_thrust"] > 0 for commands in published.values())
    finally:
        manager.logger.close()
