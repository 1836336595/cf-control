"""Waiting for a short pose interval must not repeatedly restart prelift."""

from copy import deepcopy
import math
from types import SimpleNamespace

import numpy as np
import pytest

from test_ctbr_controller_v2 import (
    _controller_module, _slung_manager, _prelift_integral_controller,
    _prelift_integral_target, _state,
)
from test_slung_observer_state_integration import _message


def setup_case(tmp_path, monkeypatch):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path, {
        "slung_payload": {"link_calibration_enabled": False}})
    clock = [0.]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    module.rospy.Time.now = lambda: SimpleNamespace(to_sec=lambda: 100. + clock[0])
    manager._publish_payload_path = lambda _state: None
    for index in range(10):
        clock[0] = index * .01
        manager._payload_state_callback(_message(100. + clock[0], position=(0., 0., .05)))
    manager.payload_takeup_start_time = 0.
    manager.payload_takeup_start_positions = anchors.copy()
    manager.payload_takeup_target_positions = anchors.copy()
    manager.payload_prelift_reference = module.SlungPreliftTrajectory(
        **manager.payload_prelift_parameters)
    manager.payload_prelift_reference.start(anchors, anchors, .09)
    manager.payload_prelift_reference.stage = "rising"
    manager.payload_prelift_reference.speed = .03
    manager.payload_prelift_reference.positions = anchors + [0., 0., .1]
    return module, manager, anchors, clock


def pending(manager, clock, dt=.004, position=(0., 0., .05)):
    clock[0] += dt
    manager._payload_state_callback(_message(100. + clock[0], position=position))
    assert manager.latest_payload_state["observer_status"] == "pending_interval"


def test_pending_keeps_prelift_speed_but_never_turns_into_a_new_control_measurement(tmp_path, monkeypatch):
    _, manager, _, clock = setup_case(tmp_path, monkeypatch)
    try:
        accepted = deepcopy(manager.latest_payload_state)
        pending(manager, clock)
        reference = manager.payload_prelift_reference
        before = reference.positions.copy()
        outputs = manager._payload_takeup_overrides(clock[0])
        assert reference.stage == "rising"
        assert reference.speed >= .03
        assert np.all(reference.positions[:, 2] > before[:, 2])
        assert not manager._payload_ready(clock[0])
        assert not manager._payload_takeup_ready(clock[0])
        for item in outputs.values():
            assert item["payload_prelift_feedback_reused"]
            assert item["payload_prelift_feedback_age_s"] == pytest.approx(.004)
            assert item["prelift_vertical_integral_active"]
            assert not item["prelift_vertical_integral_learning"]
        # Reuse must not refresh the accepted age or mutate latest source metadata.
        assert manager.latest_payload_state["sample_time"] == accepted["sample_time"]
        assert not manager.latest_payload_state["observer_quality_valid"]
    finally:
        manager.logger.close()


@pytest.mark.parametrize("failure", ["expired_receipt", "expired_source", "bad_source",
                                    "position_innovation", "derivative_limit", "gap_reset",
                                    "initializing", "rejected_then_pending"])
def test_only_recent_pending_can_reuse_prelift_geometry(tmp_path, monkeypatch, failure):
    module, manager, _, clock = setup_case(tmp_path, monkeypatch)
    try:
        if failure == "rejected_then_pending":
            clock[0] = .10
            manager._payload_state_callback(_message(100.1, position=(1., 0., .05)))
            clock[0] = .094
            manager._payload_state_callback(_message(100.094, position=(0., 0., .05)))
        else:
            pending(manager, clock)
            state = manager.latest_payload_state
            if failure == "expired_receipt":
                clock[0] = .141
            elif failure == "expired_source":
                module.rospy.Time.now = lambda: SimpleNamespace(to_sec=lambda: 100.141)
            elif failure == "bad_source":
                state["source_valid"] = False
            else:
                state["observer_status"] = failure
        previous = manager.payload_prelift_reference.positions.copy()
        outputs = manager._payload_takeup_overrides(clock[0])
        np.testing.assert_array_equal(manager.payload_prelift_reference.positions, previous)
        assert manager.payload_prelift_reference.speed == 0.
        assert all(not item.get("payload_prelift_feedback_reused", False) for item in outputs.values())
        assert all(not item["prelift_vertical_integral_active"] for item in outputs.values())
        assert math.isnan(manager.payload_ground_clearance_m)
    finally:
        manager.logger.close()


def test_pending_freezes_large_vertical_integral_without_clamping_away_its_lift():
    module = _controller_module()
    controller = _prelift_integral_controller(module)
    controller.position_integral[2] = -2.
    target = _prelift_integral_target()
    target.update(position=np.array([0., 0., .1]), velocity=np.zeros(3),
                  prelift_vertical_integral_learning=False)
    state = _state()
    state.update(position=np.zeros(3), velocity=np.zeros(3))
    command = controller.compute(state, target, .01)
    assert controller.position_integral[2] == -2.
    assert command["prelift_vertical_integral_active"]
    assert command["prelift_vertical_integral_force_n"] == pytest.approx(
        controller.config.mass * .8 * 2.)


def test_pending_preserves_existing_settle_clock_but_cannot_confirm_handoff(tmp_path, monkeypatch):
    _, manager, anchors, clock = setup_case(tmp_path, monkeypatch)
    try:
        clock[0] = .10
        # A ground-supported test load moved up coherently with all aircraft.
        raised = anchors + [0., 0., .22]
        for vehicle, position in zip(manager.vehicles, raised):
            vehicle.latest_state["position"] = position
        for index in range(10):
            clock[0] = .10 + index * .01
            # Seed a stationary elevated load directly, then let the real callback warm it.
            if index == 0:
                manager.payload_state_observer.reset()
            manager._payload_state_callback(_message(100. + clock[0], position=(0., 0., .27)))
        manager.payload_prelift_reference.stage = "holding"
        manager.payload_prelift_reference.positions = raised.copy()
        manager.payload_prelift_ready_since = .15
        manager.payload_takeup_ready_since = .16
        pending(manager, clock, position=(0., 0., .27))
        assert not manager._payload_takeup_ready(clock[0])
        assert manager.payload_prelift_ready_since == .15
        assert manager.payload_takeup_ready_since == .16
        assert manager.payload_prelift_confirmation_waiting
    finally:
        manager.logger.close()


def test_new_prelift_diagnostics_are_appended_after_existing_csv_columns():
    from ctbr_csv_logger import FlightCsvLogger
    assert FlightCsvLogger.FIELDS[-3:] == ["prelift_vertical_integral_learning",
        "payload_prelift_feedback_reused", "payload_prelift_feedback_age_s"]


def test_real_rejection_clears_cached_geometry_before_a_later_pending_frame(tmp_path, monkeypatch):
    _, manager, _, clock = setup_case(tmp_path, monkeypatch)
    try:
        assert manager.payload_prelift_accepted_state is not None
        clock[0] = .10
        manager._payload_state_callback(_message(100.1, position=(1., 0., .05)))
        assert manager.latest_payload_state["observer_status"] == "position_innovation"
        assert manager.payload_prelift_accepted_state is None
        rejected = deepcopy(manager.latest_payload_state)
        rejected.update(observer_status="pending_interval", consecutive_rejected_count=0)
        assert manager._payload_prelift_motion_state(rejected, .101) is None
    finally:
        manager.logger.close()


def test_reused_geometry_and_frozen_integral_are_written_to_real_csv(tmp_path, monkeypatch):
    import csv
    _, manager, anchors, clock = setup_case(tmp_path, monkeypatch)
    try:
        pending(manager, clock)
        target = manager._payload_takeup_overrides(clock[0])[id(manager.vehicles[0])]
        vehicle = manager.vehicles[0]
        state = dict(vehicle.latest_state, filtered_velocity=np.zeros(3),
                     filtered_acceleration=np.zeros(3))
        vehicle.controller.position_integral[2] = -2.
        command = vehicle.controller.compute(state, target, .01)
        vehicle.set_payload_diagnostics(manager.latest_payload_state)
        vehicle._write_log(clock[0], state, target, command)
        manager.logger.flush()
        with open(manager.logger.path, newline="") as stream:
            row = next(csv.DictReader(stream))
        assert row["payload_prelift_feedback_reused"] == "1"
        assert row["prelift_vertical_integral_active"] == "1"
        assert row["prelift_vertical_integral_learning"] == "0"
        assert float(row["payload_prelift_feedback_age_s"]) == pytest.approx(.004)
        assert float(row["prelift_vertical_integral_force_n"]) > 0.
        assert vehicle.controller.position_integral[2] == -2.
    finally:
        manager.logger.close()


def test_tiny_source_wait_does_not_consume_prelift_timeout_through_repeated_speed_resets(tmp_path, monkeypatch):
    _, manager, _, clock = setup_case(tmp_path, monkeypatch)
    try:
        start = manager.payload_prelift_reference.positions[0, 2]
        for index in range(1, 701):
            clock[0] = .09 + index / 70.
            # A freshly accepted pose followed by a 1 ms pending sample at every tick.
            manager._payload_state_callback(_message(100. + clock[0] - .001, position=(0., 0., .05)))
            manager._payload_state_callback(_message(100. + clock[0], position=(0., 0., .05)))
            manager._payload_takeup_overrides(clock[0])
        rise = manager.payload_prelift_reference.positions[0, 2] - start
        assert rise > .45
        assert manager.payload_prelift_reference.speed == pytest.approx(.05)
        assert manager.payload_prelift_reference.timeout_s == 60.
    finally:
        manager.logger.close()
