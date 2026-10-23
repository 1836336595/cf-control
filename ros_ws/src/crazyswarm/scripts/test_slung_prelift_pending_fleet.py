"""Fleet fault and handoff clocks across bounded observer sampling waits."""

import numpy as np
import pytest

from test_slung_prelift_pending_interval import setup_case
from test_slung_observer_state_integration import _message


def fleet_case(tmp_path, monkeypatch):
    module, manager, anchors, clock = setup_case(tmp_path, monkeypatch)
    manager.fleet_gate_open = True
    manager.last_tick = clock[0]
    # Exercise the real fleet's payload branch. Vehicle telemetry, physical
    # separation checks and publication are independent of this clock case.
    manager._all_states_ready = lambda _now: True
    manager._transport_phase_active = lambda _now: True
    manager._check_payload_physical_safety = lambda _now: False
    manager._check_payload_flight_validation = lambda _now: False
    manager._release_validation_payload = lambda _now: None
    for vehicle in manager.vehicles:
        vehicle._timer_callback = lambda *_args, **_kwargs: None
        vehicle.last_control_command = {
            "desired_force": np.array([0., 0., .65]),
            "body_rate_command": np.zeros(3),
            "collective_thrust": .65,
        }
    return module, manager, anchors, clock


def update_tick(manager, clock, now, *, pending=True, position=(0., 0., .05), valid=True):
    clock[0] = float(now)
    for vehicle in manager.vehicles:
        vehicle.latest_state.update(received_time=clock[0], mocap_sample_time_s=100. + clock[0])
    if pending:
        manager._payload_state_callback(_message(100. + clock[0] - .001,
                                                 position=position, valid=valid))
    manager._payload_state_callback(_message(100. + clock[0], position=position, valid=valid))
    manager._timer_callback(None)


def test_repeated_recent_pending_frames_do_not_accumulate_a_fleet_payload_fault(tmp_path, monkeypatch):
    _, manager, _, clock = fleet_case(tmp_path, monkeypatch)
    try:
        for index in range(1, 72):
            update_tick(manager, clock, .09 + index / 70.)
        assert manager.latest_payload_state["observer_status"] == "pending_interval"
        assert not manager._payload_ready(clock[0])
        assert manager._payload_prelift_motion_state(manager.latest_payload_state, clock[0]) is not None
        assert manager.payload_fault_since is None
        assert all(vehicle.trajectory.phase != "emergency_landing" for vehicle in manager.vehicles)
        assert not manager.payload_transport_active
        assert manager.payload_prelift_reference.speed == pytest.approx(.05)
    finally:
        manager.logger.close()


def test_pending_keeps_stable_handoff_clocks_but_only_a_new_accepted_frame_can_activate(tmp_path, monkeypatch):
    _, manager, anchors, clock = fleet_case(tmp_path, monkeypatch)
    try:
        raised = anchors + [0., 0., .22]
        for vehicle, position in zip(manager.vehicles, raised):
            vehicle.latest_state["position"] = position.copy()
        manager.payload_prelift_reference.stage = "holding"
        manager.payload_prelift_reference.positions = raised.copy()
        manager.payload_prelift_reference.speed = 0.
        manager.payload_prelift_confirm_s = manager.payload_takeup_confirm_s = .04
        manager.payload_activation_hold_s = .04
        # Warm the actual observer at the elevated stationary pose.
        manager.payload_state_observer.reset()
        for index in range(10):
            clock[0] = .10 + index * .01
            manager._payload_state_callback(_message(100. + clock[0], position=(0., 0., .27)))
        manager.payload_prelift_ready_since = .15
        manager.payload_takeup_ready_since = .16
        manager.payload_ready_since = .17
        for index in range(1, 9):
            update_tick(manager, clock, .19 + index / 70., position=(0., 0., .27))
            assert not manager.payload_transport_active
            assert manager.payload_prelift_ready_since == .15
            assert manager.payload_takeup_ready_since == .16
            assert manager.payload_ready_since == .17
        update_tick(manager, clock, clock[0] + .01, pending=False, position=(0., 0., .27))
        assert manager.payload_transport_active
        assert manager.payload_transport_start_time == clock[0]
    finally:
        manager.logger.close()


@pytest.mark.parametrize("failure", ["invalid_source", "pose_innovation"])
def test_real_rejection_still_enters_fleet_controlled_landing_after_fault_timeout(tmp_path, monkeypatch, failure):
    _, manager, _, clock = fleet_case(tmp_path, monkeypatch)
    try:
        for index in range(1, 45):
            update_tick(manager, clock, .09 + index / 70., pending=False,
                        position=((1., 0., .05) if failure == "pose_innovation" else (0., 0., .05)),
                        valid=failure != "invalid_source")
        assert not manager.payload_transport_active
        assert all(vehicle.trajectory.phase == "emergency_landing" for vehicle in manager.vehicles)
        assert all("预抬期间负载状态持续失效" in vehicle.trajectory.emergency_landing_reason
                   for vehicle in manager.vehicles)
        assert not manager.global_abort_reason
    finally:
        manager.logger.close()
