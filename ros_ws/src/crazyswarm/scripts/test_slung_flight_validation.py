"""Short real-flight validation must land once from the handoff deadline."""

import csv

import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _slung_manager


def manager_at_handoff(tmp_path, enabled=True):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path, {"slung_payload": {
        "flight_validation_enabled": enabled, "flight_validation_transport_s": 10.0,
    }})
    manager.payload_transport_active = True
    manager.payload_transport_start_time = 20.0
    manager.payload_transport_start_position = manager.latest_payload_state["position"].copy()
    # The fixture begins at TAKEUP anchors; real trajectories originate on the ground.
    for vehicle in manager.vehicles:
        vehicle.trajectory.start_position[2] = .05
    manager.last_transport_overrides = {id(v): {"transport_mode": True} for v in manager.vehicles}
    return module, manager


def test_deadline_starts_at_actual_handoff_and_is_one_shot(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        assert not manager._check_payload_flight_validation(29.999)
        positions = [v.latest_state["position"].copy() for v in manager.vehicles]
        assert manager._check_payload_flight_validation(30.0)
        assert manager.payload_validation_landing_started_time == 30.0
        assert manager.payload_validation_handoff_time == 20.0
        # Cooperative support remains active until the load is on the ground.
        assert manager.payload_transport_active
        assert not manager.global_abort_reason
        for vehicle, position in zip(manager.vehicles, positions):
            assert vehicle.trajectory.phase == "landing"
            np.testing.assert_allclose(vehicle.trajectory.landing_start_position, position)
            assert "flight_validation_complete" in vehicle.trajectory.emergency_landing_reason
        assert not manager._check_payload_flight_validation(40.0)
        assert all(v.trajectory.phase_start_time == 30.0 for v in manager.vehicles)
    finally:
        manager.logger.close()


@pytest.mark.parametrize("enabled,active,start", [(False, True, 20), (True, False, 20), (True, True, None)])
def test_disabled_or_not_handed_off_never_starts_countdown(tmp_path, enabled, active, start):
    _, manager = manager_at_handoff(tmp_path, enabled)
    try:
        manager.payload_transport_active = active
        manager.payload_transport_start_time = start
        assert not manager._check_payload_flight_validation(100.0)
        assert manager.payload_validation_landing_started_time is None
        assert all(v.trajectory.phase != "emergency_landing" for v in manager.vehicles)
    finally:
        manager.logger.close()


def test_existing_fault_is_not_relabelled_as_validation_completion(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        manager._begin_payload_emergency_landing(28, "height_drop")
        assert not manager._check_payload_flight_validation(30)
        assert manager.payload_validation_landing_started_time is None
        assert all(v.trajectory.emergency_landing_reason.endswith("height_drop") for v in manager.vehicles)
    finally:
        manager.logger.close()


def test_handoff_reset_does_not_clear_the_validation_stop_latch(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        assert manager._check_payload_flight_validation(30)
        manager._reset_payload_handoff()
        assert manager.payload_validation_landing_started_time == 30
        assert manager.payload_validation_handoff_time == 20
        assert not manager._transport_phase_active(31)
        for v in manager.vehicles:
            target = v.trajectory.evaluate(v.latest_state, 31)
            assert not target.get("zero_output", False)
            assert target["position"][2] < v.trajectory.landing_start_position[2]
    finally:
        manager.logger.close()


@pytest.mark.parametrize("seconds", [0, -1, float("nan"), float("inf")])
def test_invalid_validation_duration_is_rejected_at_startup(tmp_path, seconds):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="flight_validation_transport_s"):
        _slung_manager(module, tmp_path, {"slung_payload": {
            "flight_validation_enabled": True, "flight_validation_transport_s": seconds,
        }})


def test_validation_boolean_cannot_silently_accept_a_yaml_string(tmp_path):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="flight_validation_enabled"):
        _slung_manager(module, tmp_path, {"slung_payload": {"flight_validation_enabled": "false"}})


@pytest.mark.parametrize("name", ["landing_vertical_integral_limit",
    "landing_vertical_integral_rate_limit", "landing_vertical_feedback_limit_mps2"])
@pytest.mark.parametrize("value", [0., -1., float("nan"), float("inf")])
def test_invalid_landing_vertical_parameters_are_rejected(tmp_path, name, value):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="landing_vertical"):
        _slung_manager(module, tmp_path, {"slung_payload": {name: value}})


def test_current_physical_fault_has_priority_over_expired_validation_timer(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        now = 30.0
        manager.latest_payload_state["received_time"] = now
        for vehicle, position in zip(manager.vehicles, ([.2,0,1],[0,0,1],[0,1,1])):
            vehicle.latest_state.update(position=np.array(position), received_time=now,
                                        filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
                                        filter_derivatives_valid=True)
        assert manager._check_payload_physical_safety(now)
        assert not manager._check_payload_flight_validation(now)
        assert manager.payload_validation_landing_started_time is None
        assert manager.payload_safety_reason == "distance"
        assert all("distance" in v.trajectory.emergency_landing_reason for v in manager.vehicles)
    finally:
        manager.logger.close()


def test_valid_mocap_without_trajectory_cannot_crash_payload_fault_path(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        manager.vehicles[1].trajectory = None
        manager.latest_payload_state["received_time"] = 30.
        manager._begin_payload_emergency_landing(30., "distance: preflight regression")
        assert id(manager.vehicles[1]) not in manager.payload_emergency_support
        assert manager.vehicles[0].trajectory.phase == "emergency_landing"
        assert manager.vehicles[2].trajectory.phase == "emergency_landing"
    finally:
        manager.logger.close()


def test_fleet_tick_with_valid_states_and_missing_trajectory_does_not_crash(tmp_path):
    module, manager = manager_at_handoff(tmp_path)
    original_time = module.time.monotonic
    published, zeros = [], []
    try:
        # First vehicle evaluation creates a pre-takeoff trajectory and stops
        # the transport-phase scan; remaining vehicles are still uninitialized
        # when the manager builds their output overrides (real startup path).
        for vehicle in manager.vehicles:
            vehicle.trajectory = None
        manager.fleet_gate_open = True
        manager.last_tick = 0.
        now = 1.
        module.time.monotonic = lambda: now
        for vehicle in manager.vehicles:
            vehicle.latest_state["received_time"] = now
            vehicle.latest_state["filtered_velocity"] = np.zeros(3)
            vehicle.latest_state["filtered_acceleration"] = np.zeros(3)
            vehicle.latest_state["filter_derivatives_valid"] = True
            vehicle._publish_path = lambda _state: None
            vehicle._publish_command = lambda command: published.append(command)
            vehicle._publish_zero = lambda: zeros.append(True)
        manager._timer_callback(None)
        assert len(zeros) == 3
        assert not published
    finally:
        module.time.monotonic = original_time
        manager.logger.close()


def test_timer_rebuilds_current_tick_overrides_for_cooperative_descent(tmp_path):
    module, manager = manager_at_handoff(tmp_path)
    original_time = module.time.monotonic
    published = []
    try:
        now = 30.0
        module.time.monotonic = lambda: now
        manager.fleet_gate_open = True
        manager.last_tick = now - 1/70
        manager.latest_payload_state["received_time"] = now
        manager._transport_overrides = lambda _now: {
            id(v): {"flight_phase": ("landing" if v.trajectory.phase == "landing" else "tension_ramp"),
                    "transport_mode": True,
                    "desired_force_override": np.array([0, 0, .8])} for v in manager.vehicles}
        manager._check_payload_physical_safety = lambda _now: False
        for vehicle, position in zip(manager.vehicles, ([.5,0,1.2],[0,.5,1.2],[-.5,0,1.2])):
            vehicle.preflight_voltage_ready = True
            vehicle.latest_state.update(position=np.array(position), received_time=now,
                                        filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
                                        filter_derivatives_valid=True)
            vehicle.latest_ekf_state = dict(position=np.array(position), filtered_velocity=np.zeros(3),
                                            filtered_acceleration=np.zeros(3), filter_derivatives_valid=True,
                                            received_time=now)
            vehicle._publish_path = lambda _state: None
            vehicle._publish_command = lambda command: published.append(command)
            vehicle._publish_zero = lambda: None
        manager._timer_callback(None)
        assert all(v.flight_phase == "landing" for v in manager.vehicles)
        assert len(published) == 3
        assert all(command["transport_mode"] for command in published)
        assert all(command["collective_thrust"] > 0 for command in published)
        manager.logger.flush()
        with open(manager.logger.path) as stream:
            rows = list(csv.DictReader(stream))
        assert all(r["flight_validation_landing_started"] == "1" for r in rows[-3:])
        assert all(float(r["flight_validation_elapsed_s"]) == 10 for r in rows[-3:])
    finally:
        module.time.monotonic = original_time
        manager.logger.close()


def test_planned_descent_keeps_cooperative_support_until_payload_settles(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        manager.latest_payload_state.update(position=np.array([0,0,.4]), velocity=np.zeros(3), received_time=30.)
        assert manager._check_payload_flight_validation(30)
        assert not manager._release_validation_payload(30.1)
        assert manager.payload_transport_active
        manager.latest_payload_state.update(position=np.array([0,0,.03]), velocity=np.array([0,0,-.2]), received_time=30.2)
        assert not manager._release_validation_payload(30.2)
        for now in np.arange(30.3, 30.71, .01):
            manager.latest_payload_state.update(velocity=np.zeros(3), received_time=float(now))
            assert not manager._release_validation_payload(float(now))
        assert manager.payload_unload_start_time is not None
        now = manager.payload_unload_start_time + manager.payload_landing_unload_s + .01
        manager.latest_payload_state["received_time"] = now
        assert manager._release_validation_payload(now)
        assert not manager.payload_transport_active
        assert manager.payload_validation_landing_stage == "vehicle_descent"
        assert all(v.trajectory.phase == "landing" for v in manager.vehicles)
        assert all(v.trajectory.phase_start_time == now for v in manager.vehicles)
    finally:
        manager.logger.close()


def test_planned_vehicle_descent_keeps_current_xy_yaw_and_landing_gates(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        vehicle = manager.vehicles[0]
        position = np.array([.7, -.4, 1.1])
        yaw = .8
        trajectory = vehicle.trajectory
        trajectory.begin_controlled_landing(30, position, yaw, "planned")
        state = dict(vehicle.latest_state, position=position.copy(), velocity=np.zeros(3))
        target = trajectory.evaluate(state, 30.5)
        np.testing.assert_allclose(target["position"][:2], position[:2])
        assert target["yaw"] == yaw
        assert not target.get("zero_output", False)
        end = 30 + trajectory.active_landing_duration_s
        # Time expiry alone must never cut thrust while measured altitude is high.
        assert trajectory.evaluate(state, end + 1)["flight_phase"] == "landing"
        assert not trajectory.evaluate(state, end + 1).get("zero_output", False)
        state.update(position=np.array([.7, -.4, .05]), velocity=np.zeros(3))
        target = trajectory.evaluate(state, end + 2)
        assert target["flight_phase"] == "landing_settle"
        assert target["zero_output"]
        assert trajectory.evaluate(state, end + 3)["flight_phase"] == "landed"
    finally:
        manager.logger.close()


def test_one_landed_vehicle_does_not_disable_remaining_descent_spacing_guard(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        manager.payload_transport_active = False
        manager.payload_takeup_start_time = None
        manager.payload_validation_landing_stage = "vehicle_descent"
        for index, (vehicle, position) in enumerate(zip(
                manager.vehicles, ([0,0,.05],[.2,0,.1],[0,1,.4]))):
            vehicle.latest_state.update(position=np.array(position), received_time=30,
                                        filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
                                        filter_derivatives_valid=True)
            vehicle.trajectory.phase = "landed" if index == 0 else "landing"
        assert manager._check_payload_physical_safety(30)
        assert manager.payload_safety_reason == "distance"
        assert manager.vehicles[0].trajectory.phase == "landed"
    finally:
        manager.logger.close()
