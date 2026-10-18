"""Post-release landing must overcome force bias without bypassing touchdown gates."""

import numpy as np
import pytest

from test_ctbr_controller_v2 import _config, _controller_module, _slung_manager, _state, _target
from test_slung_flight_validation import manager_at_handoff


def landing_controller(**overrides):
    module = _controller_module()
    parameters = dict(
        independent_position_gain=np.array([3., 3., 4.]),
        independent_velocity_gain=np.array([3.12, 3.12, 3.6]),
        independent_integral_gain=np.full(3, .8),
        independent_integral_limit=np.full(3, .2),
        independent_integral_gate=.2,
        independent_max_feedback_acceleration=np.array([6., 6., 8.]),
    )
    parameters.update(overrides)
    return module.GeometricCtbrController(_config(module, **parameters))


def landing_target():
    target = _target()
    target.update(
        independent_mode=True, transport_mode=False, flight_phase="landing",
        min_collective_thrust=.3, landing_vertical_integral_active=True,
        landing_vertical_integral_limit=2., landing_vertical_integral_rate_limit=.5,
        landing_vertical_feedback_limit_mps2=8.,
    )
    return target


@pytest.mark.parametrize("force_scale", [.90, .92, 1.08, 1.10])
def test_biased_vertical_plant_reaches_measured_landing_band(tmp_path, force_scale):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    try:
        vehicle = manager.vehicles[0]
        trajectory = vehicle.trajectory
        trajectory.reset(np.array([0., 0., .05]), 0., 0.)
        trajectory.begin_controlled_landing(0., np.array([0., 0., .8]), 0., "test")
        controller = landing_controller()
        state = _state()
        state["position"][2] = .8
        reached = False
        for step in range(70 * 45):
            now = step / 70.
            target = trajectory.evaluate(state, now)
            if target.get("zero_output", False):
                # The real trajectory may stop only after measured arrival.
                assert abs(state["position"][2] - .05) <= .08
                assert abs(state["velocity"][2]) <= .15
                reached = True
                break
            target.update({key: value for key, value in landing_target().items()
                           if key.startswith("landing_vertical_")})
            target["independent_mode"] = True
            command = controller.compute(state, target, 1 / 70.)
            assert command["collective_thrust"] >= .3
            acceleration = force_scale * command["collective_thrust"] / controller.config.mass - controller.config.gravity
            state["velocity"][2] += acceleration / 70.
            state["position"][2] += state["velocity"][2] / 70.
            state["acceleration"][2] = acceleration
            # A unilateral floor supplies normal force rather than permitting
            # the low-thrust plant to descend through the resting marker z.
            if state["position"][2] <= .05:
                state["position"][2] = .05
                state["velocity"][2] = max(0., state["velocity"][2])
                state["acceleration"][2] = max(0., acceleration)
        assert reached, "landing remained airborne at z=%.4f m with integral %.4f" % (
            state["position"][2], controller.position_integral[2])
    finally:
        manager.logger.close()


def test_horizontal_error_does_not_disable_landing_vertical_bias_correction():
    controller = landing_controller()
    state = _state()
    state["position"] = np.array([.5, -.4, .15])
    controller.compute(state, landing_target(), .02)
    assert controller.position_integral[2] > 0.
    np.testing.assert_array_equal(controller.position_integral[:2], [0., 0.])


@pytest.mark.parametrize("invalid_field", ["derivatives_valid", "filter_derivatives_valid"])
def test_unavailable_kinematics_freezes_landing_integral(invalid_field):
    controller = landing_controller()
    controller.position_integral[2] = .1
    state = _state()
    state["position"][2] = .1
    state[invalid_field] = False
    controller.compute(state, landing_target(), .02)
    assert controller.position_integral[2] == .1


def test_landing_integral_growth_is_bounded_per_second():
    controller = landing_controller()
    state = _state()
    state["position"][2] = .1
    state["velocity"][2] = .8
    target = landing_target()
    target["min_collective_thrust"] = 0.0  # Isolate rate limiting from the actuator floor.
    controller.compute(state, target, .02)
    assert 0. < controller.position_integral[2] <= .010000001


@pytest.mark.parametrize("sign", [-1., 1.])
def test_landing_bias_has_its_own_positive_and_negative_cap(sign):
    controller = landing_controller()
    controller.position_integral[2] = sign * 1.99
    state = _state()
    state["position"][2] = sign * .1
    state["velocity"][2] = sign * .5
    target = landing_target()
    target["min_collective_thrust"] = 0.0  # Caps have their own test, floor anti-windup another.
    controller.compute(state, target, .05)
    assert abs(controller.position_integral[2]) == 2.


@pytest.mark.parametrize("saturation", ["positive_feedback", "negative_feedback", "upper_thrust", "lower_thrust"])
def test_saturation_does_not_accumulate_more_landing_bias(saturation):
    controller = landing_controller(max_command_thrust=.2) if saturation == "upper_thrust" else landing_controller()
    target = landing_target()
    state = _state()
    state["position"][2] = -.1 if saturation in ("positive_feedback", "upper_thrust") else .1
    if saturation.endswith("feedback"):
        target["landing_vertical_feedback_limit_mps2"] = .2
    if saturation == "lower_thrust":
        target["acceleration"][2] = -12.
    controller.compute(state, target, .02)
    assert controller.position_integral[2] == 0.


def test_landing_integral_respects_nonzero_collective_floor():
    controller = landing_controller()
    state = _state()
    state["position"][2] = 1.0
    target = landing_target()
    command = controller.compute(state, target, .02)
    assert command["collective_thrust"] == target["min_collective_thrust"]
    assert controller.position_integral[2] == 0.


@pytest.mark.parametrize("phase", ["takeoff", "height_correction", "transport"])
def test_other_phases_keep_ordinary_integral_limits_without_landing_flag(phase):
    controller = landing_controller()
    controller.position_integral[2] = .2
    state = _state()
    state["position"][2] = .1
    target = _target()
    target.update(flight_phase=phase, independent_mode=phase != "transport", transport_mode=phase == "transport")
    controller.compute(state, target, .02)
    assert controller.position_integral[2] == .2


def test_release_discards_prelift_support_bias_once_and_retains_positive_trim(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        originals = (-2.2, .1, .7)
        for vehicle, integral in zip(manager.vehicles, originals):
            vehicle.controller.position_integral[2] = integral
        manager.latest_payload_state.update(position=np.array([0., 0., .03]), velocity=np.zeros(3), received_time=30.)
        assert manager._check_payload_flight_validation(30.)
        # Starting payload lowering must retain the support still needed aloft.
        assert [v.controller.position_integral[2] for v in manager.vehicles] == list(originals)
        for now in np.arange(30., 30.51, .01):
            manager.latest_payload_state["received_time"] = float(now)
            assert not manager._release_validation_payload(float(now))
        assert manager.payload_unload_start_time is not None
        now = manager.payload_unload_start_time + manager.payload_landing_unload_s + .01
        manager.latest_payload_state["received_time"] = now
        assert manager._release_validation_payload(now)
        np.testing.assert_allclose([v.controller.position_integral[2] for v in manager.vehicles], [0., .1, .2])
        manager.vehicles[0].controller.position_integral[2] = .65
        assert not manager._release_validation_payload(now + 1.)
        assert manager.vehicles[0].controller.position_integral[2] == .65
    finally:
        manager.logger.close()


def test_manager_passes_configured_landing_rate_to_actual_controller(tmp_path):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path, {"slung_payload": {
        "landing_vertical_integral_limit": 1.25,
        "landing_vertical_integral_rate_limit": .05,
        "landing_vertical_feedback_limit_mps2": 7.5,
    }})
    try:
        manager.fleet_gate_open = True
        manager.payload_validation_landing_stage = "vehicle_descent"
        for vehicle, position in zip(manager.vehicles, ([.6, 0., .8], [0., .6, .8], [-.6, 0., .8])):
            vehicle.latest_state["position"] = np.array(position)
            vehicle.trajectory.start_position[2] = .05
        manager._begin_validation_vehicle_descent(30.)
        overrides = {}
        for vehicle in manager.vehicles:
            vehicle.latest_state["received_time"] = 30.
            vehicle._timer_callback = lambda _event, now=None, transport_override=None, vehicle=vehicle: overrides.update({id(vehicle): transport_override})
        manager.latest_payload_state["received_time"] = 30.
        manager._run_fleet_tick(None, 30.)
        vehicle = manager.vehicles[0]
        target = landing_target()
        # Only the real manager supplies the opt-in and configured bounds.
        for key in tuple(target):
            if key.startswith("landing_vertical_"):
                del target[key]
        target.update(overrides[id(vehicle)])
        state = _state()
        state["position"] = np.array([.5, -.4, .1])
        state["velocity"][2] = .5
        vehicle.controller.position_integral[:] = 0.
        for _ in range(5):
            vehicle.controller.compute(state, target, .02)
        assert .0049 <= vehicle.controller.position_integral[2] <= .0051
    finally:
        manager.logger.close()


def test_landing_timeout_alone_never_stops_airborne_vehicle(tmp_path):
    _, manager = manager_at_handoff(tmp_path)
    try:
        vehicle = manager.vehicles[0]
        state = dict(vehicle.latest_state, position=np.array([0., 0., .20]), velocity=np.zeros(3))
        vehicle.trajectory.begin_controlled_landing(30., state["position"], 0., "test")
        target = vehicle.trajectory.evaluate(state, 3600.)
        assert target["flight_phase"] == "landing"
        assert not target.get("zero_output", False)
    finally:
        manager.logger.close()
