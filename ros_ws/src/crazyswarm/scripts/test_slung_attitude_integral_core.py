"""The payload moment integral must reach tension allocation and force history."""

import copy

import numpy as np
import pytest

from slung_load_controller import (
    SlungLoadConfig, SlungLoadController, attachment_points_ros_z_up,
)


def _case(force_mode="reference_cascade", angle=.1, axis="roll"):
    config = SlungLoadConfig(
        payload_mass_kg=.023, gravity_mps2=9.80665,
        payload_size_m=np.array([.08, .06, .05]),
        attachment_points_m=attachment_points_ros_z_up([.08, .06, .05]),
        link_length_m=.85, position_gain=np.zeros(3), velocity_gain=np.zeros(3),
        integral_gain=np.zeros(3), integral_limit=np.ones(3), c1=.5,
        force_norm_epsilon=1e-9, tension_pinv_tolerance=1e-9,
        link_kq=0., link_komega=0., link_integral_gain=0.,
        link_integral_limit=np.ones(3), transport_force_mode=force_mode,
    )
    c, s = np.cos(angle), np.sin(angle)
    rotation = (np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
                if axis == "roll" else
                np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]))
    payload = dict(valid=True, position=np.zeros(3), velocity=np.zeros(3),
                   acceleration=np.zeros(3), rotation=rotation, body_rate=np.zeros(3))
    # Vertical, taut cables. Their projected wrench has roll/pitch authority,
    # but no yaw authority, even though the unconstrained 6D allocator has it.
    vehicles = [dict(position=rotation @ config.attachment_points_m[:, index]
                     + np.array([0., 0., .85]), velocity=np.zeros(3), mass=.043)
                for index in range(3)]
    target = dict(position=np.zeros(3), velocity=np.zeros(3), acceleration=np.zeros(3),
                  rotation=np.eye(3), body_rate=np.zeros(3), body_rate_dot=np.zeros(3),
                  payload_attitude_gain=np.array([.0001, .0002, .0003]),
                  payload_rate_gain=np.zeros(3),
                  payload_attitude_integral_gain_nm_per_rad_s=[.001, .001, .001],
                  payload_attitude_integral_limit_nm=.0015,
                  payload_attitude_integral_rate_limit_nmps=.01,
                  payload_attitude_integral_learning=True,
                  payload_attitude_integral_tick=1)
    return SlungLoadController(config), payload, vehicles, target


@pytest.mark.parametrize("force_mode", ["coupled", "reference_cascade"])
def test_default_and_explicit_disabled_are_exact_legacy_pd(force_mode):
    controller, payload, vehicles, target = _case(force_mode)
    baseline = controller.compute(payload, vehicles, target, .02)
    target["payload_attitude_integral_enabled"] = False
    disabled = SlungLoadController(controller.config).compute(payload, vehicles, target, .02)
    np.testing.assert_array_equal(baseline["desired_moment"], disabled["desired_moment"])
    np.testing.assert_allclose(disabled["desired_moment"],
                               [-.0001 * np.sin(.1), 0., 0.], atol=1e-18)
    for first, second in zip(baseline["vehicles"], disabled["vehicles"]):
        np.testing.assert_array_equal(first["force"], second["force"])
    assert baseline["payload_attitude_integral_enabled"] is False


@pytest.mark.parametrize("force_mode", ["coupled", "reference_cascade"])
def test_integral_moment_reaches_allocated_wrench_and_final_force_derivative(force_mode):
    controller, payload, vehicles, target = _case(force_mode)
    target["payload_attitude_gain"] = np.zeros(3)
    initial = controller.compute(payload, vehicles, target, .02)
    target["payload_attitude_integral_enabled"] = True
    learned = controller.compute(payload, vehicles, target, .02)
    np.testing.assert_allclose(learned["desired_moment"], [-2e-6, 0., 0.], atol=1e-18)
    np.testing.assert_allclose(learned["payload_attitude_integral_body_moment"],
                               [-2e-6, 0., 0.], atol=1e-18)
    allocated_moment = np.zeros(3)
    for rho, output in zip(controller.config.attachment_points_m.T, learned["vehicles"]):
        allocated_moment += np.cross(rho, payload["rotation"].T @ output["desired_tension_vector"])
    np.testing.assert_allclose(allocated_moment, learned["desired_moment"], atol=1e-16)
    changes = []
    for first, second in zip(initial["vehicles"], learned["vehicles"]):
        changes.append(np.linalg.norm(second["force"] - first["force"]))
        np.testing.assert_allclose(second["force_dot"],
                                   (second["force"] - first["force"]) / .02,
                                   atol=1e-16)
    assert max(changes) > 1e-6
    assert learned["payload_attitude_integral_learning"] is True
    assert learned["payload_attitude_integral_allocation_limited"] is False


def test_vertical_cables_block_yaw_windup_when_projected_moment_has_no_authority():
    controller, payload, vehicles, target = _case(axis="yaw")
    target.update(payload_attitude_gain=np.zeros(3), payload_attitude_integral_enabled=True)
    for tick in range(1, 21):
        target["payload_attitude_integral_tick"] = tick
        result = controller.compute(payload, vehicles, target, .02)
    np.testing.assert_array_equal(result["payload_attitude_integral_world_moment"], np.zeros(3))
    assert result["payload_attitude_integral_allocation_limited"] is True
    assert result["payload_attitude_integral_freeze_reason"] == "allocation_authority"
    assert result["payload_attitude_integral_projection_authority"] <= 1e-8


def test_nonpositive_cable_tension_rejects_only_a_worsening_increment():
    controller, payload, vehicles, target = _case(angle=.4)
    target.update(payload_attitude_gain=np.zeros(3), payload_attitude_integral_enabled=True,
                  payload_attitude_integral_gain_nm_per_rad_s=[1., 1., 1.],
                  payload_attitude_integral_limit_nm=.1,
                  payload_attitude_integral_rate_limit_nmps=10.)
    rejected = controller.compute(payload, vehicles, target, .05)
    np.testing.assert_array_equal(rejected["payload_attitude_integral_world_moment"], np.zeros(3))
    assert rejected["payload_attitude_integral_freeze_reason"] == "allocation_tension"
    target["payload_attitude_integral_gain_nm_per_rad_s"] = [.001, .001, .001]
    target["payload_attitude_integral_tick"] = 2
    accepted = controller.compute(payload, vehicles, target, .02)
    assert accepted["payload_attitude_integral_learning"] is True
    assert accepted["desired_moment"][0] < 0.


def test_reversing_bias_can_recover_tension_while_pd_allocation_is_still_infeasible():
    controller, payload, vehicles, target = _case()
    target.update(payload_attitude_gain=np.zeros(3), payload_attitude_integral_enabled=True)
    learned = controller.compute(payload, vehicles, target, .02)
    np.testing.assert_allclose(learned["payload_attitude_integral_world_moment"], [-2e-6, 0., 0.])
    # Almost remove payload support after learning. The retained moment now
    # requires compression; reversing the error must still let it recover.
    target["acceleration"][2] = -9.80665 + .001
    payload["rotation"] = payload["rotation"].T
    for index, vehicle in enumerate(vehicles):
        vehicle["position"] = (payload["rotation"]
                               @ controller.config.attachment_points_m[:, index]
                               + [0., 0., .85])
    target["payload_attitude_integral_tick"] = 2
    recovered = controller.compute(payload, vehicles, target, .01)
    np.testing.assert_allclose(recovered["payload_attitude_integral_world_moment"], [-1e-6, 0., 0.])
    assert recovered["payload_attitude_integral_learning"] is True
    assert min(output["desired_tension_signed"] for output in recovered["vehicles"]) < 0.


@pytest.mark.parametrize("invalid_dt", [np.nan, np.inf, -np.inf])
def test_nonfinite_dt_never_poison_existing_controller_state(invalid_dt):
    controller, payload, vehicles, target = _case()
    target["payload_attitude_integral_enabled"] = True
    controller.compute(payload, vehicles, target, .02)
    position_integral = controller.position_integral.copy()
    world_moment = controller.attitude_integral.world_moment
    target["payload_attitude_integral_tick"] = 2
    assert controller.compute(payload, vehicles, target, invalid_dt) is None
    np.testing.assert_array_equal(controller.position_integral, position_integral)
    np.testing.assert_array_equal(controller.attitude_integral.world_moment, world_moment)


def test_core_tick_dedup_reset_and_nonfinite_state_do_not_poison_integral():
    controller, payload, vehicles, target = _case()
    target["payload_attitude_integral_enabled"] = True
    first = controller.compute(payload, vehicles, target, .02)
    duplicate = controller.compute(payload, vehicles, target, .02)
    np.testing.assert_array_equal(first["payload_attitude_integral_world_moment"],
                                  duplicate["payload_attitude_integral_world_moment"])
    bad = copy.deepcopy(payload)
    bad["rotation"][:] = np.nan
    assert controller.compute(bad, vehicles, target, .02) is None
    target["payload_attitude_integral_tick"] = 2
    recovered = controller.compute(payload, vehicles, target, .02)
    np.testing.assert_allclose(recovered["payload_attitude_integral_world_moment"], [-4e-6, 0., 0.])
    controller.reset()
    target["payload_attitude_integral_learning"] = False
    target["payload_attitude_integral_tick"] = 3
    cleared = controller.compute(payload, vehicles, target, .02)
    np.testing.assert_array_equal(cleared["payload_attitude_integral_world_moment"], np.zeros(3))
