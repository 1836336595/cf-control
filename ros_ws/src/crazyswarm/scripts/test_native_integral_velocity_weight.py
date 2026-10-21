"""The transport I velocity coefficient must not alter PD or hover trim."""
import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _state
from test_native_position_pid_core import _fixture


def _weighted_fixture(module, weight):
    controller, target = _fixture(module, mass=.05,
        independent_position_gain=np.array([2., 3., 4.]),
        independent_velocity_gain=np.ones(3),
        independent_integral_gain=np.full(3, .2),
        independent_max_feedback_acceleration=np.full(3, 10.))
    target.update(position_hold_position=np.zeros(3), position_hold_velocity=np.zeros(3),
        position_hold_acceleration=np.zeros(3), desired_force_dot_override=np.zeros(3),
        independent_position_gain_scale=1., independent_velocity_gain_scale=1.,
        transport_position_hold_gain_scale=1., transport_position_hold_gain_scale_dot=0.,
        transport_full_pid_integral_limit=np.ones(3),
        transport_full_pid_force_limit_n=np.inf, transport_full_pid_force_rate_limit_nps=np.inf,
        transport_full_pid_integral_velocity_weight=weight)
    state = _state()
    state.update(position=np.array([.1, 0., 0.]), velocity=np.array([.2, 0., 0.]))
    return controller, target, state


@pytest.mark.parametrize("weight, expected_state, expected_rate", [
    (0., [.001, -.002, .003], [.05, -.1, .15]),
    (.5, [.005, -.003, .005], [.25, -.15, .25]),
    (1., [.009, -.004, .007], [.45, -.2, .35]),
])
def test_shared_integral_weight_changes_only_velocity_drive(weight, expected_state, expected_rate):
    """Ignoring the coefficient would integrate velocity bias at zero weight."""
    module = _controller_module()
    updated, rate, gate_open = module.position_integral_step(np.zeros(3),
        np.array([.1, -.2, .3]), np.array([.4, -.1, .2]), .02, 1., np.ones(3),
        velocity_weight=weight)
    np.testing.assert_allclose(updated, expected_state, atol=1e-16)
    np.testing.assert_allclose(rate, expected_rate, atol=1e-16)
    assert gate_open


def test_shared_integral_default_exactly_retains_unit_velocity_weight():
    """Changing the helper default would alter ordinary independent I learning."""
    module = _controller_module()
    args = (np.array([.01, -.02, .03]), np.array([.1, -.2, .3]),
            np.array([.4, -.1, .2]), .02, 1., np.ones(3))
    default = module.position_integral_step(*args)
    explicit = module.position_integral_step(*args, velocity_weight=1.)
    np.testing.assert_array_equal(default[0], explicit[0])
    np.testing.assert_array_equal(default[1], explicit[1])
    np.testing.assert_allclose(default[0], [.019, -.024, .037], atol=1e-16)


@pytest.mark.parametrize("weight", [-.1, np.nan, np.inf, -np.inf, True, np.bool_(False),
                                     None, "0.5", [0.], np.array([0.]), np.array(0.), 1j])
def test_shared_integral_rejects_invalid_velocity_weight(weight):
    """A malformed coefficient must fail before it can poison the I state."""
    module = _controller_module()
    with pytest.raises(ValueError, match="velocity_weight"):
        module.position_integral_step(np.zeros(3), np.zeros(3), np.ones(3),
            .02, 1., np.ones(3), velocity_weight=weight)


@pytest.mark.parametrize("weight, expected_integral, expected_force, expected_force_dot", [
    (0., .001, -.00003, -.0015),
    (.5, .003, -.00009, -.0045),
    (1., .005, -.00015, -.0075),
])
def test_native_integral_weight_and_unbounded_force_derivative(weight, expected_integral,
                                                              expected_force, expected_force_dot):
    """A stale I rate would produce a wrong force derivative after weighting."""
    module = _controller_module()
    controller, target, state = _weighted_fixture(module, weight)
    ordinary_i = np.array([.07, -.03, .04])
    controller.position_integral = ordinary_i.copy()
    command = controller.compute(state, target, .02)
    np.testing.assert_allclose(controller.transport_position_integral,
                               [expected_integral, 0., 0.], atol=1e-16)
    np.testing.assert_allclose(command["transport_full_pid_force"], [expected_force, 0., 0.], atol=1e-16)
    np.testing.assert_allclose(command["transport_full_pid_force_dot"],
                               [expected_force_dot, 0., 0.], atol=1e-15)
    np.testing.assert_allclose(command["desired_force_dot"],
                               [-.02+expected_force_dot, 0., 0.], atol=1e-15)
    np.testing.assert_allclose(command["transport_full_pid_pd_force"], [-.02, 0., 0.], atol=1e-16)
    np.testing.assert_array_equal(controller.position_integral, ordinary_i)
    assert command["transport_full_pid_info"]["integral_velocity_weight"] == weight
    assert command["transport_full_pid_integral_velocity_weight"] == weight


def test_zero_integral_velocity_weight_preserves_control_velocity_in_pd():
    """Zero I velocity drive must retain the selected EKF/control velocity PD."""
    module = _controller_module()
    controller, target, state = _weighted_fixture(module, 0.)
    state.update(position=np.zeros(3), velocity=np.full(3, 8.),
                 control_velocity=np.array([.2, 0., 0.]))
    command = controller.compute(state, target, .02)
    np.testing.assert_array_equal(controller.transport_position_integral, np.zeros(3))
    np.testing.assert_array_equal(command["transport_full_pid_force"], np.zeros(3))
    np.testing.assert_allclose(command["transport_full_pid_pd_force"], [-.01, 0., 0.], atol=1e-16)
    np.testing.assert_allclose(command["velocity_error"], [.2, 0., 0.], atol=1e-16)


@pytest.mark.parametrize("weight, expected_force, expected_rate, rate_limited", [
    (0., -.00003, -.0015, False),
    (1., -.00008, -.004, True),
])
def test_weighted_integral_force_derivative_uses_actual_budgeted_step(
        weight, expected_force, expected_rate, rate_limited):
    """The finite force budget must determine the applied I force derivative."""
    module = _controller_module()
    controller, target, state = _weighted_fixture(module, weight)
    target.update(transport_full_pid_force_limit_n=.06, transport_full_pid_force_rate_limit_nps=.004)
    command = controller.compute(state, target, .02)
    np.testing.assert_allclose(command["transport_full_pid_force"], [expected_force, 0., 0.], atol=1e-16)
    np.testing.assert_allclose(command["transport_full_pid_force_dot"], [expected_rate, 0., 0.], atol=1e-15)
    np.testing.assert_allclose(command["desired_force_dot"], [-.02+expected_rate, 0., 0.], atol=1e-15)
    assert command["transport_full_pid_rate_budget_limited"] == rate_limited


def test_native_default_is_exactly_unit_velocity_weight():
    """Targets predating this coefficient must reproduce their original command."""
    module = _controller_module()
    default, target, state = _weighted_fixture(module, 1.)
    explicit, explicit_target, _ = _weighted_fixture(module, 1.)
    target.pop("transport_full_pid_integral_velocity_weight")
    for tick in range(3):
        target["transport_full_pid_tick"] = tick
        explicit_target["transport_full_pid_tick"] = tick
        old = default.compute(state, target, .02)
        current = explicit.compute(state, explicit_target, .02)
        for name in ("transport_full_pid_integral_state", "raw_desired_force", "desired_force_dot"):
            np.testing.assert_array_equal(old[name], current[name])
        assert old["transport_full_pid_integral_velocity_weight"] == 1.


def test_changed_weight_on_duplicate_tick_recomputes_diagnostics_without_advancing_i():
    """Omitting the coefficient from the cache signature returns stale diagnostics."""
    module = _controller_module()
    controller, target, state = _weighted_fixture(module, 1.)
    target["transport_full_pid_tick"] = 1.
    controller.compute(state, target, .02)
    integral = controller.transport_position_integral.copy()
    force = controller.transport_full_pid_force_previous.copy()
    target["transport_full_pid_integral_velocity_weight"] = 0.
    command = controller.compute(state, target, .02)
    np.testing.assert_array_equal(controller.transport_position_integral, integral)
    np.testing.assert_array_equal(controller.transport_full_pid_force_previous, force)
    assert command["transport_full_pid_info"]["integral_velocity_weight"] == 0.
    assert command["transport_full_pid_duplicate_tick"]
    target["transport_full_pid_tick"] = 2.
    controller.compute(state, target, .02)
    np.testing.assert_allclose(controller.transport_position_integral, [.006, 0., 0.], atol=1e-16)


@pytest.mark.parametrize("weight", [-.1, np.nan, np.inf, True, [0.]])
def test_direct_native_target_rejects_invalid_velocity_weight(weight):
    """Direct compute callers must receive the same validation as shared helper users."""
    module = _controller_module()
    controller, target, state = _weighted_fixture(module, weight)
    with pytest.raises(ValueError, match="velocity_weight"):
        controller.compute(state, target, .02)


def test_transport_integral_coefficient_does_not_change_independent_integrator():
    """Forwarding the transport coefficient into ordinary I would remove hover trim."""
    module = _controller_module()
    controller, target, state = _weighted_fixture(module, 0.)
    target.update(independent_mode=True, transport_mode=False)
    state["position"] = np.array([.01, 0., 0.])
    command = controller.compute(state, target, .02)
    np.testing.assert_allclose(controller.position_integral, [.0041, 0., 0.], atol=1e-16)
    np.testing.assert_array_equal(controller.transport_position_integral, np.zeros(3))
    assert command["independent_mode"]


def test_weighted_integral_still_obeys_gate_learning_and_component_limits():
    """Weighting must retain closed-gate/frozen state and actual clipped rate."""
    module = _controller_module()
    args = (np.array([.01, 0., 0.]), np.array([.1, 0., 0.]),
            np.array([9., 0., 0.]), .02)
    updated, rate, gate_open = module.position_integral_step(*args, .1, np.ones(3), velocity_weight=0.)
    np.testing.assert_array_equal(updated, [.01, 0., 0.])
    np.testing.assert_array_equal(rate, np.zeros(3))
    assert not gate_open
    updated, rate, _ = module.position_integral_step(*args, 1., np.ones(3),
                                                   learning=False, velocity_weight=0.)
    np.testing.assert_array_equal(updated, [.01, 0., 0.])
    np.testing.assert_array_equal(rate, np.zeros(3))
    updated, rate, _ = module.position_integral_step(*args, 1., np.full(3, .0105), velocity_weight=0.)
    np.testing.assert_allclose(updated, [.0105, 0., 0.], atol=1e-16)
    np.testing.assert_allclose(rate, [.025, 0., 0.], atol=1e-16)
