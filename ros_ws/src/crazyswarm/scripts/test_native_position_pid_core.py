"""Production native transport PID must reproduce the selected offline law."""
import importlib.util
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _config, _state, _target


def _offline(module):
    path = Path(__file__).parent / "ctbr_logs/unified_pid_tuning_20261019/experiment/unified_pid_adapter.py"
    if not path.is_file():
        pytest.skip("archived offline native PID reference is not installed")
    spec = importlib.util.spec_from_file_location("native_pid_offline_reference", path)
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    return adapter.UnifiedPidExperiment(module, integral_gain_scale=3.,
        integral_limit_scale=10./3., integral_gate_m=.23, learning_start_s=0.,
        budget_mode="fixed_force", force_budget_n=.06, force_rate_nps=.004)


def _fixture(module, **config):
    values = dict(independent_position_gain=np.array([2., 3., 4.]),
        independent_velocity_gain=np.array([1., 1.5, 2.]),
        independent_integral_gain=np.array([.4, .5, .6]),
        independent_integral_limit=np.array([.02, .03, .04]),
        independent_max_feedback_acceleration=np.array([.35, .45, .55]))
    values.update(config)
    controller = module.GeometricCtbrController(_config(module, **values))
    target = _target(acceleration=[.01, -.02, .03])
    target.update(transport_mode=True, desired_force_override=np.array([.02, -.01, .55]),
        desired_force_dot_override=np.array([.003, -.002, .001]),
        ramp_position_hold_gain_scale=.7, transport_position_hold_gain_scale=1.1,
        transport_position_hold_gain_scale_dot=.12,
        transport_blend=1., transport_blend_dot=.0,
        position_hold_position=np.array([.01, -.02, .03]),
        position_hold_velocity=np.array([.03, -.02, .01]),
        position_hold_acceleration=np.array([.02, -.01, .04]),
        independent_position_gain_scale=np.array([1., .8, 1.2]),
        independent_velocity_gain_scale=np.array([.9, 1.1, 1.]),
        transport_feedback_controller="native_position_pid",
        transport_full_pid_learning=True, transport_full_pid_integral_gain_scale=3.,
        transport_full_pid_integral_limit=np.array([.02, .03, .04])*10./3.,
        transport_full_pid_integral_gate_m=.23,
        transport_full_pid_force_limit_n=.06, transport_full_pid_force_rate_limit_nps=.004)
    return controller, target


def test_native_force_and_derivative_match_offline_clamped_pid_without_double_pd():
    """Catches repeated PD, whole-PID budgeting and missing I force derivative."""
    module = _controller_module()
    controller, target = _fixture(module)
    reference = module.GeometricCtbrController(deepcopy(controller.config))
    experiment = _offline(module)
    ordinary_i = np.array([.07, -.03, .04])
    controller.position_integral = ordinary_i.copy()
    reference.position_integral = ordinary_i.copy()
    for tick in range(70):
        state = _state(acceleration=[.04, -.03, .02])
        state["position"] = np.array([.10, -.08, .07]) + tick*np.array([.0001, -.0002, .0001])
        state["velocity"] = np.array([.02, -.01, .03])
        target["transport_blend"] = .4 if tick < 20 else 1.
        target["transport_blend_dot"] = .2 if tick < 20 else 0.
        target["transport_full_pid_tick"] = tick
        learning = tick < 45
        target["transport_full_pid_learning"] = learning
        baseline = dict(target, transport_feedback_controller="differential_integral")
        mapped, info = experiment.transform(reference, state, baseline, .02, tick*.02,
            tick_key=tick, learning=learning, freeze_reason="" if learning else "external_freeze")
        expected = reference.compute(state, mapped, .02)
        actual = controller.compute(state, target, .02)
        np.testing.assert_allclose(actual["raw_desired_force"], expected["raw_desired_force"], atol=2e-14)
        np.testing.assert_allclose(actual["desired_force_dot"], expected["desired_force_dot"], atol=2e-14)
        np.testing.assert_allclose(controller.transport_position_integral, info["integral_state"], atol=2e-14)
        np.testing.assert_array_equal(controller.position_integral, ordinary_i)


def test_duplicate_tick_recomputes_pd_but_cannot_advance_integral_or_budget():
    """A manager target rebuild during one tick must not spend a second I step."""
    module = _controller_module()
    controller, target = _fixture(module)
    state = _state(acceleration=[.1, -.2, .05])
    state["position"] = np.array([.1, -.1, .1])
    target["transport_full_pid_tick"] = 2.
    first = controller.compute(state, target, .02)
    integral = controller.transport_position_integral.copy()
    budget = controller.transport_full_pid_force_previous.copy()
    target["position_hold_position"] = np.array([.02, .01, -.01])
    second = controller.compute(state, target, .02)
    np.testing.assert_array_equal(controller.transport_position_integral, integral)
    np.testing.assert_array_equal(controller.transport_full_pid_force_previous, budget)
    assert not np.allclose(first["raw_desired_force"], second["raw_desired_force"])


def test_native_optional_config_falls_back_and_transport_reset_preserves_hover_trim():
    """None-valued optional gains use ordinary gains and reset only transport I."""
    module = _controller_module()
    controller, target = _fixture(module, independent_position_gain=None,
        independent_velocity_gain=None, independent_integral_gain=None,
        independent_integral_limit=None, independent_max_feedback_acceleration=None)
    target.pop("transport_full_pid_integral_limit")
    state = _state()
    state["position"] = np.array([.1, -.1, .1])
    controller.position_integral[:] = [.1, .2, .3]
    command = controller.compute(state, target, .02)
    assert np.all(np.isfinite(command["raw_desired_force"]))
    controller.reset_transport_position_pid()
    np.testing.assert_array_equal(controller.transport_position_integral, np.zeros(3))
    np.testing.assert_array_equal(controller.transport_full_pid_force_previous, np.zeros(3))
    np.testing.assert_array_equal(controller.position_integral, [.1, .2, .3])


def test_unload_weight_scales_only_extra_integral_force_with_product_derivative():
    """Unload must retain PD and include wdot*b as well as w*bdot."""
    module = _controller_module()
    weighted, target = _fixture(module)
    control, control_target = _fixture(module)
    state = _state(acceleration=[.1, -.2, .05])
    state["position"] = np.array([.1, -.1, .1])
    target.update(transport_full_pid_weight=.3, transport_full_pid_weight_dot=-.2,
                  transport_blend=.4, transport_blend_dot=.15)
    control_target.update(transport_blend=.4, transport_blend_dot=.15)
    unweighted = control.compute(state, control_target, .02)
    command = weighted.compute(state, target, .02)
    bias = control.transport_full_pid_force_previous
    bias_dot = bias/.02
    np.testing.assert_allclose(command["raw_desired_force"]-unweighted["raw_desired_force"],
                               -.7*.4*bias, atol=2e-14)
    np.testing.assert_allclose(command["desired_force_dot"]-unweighted["desired_force_dot"],
        -.7*(.4*bias_dot+.15*bias)-.2*.4*bias, atol=2e-14)
    np.testing.assert_allclose(command["transport_full_pid_force"], .4*.3*bias, atol=2e-14)
    np.testing.assert_allclose(command["transport_full_pid_force_dot"],
        .4*.3*bias_dot+(.15*.3-.4*.2)*bias, atol=2e-14)


def test_bad_derivatives_freeze_new_integral_without_removing_existing_bias():
    """The gate closes learning while the bounded existing trim remains applied."""
    module = _controller_module()
    controller, target = _fixture(module)
    state = _state()
    state["position"] = np.array([.1, -.1, .1])
    controller.compute(state, target, .02)
    integral = controller.transport_position_integral.copy()
    state["derivatives_valid"] = False
    state["velocity"][:] = np.nan
    command = controller.compute(state, target, .02)
    np.testing.assert_array_equal(controller.transport_position_integral, integral)
    assert np.linalg.norm(command["transport_full_pid_force"]) > 0.
    assert not command["transport_full_pid_learning"]
    assert command["transport_full_pid_freeze_reason"] == "vehicle_derivatives"


def test_numeric_native_pid_replaces_pd_and_budgets_only_extra_i():
    """Hand-derived force/derivative regression independent of archived logs."""
    module = _controller_module()
    controller, target = _fixture(module, mass=.05,
        independent_position_gain=np.array([2., 3., 4.]),
        independent_velocity_gain=np.ones(3), independent_integral_gain=np.full(3, .2),
        independent_max_feedback_acceleration=np.full(3, 10.))
    state = _state(acceleration=[.3, 0., 0.])
    state.update(position=np.array([.1, 0., 0.]), velocity=np.array([.2, 0., 0.]))
    target.update(position_hold_position=np.zeros(3), position_hold_velocity=np.zeros(3),
        position_hold_acceleration=np.array([.1, 0., 0.]),
        independent_position_gain_scale=1., independent_velocity_gain_scale=1.,
        transport_position_hold_gain_scale=1., transport_position_hold_gain_scale_dot=0.)
    # ep=.1, ev=.2 -> I=.005, PD=-.4 m/s². Native extra I=-.00015 N;
    # the .004 N/s rate budget admits -.00008 N, with derivative -.004 N/s.
    command = controller.compute(state, target, .02)
    np.testing.assert_allclose(command["raw_desired_force"], [-.00008, -.01, .55], atol=1e-15)
    np.testing.assert_allclose(command["desired_force_dot"], [-.031, -.002, .001], atol=1e-15)
    np.testing.assert_allclose(command["transport_full_pid_pd_force"], [-.02, 0., 0.], atol=1e-15)
    assert command["transport_full_pid_rate_budget_limited"]
    assert not command["transport_full_pid_force_budget_limited"]
    # A large stored native I produces -.09 N. Its .06 N cap leaves the
    # separate -.02 N PD unchanged; the total feedback is consequently -.08 N.
    controller.reset_transport_position_pid()
    controller.transport_position_integral[:] = [3., 0., 0.]
    target.update(transport_full_pid_learning=False, transport_full_pid_force_rate_limit_nps=100.)
    command = controller.compute(state, target, .02)
    np.testing.assert_allclose(command["raw_desired_force"], [-.06, -.01, .55], atol=1e-15)
    np.testing.assert_allclose(command["transport_full_pid_force"], [-.06, 0., 0.], atol=1e-15)
    assert command["transport_full_pid_force_budget_limited"]


def test_cached_force_hold_freezes_bias_across_ticks_but_external_freeze_keeps_slewing():
    """Cached payload feedback cannot spend rate budget or learn another I step."""
    module = _controller_module()
    controller, target = _fixture(module)
    controller.transport_position_integral[:] = [.05, -.04, .03]
    state = _state(acceleration=[.1, -.2, .05])
    state["position"] = np.array([.1, -.1, .1])
    target.update(transport_full_pid_tick=1., transport_full_pid_learning=False,
        transport_full_pid_freeze_reason="external_freeze", transport_blend=.4,
        transport_blend_dot=.2, transport_full_pid_weight=.6,
        transport_full_pid_weight_dot=0.)
    first = controller.compute(state, target, .02)
    integral = controller.transport_position_integral.copy()
    budget = controller.transport_full_pid_force_previous.copy()
    assert first["transport_full_pid_rate_budget_limited"]
    assert np.linalg.norm(budget) > 0.
    previous_pd = first["transport_full_pid_pd_force"]
    for tick in (2., 3.):
        target.update(transport_full_pid_tick=tick, transport_full_pid_force_hold=True,
            transport_full_pid_learning=True, transport_full_pid_freeze_reason="")
        target["position_hold_position"] = np.array([.01*tick, .01, -.01])
        command = controller.compute(state, target, .02)
        np.testing.assert_array_equal(controller.transport_position_integral, integral)
        np.testing.assert_array_equal(controller.transport_full_pid_force_previous, budget)
        np.testing.assert_array_equal(command["transport_full_pid_info"]["integral_force_dot"], np.zeros(3))
        np.testing.assert_allclose(command["transport_full_pid_force"], .4*.6*budget, atol=1e-15)
        np.testing.assert_allclose(command["transport_full_pid_force_dot"], .2*.6*budget, atol=1e-15)
        assert not np.allclose(command["transport_full_pid_pd_force"], previous_pd)
        previous_pd = command["transport_full_pid_pd_force"]
    target.update(transport_full_pid_tick=4., transport_full_pid_force_hold=False,
        transport_full_pid_learning=False, transport_full_pid_freeze_reason="external_freeze")
    command = controller.compute(state, target, .02)
    np.testing.assert_array_equal(controller.transport_position_integral, integral)
    np.testing.assert_allclose(np.linalg.norm(controller.transport_full_pid_force_previous-budget),
                               .004*.02, atol=1e-15)
    assert np.linalg.norm(command["transport_full_pid_info"]["integral_force_dot"]) > 0.
