"""Tick-by-tick production comparison with the validated offline PID adapter.

The archived adapter is optional because local flight logs are not distributed
with the repository.  Its independent PID shadow is the reference; neither
controller's force, filter, attitude, nor rate computation is replaced here.
"""

from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from test_ctbr_controller_v2 import _config, _controller_module, _state, _target


SCRIPT_DIR = Path(__file__).resolve().parent
ADAPTER = (SCRIPT_DIR / "ctbr_logs" / "unified_pid_tuning_20261019"
           / "experiment" / "unified_pid_adapter.py")
OUTPUTS = ("raw_desired_force", "desired_force", "desired_force_dot",
           "computed_rotation", "computed_body_rate", "raw_body_rate_command",
           "body_rate_command", "collective_thrust")
ORDINARY_INTEGRAL = np.array([.11, -.12, .13])


def _experiment(module):
    if not ADAPTER.is_file():
        pytest.skip("archived offline native PID reference is not installed")
    spec = importlib.util.spec_from_file_location("native_pid_archive_reference", ADAPTER)
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    return adapter.UnifiedPidExperiment(module, integral_gain_scale=3.,
        integral_limit_scale=10. / 3., integral_gate_m=.23,
        learning_start_s=17., hold_scale_mode="matched", budget_mode="fixed_force",
        force_budget_n=.06, force_rate_nps=.004)


def _pair():
    module = _controller_module()
    config = _config(module,
        independent_position_gain=np.array([4., 3., 4.5]),
        independent_velocity_gain=np.array([2., 1.6, 2.2]),
        independent_integral_gain=np.array([.8, .7, 1.1]),
        independent_integral_limit=np.array([.2, .25, .3]),
        independent_integral_gate=.05,
        independent_max_feedback_acceleration=np.array([3., 2.5, 3.5]),
        transport_force_filter_tau_s=.04,
        transport_body_rate_filter_tau_s=.03)
    native = module.GeometricCtbrController(config)
    reference = module.GeometricCtbrController(deepcopy(config))
    for controller in (native, reference):
        controller.position_integral[:] = ORDINARY_INTEGRAL
    return native, reference, _experiment(module)


def _inputs(tick, blend=1., offset=.08, source="control", handoff=True):
    """Moving hold reference deliberately differs from the main flight target."""
    t = .02 * tick
    hold_position = np.array([.03 + .012*t, -.02 - .008*t, .04 + .006*t])
    hold_velocity = np.array([.012, -.008, .006])
    hold_acceleration = np.array([-.004, .003, .002])
    state = _state(acceleration=[.021, -.014, .017],
        rotation=np.array([[.99980001, -.01999867, 0.],
                           [.01999867, .99980001, 0.], [0., 0., 1.]]))
    state["position"] = hold_position + [offset*(1.+.03*np.sin(t)), -.2*offset, .015]
    state["velocity"] = np.array([.047, -.027, .019])
    if source in ("filtered", "control", "invalid_source", "invalid_filter"):
        state.update(filtered_velocity=np.array([.038, -.022, .027]),
            filtered_acceleration=np.array([.019, -.011, .023]),
            filter_derivatives_valid=True)
    if source in ("control", "invalid_source", "invalid_filter"):
        state.update(control_velocity=np.array([.031, -.018, .021]),
            control_acceleration=np.array([.014, -.009, .019]))
    if source == "invalid_source":
        state["derivatives_valid"] = False
    if source == "invalid_filter":
        state["filter_derivatives_valid"] = False
    target = _target(acceleration=[.01, -.02, .03], jerk=[.006, -.004, .002],
                     yaw=.12 + .01*t, yaw_rate=.01)
    target.update(position=np.array([-.04, .05, -.02]),
        velocity=np.array([-.01, .02, -.03]), transport_mode=True,
        transport_blend=blend, transport_blend_dot=.013,
        desired_force_override=np.array([.025+.001*t, -.018, .59+.002*t]),
        desired_force_dot_override=np.array([.001, -.002, .003]),
        position_hold_position=hold_position,
        position_hold_velocity=hold_velocity,
        position_hold_acceleration=hold_acceleration,
        ramp_position_hold_gain_scale=.3,
        transport_position_hold_gain_scale=.65 + .015*t,
        transport_position_hold_gain_scale_dot=.015,
        independent_position_gain_scale=np.array([1., .9, .8]),
        independent_velocity_gain_scale=np.array([.7, .8, 1.]),
        transport_hold_integral_force=np.zeros(3),
        transport_hold_integral_force_dot=np.zeros(3),
        transport_feedback_controller="native_position_pid",
        transport_full_pid_integral_gain_scale=3.,
        transport_full_pid_integral_limit=np.array([.2, .25, .3])*10./3.,
        transport_full_pid_integral_gate_m=.23,
        transport_full_pid_force_limit_n=.06,
        transport_full_pid_force_rate_limit_nps=.004)
    if handoff:
        target["handoff_force"] = np.array([.012, -.023, .53])
    return state, target


def _tick(native, reference, experiment, state, target, now, learning=True,
          freeze_reason="", dt=.02):
    healthy = (state.get("derivatives_valid", True)
               and state.get("filter_derivatives_valid", True))
    production_target = dict(target, transport_full_pid_tick=now,
        transport_full_pid_learning=bool(learning and healthy and now >= 17.))
    archived_target = dict(target, transport_feedback_controller="differential_integral")
    mapped, info = experiment.transform(reference, state, archived_target, dt, now,
        tick_key=now, learning=learning, freeze_reason=freeze_reason)
    expected = reference.compute(state, mapped, dt)
    actual = native.compute(state, production_target, dt)
    for name in OUTPUTS:
        np.testing.assert_allclose(actual[name], expected[name], rtol=0., atol=2e-12,
            err_msg=f"{name}, time={now}, beta={target['transport_blend']}")
    np.testing.assert_array_equal(native.position_integral, ORDINARY_INTEGRAL)
    np.testing.assert_array_equal(reference.position_integral, ORDINARY_INTEGRAL)
    np.testing.assert_allclose(native.transport_position_integral,
        info["integral_state"], rtol=0., atol=1e-14)
    return actual, info


@pytest.mark.parametrize("blend", [0., .4, 1.])
@pytest.mark.parametrize("offset", [.08, 5.])
@pytest.mark.parametrize("source", ["raw", "filtered", "control",
                                    "invalid_source", "invalid_filter"])
@pytest.mark.parametrize("handoff", [False, True])
def test_production_matches_archived_full_transport_tick_sequence(blend, offset, source, handoff):
    """Catch doubled PD, whole-PID budgeting, wrong state selection and F_dot loss."""
    native, reference, experiment = _pair()
    for tick in range(24):
        state, target = _inputs(tick, blend, offset, source, handoff)
        _tick(native, reference, experiment, state, target, 17.8+.02*tick)


def test_learning_start_and_freezes_preserve_native_integral_and_applied_budget():
    """The 17 s transition and closed native gate must match the offline shadow."""
    native, reference, experiment = _pair()
    learned = False
    for tick in range(80):
        now = 16.8+.02*tick
        outside_gate = 30 <= tick < 40
        source = "invalid_source" if 50 <= tick < 55 else "control"
        state, target = _inputs(tick, offset=.3 if outside_gate else .08, source=source)
        learning = not 60 <= tick < 65
        before = native.transport_position_integral.copy()
        _, info = _tick(native, reference, experiment, state, target, now,
            learning=learning, freeze_reason="landing" if not learning else "")
        if now < 17. or outside_gate or source == "invalid_source" or not learning:
            np.testing.assert_array_equal(native.transport_position_integral, before)
        elif np.linalg.norm(info["native_integral_state_delta"]) > 0.:
            learned = True
    assert learned


def test_ramp_to_full_transport_matches_with_existing_force_and_rate_filter_history():
    """Changing beta and the ramp/full hold scale must preserve the same handoff."""
    native, reference, experiment = _pair()
    for tick in range(100):
        blend = float(np.clip((tick-10)/60., 0., 1.))
        state, target = _inputs(tick, blend=blend)
        target["transport_blend_dot"] = 1./1.2 if 10 < tick < 70 else 0.
        _tick(native, reference, experiment, state, target, 17.8+.02*tick)


def test_actual_extra_integral_uses_point_zero_six_norm_and_point_zero_zero_four_rate():
    """Only the clamped native I difference consumes the Euclidean force budget."""
    native, reference, experiment = _pair()
    previous = np.zeros(3)
    saw_rate_limit = saw_force_limit = False
    for tick in range(1100):
        state, target = _inputs(tick)
        state["control_velocity"] = np.array([.42, -.25, .19])
        _, info = _tick(native, reference, experiment, state, target, 18.+.02*tick)
        applied = np.asarray(info["applied_extra_i_before_blend_n"])
        assert np.linalg.norm(applied) <= .06+1e-13
        assert np.linalg.norm(applied-previous) <= .004*.02+1e-13
        previous = applied.copy()
        saw_rate_limit |= info["rate_budget_limited"]
        saw_force_limit |= info["force_budget_limited"]
    assert saw_rate_limit and saw_force_limit
    np.testing.assert_allclose(np.linalg.norm(previous), .06, atol=1e-12)
