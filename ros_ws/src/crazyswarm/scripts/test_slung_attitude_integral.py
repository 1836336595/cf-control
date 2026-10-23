"""Moment integral sign, reference frames and bounded state transitions."""

import numpy as np
import pytest


def _rotation(axis, angle):
    vector = np.asarray(axis, dtype=float)
    vector /= np.linalg.norm(vector)
    x, y, z = vector
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return (np.eye(3) + np.sin(angle) * skew
            + (1. - np.cos(angle)) * skew @ skew)


def _integral(**kwargs):
    from slung_attitude_integral import SlungAttitudeMomentIntegral
    parameters = dict(gain_nm_per_rad_s=[2., 3., 4.], limit_nm=10.,
                      rate_limit_nmps=100.)
    parameters.update(kwargs)
    return SlungAttitudeMomentIntegral(**parameters)


def _value(snapshot, name):
    return snapshot["payload_attitude_integral_" + name]


def test_correcting_sign_and_gain_axes_follow_the_desired_frame():
    integral = _integral()
    desired = _rotation([0, 0, 1], np.pi / 2.)
    measured = desired @ _rotation([1, 0, 0], .2)
    state = integral.update(measured, desired, .1, tick=1)

    np.testing.assert_allclose(_value(state, "error_goal"), [.2, 0., 0.], atol=1e-15)
    np.testing.assert_allclose(_value(state, "world_moment"), [0., -.04, 0.], atol=1e-15)
    np.testing.assert_allclose(_value(state, "body_moment"), [-.04, 0., 0.], atol=1e-15)
    np.testing.assert_allclose(_value(state, "world_moment_dot"), [0., -.4, 0.], atol=1e-15)


def test_frozen_world_bias_does_not_rotate_with_either_payload_frame():
    integral = _integral()
    integral.update(_rotation([1, 0, 0], .2), np.eye(3), .1, tick=1)
    measured = _rotation([0, 0, 1], np.pi / 2.)
    desired = _rotation([0, 1, 0], .3)
    state = integral.update(measured, desired, .2, learning=False,
                            freeze_reason="host_limit", tick=2)

    np.testing.assert_allclose(_value(state, "world_moment"), [-.04, 0., 0.], atol=1e-15)
    np.testing.assert_allclose(_value(state, "body_moment"), [0., .04, 0.], atol=1e-15)
    np.testing.assert_array_equal(_value(state, "world_moment_dot"), np.zeros(3))
    assert _value(state, "freeze_reason") == "host_limit"
    assert _value(state, "learning") is False


def test_noncommuting_moving_goals_accumulate_each_error_in_world_axes():
    integral = _integral(gain_nm_per_rad_s=[1., 1., 1.])
    integral.update(_rotation([1, 0, 0], .2), np.eye(3), .1, tick=1)
    desired = _rotation([0, 0, 1], np.pi / 2.)
    state = integral.update(desired @ _rotation([1, 0, 0], .3), desired,
                            .2, tick=2)
    np.testing.assert_allclose(_value(state, "world_moment"), [-.02, -.06, 0.], atol=1e-15)


def test_rate_and_norm_limits_bound_the_actual_state_and_reverse_without_windup():
    integral = _integral(gain_nm_per_rad_s=[1., 1., 1.], limit_nm=.1,
                         rate_limit_nmps=.2)
    measured = _rotation([1, 0, 0], 1.)
    first = integral.update(measured, np.eye(3), .1, tick=1)
    np.testing.assert_allclose(_value(first, "world_moment"), [-.02, 0., 0.])
    assert _value(first, "rate_limit_active")
    for tick in range(2, 21):
        capped = integral.update(measured, np.eye(3), .1, tick=tick)
    np.testing.assert_allclose(_value(capped, "world_moment"), [-.1, 0., 0.])
    assert _value(capped, "limit_active")
    reversed_state = integral.update(measured.T, np.eye(3), .1, tick=21)
    np.testing.assert_allclose(_value(reversed_state, "world_moment"), [-.08, 0., 0.])
    np.testing.assert_allclose(_value(reversed_state, "world_moment_dot"), [.2, 0., 0.])


def test_duplicate_ticks_integrate_once_and_disabled_clears_memory():
    integral = _integral()
    measured = _rotation([0, 1, 0], .1)
    first = integral.update(measured, np.eye(3), .1, tick=4)
    duplicate = integral.update(measured, np.eye(3), .1, tick=4)
    np.testing.assert_array_equal(_value(first, "world_moment"),
                                  _value(duplicate, "world_moment"))
    assert _value(duplicate, "learning") is False
    assert _value(duplicate, "freeze_reason") == "duplicate_tick"
    changed = integral.update(_rotation([0, 0, 1], np.pi / 2.), np.eye(3), .1,
                              tick=4, learning=False, freeze_reason="landing",
                              weight=.5, weight_dot=-.2)
    np.testing.assert_allclose(_value(changed, "body_moment"), [-.015, 0., 0.], atol=1e-15)
    np.testing.assert_allclose(_value(changed, "world_moment_dot"), [0., .006, 0.], atol=1e-15)
    assert _value(changed, "freeze_reason") == "landing"
    integral.update(measured, np.eye(3), .1, tick=5, enabled=False)
    next_state = integral.update(measured, np.eye(3), .1, tick=6)
    np.testing.assert_allclose(_value(next_state, "world_moment"), [0., -.03, 0.])
    integral.reset()
    reset_state = integral.update(measured, np.eye(3), .1, tick=7, learning=False)
    np.testing.assert_array_equal(_value(reset_state, "body_moment"), np.zeros(3))


def test_unload_weight_scales_output_and_reports_its_actual_world_derivative():
    integral = _integral()
    measured = _rotation([1, 0, 0], .2)
    integral.update(measured, np.eye(3), .1, tick=1)
    state = integral.update(measured, np.eye(3), .1, tick=2, learning=False,
                            weight=.25, weight_dot=-.5)
    np.testing.assert_allclose(_value(state, "body_moment"), [-.01, 0., 0.])
    np.testing.assert_allclose(_value(state, "world_moment_dot"), [.02, 0., 0.])
    np.testing.assert_allclose(_value(state, "world_moment"), [-.04, 0., 0.])


def test_disabled_yaw_never_learns_goal_yaw_or_outputs_measured_body_yaw():
    integral = _integral()
    state = integral.update(_rotation([0, 0, 1], .3), np.eye(3), .1,
                            tick=1, yaw_enabled=False)
    np.testing.assert_array_equal(_value(state, "world_moment"), np.zeros(3))
    integral.update(_rotation([1, 0, 0], .2), np.eye(3), .1, tick=2)
    state = integral.update(_rotation([0, 1, 0], np.pi / 2.), np.eye(3), .1,
                            tick=3, learning=False, yaw_enabled=False)
    assert _value(state, "body_moment")[2] == 0.
    assert _value(state, "yaw_enabled") is False


def test_disabling_yaw_erases_stored_yaw_before_reenable():
    integral = _integral()
    yawed = _rotation([0, 0, 1], .3)
    integral.update(yawed, np.eye(3), .1, tick=1)
    disabled = integral.update(yawed, np.eye(3), .1, tick=2,
                               learning=False, yaw_enabled=False)
    np.testing.assert_array_equal(_value(disabled, "world_moment"), np.zeros(3))
    restored = integral.update(yawed, np.eye(3), .1, tick=3, learning=False)
    np.testing.assert_array_equal(_value(restored, "body_moment"), np.zeros(3))


def test_invalid_measurement_holds_finite_state_without_poisoning_next_sample():
    integral = _integral()
    measured = _rotation([1, 0, 0], .2)
    integral.update(measured, np.eye(3), .1, tick=1)
    bad = integral.update(np.full((3, 3), np.nan), np.eye(3), .1, tick=2)
    np.testing.assert_allclose(_value(bad, "world_moment"), [-.04, 0., 0.])
    assert not _value(bad, "learning")
    assert _value(bad, "freeze_reason") == "invalid_state"
    assert np.all(np.isfinite(_value(bad, "body_moment")))
    recovered = integral.update(measured, np.eye(3), .1, tick=3)
    np.testing.assert_allclose(_value(recovered, "world_moment"), [-.08, 0., 0.])


def test_allocation_rejection_preserves_state_and_accepts_immediate_reversal():
    integral = _integral()
    measured = _rotation([1, 0, 0], .2)
    state = integral.update(measured, np.eye(3), .1, tick=1,
                            accept=lambda candidate, previous: "allocation_tension")
    np.testing.assert_array_equal(_value(state, "world_moment"), np.zeros(3))
    np.testing.assert_array_equal(_value(state, "world_moment_dot"), np.zeros(3))
    assert _value(state, "allocation_limited")
    assert _value(state, "freeze_reason") == "allocation_tension"
    state = integral.update(measured.T, np.eye(3), .1, tick=2,
                            accept=lambda candidate, previous: True)
    np.testing.assert_allclose(_value(state, "world_moment"), [.04, 0., 0.])


@pytest.mark.parametrize("parameters", [
    dict(gain_nm_per_rad_s=[1., -1., 1.]),
    dict(gain_nm_per_rad_s=[1., np.inf, 1.]),
    dict(gain_nm_per_rad_s=[True, 1., 1.]),
    dict(limit_nm=0.), dict(limit_nm=np.nan), dict(rate_limit_nmps=-1.),
])
def test_unsafe_configuration_is_rejected(parameters):
    with pytest.raises(ValueError):
        _integral(**parameters)
