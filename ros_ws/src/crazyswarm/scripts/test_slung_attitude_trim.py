"""Slow aircraft reference trim must remain geometrically and temporally safe."""

import importlib

import numpy as np
import pytest


ATTACHMENTS = np.array([[.2, -.1, -.1], [0., .1, -.1], [0., 0., 0.]])


def rotation_y(angle):
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[cosine, 0., sine], [0., 1., 0.], [-sine, 0., cosine]])


def rotation_z(angle):
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[cosine, -sine, 0.], [sine, cosine, 0.], [0., 0., 1.]])


def make_trim(**kwargs):
    try:
        module = importlib.import_module("slung_attitude_trim")
    except ModuleNotFoundError:
        pytest.fail("the attitude reference trim helper has not been implemented")
    return module.SlungAttitudeReferenceTrim(ATTACHMENTS, **kwargs)


def test_positive_payload_pitch_raises_front_aircraft_and_lowers_rear_aircraft():
    trim = make_trim(gain_per_s=.1, max_rate_mps=1., max_acceleration_mps2=1.)
    first = trim.update(rotation_y(.02), np.eye(3), 0.)
    np.testing.assert_array_equal(first, np.zeros((3, 3, 3)))
    position, velocity, acceleration = trim.update(None, np.eye(3), 1., learning=False)
    np.testing.assert_allclose(position, [[0., 0., .0004], [0., 0., -.0002],
                                         [0., 0., -.0002]], atol=1e-15)
    np.testing.assert_allclose(trim.integral, [0., .002, 0.], atol=1e-15)
    np.testing.assert_array_equal(velocity, np.zeros((3, 3)))
    np.testing.assert_array_equal(acceleration, np.zeros((3, 3)))


def test_goal_frame_yaw_error_produces_world_xy_trim_with_zero_mean():
    trim = make_trim(gain_per_s=.1, max_rate_mps=1., max_acceleration_mps2=1.)
    desired = rotation_z(np.pi / 2.)
    trim.update(desired @ rotation_z(.1), desired, 0.)
    position, _, _ = trim.update(None, desired, 1., learning=False)
    np.testing.assert_allclose(position, [[.002, 0., 0.], [-.001, .001, 0.],
                                         [-.001, -.001, 0.]], atol=1e-15)
    np.testing.assert_allclose(trim.attitude_error, [0., 0., .1], atol=1e-15)
    np.testing.assert_allclose(position.sum(axis=0), 0., atol=1e-15)


def test_nonidentity_goal_uses_error_in_goal_coordinates():
    trim = make_trim(gain_per_s=[.1, .2, .3], max_rate_mps=1., max_acceleration_mps2=1.)
    desired = rotation_z(np.pi / 2.)
    trim.update(desired @ rotation_y(.02), desired, 0.)
    np.testing.assert_allclose(trim.attitude_error, [0., .02, 0.], atol=1e-15)
    np.testing.assert_allclose(trim.integral, [0., .004, 0.], atol=1e-15)


def test_attachment_centroid_does_not_create_common_translation():
    original = make_trim(max_rate_mps=1., max_acceleration_mps2=1.)
    shifted = original.__class__(ATTACHMENTS + np.array([[1.], [-2.], [3.]]),
                                 max_rate_mps=1., max_acceleration_mps2=1.)
    for trim in (original, shifted):
        trim.update(rotation_y(.03) @ rotation_z(.04), np.eye(3), 0.)
    for actual, expected in zip(shifted.evaluate(.4), original.evaluate(.4)):
        np.testing.assert_allclose(actual, expected, atol=1e-15)
        np.testing.assert_allclose(actual.sum(axis=0), 0., atol=1e-15)


@pytest.mark.parametrize("limits", [dict(max_rate_mps=.0001, max_acceleration_mps2=1.),
                                    dict(max_rate_mps=.1, max_acceleration_mps2=.001),
                                    dict(max_rate_mps=.001, max_acceleration_mps2=.001)])
def test_entire_segments_obey_aircraft_norm_limits_including_derivative_extrema(limits):
    trim = make_trim(gain_per_s=10., **limits)
    critical_times = [0., (3. - np.sqrt(3.)) / 6., .5,
                      (3. + np.sqrt(3.)) / 6., 1.]
    for second in range(65):
        trim.update(rotation_y(.4) @ rotation_z(.2), np.eye(3), float(second))
        for fraction in critical_times:
            position, velocity, acceleration = trim.evaluate(second + fraction)
            assert np.max(np.linalg.norm(position, axis=1)) <= .01 + 1e-14
            assert np.max(np.linalg.norm(velocity, axis=1)) <= limits["max_rate_mps"] + 1e-14
            assert np.max(np.linalg.norm(acceleration, axis=1)) <= limits["max_acceleration_mps2"] + 1e-14
            for values in (position, velocity, acceleration):
                np.testing.assert_allclose(values.sum(axis=0), 0., atol=1e-14)


def test_quintic_rates_match_finite_differences_and_join_with_zero_acceleration():
    trim = make_trim(gain_per_s=.1, max_rate_mps=1., max_acceleration_mps2=1.)
    trim.update(rotation_y(.04), np.eye(3), 0.)
    epsilon = 1e-4
    for now in (.12, .5, .87):
        left, middle, right = [trim.evaluate(t) for t in (now-epsilon, now, now+epsilon)]
        np.testing.assert_allclose((right[0]-left[0])/(2*epsilon), middle[1], atol=1e-10)
        np.testing.assert_allclose((right[1]-left[1])/(2*epsilon), middle[2], atol=1e-10)
    before = trim.evaluate(1.-epsilon)
    join = trim.update(rotation_y(-.04), np.eye(3), 1.)
    after = trim.evaluate(1.+epsilon)
    np.testing.assert_allclose(join[0], before[0], atol=1e-12)
    np.testing.assert_allclose(join[0], after[0], atol=1e-12)
    np.testing.assert_allclose(join[1:], 0., atol=1e-15)
    assert np.max(np.abs(before[2])) < 5e-6
    assert np.max(np.abs(after[2])) < 5e-6


def test_position_saturation_has_no_windup_or_reversal_delay():
    trim = make_trim(gain_per_s=10., max_rate_mps=1., max_acceleration_mps2=1.)
    trim.update(rotation_y(.4), np.eye(3), 0.)
    np.testing.assert_allclose(trim.integral, [0., .05, 0.], atol=1e-15)
    assert trim.saturated
    for now in range(1, 20):
        trim.update(rotation_y(.4), np.eye(3), float(now))
    np.testing.assert_allclose(trim.integral, [0., .05, 0.], atol=1e-15)
    trim.update(rotation_y(-.4), np.eye(3), 20.)
    assert trim.integral[1] < 0.
    assert trim.evaluate(20.2)[0][0, 2] < .01
    assert trim.evaluate(21.)[0][0, 2] < 0.


def test_zero_error_stops_learning_without_erasing_the_learned_bias():
    trim = make_trim(max_rate_mps=1., max_acceleration_mps2=1.)
    trim.update(rotation_y(.03), np.eye(3), 0.)
    integral = trim.integral.copy()
    trim.update(np.eye(3), np.eye(3), 1.)
    np.testing.assert_array_equal(trim.integral, integral)
    for now in (1., 1.5, 2.):
        np.testing.assert_array_equal(trim.evaluate(now)[1:], np.zeros((2, 3, 3)))


def test_constant_quasistatic_pitch_bias_decreases_with_the_executed_trim():
    trim = make_trim(gain_per_s=.1)
    observed_pitch = .04
    for second in range(100):
        # A positive learned front rise cancels the positive static tilt.
        executed_pitch = trim.evaluate(float(second))[0][0, 2] / .2
        observed_pitch = .04 - executed_pitch
        trim.update(rotation_y(observed_pitch), np.eye(3), float(second))
    assert 0. <= observed_pitch < .002
    assert trim.evaluate(100.)[0][0, 2] > 0.


def test_limiter_diagnostics_describe_the_requested_segment():
    trim = make_trim(gain_per_s=.1, max_rate_mps=.0001, max_acceleration_mps2=1.)
    trim.update(rotation_y(.1), np.eye(3), 0.)
    assert trim.rate_limited
    assert not trim.acceleration_limited
    assert not trim.saturated
    trim = make_trim(gain_per_s=.1, max_rate_mps=1., max_acceleration_mps2=.001)
    trim.update(rotation_y(.1), np.eye(3), 0.)
    assert not trim.rate_limited
    assert trim.acceleration_limited
    assert not trim.saturated


def test_gate_before_learning_does_not_lock_the_goal_or_build_integral():
    trim = make_trim()
    trim.update(rotation_y(.4), rotation_z(.2), 0., learning=False)
    assert not trim.initialized
    assert trim.desired_rotation is None
    np.testing.assert_array_equal(trim.integral, np.zeros(3))
    trim.update(rotation_y(.02), np.eye(3), 1.)
    assert trim.initialized
    assert trim.integral[1] > 0.


def test_returned_arrays_cannot_mutate_the_integral_or_cached_trajectory():
    trim = make_trim()
    trim.update(rotation_y(.2), np.eye(3), 0.)
    original = trim.evaluate(.4)
    trim.integral[:] = 10.
    trim.attitude_error[:] = 10.
    trim.desired_rotation[:] = 10.
    mutated = trim.evaluate(.4)
    for array in mutated:
        array[:] = 10.
    for actual, expected in zip(trim.evaluate(.4), original):
        np.testing.assert_array_equal(actual, expected)


def test_gate_freezes_integral_and_finishes_the_existing_segment_smoothly():
    trim = make_trim()
    trim.update(rotation_y(.4), np.eye(3), 0.)
    integral = trim.integral.copy()
    expected = trim.evaluate(.3)
    actual = trim.update(None, np.eye(3), .3, learning=False)
    for a, b in zip(actual, expected):
        np.testing.assert_array_equal(a, b)
    assert not trim.learning
    assert np.any(actual[1] != 0.)
    for now in (.9, 1., 5., 30.):
        trim.update(None, np.eye(3), now, learning=False)
        np.testing.assert_array_equal(trim.integral, integral)
    np.testing.assert_array_equal(trim.evaluate(30.)[1:], np.zeros((2, 3, 3)))


def test_freeze_latches_learning_until_reset_and_keeps_c2_motion():
    trim = make_trim()
    trim.update(rotation_y(.3), np.eye(3), 0.)
    integral = trim.integral.copy()
    expected = trim.evaluate(.4)
    trim.freeze()
    actual = trim.update(rotation_y(-.3), np.eye(3), .4)
    np.testing.assert_array_equal(trim.integral, integral)
    for a, b in zip(actual, expected):
        np.testing.assert_array_equal(a, b)
    trim.reset()
    np.testing.assert_array_equal(trim.evaluate(100.), np.zeros((3, 3, 3)))
    trim.update(rotation_y(-.3), np.eye(3), 0.)
    assert trim.integral[1] < 0.


def test_duplicate_timestamp_cannot_double_integrate_or_change_cached_error():
    trim = make_trim()
    expected = trim.update(rotation_y(.4), np.eye(3), 0.)
    integral, error = trim.integral.copy(), trim.attitude_error.copy()
    actual = trim.update(rotation_y(-.4), np.eye(3), 0.)
    np.testing.assert_array_equal(trim.integral, integral)
    np.testing.assert_array_equal(trim.attitude_error, error)
    np.testing.assert_array_equal(actual, expected)


def test_read_only_evaluation_does_not_consume_adaptation_or_clock_gaps():
    trim = make_trim(gain_per_s=.1, max_rate_mps=1., max_acceleration_mps2=1.)
    trim.update(rotation_y(.02), np.eye(3), 0.)
    integral = trim.integral.copy()
    trim.evaluate(1000.)
    np.testing.assert_array_equal(trim.integral, integral)
    trim.update(rotation_y(.02), np.eye(3), 100.)
    np.testing.assert_allclose(trim.integral, [0., .004, 0.], atol=1e-15)


def test_disabled_trim_never_requires_attitude_or_generates_a_reference():
    trim = make_trim(enabled=False)
    np.testing.assert_array_equal(trim.update(None, None, 0.), np.zeros((3, 3, 3)))
    np.testing.assert_array_equal(trim.update(rotation_y(.4), np.eye(3), 100.),
                                  np.zeros((3, 3, 3)))
    np.testing.assert_array_equal(trim.integral, np.zeros(3))
    assert not trim.learning


def test_principal_log_handles_identity_large_angles_and_pi():
    for angle in (0., 1e-10, .8, np.pi):
        trim = make_trim(gain_per_s=.001, max_rate_mps=1., max_acceleration_mps2=1.)
        trim.update(rotation_y(angle), np.eye(3), 0.)
        np.testing.assert_allclose(trim.attitude_error, [0., angle, 0.], atol=1e-14)


@pytest.mark.parametrize("name,value", [("gain_per_s", -1.), ("gain_per_s", [1., 2.]),
    ("gain_per_s", [1., np.nan, 2.]), ("gain_per_s", "invalid"),
    ("max_offset_m", 0.), ("max_offset_m", np.inf), ("max_rate_mps", -.1),
    ("max_acceleration_mps2", np.nan), ("adaptation_interval_s", 0.),
    ("adaptation_interval_s", [1.]), ("enabled", "false")])
def test_invalid_parameters_are_rejected(name, value):
    with pytest.raises(ValueError):
        make_trim(**{name: value})


@pytest.mark.parametrize("name,value", [("gain_per_s", True), ("gain_per_s", False),
    ("gain_per_s", [.1, True, .2]), ("gain_per_s", np.array([False, True, False])),
    ("max_offset_m", True), ("max_rate_mps", True),
    ("max_acceleration_mps2", np.bool_(True)), ("adaptation_interval_s", True),
    ("gain_per_s", ".1"), ("gain_per_s", [.1, ".2", .3]),
    ("max_offset_m", ".01"), ("max_rate_mps", ".001"),
    ("max_acceleration_mps2", ".001"), ("adaptation_interval_s", "1.")])
def test_booleans_and_numeric_strings_cannot_silently_become_numeric_parameters(name, value):
    with pytest.raises(ValueError):
        make_trim(**{name: value})


def test_numeric_zero_gain_is_valid_and_does_not_learn():
    trim = make_trim(gain_per_s=[0., 0, np.float64(0)])
    trim.update(rotation_y(.1), np.eye(3), 0.)
    np.testing.assert_array_equal(trim.integral, np.zeros(3))
    np.testing.assert_array_equal(trim.evaluate(1.), np.zeros((3, 3, 3)))


@pytest.mark.parametrize("attachments", [[], np.zeros((3, 3)), [[0., 1., 2.],
    [0., 0., 0.], [0., 0., 0.]], [[0., 1.], [0., 0.], [0., 0.]],
    [[0., np.nan, 0.], [0., 1., 0.], [0., 0., 1.]], [["bad"] * 3] * 3])
def test_unobservable_or_invalid_attachment_geometry_is_rejected(attachments):
    cls = make_trim().__class__
    with pytest.raises(ValueError):
        cls(attachments)


@pytest.mark.parametrize("bad", [np.zeros((3, 3)), np.diag([1., 1., -1.]),
    np.eye(2), [[np.nan, 0., 0.], [0., 1., 0.], [0., 0., 1.]], "bad"])
def test_invalid_rotation_does_not_mutate_integral_or_consume_timestamp(bad):
    trim = make_trim()
    trim.update(rotation_y(.02), np.eye(3), 0.)
    integral = trim.integral.copy()
    for measured, desired in ((bad, np.eye(3)), (np.eye(3), bad)):
        with pytest.raises(ValueError):
            trim.update(measured, desired, 1.)
        np.testing.assert_array_equal(trim.integral, integral)
    trim.update(rotation_y(.02), np.eye(3), 1.)
    assert trim.integral[1] > integral[1]


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), "bad", [1.], -1.])
def test_invalid_or_backward_time_does_not_mutate_integral(bad):
    trim = make_trim()
    trim.update(rotation_y(.02), np.eye(3), 0.)
    integral = trim.integral.copy()
    with pytest.raises(ValueError):
        trim.update(rotation_y(.02), np.eye(3), bad)
    np.testing.assert_array_equal(trim.integral, integral)


def test_changing_the_fixed_goal_requires_explicit_reset():
    trim = make_trim()
    trim.update(rotation_y(.02), np.eye(3), 0.)
    integral = trim.integral.copy()
    with pytest.raises(ValueError):
        trim.update(rotation_y(.02), rotation_z(.2), 1.)
    np.testing.assert_array_equal(trim.integral, integral)
    trim.reset()
    trim.update(rotation_z(.2) @ rotation_y(.02), rotation_z(.2), 0.)
    np.testing.assert_allclose(trim.desired_rotation, rotation_z(.2), atol=1e-15)
