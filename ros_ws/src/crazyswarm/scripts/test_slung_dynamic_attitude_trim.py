"""Moving attitude goals must not rotate or differentiate learned world offsets."""

import numpy as np
import pytest

import slung_attitude_trim


ATTACHMENTS = np.array([[.2, -.1, -.1], [0., .1, -.1], [0., 0., 0.]])


def rotation_x(angle):
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[1., 0., 0.], [0., cosine, -sine], [0., sine, cosine]])


def rotation_y(angle):
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[cosine, 0., sine], [0., 1., 0.], [-sine, 0., cosine]])


def rotation_z(angle):
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[cosine, -sine, 0.], [sine, cosine, 0.], [0., 0., 1.]])


def make_trim(**kwargs):
    cls = getattr(slung_attitude_trim, "SlungDynamicAttitudeReferenceTrim", None)
    assert cls is not None, "dynamic attitude reference trim has not been implemented"
    return cls(ATTACHMENTS, **kwargs)


def test_full_so3_goal_maps_body_error_into_world_offset_and_world_integral():
    trim = make_trim(gain_per_s=[.1, .2, .3], max_rate_mps=1.,
                     max_acceleration_mps2=1.)
    desired = rotation_z(np.pi / 2.) @ rotation_y(np.pi / 2.)
    np.testing.assert_array_equal(
        trim.update(desired @ rotation_y(.02), desired, 0.), np.zeros((3, 3, 3)))
    position, velocity, acceleration = trim.evaluate(1.)
    np.testing.assert_allclose(position, [[0., .0008, 0.], [0., -.0004, 0.],
                                         [0., -.0004, 0.]], atol=1e-15)
    np.testing.assert_allclose(trim.attitude_error, [0., .02, 0.], atol=1e-15)
    np.testing.assert_allclose(trim.integral, [-.004, 0., 0.], atol=1e-15)
    np.testing.assert_array_equal(velocity, np.zeros((3, 3)))
    np.testing.assert_array_equal(acceleration, np.zeros((3, 3)))


def test_new_goal_adds_world_increment_without_reorienting_previous_offset():
    trim = make_trim(max_rate_mps=1., max_acceleration_mps2=1.)
    trim.update(rotation_y(.02), np.eye(3), 0.)
    desired = rotation_z(np.pi)
    before = trim.evaluate(.4)
    actual = trim.update(desired @ rotation_z(.04), desired, .4)
    for values, expected in zip(actual, before):
        np.testing.assert_array_equal(values, expected)
    np.testing.assert_allclose(trim.desired_rotation, desired, atol=1e-15)
    trim.update(desired @ rotation_z(.04), desired, 1.)
    np.testing.assert_allclose(trim.evaluate(2.)[0],
        [[0., .0008, .0004], [-.0004, -.0004, -.0002],
         [.0004, -.0004, -.0002]], atol=1e-15)
    np.testing.assert_allclose(trim.integral, [0., .002, .004], atol=1e-15)


@pytest.mark.parametrize("perfect_tracking", [True, False])
def test_moving_goal_alone_never_creates_a_trim(perfect_tracking):
    trim = make_trim(gain_per_s=.1 if perfect_tracking else 0.)
    for now in np.linspace(0., 8., 161):
        desired = rotation_z(.7 * now) @ rotation_y(.3 * np.sin(now)) @ rotation_x(.2)
        measured = desired if perfect_tracking else desired @ rotation_y(.1)
        np.testing.assert_array_equal(trim.update(measured, desired, now),
                                      np.zeros((3, 3, 3)))
        np.testing.assert_array_equal(trim.integral, np.zeros(3))


def test_zero_error_preserves_learned_world_offset_while_goal_moves():
    trim = make_trim(max_rate_mps=1., max_acceleration_mps2=1.)
    trim.update(rotation_y(.02), np.eye(3), 0.)
    expected = trim.evaluate(1.)
    integral = trim.integral.copy()
    for now in (1., 1.4, 2., 3., 5.):
        desired = rotation_z(now) @ rotation_y(.6 * now)
        actual = trim.update(desired, desired, now)
        for values, reference in zip(actual, expected):
            np.testing.assert_array_equal(values, reference)
        np.testing.assert_array_equal(trim.integral, integral)


@pytest.mark.parametrize("limits", [dict(max_rate_mps=.0001, max_acceleration_mps2=1.),
                                    dict(max_rate_mps=.1, max_acceleration_mps2=.001),
                                    dict(max_rate_mps=.001, max_acceleration_mps2=.001)])
@pytest.mark.parametrize("duration", [.5, 1., 2.])
def test_moving_goals_obey_all_bounds_for_entire_segments(limits, duration):
    trim = make_trim(gain_per_s=10., max_offset_m=.002,
                     adaptation_interval_s=duration, **limits)
    critical_fractions = [0., (3. - np.sqrt(3.)) / 6., .5,
                          (3. + np.sqrt(3.)) / 6., 1.]
    for segment in range(60):
        now = segment * duration
        desired = rotation_z(.6 * segment) @ rotation_y(.4 * np.sin(.3 * segment))
        trim.update(desired @ rotation_y(.4) @ rotation_z(.2), desired, now)
        for fraction in critical_fractions:
            position, velocity, acceleration = trim.evaluate(now + fraction * duration)
            assert np.max(np.linalg.norm(position, axis=1)) <= .002 + 1e-14
            assert np.max(np.linalg.norm(velocity, axis=1)) <= limits["max_rate_mps"] + 1e-14
            assert np.max(np.linalg.norm(acceleration, axis=1)) <= limits["max_acceleration_mps2"] + 1e-14
            for values in (position, velocity, acceleration):
                np.testing.assert_allclose(values.sum(axis=0), 0., atol=1e-14)


def test_changing_goals_keeps_analytic_derivatives_and_c2_segment_joins():
    trim = make_trim(max_rate_mps=1., max_acceleration_mps2=1.)
    epsilon = 1e-4
    for second in range(4):
        desired = rotation_z(.9 * second) @ rotation_x(.2 * second)
        before = trim.evaluate(float(second))
        join = trim.update(desired @ rotation_y(.03), desired, float(second))
        for values, expected in zip(join, before):
            np.testing.assert_array_equal(values, expected)
        for fraction in (.12, .5, .87):
            now = second + fraction
            new_desired = rotation_z(.9 * now) @ rotation_x(.2 * now)
            trim.update(new_desired @ rotation_y(.03), new_desired, now)
            left, middle, right = [trim.evaluate(t) for t in
                                   (now - epsilon, now, now + epsilon)]
            np.testing.assert_allclose((right[0] - left[0]) / (2. * epsilon),
                                       middle[1], atol=1e-10)
            np.testing.assert_allclose((right[1] - left[1]) / (2. * epsilon),
                                       middle[2], atol=1e-10)
        np.testing.assert_array_equal(trim.evaluate(second + 1.)[1:],
                                      np.zeros((2, 3, 3)))
        assert np.max(np.abs(trim.evaluate(second + 1. - epsilon)[2])) < 5e-6


def test_projection_limits_endpoint_without_world_integral_windup():
    trim = make_trim(gain_per_s=10., max_rate_mps=1., max_acceleration_mps2=1.)
    trim.update(rotation_y(.4), np.eye(3), 0.)
    np.testing.assert_allclose(trim.integral, [0., .05, 0.], atol=1e-15)
    for now in range(1, 10):
        trim.update(rotation_y(.4), np.eye(3), float(now))
        np.testing.assert_allclose(trim.integral, [0., .05, 0.], atol=1e-15)
    # The same world angular increment acts on different attachment orientations.
    # Its diagnostic sum is therefore not a rotation that reconstructs the offset.
    desired = rotation_y(np.pi)
    trim.update(desired @ rotation_y(.4), desired, 10.)
    np.testing.assert_allclose(trim.evaluate(11.)[0][:, 2], [-.01, .005, .005],
                               atol=1e-15)
    np.testing.assert_allclose(trim.integral, [0., .051265822784810126, 0.],
                               atol=1e-15)
    assert trim.saturated


def test_limit_diagnostics_and_world_integral_follow_the_accepted_step():
    trim = make_trim(max_rate_mps=.0001, max_acceleration_mps2=1.)
    desired = rotation_z(np.pi / 2.)
    trim.update(desired @ rotation_y(.1), desired, 0.)
    assert trim.rate_limited and not trim.acceleration_limited and not trim.saturated
    np.testing.assert_allclose(trim.integral, [-.0002666666666666667, 0., 0.],
                               atol=1e-15)
    assert np.max(np.linalg.norm(trim.evaluate(.5)[1], axis=1)) == pytest.approx(.0001)
    trim = make_trim(max_rate_mps=1., max_acceleration_mps2=.001)
    trim.update(rotation_y(.1), np.eye(3), 0.)
    assert trim.acceleration_limited and not trim.rate_limited and not trim.saturated


@pytest.mark.parametrize("latch", [False, True])
def test_freeze_finishes_existing_world_segment_even_as_goal_moves(latch):
    trim = make_trim()
    trim.update(rotation_y(.4), np.eye(3), 0.)
    integral, endpoint = trim.integral.copy(), trim.evaluate(1.)
    if latch:
        trim.freeze()
    for now in (.3, .8, 1., 2., 30.):
        expected = trim.evaluate(now)
        desired = rotation_z(now) @ rotation_y(now)
        actual = trim.update(desired @ rotation_y(-.4), desired, now,
                             learning=latch)
        for values, reference in zip(actual, expected):
            np.testing.assert_array_equal(values, reference)
        np.testing.assert_array_equal(trim.integral, integral)
        assert not trim.learning
    for values, reference in zip(trim.evaluate(30.), endpoint):
        np.testing.assert_array_equal(values, reference)


def test_same_tick_keeps_output_goal_error_and_integral_unchanged():
    trim = make_trim()
    expected = trim.update(rotation_y(.4), np.eye(3), 0.)
    integral, error, desired = trim.integral, trim.error, trim.goal_rotation
    different_goal = rotation_z(.7) @ rotation_y(-.3)
    actual = trim.update(different_goal @ rotation_y(-.4), different_goal, 0.,
                         learning=False)
    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(trim.integral, integral)
    np.testing.assert_array_equal(trim.error, error)
    np.testing.assert_array_equal(trim.goal_rotation, desired)


def test_gated_start_gap_and_read_only_evaluation_do_not_add_learning_steps():
    trim = make_trim(max_rate_mps=1., max_acceleration_mps2=1.)
    trim.update(rotation_y(.4), rotation_z(.2), 0., learning=False)
    assert not trim.initialized and trim.desired_rotation is None
    trim.update(rotation_y(.02), np.eye(3), 1.)
    integral = trim.integral.copy()
    trim.evaluate(1000.)
    np.testing.assert_array_equal(trim.integral, integral)
    trim.update(rotation_y(.02), np.eye(3), 100.)
    np.testing.assert_allclose(trim.integral, [0., .004, 0.], atol=1e-15)


def test_attachment_centroid_does_not_affect_dynamic_world_learning():
    original = make_trim()
    shifted = original.__class__(ATTACHMENTS + np.array([[1.], [-2.], [3.]]))
    for now in (0., 1., 2.):
        desired = rotation_z(now) @ rotation_x(.4)
        for trim in (original, shifted):
            trim.update(desired @ rotation_y(.05), desired, now)
        for values, expected in zip(shifted.evaluate(now + .4), original.evaluate(now + .4)):
            np.testing.assert_allclose(values, expected, atol=1e-15)


def test_reset_and_defensive_copies_preserve_the_base_api():
    trim = make_trim()
    desired = rotation_z(.7)
    trim.update(desired @ rotation_y(.1), desired, 0.)
    expected = trim.evaluate(.4)
    trim.integral[:] = 10.
    trim.error[:] = 10.
    trim.goal_rotation[:] = 10.
    for values in trim.update(None, desired, .4, learning=False):
        values[:] = 10.
    for values, reference in zip(trim.evaluate(.4), expected):
        np.testing.assert_array_equal(values, reference)
    trim.freeze()
    trim.reset()
    assert not trim.initialized and not trim.frozen
    np.testing.assert_array_equal(trim.evaluate(100.), np.zeros((3, 3, 3)))
    trim.update(rotation_y(-.1), np.eye(3), 0.)
    assert trim.integral[1] < 0.


def test_disabled_dynamic_trim_needs_no_attitudes_and_never_generates_a_reference():
    trim = make_trim(enabled=False)
    np.testing.assert_array_equal(trim.update(None, None, 0.), np.zeros((3, 3, 3)))
    np.testing.assert_array_equal(trim.update(rotation_y(.4), rotation_z(.8), 10.),
                                  np.zeros((3, 3, 3)))
    assert not trim.learning and not trim.initialized


@pytest.mark.parametrize("bad", [np.zeros((3, 3)), np.diag([1., 1., -1.]),
    [[np.nan, 0., 0.], [0., 1., 0.], [0., 0., 1.]], "bad"])
def test_bad_rotation_is_rejected_without_consuming_the_tick_or_changing_state(bad):
    trim = make_trim()
    trim.update(rotation_y(.02), np.eye(3), 0.)
    integral, error, desired = trim.integral, trim.error, trim.goal_rotation
    expected = trim.evaluate(1.)
    for measured, goal in ((bad, np.eye(3)), (np.eye(3), bad)):
        for now in (0., 1.):
            with pytest.raises(ValueError):
                trim.update(measured, goal, now)
            np.testing.assert_array_equal(trim.integral, integral)
            np.testing.assert_array_equal(trim.error, error)
            np.testing.assert_array_equal(trim.goal_rotation, desired)
            np.testing.assert_array_equal(trim.evaluate(1.), expected)
    trim.update(rotation_y(.02), rotation_z(.1), 1.)
    assert trim.integral[1] > integral[1]


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), "bad", [1.], -1., .2])
def test_invalid_or_backward_time_keeps_the_previous_dynamic_segment(bad):
    trim = make_trim()
    trim.update(rotation_y(.02), np.eye(3), .3)
    integral, expected = trim.integral, trim.evaluate(.8)
    with pytest.raises(ValueError):
        trim.update(rotation_y(.02), rotation_z(.2), bad)
    np.testing.assert_array_equal(trim.integral, integral)
    np.testing.assert_array_equal(trim.evaluate(.8), expected)


def test_nonfinite_requested_increment_does_not_consume_the_learning_tick():
    trim = make_trim(gain_per_s=np.finfo(float).max)
    trim.update(np.eye(3), np.eye(3), 0.)
    with pytest.raises(ValueError):
        trim.update(rotation_y(np.pi), np.eye(3), 1.)
    np.testing.assert_array_equal(trim.integral, np.zeros(3))
    np.testing.assert_array_equal(trim.error, np.zeros(3))
    trim.update(np.eye(3), rotation_z(.1), 1., learning=False)
