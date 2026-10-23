"""General SO(3) braking keeps rotation, body rate, and angular acceleration coherent."""

import importlib

import numpy as np
import pytest

from slung_reference import _rotation_exp


def _hat(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def _brake(rotation=None, rate=None, acceleration=None, start=10., duration=5.):
    module = importlib.import_module("slung_circle_reference")
    return module.AttitudeBrakeReference(
        _rotation_exp(np.array([.21, -.32, .47])) if rotation is None else rotation,
        [.35, -.21, .16] if rate is None else rate,
        [.08, .11, -.06] if acceleration is None else acceleration,
        start, duration)


def test_initial_state_is_preserved_before_and_at_start():
    rotation = _rotation_exp(np.array([.21, -.32, .47]))
    reference = _brake(rotation=rotation)
    for now in (0., 10.):
        sample = reference.evaluate(now)
        np.testing.assert_array_equal(sample["rotation"], rotation)
        np.testing.assert_array_equal(sample["body_rate"], [.35, -.21, .16])
        np.testing.assert_array_equal(sample["body_rate_dot"], [.08, .11, -.06])


def test_final_rotation_uses_the_bounded_rotation_vector_endpoint_and_zero_rates():
    initial = _rotation_exp(np.array([.21, -.32, .47]))
    reference = _brake(rotation=initial)
    endpoint = 2.5*np.array([.35, -.21, .16])+(25./12.)*np.array([.08, .11, -.06])
    expected = initial@_rotation_exp(endpoint)
    for now in (15., 20., 1.e9):
        sample = reference.evaluate(now)
        np.testing.assert_allclose(sample["rotation"], expected, atol=1e-14)
        np.testing.assert_array_equal(sample["body_rate"], np.zeros(3))
        np.testing.assert_array_equal(sample["body_rate_dot"], np.zeros(3))


@pytest.mark.parametrize("scale", [1.e-8, 1., 8.])
@pytest.mark.parametrize("elapsed", [.0001, .4, 1.7, 3.3, 4.9999])
def test_noncollinear_rotation_body_rates_and_acceleration_match_derivatives(scale, elapsed):
    reference = _brake(rate=scale*np.array([.35, -.21, .16]),
                       acceleration=scale*np.array([.08, .11, -.06]))
    now, step = 10.+elapsed, 1.e-5
    before, sample, after = [reference.evaluate(t) for t in (now-step, now, now+step)]
    np.testing.assert_allclose((after["rotation"]-before["rotation"])/(2.*step),
                               sample["rotation"]@_hat(sample["body_rate"]),
                               atol=3e-8, rtol=2e-7)
    np.testing.assert_allclose((after["body_rate"]-before["body_rate"])/(2.*step),
                               sample["body_rate_dot"], atol=4e-8, rtol=2e-7)
    np.testing.assert_allclose(sample["rotation"].T@sample["rotation"], np.eye(3), atol=2e-14)
    assert np.linalg.det(sample["rotation"]) == pytest.approx(1., abs=2e-14)


def test_reference_reaches_initial_and_final_states_with_c2_continuity():
    reference = _brake()
    step = 1.e-5
    initial, following, second = [reference.evaluate(10.+offset*step) for offset in (0., 1., 2.)]
    np.testing.assert_allclose((-3.*initial["rotation"]+4.*following["rotation"]-second["rotation"])/(2.*step),
                               initial["rotation"]@_hat(initial["body_rate"]), atol=1e-9)
    np.testing.assert_allclose((-3.*initial["body_rate"]+4.*following["body_rate"]-second["body_rate"])/(2.*step),
                               initial["body_rate_dot"], atol=1e-9)
    preceding, final = reference.evaluate(15.-step), reference.evaluate(15.)
    np.testing.assert_allclose(preceding["rotation"], final["rotation"], atol=1e-12)
    np.testing.assert_allclose(preceding["body_rate"], final["body_rate"], atol=2e-11)
    np.testing.assert_allclose(preceding["body_rate_dot"], final["body_rate_dot"], atol=2e-6)


@pytest.mark.parametrize("rate,acceleration", [(.2, 0.), (-.2, .03), (.1, -.08)])
def test_yaw_only_brake_matches_the_existing_scalar_quintic(rate, acceleration):
    from slung_landing import SlungLandingProfile
    yaw, duration = -.8, 5.
    rotation = _rotation_exp(np.array([0., 0., yaw]))
    reference = _brake(rotation=rotation, rate=[0., 0., rate],
                       acceleration=[0., 0., acceleration])
    endpoint = yaw+.5*duration*rate+duration**2*acceleration/12.
    coefficients = SlungLandingProfile._quintic_coefficients(
        np.array([yaw]), np.array([rate]), np.array([acceleration]),
        np.array([endpoint]), duration)
    for elapsed in np.linspace(0., duration, 31):
        angle, speed, alpha = SlungLandingProfile._sample_segment(
            [elapsed/duration], duration, coefficients)
        sample = reference.evaluate(10.+elapsed)
        np.testing.assert_allclose(sample["rotation"], _rotation_exp(np.array([0., 0., angle[0, 0]])),
                                   atol=2e-14)
        np.testing.assert_allclose(sample["body_rate"], [0., 0., speed[0, 0]], atol=2e-14)
        np.testing.assert_allclose(sample["body_rate_dot"], [0., 0., alpha[0, 0]], atol=2e-14)


@pytest.mark.parametrize("rotation", [np.zeros((3, 3)), np.diag([1., 1., -1.]),
                                        np.eye(2), np.full((3, 3), np.nan)])
def test_invalid_initial_rotations_are_rejected(rotation):
    with pytest.raises(ValueError):
        _brake(rotation=rotation)


@pytest.mark.parametrize("argument,value", [("rate", [0., 0.]), ("rate", [np.inf, 0., 0.]),
    ("rate", [True, 0., 0.]), ("acceleration", [0., np.nan, 0.]),
    ("acceleration", [[0., 0., 0.]]), ("start", np.inf), ("start", True),
    ("duration", 0.), ("duration", -1.), ("duration", np.nan), ("duration", True)])
def test_invalid_initial_kinematics_and_timing_are_rejected(argument, value):
    with pytest.raises(ValueError):
        _brake(**{argument: value})


def test_inputs_and_returned_values_are_independent_and_evaluation_time_is_validated():
    rotation = _rotation_exp(np.array([.1, .2, .3]))
    rate, acceleration = np.array([.2, -.1, .3]), np.array([.1, .2, -.1])
    reference = _brake(rotation=rotation, rate=rate, acceleration=acceleration)
    expected = reference.evaluate(11.)
    rotation[:] = 99.
    rate[:] = 99.
    acceleration[:] = 99.
    first = reference.evaluate(11.)
    for name, value in first.items():
        np.testing.assert_array_equal(value, expected[name])
        value[:] = 99.
    for name, value in reference.evaluate(11.).items():
        np.testing.assert_array_equal(value, expected[name])
    with pytest.raises(ValueError):
        reference.evaluate(np.nan)


@pytest.mark.parametrize("argument,value", [("duration", 1.e200), ("rate", [1.e200, 0., 0.])])
def test_unrepresentable_rotation_profile_is_rejected_at_construction(argument, value):
    with pytest.raises(ValueError):
        _brake(**{argument: value})
