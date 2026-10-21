"""ROS-free regressions for the transport differential hold-force integral."""

import numpy as np
import pytest

from slung_transport_integral import SlungDifferentialHoldIntegral


def test_common_position_error_does_not_change_fleet_force():
    integral = SlungDifferentialHoldIntegral()
    errors = np.tile([0.30, -0.12, 0.21], (3, 1))
    bias, rate = integral.update(errors, 0.02)

    np.testing.assert_allclose(bias, 0.0, atol=1e-16)
    np.testing.assert_allclose(rate, 0.0, atol=1e-16)
    np.testing.assert_allclose(integral.differential_error_m, 0.0, atol=1e-16)


def test_relative_position_error_learns_opposing_force_in_each_axis():
    integral = SlungDifferentialHoldIntegral(
        gain_npm_s=0.2, limit_n=0.5, rate_limit_nps=1.0)
    errors = np.array([[0.3, 0.1, 0.4], [0.2, 0.3, 0.2], [0.1, 0.2, 0.3]])
    bias, rate = integral.update(errors, 0.5)

    np.testing.assert_allclose(
        integral.differential_error_m,
        [[0.1, -0.1, 0.1], [0.0, 0.1, -0.1], [-0.1, 0.0, 0.0]], atol=1e-15)
    np.testing.assert_allclose(
        rate, [[-0.02, 0.02, -0.02], [0.0, -0.02, 0.02], [0.02, 0.0, 0.0]],
        atol=1e-15)
    np.testing.assert_allclose(bias, rate * 0.5, atol=1e-15)


def test_force_bound_scales_the_fleet_together_without_changing_total_force():
    integral = SlungDifferentialHoldIntegral(gain_npm_s=1.0)
    bias, rate = integral.update([[-3, 0, 0], [1, 0, 0], [2, 0, 0]], 20.0)

    np.testing.assert_allclose(bias, [[0.03, 0, 0], [-0.01, 0, 0], [-0.02, 0, 0]])
    np.testing.assert_allclose(rate, [[0.0015, 0, 0], [-0.0005, 0, 0], [-0.001, 0, 0]])


def test_rate_bound_is_reapplied_after_joint_force_projection():
    integral = SlungDifferentialHoldIntegral()
    directions = np.array([[1, 0, 0], [-0.5, np.sqrt(3) / 2, 0],
                           [-0.5, -np.sqrt(3) / 2, 0]])
    previous, _ = integral.update(-directions, 15.0)
    np.testing.assert_allclose(np.linalg.norm(previous, axis=1), 0.03)

    # Outward motion of the first row contracts the entire proposed fleet.
    # Without a second rate bound, this contraction moves the second row
    # inward by more than 0.002 N during this one-second step.
    errors = np.array([[-1, 0, 0], [-0.5, np.sqrt(3) / 2, 0],
                       [1.5, -np.sqrt(3) / 2, 0]])
    bias, rate = integral.update(errors, 1.0)

    assert np.isclose(np.max(np.linalg.norm(rate, axis=1)), 0.002)
    assert np.max(np.linalg.norm(bias, axis=1)) <= 0.03 + 1e-14
    np.testing.assert_allclose(rate, bias - previous, atol=1e-15)
    np.testing.assert_allclose(bias.sum(axis=0), 0.0, atol=1e-14)
    np.testing.assert_allclose(rate.sum(axis=0), 0.0, atol=1e-14)


def test_small_positive_limits_do_not_introduce_an_implicit_dead_zone():
    integral = SlungDifferentialHoldIntegral(
        gain_npm_s=1e-40, limit_n=1e-40, rate_limit_nps=1e-40)
    bias, rate = integral.update([[-1, 0, 0], [0, 0, 0], [1, 0, 0]], 0.5)

    np.testing.assert_allclose(bias, [[5e-41, 0, 0], [0, 0, 0], [-5e-41, 0, 0]],
                               atol=0.0)
    np.testing.assert_allclose(rate, [[1e-40, 0, 0], [0, 0, 0], [-1e-40, 0, 0]],
                               atol=0.0)


def test_large_changing_errors_preserve_force_rate_and_zero_sum_bounds_each_step():
    integral = SlungDifferentialHoldIntegral()
    rng = np.random.default_rng(138)
    previous = np.zeros((3, 3))
    for step in range(800):
        errors = rng.normal(size=(3, 3)) * (100 if step % 2 else 0.5)
        dt = (0.001, 0.02, 0.25, 4.0)[step % 4]
        bias, rate = integral.update(errors, dt)

        assert np.max(np.linalg.norm(bias, axis=1)) <= 0.03 + 1e-14
        assert np.max(np.linalg.norm(rate, axis=1)) <= 0.002 + 1e-14
        np.testing.assert_allclose(bias.sum(axis=0), 0.0, atol=1e-13)
        np.testing.assert_allclose(rate.sum(axis=0), 0.0, atol=1e-13)
        np.testing.assert_allclose(rate, (bias - previous) / dt, atol=1e-13)
        previous = bias


def test_freeze_holds_learned_bias_and_returns_zero_rate():
    integral = SlungDifferentialHoldIntegral()
    learned, _ = integral.update([[-1, 0, 0], [0, 0, 0], [1, 0, 0]], 1.0)

    frozen, rate = integral.update([[2, 1, 0], [-1, 1, 0], [-1, 1, 0]], 0.5,
                                   learning=False)
    np.testing.assert_array_equal(frozen, learned)
    np.testing.assert_array_equal(rate, np.zeros((3, 3)))
    np.testing.assert_allclose(integral.differential_error_m,
                               [[2, 0, 0], [-1, 0, 0], [-1, 0, 0]])
    frozen, rate = integral.update(None, 0.5, learning=False)
    np.testing.assert_array_equal(frozen, learned)
    np.testing.assert_array_equal(rate, np.zeros((3, 3)))


def test_returned_arrays_and_error_snapshot_do_not_alias_callers():
    integral = SlungDifferentialHoldIntegral()
    errors = np.array([[-1.0, 0, 0], [0, 0, 0], [1, 0, 0]])
    bias, rate = integral.update(errors, 1.0)
    learned = bias.copy()
    errors[:] = 8.0
    bias[:] = 7.0
    rate[:] = 6.0

    frozen, frozen_rate = integral.update(None, 1.0, learning=False)
    np.testing.assert_array_equal(frozen, learned)
    np.testing.assert_array_equal(frozen_rate, np.zeros((3, 3)))
    np.testing.assert_allclose(integral.differential_error_m,
                               [[-1, 0, 0], [0, 0, 0], [1, 0, 0]])


def test_reset_clears_bias_and_error_before_reuse():
    integral = SlungDifferentialHoldIntegral()
    integral.update([[-1, 0, 0], [0, 0, 0], [1, 0, 0]], 5.0)
    integral.reset()

    np.testing.assert_array_equal(integral.bias, np.zeros((3, 3)))
    np.testing.assert_array_equal(integral.differential_error_m, np.zeros((3, 3)))
    bias, rate = integral.update(np.zeros((3, 3)), 1.0)
    np.testing.assert_array_equal(bias, np.zeros((3, 3)))
    np.testing.assert_array_equal(rate, np.zeros((3, 3)))


@pytest.mark.parametrize("name", ["gain_npm_s", "limit_n", "rate_limit_nps"])
@pytest.mark.parametrize("value", [0.0, -0.1, np.nan, np.inf, -np.inf, None, "bad"])
def test_parameters_require_finite_positive_values(name, value):
    with pytest.raises(ValueError):
        SlungDifferentialHoldIntegral(**{name: value})


@pytest.mark.parametrize("learning", [True, False])
@pytest.mark.parametrize("dt", [0.0, -0.1, np.nan, np.inf, -np.inf, None, "bad"])
def test_updates_require_finite_positive_dt_even_while_frozen(dt, learning):
    integral = SlungDifferentialHoldIntegral()
    with pytest.raises(ValueError):
        integral.update(np.zeros((3, 3)), dt, learning=learning)


@pytest.mark.parametrize("learning", [True, False])
@pytest.mark.parametrize("errors", [
    np.zeros(9), np.zeros((1, 9)), np.zeros((3, 2)), np.zeros((4, 3)),
    np.zeros((3, 3, 1)), 0.0, [], np.full((3, 3), np.nan),
    [[0, 0, 0], [0, np.inf, 0], [0, 0, 0]],
])
def test_provided_errors_require_exact_finite_three_by_three_shape(errors, learning):
    integral = SlungDifferentialHoldIntegral()
    with pytest.raises(ValueError):
        integral.update(errors, 0.02, learning=learning)


def test_learning_requires_errors_instead_of_reusing_a_stale_sample():
    integral = SlungDifferentialHoldIntegral()
    integral.update([[-1, 0, 0], [0, 0, 0], [1, 0, 0]], 0.02)
    with pytest.raises(ValueError):
        integral.update(None, 0.02)


def test_invalid_update_does_not_change_learned_state():
    integral = SlungDifferentialHoldIntegral()
    learned, _ = integral.update([[-1, 0, 0], [0, 0, 0], [1, 0, 0]], 0.02)
    previous_error = integral.differential_error_m.copy()
    with pytest.raises(ValueError):
        integral.update(np.ones((3, 3)), -1.0)

    np.testing.assert_array_equal(integral.bias, learned)
    np.testing.assert_array_equal(integral.differential_error_m, previous_error)


def test_feasible_static_force_bias_converges_to_smaller_relative_pose_error():
    integral = SlungDifferentialHoldIntegral()
    stiffness_npm = 0.4
    response_time_s = 0.3
    dt = 0.05
    disturbance_n = np.array([[0.012, -0.006, 0.004], [-0.004, 0.010, -0.002],
                              [-0.008, -0.004, -0.002]])
    common_error_m = np.array([0.02, -0.01, 0.015])
    errors = common_error_m + disturbance_n / stiffness_npm
    initial_relative_rms = np.sqrt(np.mean((errors - errors.mean(axis=0)) ** 2))

    # An overdamped position loop relaxes toward force/stiffness equilibrium.
    # The real integral supplies a slowly learned force correction each step.
    for _ in range(2400):
        bias, _ = integral.update(errors, dt)
        equilibrium_m = common_error_m + (disturbance_n + bias) / stiffness_npm
        errors += dt / response_time_s * (equilibrium_m - errors)

    final_relative_rms = np.sqrt(np.mean((errors - errors.mean(axis=0)) ** 2))
    assert final_relative_rms < 0.01 * initial_relative_rms
    np.testing.assert_allclose(bias, -disturbance_n, atol=1e-4)
    np.testing.assert_allclose(errors.mean(axis=0), common_error_m, atol=1e-13)
