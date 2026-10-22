"""Physical handoff regressions, independent of ROS transports."""

import numpy as np

from slung_transport_guard import SlungTransportGuard, SlungSupportTrim


def test_closing_pair_triggers_before_actual_distance_boundary():
    guard = SlungTransportGuard(min_distance_m=0.30, prediction_s=0.35, confirm_s=0.10)
    positions = np.array([[0.40, 0, 1], [0, 0, 1], [0, 1, 1]])
    velocities = np.array([[-0.4, 0, 0], [0, 0, 0], [0, 0, 0]])
    assert not guard.check(0.0, positions, velocities)["reason"]
    result = guard.check(0.11, positions, velocities)
    assert "distance" in result["reason"]
    assert np.isclose(result["minimum_distance_m"], 0.4)
    assert np.isclose(result["predicted_distance_m"], 0.26)


def test_actual_close_pair_does_not_wait_for_confirmation():
    guard = SlungTransportGuard(min_distance_m=0.30, confirm_s=0.20)
    positions = np.array([[0.29, 0, 1], [0, 0, 1], [0, 1, 1]])
    assert "distance" in guard.check(0.0, positions, np.zeros((3, 3)))["reason"]


def test_height_drop_and_saturation_timers_reset_after_recovery():
    guard = SlungTransportGuard(height_error_m=0.08, confirm_s=0.20)
    p = np.array([[0.5, 0, 1], [0, 0, 1], [0, 1, 1]])
    v = np.zeros((3, 3))
    assert not guard.check(0.0, p, v, height_error=0.09)["reason"]
    assert not guard.check(0.15, p, v, height_error=0.0)["reason"]
    assert not guard.check(0.30, p, v, height_error=0.09)["reason"]
    assert "height_drop" in guard.check(0.51, p, v, height_error=0.09)["reason"]
    guard.reset()
    assert not guard.check(1.0, p, v, rate_saturated=True)["reason"]
    assert "rate_saturation" in guard.check(1.21, p, v, rate_saturated=True)["reason"]


def test_support_trim_keeps_only_handoff_residual_and_unwinds_overshoot():
    trim = SlungSupportTrim(limit_n=0.25, integral_gain=0.2, rate_limit_nps=0.03)
    bias = trim.initialize([0.8, 0.8, 0.8], [0.6, 0.85, 0.7])
    np.testing.assert_allclose(bias, [0.2, 0, 0.1])
    # A 0.6 N model output must become 0.8 N, not 1.4 N.
    assert np.isclose(0.6 + bias[0], 0.8)
    for _ in range(100):
        bias, derivative = trim.update(-0.1, 0.01)
    np.testing.assert_allclose(bias, [0.18, 0, 0.08], atol=1e-12)
    assert np.max(np.abs(derivative)) <= 0.03


def test_support_trim_is_bounded_and_cannot_add_force_before_initialization():
    trim = SlungSupportTrim(limit_n=0.25, integral_gain=0.2, rate_limit_nps=0.03)
    np.testing.assert_allclose(trim.update(1.0, .02)[0], 0)
    trim.initialize([2, 2, 2], [0, 0, 0])
    bias, derivative = trim.update(1.0, .02)
    np.testing.assert_allclose(bias, .25)
    np.testing.assert_allclose(derivative, 0)
