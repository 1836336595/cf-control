"""Per-vehicle support trim starts from handoff residual within force headroom."""

import numpy as np

from slung_transport_guard import SlungSupportTrim


def test_available_handoff_residual_is_preserved_per_vehicle():
    trim = SlungSupportTrim(limit_n=.25)
    bias = trim.initialize([.85, .85, .82], [.79, .45, .43], [1.0, 1.0, 1.0])
    np.testing.assert_allclose(bias, [.06, .40, .39])
    np.testing.assert_allclose(trim.requested_bias, [.06, .40, .39])
    np.testing.assert_allclose(trim.limits, [.21, .40, .39])
    np.testing.assert_array_equal(trim.initialization_clipped, [False, False, False])


def test_support_residual_is_clipped_only_by_available_vertical_force():
    trim = SlungSupportTrim(limit_n=.25)
    bias = trim.initialize([.85, .85, .82], [.45, .45, .43], [.70, .70, .70])
    np.testing.assert_allclose(bias, [.25, .25, .27])
    np.testing.assert_allclose(trim.limits, [.25, .25, .27])
    np.testing.assert_array_equal(trim.initialization_clipped, [True, True, True])


def test_missing_available_force_preserves_legacy_trim_cap():
    trim = SlungSupportTrim(limit_n=.25)
    bias = trim.initialize([.85, .85, .82], [.45, .45, .43])
    np.testing.assert_allclose(bias, [.25, .25, .25])
    np.testing.assert_allclose(trim.limits, [.25, .25, .25])
    np.testing.assert_array_equal(trim.initialization_clipped, [True, True, True])


def test_integral_cannot_run_past_per_vehicle_limit_and_reset_clears_diagnostics():
    trim = SlungSupportTrim(limit_n=.25)
    trim.initialize([.85, .85, .82], [.79, .45, .43], [1.0, 1.0, 1.0])
    for _ in range(100):
        bias, derivative = trim.update(1.0, .1)
    np.testing.assert_allclose(bias, [.21, .40, .39])
    np.testing.assert_allclose(derivative, [0.0, 0.0, 0.0])
    trim.reset()
    np.testing.assert_allclose(trim.bias, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(trim.requested_bias, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(trim.limits, [.25, .25, .25])
    np.testing.assert_array_equal(trim.initialization_clipped, [False, False, False])
