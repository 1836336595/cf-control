"""Measurement noise must pass through the observer without changing physics."""

import numpy as np
import pytest

import simulate_slung_handoff as replay


def test_zero_measurement_noise_preserves_rotation_and_rng_state():
    first = np.random.default_rng(123)
    second = np.random.default_rng(123)
    rotation = np.eye(3)
    actual = replay.payload_rotation_measurement(rotation, [0,0,0], first)
    np.testing.assert_array_equal(actual, rotation)
    assert first.random() == second.random()


def test_seeded_noise_is_a_rotation_and_never_mutates_truth():
    truth = np.eye(3)
    a = replay.payload_rotation_measurement(truth, [.005,.004,.004], np.random.default_rng(20261016))
    b = replay.payload_rotation_measurement(truth, [.005,.004,.004], np.random.default_rng(20261016))
    np.testing.assert_array_equal(a,b)
    np.testing.assert_array_equal(truth,np.eye(3))
    np.testing.assert_allclose(a.T@a,np.eye(3),atol=1e-12)
    assert np.isclose(np.linalg.det(a),1)
    assert not np.array_equal(a,truth)


@pytest.mark.parametrize('std', [[-.01,0,0],[0,0],[float('nan'),0,0]])
def test_invalid_noise_configuration_fails_before_simulation(std):
    with pytest.raises(ValueError,match='payload_rotation_noise_std_rad'):
        replay.resolve_simulation_settings({'simulation':{'payload_rotation_noise_std_rad':std}})
