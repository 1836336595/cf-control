"""Session-only rope length estimates must require stable unique geometry."""

import numpy as np
import pytest

from slung_link_calibration import SlungLinkCalibrator


NOMINAL = np.array([.709, .699, .654])


def fill(calibrator, start=0., end=5., value=(.712, .701, .679), dt=.02):
    result = None
    for sample in np.arange(start, end + dt / 2, dt):
        accepted = calibrator.update(float(sample), value)
        if accepted is not None:
            result = accepted
    return result


def test_stable_full_five_second_window_applies_only_positive_bounded_lengths():
    calibrator = SlungLinkCalibrator(NOMINAL)
    assert fill(calibrator, end=4.98) is None
    assert calibrator.status == 'collecting'
    candidate = calibrator.update(5., [.712, .701, .679])
    np.testing.assert_allclose(candidate, [.712, .701, .679])
    assert calibrator.status == 'applied'
    assert calibrator.applied_time == 5.
    np.testing.assert_allclose(calibrator.nominal, NOMINAL)
    np.testing.assert_allclose(calibrator.effective, candidate)
    assert calibrator.update(6., [.73, .72, .68]) is None
    np.testing.assert_allclose(calibrator.effective, candidate)


def test_duplicates_cannot_extend_the_window_or_supply_minimum_samples():
    calibrator = SlungLinkCalibrator(NOMINAL)
    for _ in range(500):
        assert calibrator.update(0., [.712, .701, .679]) is None
    assert calibrator.sample_count == 1
    assert calibrator.window_elapsed_s == 0.
    sparse = SlungLinkCalibrator(NOMINAL, max_sample_gap_s=1.)
    for t in np.arange(0., 5.01, .1):
        assert sparse.update(t, [.712, .701, .679]) is None
    assert sparse.status == 'insufficient_samples'


def test_trimmed_estimate_ignores_isolated_high_spike():
    calibrator = SlungLinkCalibrator(NOMINAL)
    result = None
    for i in range(251):
        distance = np.array([.712, .701, .679]) + .0005 * np.sin(i)
        if i == 100:
            distance[2] = .9
        candidate = calibrator.update(i / 50., distance)
        if candidate is not None:
            result = candidate
    assert result is not None
    np.testing.assert_allclose(result, [.712, .701, .679], atol=5e-5)
    assert calibrator.spread_m[2] < .0011


def test_large_spread_or_drift_remains_unapplied_until_stable_window():
    unstable = SlungLinkCalibrator(NOMINAL)
    for i in range(251):
        assert unstable.update(i / 50., [.712, .701, .679 + .006 * np.sin(i)]) is None
    assert unstable.status == 'unstable'
    drifting = SlungLinkCalibrator(NOMINAL, max_spread_m=.03)
    for i in range(251):
        assert drifting.update(i / 50., [.712, .701, .662 + .004 * i / 50.]) is None
    assert drifting.status == 'unstable'
    assert fill(unstable, start=5.02, end=10.02) is not None


def test_out_of_bounds_candidate_is_rejected_not_clipped_or_applied():
    calibrator = SlungLinkCalibrator(NOMINAL)
    assert fill(calibrator, value=(.712, .701, .69)) is None
    assert calibrator.status == 'out_of_bounds'
    np.testing.assert_allclose(calibrator.effective, NOMINAL)
    assert calibrator.candidate[2] == pytest.approx(.69)


def test_slack_geometry_never_shortens_or_qualifies_even_with_other_mismatch():
    calibrator = SlungLinkCalibrator(NOMINAL)
    assert fill(calibrator, value=(.68, .701, .679)) is None
    assert calibrator.reason == 'slack_geometry'
    np.testing.assert_allclose(calibrator.effective, NOMINAL)
    assert fill(calibrator, start=5.02, end=10.02, value=(.705, .701, .679)) is not None
    np.testing.assert_allclose(calibrator.effective, [.709, .701, .679])


def test_within_tolerance_keeps_monitoring_until_large_positive_mismatch():
    calibrator = SlungLinkCalibrator(NOMINAL)
    assert fill(calibrator, value=(.712, .701, .66)) is None
    assert calibrator.status == 'within_tolerance'
    assert calibrator.applied_time is None
    assert fill(calibrator, start=5.02, end=10.02) is not None


@pytest.mark.parametrize('kind', ['ineligible', 'invalid', 'backwards', 'gap'])
def test_unusable_geometry_or_time_resets_the_collection(kind):
    calibrator = SlungLinkCalibrator(NOMINAL)
    fill(calibrator, end=2.)
    if kind == 'ineligible':
        assert calibrator.update(2.02, [.712, .701, .679], eligible=False) is None
    elif kind == 'invalid':
        assert calibrator.update(2.02, [.712, .701, float('nan')]) is None
    elif kind == 'backwards':
        assert calibrator.update(1., [.712, .701, .679]) is None
    else:
        assert calibrator.update(2.5, [.712, .701, .679]) is None
    assert calibrator.window_elapsed_s == 0.
    assert calibrator.sample_count <= 1
    assert calibrator.applied_time is None


def test_nominal_and_effective_are_copies_and_reset_keeps_applied_latch():
    supplied = NOMINAL.copy()
    calibrator = SlungLinkCalibrator(supplied)
    supplied[:] = 0
    exposed = calibrator.nominal
    exposed[:] = 0
    np.testing.assert_allclose(calibrator.nominal, NOMINAL)
    fill(calibrator)
    effective = calibrator.effective
    effective[:] = 0
    calibrator.reset_window('state_lost')
    assert calibrator.status == 'applied'
    assert calibrator.applied_time is not None
    assert calibrator.update(20., [.712, .701, .68]) is None
    np.testing.assert_allclose(calibrator.effective, [.712, .701, .679])


def test_exact_tolerance_boundary_does_not_force_a_calibration():
    calibrator = SlungLinkCalibrator(NOMINAL)
    assert fill(calibrator, value=(.709, .699, .669)) is None
    assert calibrator.status == 'within_tolerance'


def test_irregular_sensor_period_retains_full_observation_extent():
    calibrator = SlungLinkCalibrator(NOMINAL)
    candidate = None
    for i in range(1, 361):
        accepted = calibrator.update(i / 70., [.712, .701, .679])
        if accepted is not None:
            candidate = accepted
            break
    assert candidate is not None
    assert calibrator.window_elapsed_s >= 5. - 1e-9
    assert calibrator.applied_time >= 5. + 1 / 70. - 1e-9


@pytest.mark.parametrize('kwargs', [dict(window_s=0), dict(max_adjustment_m=-1),
    dict(min_samples=1), dict(trim_fraction=.5), dict(max_sample_gap_s=0)])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        SlungLinkCalibrator(NOMINAL, **kwargs)
