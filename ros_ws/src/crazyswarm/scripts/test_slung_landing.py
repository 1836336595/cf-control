"""C2 payload landing references and robust measured touchdown confirmation."""

import csv
import json
from pathlib import Path
import time

import numpy as np
import pytest

from slung_landing import PayloadTouchdownDetector, SlungLandingProfile


def test_recorded_feasible_state_lands_without_stretching_initial_acceleration():
    # This is the nearest earlier logged state reproducing the constructor
    # failure, not the unlogged state at the actual exception.
    p = np.array([.2703441689875301, .019401254937344334, .5975047200411108])
    v = np.array([-.016109275473236755, -.007141530014222486, .007881279383302584])
    a = np.array([-.06258183776592048, -.0008458236140853042, .05734148851088986])
    ground = .03507010219448599
    before = time.perf_counter()
    profile = SlungLandingProfile(p, v, a, ground, 10., 4., .12, .15)
    assert time.perf_counter() - before < .1  # CTBR command watchdog period.
    assert 4. <= profile.duration < 30.
    for actual, expected in zip(profile.evaluate(10.), (p, v, a)):
        np.testing.assert_allclose(actual, expected, atol=1e-12)
    values = profile._sample(np.linspace(0., 1., 10001))
    position, velocity, acceleration = values
    assert np.max(np.linalg.norm(velocity, axis=1)) <= .12 + 1e-9
    assert np.max(np.linalg.norm(acceleration, axis=1)) <= .15 + 1e-9
    assert np.min(position[:, 2]) >= ground - 1e-10
    endpoint = np.array([p[0], p[1], ground])
    for actual, expected in zip(profile.evaluate(10. + profile.duration),
                                (endpoint, np.zeros(3), np.zeros(3))):
        np.testing.assert_allclose(actual, expected, atol=1e-12)


def test_speed_boundary_with_outward_acceleration_is_explicitly_infeasible():
    with pytest.raises(ValueError, match='speed.*outward'):
        SlungLandingProfile([0., 0., .6], [.12, 0., 0.], [.01, 0., 0.],
                            .025, 0., 4.)


def test_ground_boundary_with_downward_velocity_is_explicitly_infeasible():
    with pytest.raises(ValueError, match='ground.*downward'):
        SlungLandingProfile([0., 0., .025], [0., 0., -.01], [0., 0., 0.],
                            .025, 0., 4.)


def test_initial_acceleration_above_configured_cap_is_not_increased():
    p, v, a = np.array([0., 0., .6]), np.array([.01, .02, 0.]), np.array([.5, 0., .4])
    profile = SlungLandingProfile(p, v, a, .025, 0., 4.)
    position, velocity, acceleration = profile._sample(np.linspace(0., 1., 10001))
    np.testing.assert_allclose(acceleration[0], a, atol=1e-12)
    assert np.max(np.linalg.norm(velocity, axis=1)) <= .12 + 1e-9
    assert np.max(np.linalg.norm(acceleration, axis=1)) <= np.linalg.norm(a) + 1e-9
    assert np.min(position[:, 2]) >= .025 - 1e-10


def test_braking_and_descent_junctions_preserve_position_velocity_acceleration():
    profile = SlungLandingProfile(
        [.2703441689875301, .019401254937344334, .5975047200411108],
        [-.016109275473236755, -.007141530014222486, .007881279383302584],
        [-.06258183776592048, -.0008458236140853042, .05734148851088986],
        .03507010219448599, 10., 4.)
    assert len(profile._segments) > 1
    for elapsed in profile._segment_ends[:-1]:
        before = profile.evaluate(10. + elapsed - 1e-7)
        at = profile.evaluate(10. + elapsed)
        after = profile.evaluate(10. + elapsed + 1e-7)
        for index, tolerance in enumerate((3e-8, 3e-8, 1e-6)):
            np.testing.assert_allclose(before[index], at[index], atol=tolerance, rtol=0.)
            np.testing.assert_allclose(after[index], at[index], atol=tolerance, rtol=0.)
    # Verify approach to the initial and final p/v/a, not only the endpoint
    # values returned by evaluate's endpoint guards.
    for actual, expected in zip(profile.evaluate(10. + 1e-7), profile.evaluate(10.)):
        np.testing.assert_allclose(actual, expected, atol=1e-6, rtol=0.)
    end = 10. + profile.duration
    for actual, expected in zip(profile.evaluate(end - 1e-7), profile.evaluate(end)):
        np.testing.assert_allclose(actual, expected, atol=1e-7, rtol=0.)


def test_speed_ceiling_with_inward_acceleration_can_brake_continuously():
    profile = SlungLandingProfile([0., 0., .6], [.12, 0., 0.], [-.1, 0., 0.],
                                  .025, 0., 4.)
    _, velocity, acceleration = profile._sample(np.linspace(0., 1., 10001))
    assert np.max(np.linalg.norm(velocity, axis=1)) <= .12 + 1e-9
    assert np.max(np.linalg.norm(acceleration, axis=1)) <= .15 + 1e-9


def test_recorded_last_second_states_have_bounded_landing_references():
    path = (Path(__file__).parent /
            'ctbr_logs/analysis_20261020_163110/landing_reproduction.json')
    if not path.exists():
        pytest.skip('local recorded landing diagnosis is not available')
    states = json.loads(path.read_text())['last_second_constructor_results']
    assert len(states) == 78
    for row in states:
        state = row['state']
        profile = SlungLandingProfile(state['position'], state['velocity'],
                                      state['acceleration'], state['ground_z'], 0., 4.)
        position, velocity, acceleration = profile._sample(np.linspace(0., 1., 1001))
        assert np.max(np.linalg.norm(velocity, axis=1)) <= max(.12, np.linalg.norm(state['velocity'])) + 1e-9
        assert np.max(np.linalg.norm(acceleration, axis=1)) <= max(.15, np.linalg.norm(state['acceleration'])) + 1e-9
        assert np.min(position[:, 2]) >= state['ground_z'] - 1e-9


def test_landing_reference_is_c2_at_start_and_end():
    p, v, a = np.array([.2, -.1, .6]), np.array([.02, -.01, -.03]), np.array([.01, 0., -.02])
    profile = SlungLandingProfile(p, v, a, .025, 10., 2.)
    actual = profile.evaluate(10.)
    for value, expected in zip(actual, (p, v, a)):
        np.testing.assert_allclose(value, expected, atol=1e-12)
    actual = profile.evaluate(10. + profile.duration)
    for value, expected in zip(actual, ([.2, -.1, .025], np.zeros(3), np.zeros(3))):
        np.testing.assert_allclose(value, expected, atol=1e-11)
    np.testing.assert_allclose(profile.evaluate(1000.)[0], [.2, -.1, .025])


def test_duration_expands_until_reference_speed_and_acceleration_are_bounded():
    profile = SlungLandingProfile([0, 0, 1.2], [0, 0, 0], [0, 0, 0], .025, 0., 1.)
    values = [profile.evaluate(t) for t in np.linspace(0, profile.duration, 1001)]
    assert profile.duration >= 1.875 * 1.175 / .12
    assert max(np.linalg.norm(v) for _, v, _ in values) <= .12 + 1e-7
    assert max(np.linalg.norm(a) for _, _, a in values) <= .15 + 1e-7
    assert min(p[2] for p, _, _ in values) >= .025 - 1e-10


def test_full_vector_limits_include_initial_horizontal_motion():
    profile = SlungLandingProfile([0, 0, .6], [.05, -.02, -.02], [0, 0, 0], .025, 0., 1.)
    values = [profile.evaluate(t) for t in np.linspace(0, profile.duration, 1001)]
    assert max(np.linalg.norm(v) for _, v, _ in values) <= .12 + 1e-7
    np.testing.assert_allclose(values[-1][0], [0, 0, .025])


@pytest.mark.parametrize("kwargs", [
    dict(start_position=[0, 0, float('nan')]), dict(start_velocity=[0, 0]),
    dict(ground_z=float('inf')), dict(min_duration_s=0), dict(max_speed_mps=0),
])
def test_invalid_landing_inputs_are_rejected(kwargs):
    settings = dict(start_position=[0, 0, .6], start_velocity=[0, 0, 0],
                    start_acceleration=[0, 0, 0], ground_z=.025, now=0., min_duration_s=1.)
    settings.update(kwargs)
    with pytest.raises(ValueError):
        SlungLandingProfile(**settings)


def test_stable_ground_window_ignores_isolated_noisy_velocity_spikes():
    detector = PayloadTouchdownDetector()
    detected = False
    for i in range(41):
        t = i / 100.
        vz = -.2 if i in (12, 25, 36) else .005
        detected = detector.update(t, .009 + .001 * np.sin(i), vz) or detected
    assert detected
    assert detector.detected


def test_near_ground_height_spike_does_not_discard_a_stable_window():
    detector = PayloadTouchdownDetector()
    for i in range(31):
        height = .023 if i == 25 else .009
        ready = detector.update(i / 100., height, 0.)
    assert ready


def test_alternating_velocity_noise_requires_still_measured_height():
    detector = PayloadTouchdownDetector()
    assert any(detector.update(i / 100., .009, .1 if i % 2 else -.1)
               for i in range(41))
    detector = PayloadTouchdownDetector()
    assert not any(detector.update(i / 100., .024 - .06 * i / 100.,
                                   .1 if i % 2 else -.1) for i in range(31))


def test_airborne_or_sustained_vertical_motion_cannot_confirm_touchdown():
    for height, velocity in ((.3, 0.), (.009, .1)):
        detector = PayloadTouchdownDetector()
        assert not any(detector.update(i / 100., height, velocity) for i in range(101))
    detector = PayloadTouchdownDetector()
    assert not any(detector.update(i / 100., .024 - .06 * i / 100., -.06)
                   for i in range(31))


def test_stable_hover_inside_collection_tolerance_is_not_contact():
    detector = PayloadTouchdownDetector()
    assert not any(detector.update(i / 100., .009 + .014, 0.) for i in range(101))


@pytest.mark.parametrize('margin', [0., -.001, .016, float('nan')])
def test_contact_margin_must_be_positive_and_within_collection_tolerance(margin):
    with pytest.raises(ValueError):
        PayloadTouchdownDetector(contact_margin_m=margin)


def test_invalid_sample_resets_window_and_timeout_never_forces_release():
    detector = PayloadTouchdownDetector()
    for i in range(20):
        assert not detector.update(i / 100., .009, 0.)
    assert not detector.update(.2, None, None)
    assert detector.ready_since is None
    for i in range(21, 50):
        assert not detector.update(i / 100., .009, 0.)
    assert detector.update(.51, .009, 0.)
    detector.reset()
    assert not detector.detected
    assert detector.timed_out(5., 0.)
    assert not detector.detected


def test_sample_gap_and_time_reversal_do_not_confirm_a_sparse_window():
    detector = PayloadTouchdownDetector()
    assert not detector.update(0., .009, 0.)
    assert not detector.update(1., .009, 0.)
    assert not detector.update(.9, .009, 0.)
    assert not detector.detected


def test_duplicate_measurement_is_ignored_without_extending_or_resetting_window():
    detector = PayloadTouchdownDetector()
    for i in range(21):
        assert not detector.update(i / 100., .009, 0.)
    count = len(detector._samples)
    assert not detector.update(.2, .023, -.2)
    assert len(detector._samples) == count
    assert not detector.update(.29, .009, 0.)
    assert detector.update(.3, .009, 0.)
    sparse = PayloadTouchdownDetector()
    assert not sparse.update(0., .009, 0.)
    for _ in range(100):
        assert not sparse.update(0., .009, 0.)
    assert len(sparse._samples) == 1


def test_recorded_noisy_ground_segment_confirms_before_distance_fault():
    path = Path(__file__).parent / 'ctbr_logs/multi_ctbr_20261016_145240.csv'
    if not path.exists():
        pytest.skip('local recorded flight is not available')
    detector = PayloadTouchdownDetector()
    first = None
    with path.open() as stream:
        for row in csv.DictReader(stream):
            t = float(row['control_time_s'])
            if row['vehicle_id'] != '3' or not 59.465 <= t < 62.1065:
                continue
            if detector.update(t, float(row['payload_ground_clearance_m']),
                               float(row['payload_velocity_z'])):
                first = t
                break
    assert first is not None
    assert 61.0 < first < 62.1065
