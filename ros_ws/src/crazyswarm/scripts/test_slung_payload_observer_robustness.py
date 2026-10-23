"""Pose-only observer regressions for rejected and irregular mocap samples."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from slung_load_controller import PayloadStateObserver  # noqa: E402


def _rotation(vector):
    vector = np.asarray(vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    x, y, z = vector / angle
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * skew @ skew


def _ready_observer(**kwargs):
    observer = PayloadStateObserver(**kwargs)
    for index in range(10):
        state = observer.update(np.zeros(3), np.eye(3), index * .01)
    assert state["derivatives_valid"]
    return observer, state


def _assert_same_estimate(actual, expected):
    for key in ("position", "velocity", "acceleration", "rotation", "body_rate"):
        np.testing.assert_allclose(actual[key], expected[key], atol=1e-14)
    assert actual["sample_time"] == expected["sample_time"]
    assert actual["sample_count"] == expected["sample_count"]


@pytest.mark.parametrize("timestamp", [.09, .08])
def test_duplicate_and_reversed_time_do_not_restart_or_mutate_estimator(timestamp):
    observer, before = _ready_observer()
    state = observer.update([10., 0., 0.], _rotation([0., 0., .3]), timestamp)
    assert state["sample_count"] == before["sample_count"]
    _assert_same_estimate(state, before)
    assert not state["observer_updated"]
    assert not state["observer_quality_valid"]
    assert state["observer_status"] == "nonmonotonic_time"
    assert state["reset_count"] == before["reset_count"]
    assert state["rejected_count"] == before["rejected_count"] + 1
    recovered = observer.update(np.zeros(3), np.eye(3), .10)
    assert recovered["observer_quality_valid"]


def test_tiny_dt_burst_accumulates_real_elapsed_time_without_derivative_spikes():
    observer, before = _ready_observer()
    for index in range(1, 401):
        state = observer.update([.001 * (-1) ** index, 0., 0.], np.eye(3), .09 + index * .00001)
        assert not state["observer_updated"]
        _assert_same_estimate(state, before)
    state = observer.update(np.zeros(3), np.eye(3), .095)
    assert state["observer_updated"]
    assert state["sample_count"] == before["sample_count"] + 1
    assert state["update_dt_s"] == pytest.approx(.005)
    assert state["input_dt_s"] == pytest.approx(.001)
    np.testing.assert_allclose(state["velocity"], 0.)
    np.testing.assert_allclose(state["acceleration"], 0.)


@pytest.mark.parametrize("position,rotation,status", [
    ([.5, 0., 0.], np.eye(3), "position_innovation"),
    ([0., 0., 0.], _rotation([0., 0., .2]), "attitude_innovation"),
])
def test_pose_spikes_never_enter_trusted_estimate_and_next_good_sample_recovers(position, rotation, status):
    observer, before = _ready_observer(max_velocity_mps=1., max_acceleration_mps2=8., max_body_rate_rps=2.)
    state = observer.update(position, rotation, .10)
    assert not state["derivatives_valid"]
    assert state["observer_status"] == status
    _assert_same_estimate(state, before)
    recovered = observer.update(np.zeros(3), np.eye(3), .11)
    assert recovered["observer_quality_valid"]
    np.testing.assert_allclose(recovered["acceleration"], 0.)
    np.testing.assert_allclose(recovered["body_rate"], 0.)


@pytest.mark.parametrize("limits,position,rotation,flag", [
    ({"max_velocity_mps": .01}, [.003, 0., 0.], np.eye(3), "velocity_clipped"),
    ({"max_acceleration_mps2": .1}, [.001, 0., 0.], np.eye(3), "acceleration_clipped"),
    ({"max_body_rate_rps": .1}, [0., 0., 0.], _rotation([0., 0., .02]), "body_rate_clipped"),
])
def test_candidate_at_derivative_limit_is_rejected_instead_of_trusted_clipping(limits, position, rotation, flag):
    observer, before = _ready_observer(**limits)
    state = observer.update(position, rotation, .10)
    assert not state["derivatives_valid"]
    assert state[flag]
    assert state["observer_status"] == "derivative_limit"
    _assert_same_estimate(state, before)


def test_candidate_exactly_on_physical_boundary_is_rejected_before_commit():
    observer = PayloadStateObserver(max_velocity_mps=.008)
    before = observer.update(np.zeros(3), np.eye(3), 0.)
    state = observer.update([.001, 0., 0.], np.eye(3), .01)
    assert state["velocity_clipped"]
    assert state["observer_status"] == "derivative_limit"
    assert not state["observer_updated"]
    _assert_same_estimate(state, before)


@pytest.mark.parametrize("position,rotation,timestamp", [
    ([float("nan"), 0., 0.], np.eye(3), .10),
    ([0., 0., 0.], np.full((3, 3), float("nan")), .10),
    ([0., 0., 0.], np.eye(3), float("nan")),
])
def test_nonfinite_input_does_not_clear_or_contaminate_accepted_estimate(position, rotation, timestamp):
    observer, before = _ready_observer()
    state = observer.update(position, rotation, timestamp)
    assert state["observer_status"] == "invalid_input"
    assert not state["observer_updated"]
    assert state["reset_count"] == before["reset_count"]
    _assert_same_estimate(state, before)
    assert observer.update(np.zeros(3), np.eye(3), .11)["observer_quality_valid"]


def test_continuous_bad_samples_do_not_reset_to_a_fresh_bad_pose():
    observer, before = _ready_observer(max_velocity_mps=1., max_acceleration_mps2=8., max_body_rate_rps=2.)
    for index in range(1, 61):
        state = observer.update([10., 0., 0.], _rotation([0., 0., .4]), .09 + .01 * index)
        assert not state["observer_quality_valid"]
        assert state["reset_count"] == before["reset_count"]
        _assert_same_estimate(state, before)
        assert state["consecutive_rejected_count"] == index
    recovered = observer.update(np.zeros(3), np.eye(3), .70)
    assert recovered["observer_quality_valid"]
    assert recovered["consecutive_rejected_count"] == 0
    assert recovered["rejected_count"] == 60
    np.testing.assert_allclose(recovered["acceleration"], 0.)


def test_persistent_point_two_radian_spike_cannot_open_its_own_innovation_gate():
    observer, before = _ready_observer()
    for index in range(1, 21):
        state = observer.update(np.zeros(3), _rotation([0., 0., .2]), .09 + index * .01)
        assert not state["observer_quality_valid"]
        assert state["observer_status"] == "attitude_innovation"
        _assert_same_estimate(state, before)


def test_actual_source_gap_resets_and_needs_fresh_warmup_samples():
    observer, before = _ready_observer()
    state = observer.update([.2, 0., 0.], np.eye(3), .30)
    assert not state["derivatives_valid"]
    assert state["observer_updated"]
    assert state["observer_status"] == "gap_reset"
    assert state["reset_count"] == before["reset_count"] + 1
    assert state["sample_count"] == 1
    assert state["sample_time"] == .30
    assert not observer.update([.2, 0., 0.], np.eye(3), .31)["observer_quality_valid"]
    assert observer.update([.2, 0., 0.], np.eye(3), .32)["observer_quality_valid"]


def test_uniform_velocity_and_rate_match_nominal_observer_response():
    observer = PayloadStateObserver(velocity_gain=.20, angular_rate_gain=.20)
    expected_velocity = np.array([.1, -.04, .02])
    expected_rate = np.array([.03, -.08, .12])
    accepted = []
    for index in range(201):
        timestamp = index * .01
        state = observer.update(expected_velocity * timestamp, _rotation(expected_rate * timestamp), timestamp)
        accepted.append(state["observer_updated"])
    assert all(accepted)
    assert state["observer_quality_valid"]
    np.testing.assert_allclose(state["velocity"], expected_velocity, atol=.0001)
    np.testing.assert_allclose(state["acceleration"], 0., atol=.002)
    np.testing.assert_allclose(state["body_rate"], expected_rate, atol=.0001)


def test_nominal_updates_preserve_hand_checked_original_response_without_added_lag():
    observer = PayloadStateObserver()
    observer.update(np.zeros(3), np.eye(3), 0.)
    state = observer.update([.001, 0., 0.], _rotation([0., 0., .005]), .01)
    np.testing.assert_allclose(state["position"], [.00035, 0., 0.], atol=1e-14)
    np.testing.assert_allclose(state["velocity"], [.008, 0., 0.], atol=1e-14)
    np.testing.assert_allclose(state["acceleration"], [.2, 0., 0.], atol=1e-14)
    np.testing.assert_allclose(state["body_rate"], [0., 0., .04], atol=1e-14)
    state = observer.update([.002, 0., 0.], _rotation([0., 0., .01]), .02)
    np.testing.assert_allclose(state["position"], [.000986, 0., 0.], atol=1e-14)
    np.testing.assert_allclose(state["velocity"], [.02248, 0., 0.], atol=1e-14)
    np.testing.assert_allclose(state["acceleration"], [.512, 0., 0.], atol=1e-14)
    np.testing.assert_allclose(state["body_rate"], [0., 0., .1028], atol=1e-14)
    np.testing.assert_allclose(state["rotation"], _rotation([0., 0., .0048975]), atol=1e-14)


def test_unsteady_body_axes_and_irregular_samples_track_motion_without_rejection():
    observer = PayloadStateObserver(max_velocity_mps=1., max_acceleration_mps2=4., max_body_rate_rps=2.)
    timestamp = 0.
    rotation = np.eye(3)
    observer.update(np.zeros(3), rotation, timestamp)
    intervals = [.005, .007, .013, .009, .011, .006, .014, .008, .012, .015]
    for index in range(600):
        dt = intervals[index % len(intervals)]
        midpoint = timestamp + .5 * dt
        true_rate = np.array([.12 + .05 * math.sin(midpoint), -.09 + .03 * math.cos(.7 * midpoint), .15 + .08 * math.sin(.9 * midpoint)])
        rotation = rotation @ _rotation(true_rate * dt)
        timestamp += dt
        position = np.array([.03 * math.sin(.8 * timestamp), .02 * math.sin(.6 * timestamp), .01 * math.sin(timestamp)])
        state = observer.update(position, rotation, timestamp)
        assert state["observer_updated"]
        assert state["observer_status"] in ("initializing", "updated")
    expected_velocity = np.array([.024 * math.cos(.8 * timestamp), .012 * math.cos(.6 * timestamp), .01 * math.cos(timestamp)])
    np.testing.assert_allclose(state["velocity"], expected_velocity, atol=.003)
    np.testing.assert_allclose(state["body_rate"], true_rate, atol=.01)
    assert state["observer_quality_valid"]


def test_shorter_accepted_intervals_do_not_amplify_equal_pose_noise():
    normal, _ = _ready_observer()
    short, _ = _ready_observer()
    nominal_state = normal.update([.001, 0., 0.], _rotation([.005, 0., 0.]), .10)
    short_state = short.update([.001, 0., 0.], _rotation([.005, 0., 0.]), .095)
    assert short_state["observer_updated"]
    assert np.linalg.norm(short_state["velocity"]) <= np.linalg.norm(nominal_state["velocity"])
    assert np.linalg.norm(short_state["acceleration"]) <= np.linalg.norm(nominal_state["acceleration"])
    assert np.linalg.norm(short_state["body_rate"]) <= np.linalg.norm(nominal_state["body_rate"])


def test_snapshot_arrays_are_copies_and_report_distinct_source_and_accepted_time():
    observer, first = _ready_observer()
    first["velocity"][0] = 1e6
    rejected = observer.update([1., 0., 0.], np.eye(3), .10)
    assert rejected["source_stamp"] == .10
    assert rejected["sample_time"] == .09
    assert rejected["velocity"][0] == 0.
