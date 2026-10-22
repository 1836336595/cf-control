"""Replay must distinguish planned validation descent from transport failures."""

import numpy as np
import pytest

import simulate_slung_handoff as replay


def states(z=.007, vz=0.):
    return [dict(position=np.array([i, 0., z]), velocity=np.array([0., 0., vz]))
            for i in range(3)]


def test_planned_landing_requires_enabled_latch_and_all_vehicle_reasons():
    reasons = ["payload_load: flight_validation_complete: 10.000 s"] * 3
    assert replay.is_planned_validation_landing(True, 39., reasons)
    assert not replay.is_planned_validation_landing(False, 39., reasons)
    assert not replay.is_planned_validation_landing(True, None, reasons)
    assert not replay.is_planned_validation_landing(True, 39., reasons[:2])
    assert not replay.is_planned_validation_landing(
        True, 39., reasons[:2] + ["payload_load: height_drop"])
    assert not replay.is_planned_validation_landing(True, 39., [""] * 3)


def test_short_validation_never_passes_an_unfinished_transport_or_descent():
    check = replay.validation_window_failure
    assert check(None, None, 10., False) == "handoff_timeout"
    assert check(29., None, 10., False) == "validation_landing_not_started"
    assert check(29., 38.9, 10., True) == "validation_transport_window_incomplete"
    assert check(29., 39., 10., False) == "validation_landing_incomplete"
    assert check(29., 39., 10., True) is None


@pytest.mark.parametrize("phase", ["landing", "landing_settle", "landed"])
def test_normal_planned_landing_phases_are_not_terminal_failures(phase):
    assert replay.validation_terminal_failure([phase] * 3) is None


def test_any_true_emergency_or_abort_fails_despite_planned_completion_marker():
    # The marker can survive a phase transition. A real emergency phase must
    # always take precedence over the marker and validation landing latch.
    reasons = ["payload_load: flight_validation_complete: 10.000 s"] * 3
    assert replay.is_planned_validation_landing(True, 39., reasons)
    assert replay.validation_terminal_failure(
        ["landing", "emergency_landing", "landing"]) == "emergency_landing"
    assert replay.validation_terminal_failure(
        ["landing", "aborted", "landing"]) == "aborted"


def test_zero_thrust_near_floor_and_low_speed_must_hold_for_half_second():
    monitor = replay.ValidationLandingMonitor(ground_center_z=.007)
    assert not monitor.update(40., states(), [0., 0., 0.])
    assert not monitor.update(40.49, states(), [0., 0., 0.])
    assert monitor.update(40.5, states(), [0., 0., 0.])


@pytest.mark.parametrize("bad_states,bad_thrust", [
    (states(z=.7), [0., 0., 0.]),
    (states(vz=-.16), [0., 0., 0.]),
    (states(z=float("nan")), [0., 0., 0.]),
    (states(), [0., .01, 0.]),
    (states(), [0., float("nan"), 0.]),
    (states()[:2], [0., 0.]),
])
def test_incomplete_or_unsettled_physical_landing_restarts_hold(bad_states, bad_thrust):
    monitor = replay.ValidationLandingMonitor(ground_center_z=.007)
    assert not monitor.update(40., states(), [0., 0., 0.])
    assert not monitor.update(40.4, bad_states, bad_thrust)
    assert not monitor.update(40.6, states(), [0., 0., 0.])
    assert not monitor.update(41., states(), [0., 0., 0.])
    assert monitor.update(41.1, states(), [0., 0., 0.])


def test_validation_ground_reference_preserves_takeup_geometry():
    class Trajectory:
        start_position = np.array([.2, .3, .7])
        takeoff_position = np.array([.2, .3, 1.2])
    trajectory = Trajectory()
    replay.set_validation_ground_reference([trajectory], .007)
    np.testing.assert_allclose(trajectory.start_position, [.2, .3, .007])
    np.testing.assert_allclose(trajectory.takeoff_position, [.2, .3, 1.2])
