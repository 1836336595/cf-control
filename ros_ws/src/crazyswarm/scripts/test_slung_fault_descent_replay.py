"""Fault descent verification is opt-in and cannot excuse unrelated failures."""

import pytest

import simulate_slung_handoff as replay


def test_height_fault_continuation_requires_explicit_option_fresh_payload_and_landing_latch():
    reason = 'payload_load: height_drop: injected verification'
    check = replay.is_expected_controlled_fault_descent
    assert check(True, reason, ['landing'] * 3, 33., True)
    assert check(True, reason, ['landing_settle', 'landed', 'landing'], 33., True)
    assert not check(False, reason, ['landing'] * 3, 33., True)
    assert not check(True, reason, ['landing'] * 3, None, True)
    assert not check(True, reason, ['landing'] * 3, 33., False)


@pytest.mark.parametrize('reason,phases', [
    ('payload_load: distance: .2 m', ['landing'] * 3),
    ('payload_load: height_drop_unconfirmed', ['landing'] * 3),
    ('payload_load: touchdown_timeout: invalid contact', ['landing'] * 3),
    ('payload_load: flight_validation_complete: 10 s', ['landing'] * 3),
    ('payload_load: height_drop: injected verification', ['landing', 'emergency_landing', 'landing']),
    ('payload_load: height_drop: injected verification', ['landing', 'aborted', 'landing']),
    ('payload_load: height_drop: injected verification', ['landing', 'landing']),
])
def test_fault_verification_never_accepts_distance_emergency_abort_or_other_reason(reason, phases):
    assert not replay.is_expected_controlled_fault_descent(True, reason, phases, 33., True)
