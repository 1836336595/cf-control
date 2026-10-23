"""Physical replay schedules must retain sensor clocks and fixed-rate compatibility."""

from copy import deepcopy

import numpy as np
import pytest
import yaml

import simulate_slung_handoff as replay


def test_empty_schedule_keeps_exact_70_hz_deadlines():
    deadlines = list(replay.control_tick_deadlines(70., [], 1.))
    assert len(deadlines) == 70
    assert deadlines == [(tick + 1) / 70. for tick in range(70)]
    assert replay.resolve_simulation_settings({})['control_intervals_s'] == []


def test_nonuniform_schedule_repeats_with_cumulative_physical_deadlines():
    deadlines = list(replay.control_tick_deadlines(70., [.01, .02, .03], .10))
    np.testing.assert_allclose(deadlines, [.01, .03, .06, .07, .09, .10], atol=1e-15)
    assert len(deadlines) == 6


def test_time_limit_is_physical_time_not_nominal_control_tick_count():
    fast = list(replay.control_tick_deadlines(70., [.005], 1.))
    slow = list(replay.control_tick_deadlines(70., [.025], 1.))
    assert len(fast) == 200
    assert len(slow) == 40
    assert fast[-1] == slow[-1] == 1.


def test_last_partial_interval_stops_at_physical_time_limit():
    assert list(replay.control_tick_deadlines(70., [.03], .10)) == [.03, .06, .09, .10]


@pytest.mark.parametrize('intervals', [[0.], [-.01], [float('nan')], [float('inf')],
                                      [[.01]], None, '.01'])
def test_bad_control_interval_sequences_are_rejected(intervals):
    with pytest.raises(ValueError, match='control_intervals_s'):
        replay.resolve_simulation_settings({'control_intervals_s': intervals})


def test_simulation_settings_record_real_interval_pattern():
    settings = replay.resolve_simulation_settings({'control_intervals_s': [.01, .022]})
    assert settings['control_intervals_s'] == [.01, .022]


@pytest.mark.parametrize('rate,limit', [(0., 1.), (float('nan'), 1.), (70., 0.)])
def test_invalid_nominal_clock_or_time_limit_is_rejected(rate, limit):
    with pytest.raises(ValueError):
        list(replay.control_tick_deadlines(rate, [], limit))


def test_interval_only_overlay_preserves_the_configured_simulation_plant(tmp_path):
    """Adding measured timing must preserve motor settings and sensor delays."""
    config_path = tmp_path / 'slung_simulation.yaml'
    document = yaml.safe_load(replay.SIMULATION_CONFIG_PATH.read_text())
    baseline = document['slung_simulation']
    baseline.update(brushless_motor_time_constant_s=.06, pose_delay_s=.08, command_delay_s=.013)
    baseline['model_validation_metadata'] = {'source': 'fixture', 'tags': ['original'],
                                             'nested': {'keep': 42, 'replace': 1}}
    config_path.write_text(yaml.safe_dump(document, sort_keys=False))
    original_document = deepcopy(document)
    overlay = {'slung_simulation': {'control_intervals_s': [.01, .022],
        'model_validation_metadata': {'tags': ['new'], 'nested': {'replace': 2}}},
        'slung_payload': {'flight_validation_enabled': False}}
    original_overlay = deepcopy(overlay)
    loaded = replay.load_simulation_config(path=config_path, parameter_overrides=overlay)
    settings = replay.resolve_simulation_settings(loaded)
    assert settings['brushless_motor_time_constant_s'] == .06
    assert settings['pose_delay_s'] == .08
    assert settings['command_delay_s'] == .013
    assert settings['control_intervals_s'] == [.01, .022]
    assert 'slung_payload' not in loaded
    assert 'slung_simulation' not in loaded
    for key, value in baseline.items():
        if key not in ('control_intervals_s', 'model_validation_metadata'):
            assert loaded[key] == value
    assert loaded['model_validation_metadata'] == {
        'source': 'fixture', 'tags': ['new'], 'nested': {'keep': 42, 'replace': 2}}
    # Loaded data belongs to this run, independent of caller and future loads.
    loaded['control_intervals_s'].append(.03)
    loaded['model_validation_metadata']['tags'].append('loaded-only')
    assert overlay == original_overlay
    assert yaml.safe_load(config_path.read_text()) == original_document
    second = replay.load_simulation_config(path=config_path, parameter_overrides=overlay)
    assert second['control_intervals_s'] == [.01, .022]
    assert second['model_validation_metadata']['tags'] == ['new']
