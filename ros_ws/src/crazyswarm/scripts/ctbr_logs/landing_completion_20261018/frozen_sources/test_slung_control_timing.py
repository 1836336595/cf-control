"""Physical replay schedules must retain sensor clocks and fixed-rate compatibility."""

from copy import deepcopy

import numpy as np
import pytest
import yaml

import simulate_slung_handoff as replay
import test_ctbr_controller_v2 as controller_helpers


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
        replay.resolve_simulation_settings({'simulation': {'control_intervals_s': intervals}})


def test_simulation_settings_record_real_interval_pattern():
    settings = replay.resolve_simulation_settings({'simulation': {'control_intervals_s': [.01, .022]}})
    assert settings['control_intervals_s'] == [.01, .022]


@pytest.mark.parametrize('rate,limit', [(0., 1.), (float('nan'), 1.), (70., 0.)])
def test_invalid_nominal_clock_or_time_limit_is_rejected(rate, limit):
    with pytest.raises(ValueError):
        list(replay.control_tick_deadlines(rate, [], limit))


def test_interval_only_overlay_preserves_the_configured_simulation_plant(tmp_path, monkeypatch):
    """Adding measured timing must not silently switch PID or replace delays."""
    config_directory = tmp_path / 'config'
    config_directory.mkdir()
    current = controller_helpers.CONTROLLER_CONFIG_PATH.parent
    for name in ('ctbr_controller.yaml', 'ctbr_vehicle.yaml', 'slung_payload.yaml'):
        (config_directory / name).write_text((current / name).read_text())
    payload_path = config_directory / 'slung_payload.yaml'
    document = yaml.safe_load(payload_path.read_text())
    baseline = document['slung_payload']['simulation']
    baseline.update(onboard_controller='pid', pose_delay_s=.08, command_delay_s=.013)
    baseline['model_validation_metadata'] = {'source': 'fixture', 'tags': ['original'],
                                             'nested': {'keep': 42, 'replace': 1}}
    payload_path.write_text(yaml.safe_dump(document, sort_keys=False))
    monkeypatch.setattr(controller_helpers, 'CONTROLLER_CONFIG_PATH',
                        config_directory / 'ctbr_controller.yaml')
    module = controller_helpers._controller_module()
    overlay = {'slung_payload': {'simulation': {'control_intervals_s': [.01, .022],
        'model_validation_metadata': {'tags': ['new'], 'nested': {'replace': 2}}}}}
    original_overlay = deepcopy(overlay)
    manager, _ = controller_helpers._slung_manager(module, tmp_path / 'logs', overlay)
    try:
        loaded = module.rospy.get_param('/slung_payload')['simulation']
        settings = replay.resolve_simulation_settings(module.rospy.get_param('/slung_payload'))
        assert settings['onboard_controller'] == 'pid'
        assert settings['pose_delay_s'] == .08
        assert settings['command_delay_s'] == .013
        assert settings['control_intervals_s'] == [.01, .022]
        for key, value in baseline.items():
            if key not in ('control_intervals_s', 'model_validation_metadata'):
                assert loaded[key] == value
        assert loaded['model_validation_metadata'] == {
            'source': 'fixture', 'tags': ['new'], 'nested': {'keep': 42, 'replace': 2}}
        # Mutating the loaded ROS stub data must not mutate the caller's overlay.
        loaded['control_intervals_s'].append(.03)
        loaded['model_validation_metadata']['tags'].append('loaded-only')
        assert overlay == original_overlay
    finally:
        manager.logger.close()
