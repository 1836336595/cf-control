"""Released-load descent tuning must not change takeoff or native transport."""

from copy import deepcopy
import csv

import numpy as np
import pytest

from test_ctbr_controller_v2 import _config, _controller_module, _slung_manager, _state, _target
from test_slung_landing_integral import landing_controller, landing_target


GAINS = dict(landing_position_gain_z=5., landing_velocity_gain_z=4.,
             landing_integral_gain_z=1.6)


def test_landing_only_gains_reach_force_and_integral_diagnostics_without_mutating_base():
    controller = landing_controller(**GAINS)
    controller.position_integral[:] = [.02, -.03, .4]
    originals = [value.copy() for value in (
        controller.config.independent_position_gain,
        controller.config.independent_velocity_gain,
        controller.config.independent_integral_gain)]
    state = _state()
    state['position'] = np.array([.01, -.02, .15])
    state['velocity'] = np.array([0., 0., -.03])
    target = landing_target()
    target.update(independent_position_gain_scale=np.array([4., 4., 2.]),
                  independent_velocity_gain_scale=np.array([2., 2., 2.]))
    command = controller.compute(state, target, .02)
    integral_z = .4 + .02 * (-.03 + .5 * .15)
    expected_z = controller.config.mass * (
        controller.config.gravity - 5. * .15 - 4. * (-.03) - 1.6 * integral_z)
    # Landing's explicit Z gain overrides only Z. Shared TAKEUP multipliers
    # continue applying to XY, rather than multiplying the new Z gains again.
    np.testing.assert_allclose(command['raw_desired_force'][2], expected_z, atol=1e-14)
    np.testing.assert_allclose(command['landing_vertical_integral_force_n'],
                               -controller.config.mass * 1.6 * integral_z, atol=1e-14)
    np.testing.assert_allclose(controller.position_integral[2], integral_z, atol=1e-14)
    assert command['landing_vertical_integral_active']
    assert command['landing_pid_active']
    np.testing.assert_array_equal(command['landing_pid_gains_z'], [5., 4., 1.6])
    for actual, saved in zip((controller.config.independent_position_gain,
                             controller.config.independent_velocity_gain,
                             controller.config.independent_integral_gain), originals):
        np.testing.assert_array_equal(actual, saved)


@pytest.mark.parametrize('phase', ['takeoff', 'prelift', 'ordinary_landing', 'transport'])
def test_new_gains_do_not_leak_to_takeoff_prelift_or_native_transport(phase):
    tuned = landing_controller(**GAINS)
    baseline = landing_controller()
    for controller in (tuned, baseline):
        controller.position_integral[:] = [.02, -.03, .1]
    state = _state()
    state['position'] = np.array([.03, -.01, .06])
    state['velocity'] = np.array([.01, -.02, -.015])
    target = _target()
    target.update(independent_mode=True, flight_phase='takeoff')
    if phase == 'prelift':
        target.update(prelift_vertical_integral_active=True,
                      prelift_vertical_integral_limit=8.,
                      prelift_vertical_integral_rate_limit=.5,
                      prelift_vertical_feedback_limit_mps2=12.)
    elif phase == 'ordinary_landing':
        target['flight_phase'] = 'landing'
    elif phase == 'transport':
        target.update(independent_mode=False, transport_mode=True,
                      # Even an accidentally retained landing opt-in must
                      # not alter native PID while the payload is suspended.
                      landing_vertical_integral_active=True,
                      flight_phase='landing', transport_blend=1.,
                      transport_feedback_controller='native_position_pid',
                      transport_full_pid_learning=True,
                      transport_full_pid_tick=1.,
                      desired_force_override=np.array([.01, -.02, .5]),
                      desired_force_dot_override=np.zeros(3),
                      transport_position_hold_gain_scale=1.5)
    for tick in range(5):
        target['transport_full_pid_tick'] = float(tick)
        actual = tuned.compute(state, target, .02)
        reference = baseline.compute(state, deepcopy(target), .02)
        for field in ('raw_desired_force', 'desired_force', 'body_rate_command',
                      'transport_full_pid_force', 'transport_full_pid_pd_force'):
            np.testing.assert_array_equal(actual[field], reference[field])
        assert not actual['landing_pid_active']
        np.testing.assert_array_equal(actual['landing_pid_gains_z'], np.zeros(3))
        np.testing.assert_array_equal(tuned.position_integral, baseline.position_integral)
        np.testing.assert_array_equal(tuned.transport_position_integral,
                                      baseline.transport_position_integral)


def test_omitted_landing_fields_keep_legacy_scaled_independent_landing():
    controller = landing_controller()
    target = landing_target()
    target['independent_position_gain_scale'] = np.array([4., 4., 1.5])
    target['independent_velocity_gain_scale'] = np.array([2., 2., 1.25])
    controller.position_integral[2] = .3
    state = _state()
    state['position'][2] = .1
    state['velocity'][2] = -.02
    command = controller.compute(state, target, .01)
    iz = .3 + .01 * (-.02 + .5 * .1)
    expected = controller.config.mass * (
        controller.config.gravity - 4. * 1.5 * .1 - 3.6 * 1.25 * (-.02) - .8 * iz)
    np.testing.assert_allclose(command['raw_desired_force'][2], expected, atol=1e-14)


def test_landing_integral_antiwindup_uses_selected_gain():
    controller = landing_controller(landing_position_gain_z=0.,
        landing_velocity_gain_z=0., landing_integral_gain_z=12.)
    controller.position_integral[2] = .24
    state = _state()
    state['position'][2] = .1
    state['velocity'][2] = .1
    target = landing_target()
    target['min_collective_thrust'] = .3
    # Candidate force is below the 0.3 N floor with the landing-only gain,
    # even though it would be above the floor with shared Ki=0.8.
    controller.compute(state, target, .02)
    assert controller.position_integral[2] == .24


@pytest.mark.parametrize('field', tuple(GAINS))
@pytest.mark.parametrize('bad', [True, np.bool_(False), '1.2', [1.2], -1., np.nan, np.inf])
def test_invalid_landing_gain_is_rejected_before_control(field, bad):
    module = _controller_module()
    with pytest.raises(ValueError, match=field):
        _config(module, **{field: bad})


def test_per_cf_landing_gain_loading_restores_shared_cf4_transport_gain(tmp_path):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    try:
        expected = {3: (4., 3.6, .8), 4: (5., 4., 1.6), 5: (4., 3.6, .8)}
        for vehicle in manager.vehicles:
            cfg = vehicle.controller.config
            assert tuple(getattr(cfg, field) for field in GAINS) == expected[vehicle.vehicle_id]
            assert cfg.independent_position_gain[2] == 4.
            assert cfg.independent_velocity_gain[2] == 3.6
            assert cfg.independent_integral_gain[2] == .8
    finally:
        manager.logger.close()


def test_real_parameter_loader_rejects_invalid_landing_scalar(tmp_path):
    module = _controller_module()
    with pytest.raises(ValueError, match='landing_integral_gain_z'):
        _slung_manager(module, tmp_path,
            {'ctbr_controller_cf4': {'landing_integral_gain_z': True}})


def test_fleet_release_descent_persists_each_effective_landing_pid_to_csv(tmp_path, monkeypatch):
    from test_slung_landing_failure_containment import flight_case

    _, manager, _, published, tick = flight_case(tmp_path, monkeypatch)
    try:
        # Switch at the same lifecycle boundary as measured payload release.
        # All subsequent manager/vehicle control and CSV paths are real;
        # only ROS transport and received model states use the fixture.
        manager._reset_payload_handoff()
        manager.payload_validation_landing_stage = 'vehicle_descent'
        manager._begin_validation_vehicle_descent(18.02)
        tick(18.04)
        manager.logger.flush()
        with open(manager.logger.path, newline='') as stream:
            rows = list(csv.DictReader(stream))
        for vehicle in manager.vehicles:
            command = published[id(vehicle)][-1]
            assert command['landing_pid_active']
            expected = [5., 4., 1.6] if vehicle.vehicle_id == 4 else [4., 3.6, .8]
            np.testing.assert_array_equal(command['landing_pid_gains_z'], expected)
            row = next(row for row in reversed(rows)
                       if int(row['vehicle_id']) == vehicle.vehicle_id)
            assert row['flight_validation_landing_stage'] == 'vehicle_descent'
            assert row['flight_phase'] == 'landing'
            assert row['landing_pid_active'] == '1'
            saved = [float(row['landing_pid_gain_' + label + '_z'])
                     for label in ('p', 'd', 'i')]
            np.testing.assert_array_equal(saved, expected)
            assert float(row['landing_vertical_integral_force_n']) == pytest.approx(
                command['landing_vertical_integral_force_n'])
    finally:
        manager.logger.close()
