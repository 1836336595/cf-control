"""Coordinate, causality, rejection and ROS integration checks for beta."""

import csv
import math
import numpy as np
import pytest
from scipy.linalg import expm

from slung_velocity_free import VelocityFreePayloadObserver, se3_step
from slung_load_controller import _hat, _rotation_exp, PayloadStateObserver
from test_slung_observer_state_integration import _manager, _message
from test_ctbr_controller_v2 import _controller_module, _slung_manager


def observer(**kwargs):
    return VelocityFreePayloadObserver(max_velocity_mps=[1.]*3,
        max_body_rate_rps=[8.]*3, max_acceleration_mps2=[8.]*3, **kwargs)


def test_se3_step_matches_matrix_exponential_with_nonzero_translation_and_rotation():
    r = _rotation_exp([.2, -.1, .5])
    p = np.array([.8, -.3, .6])
    beta = np.array([.3, -.2, .1, .4, .15, -.2])
    x = np.eye(4); x[:3, :3] = r; x[:3, 3] = p
    hat = np.zeros((4, 4)); hat[:3, :3] = _hat(beta[:3]); hat[:3, 3] = beta[3:]
    expected = x @ expm(.027 * hat)
    rn, pn = se3_step(r, p, beta, .027)
    np.testing.assert_allclose(rn, expected[:3, :3], atol=1.e-13)
    np.testing.assert_allclose(pn, expected[:3, 3], atol=1.e-13)


def test_beta_matches_matlab_se3_adjoint_and_weighted_gradient():
    o = observer()
    rh = _rotation_exp([.1, .05, -.2]); ph = np.array([.1, .2, .6])
    o.update(ph, rh, 1.)
    r = rh @ _rotation_exp([.002, -.003, .004]); p = ph + [.001, -.002, .0005]
    rd = _rotation_exp([-.1, .15, .3]); pd = np.array([.12, .18, .6])
    vd = np.array([.05, -.03, .02]); wd = np.array([.02, -.04, .03])
    o.set_reference(dict(position=pd, rotation=rd, velocity=vd, body_rate=wd), 1.01)
    state = o.update(p, r, 1.01)
    xe = np.eye(4); xe[:3, :3] = rd.T @ r; xe[:3, 3] = rd.T @ (p-pd)
    invxe = np.linalg.inv(xe)
    ad = np.zeros((6, 6)); ad[:3,:3] = ad[3:,3:] = invxe[:3,:3]
    ad[3:,:3] = _hat(invxe[:3,3]) @ invxe[:3,:3]
    rh_aux = state['auxiliary_rotation']; ph_aux = state['auxiliary_position']
    xt = np.eye(4); xt[:3,:3] = rh_aux.T @ r; xt[:3,3] = rh_aux.T @ (p-ph_aux)
    w = np.diag(o.weights); d = 2.
    m = w @ xt[:3,:3] - xt[:3,:3].T @ w
    psi = np.r_[.5*np.array([m[2,1],m[0,2],m[1,0]]), d/2*xt[:3,:3].T @ xt[:3,3]]
    gain = np.diag(np.r_[o.kappa_attitude/o.wbar, np.full(3, o.kappa_position*2/d)])
    expected = ad @ np.r_[wd, rd.T @ vd] + gain @ psi
    np.testing.assert_allclose(state['beta_body'], expected, atol=1.e-12)
    np.testing.assert_allclose(state['velocity'], r @ expected[3:], atol=1.e-12)


def test_constant_motion_converges_without_any_measured_velocity():
    o = observer()
    for i in range(501):
        t = i*.01
        state = o.update([.12*t, -.08*t, .6], _rotation_exp([0.,0.,.2*t]), 1.+t)
    assert state['observer_quality_valid']
    np.testing.assert_allclose(state['velocity'], [.12,-.08,0.], atol=.002)
    np.testing.assert_allclose(state['body_rate'], [0.,0.,.2], atol=.002)
    np.testing.assert_allclose(o.auxiliary_rotation.T @ o.auxiliary_rotation, np.eye(3), atol=1.e-12)


@pytest.mark.parametrize('bad', ['duplicate', 'backward', 'jump', 'nan', 'short'])
def test_rejected_or_pending_frames_do_not_advance_auxiliary_or_refresh_accepted_time(bad):
    o = observer()
    for i in range(6): o.update([.001*i,0,.6], np.eye(3), 1.+.01*i)
    before = o._snapshot()
    t, p = 1.06, np.array([.006,0,.6])
    if bad == 'duplicate': t = 1.05
    if bad == 'backward': t = 1.04
    if bad == 'jump': p[0] = 1.
    if bad == 'nan': p[0] = math.nan
    if bad == 'short': t = 1.051
    after = o.update(p, np.eye(3), t)
    assert not after['observer_quality_valid']
    assert after['sample_time'] == before['sample_time']
    for key in ['auxiliary_position', 'auxiliary_rotation', 'beta_body']:
        np.testing.assert_array_equal(after[key], before[key])


def test_persistent_bad_frames_cannot_restart_observer_and_real_gap_rewarms():
    o = observer()
    for i in range(5): o.update([0,0,.6], np.eye(3), 1.+.01*i)
    resets = o.reset_count
    for i in range(5,60):
        state = o.update([1,0,.6], np.eye(3), 1.+.01*i)
        assert not state['observer_quality_valid']
        assert state['sample_time'] == 1.04
    assert o.reset_count == resets
    state = o.update([1,0,.6], np.eye(3), 2.)
    assert state['observer_status'] == 'gap_reset'
    assert not state['derivatives_valid']
    np.testing.assert_array_equal(state['beta_body'], np.zeros(6))


def test_reference_expires_without_affecting_sample_age():
    o = observer()
    for i in range(5): o.update([0,0,.6], np.eye(3), 1.+.01*i)
    o.set_reference(dict(position=[0,0,.6], velocity=[.1,0,0]), 1.04)
    assert o.update([0,0,.6], np.eye(3), 1.05)['beta_reference_active']
    for i in range(6,12): state = o.update([0,0,.6], np.eye(3), 1.+.01*i)
    assert not state['beta_reference_active']
    assert state['sample_time'] == 1.11


def test_beta_bound_rejects_candidate_instead_of_returning_clipped_healthy_rate():
    o = observer(kappa_attitude=20.)
    o.max_body_rate = np.full(3, .1)
    for i in range(5): o.update([0,0,.6], np.eye(3), 1.+i*.01)
    state = o.update([0,0,.6], _rotation_exp([.02,0,0]), 1.05)
    assert state['observer_status'] == 'beta_limit'
    assert not state['observer_quality_valid']
    assert state['body_rate_clipped']
    np.testing.assert_array_equal(state['body_rate'], np.zeros(3))


def test_slung_core_uses_matlab_full_se3_wrench_in_beta_mode(tmp_path):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path,
        {'slung_payload': {'state_feedback_mode': 'velocity_free_beta'}})
    try:
        core = manager.slung_controller
        np.testing.assert_allclose(
            core.config.se3_W, np.diag(manager.payload_state_observer.weights), atol=0.)
        expected_d = (2. * core.config.payload_mass_kg
                      * np.mean(core.config.position_gain) / core.config.se3_kx)
        assert core.config.se3_d == expected_d
        beta = np.array([.1, .2, -.15, .03, -.02, .01])
        state = dict(valid=True, position=np.array([0.,0.,.6]), rotation=np.eye(3),
            velocity=beta[3:].copy(), body_rate=beta[:3].copy(), acceleration=np.zeros(3),
            auxiliary_position=np.array([0.,0.,.6]), auxiliary_rotation=np.eye(3),
            feedback_mode='velocity_free_beta', raw_velocity=np.full(3,99.), raw_body_rate=np.full(3,99.))
        vehicles = [dict(position=state['position']+core.config.attachment_points_m[:, i]+[0,0,.8],
                         velocity=np.zeros(3), mass=.043) for i in range(3)]
        target = dict(position=state['position'].copy(), velocity=np.zeros(3),
            acceleration=np.zeros(3), rotation=np.eye(3),
            body_rate=np.zeros(3), body_rate_dot=np.zeros(3),
            payload_attitude_gain=np.ones(3)*.001, payload_rate_gain=np.ones(3)*.002)
        result = core.compute(state, vehicles, target, .01)
        force_limit = core.config.se3_force_limit_n
        gravity_load = core.config.payload_mass_kg * core.config.gravity_mps2
        expected_force = np.array([0., 0., force_limit * np.tanh(gravity_load / force_limit)])
        np.testing.assert_allclose(result['desired_force'], expected_force, atol=1.e-12)
        np.testing.assert_allclose(result['desired_moment'], np.zeros(3), atol=1.e-12)
        np.testing.assert_allclose(result['payload_velocity_error'], beta[3:])
        assert np.linalg.norm(result['payload_se3_upsilon']) == 0.
        assert np.linalg.norm(result['payload_se3_psi_error']) == 0.
        assert np.linalg.norm(result['payload_se3_psi_tilde']) == 0.
        np.testing.assert_array_equal(core.position_integral, np.zeros(3))
        core.reset();state['feedback_mode']='pose_observer'
        state.pop('auxiliary_position'); state.pop('auxiliary_rotation')
        legacy = core.compute(state, vehicles, target, .01)
        expected_legacy = (
            np.cross(beta[:3], core.config.payload_inertia_kgm2*beta[:3])
            - .002 * beta[:3])
        np.testing.assert_allclose(legacy['desired_moment'], expected_legacy, atol=1.e-12)
    finally:
        manager.logger.close()


def test_slung_core_se3_wrench_matches_matlab_adjoint_and_potential_terms(tmp_path):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path,
        {'slung_payload': {'state_feedback_mode': 'velocity_free_beta'}})
    try:
        core = manager.slung_controller
        rotation = _rotation_exp([.035, -.018, .052])
        position = np.array([.014, -.009, .624])
        auxiliary_rotation = _rotation_exp([-.012, .021, -.006])
        auxiliary_position = np.array([.003, .004, .611])
        beta = np.array([.08, -.05, .03, .04, -.02, .01])
        state = dict(
            valid=True, position=position, rotation=rotation,
            velocity=rotation @ beta[3:], body_rate=beta[:3],
            acceleration=np.zeros(3), auxiliary_rotation=auxiliary_rotation,
            auxiliary_position=auxiliary_position, feedback_mode='velocity_free_beta')
        vehicles = [dict(
            position=position + core.config.attachment_points_m[:, i] + [0., 0., .8],
            velocity=np.zeros(3), mass=.043) for i in range(3)]
        target = dict(
            position=np.array([-.006, .008, .605]),
            velocity=np.array([.035, -.024, .012]),
            acceleration=np.array([.004, -.002, .003]),
            rotation=_rotation_exp([-.014, .027, .041]),
            body_rate=np.array([.021, -.013, .009]),
            body_rate_dot=np.array([-.003, .005, .002]),
            payload_yaw_enabled=True)
        result = core.compute(state, vehicles, target, .01)

        def adjoint(r, p):
            value = np.zeros((6, 6))
            value[:3, :3] = value[3:, 3:] = r
            value[3:, :3] = _hat(p) @ r
            return value

        def vee(matrix):
            return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])

        rd, pd = target['rotation'], target['position']
        omega_d, alpha_d = target['body_rate'], target['body_rate_dot']
        error_rotation = rd.T @ rotation
        error_position = rd.T @ (position - pd)
        inverse_error_rotation = error_rotation.T
        inverse_error_position = -inverse_error_rotation @ error_position
        adjoint_inverse_error = adjoint(inverse_error_rotation, inverse_error_position)
        reference_twist = np.r_[omega_d, rd.T @ target['velocity']]
        reference_twist_dot = np.r_[
            alpha_d,
            -_hat(omega_d) @ (rd.T @ target['velocity'])
            + rd.T @ target['acceleration']]
        transported_twist = adjoint_inverse_error @ reference_twist
        ad = np.zeros((6, 6))
        ad[:3, :3] = _hat(transported_twist[:3])
        ad[3:, :3] = _hat(transported_twist[3:])
        ad[3:, 3:] = _hat(transported_twist[:3])
        spatial_inertia = np.zeros((6, 6))
        spatial_inertia[:3, :3] = np.diag(core.config.payload_inertia_kgm2)
        spatial_inertia[3:, 3:] = core.config.payload_mass_kg * np.eye(3)
        upsilon = (-ad.T @ spatial_inertia @ transported_twist
                   + spatial_inertia @ adjoint_inverse_error @ reference_twist_dot)

        psi_error = np.r_[
            .5 * vee(core.config.se3_W @ error_rotation
                     - error_rotation.T @ core.config.se3_W),
            .5 * core.config.se3_d * error_rotation.T @ error_position]
        tilde_rotation = auxiliary_rotation.T @ rotation
        tilde_position = auxiliary_rotation.T @ (position - auxiliary_position)
        psi_tilde = np.r_[
            .5 * vee(core.config.se3_W @ tilde_rotation
                     - tilde_rotation.T @ core.config.se3_W),
            .5 * core.config.se3_d * tilde_rotation.T @ tilde_position]
        gravity = np.array([0., 0., core.config.gravity_mps2])
        wrench = (upsilon - core.config.se3_kx * psi_error
                  - core.config.se3_kxtilde * psi_tilde
                  + np.r_[np.zeros(3), core.config.payload_mass_kg * rotation.T @ gravity])
        expected_moment = core.config.se3_moment_limit_nm * np.tanh(
            wrench[:3] / core.config.se3_moment_limit_nm)
        expected_force = rotation @ (
            core.config.se3_force_limit_n * np.tanh(
                wrench[3:] / core.config.se3_force_limit_n))

        np.testing.assert_allclose(result['payload_se3_upsilon'], upsilon, atol=1.e-12)
        np.testing.assert_allclose(result['payload_se3_psi_error'], psi_error, atol=1.e-12)
        np.testing.assert_allclose(result['payload_se3_psi_tilde'], psi_tilde, atol=1.e-12)
        np.testing.assert_allclose(result['desired_moment'], expected_moment, atol=1.e-12)
        np.testing.assert_allclose(result['desired_force'], expected_force, atol=1.e-12)
    finally:
        manager.logger.close()


def test_ros_ignores_poisoned_twist_and_uses_beta_for_all_payload_derivatives():
    manager = _manager()
    manager.payload_state_observer = observer()
    for i in range(11):
        manager._payload_state_callback(_message(10.+.01*i, (.001*i,0,.53), .001*i))
    state = manager.latest_payload_state
    assert manager._payload_ready(state['received_time'])
    assert state['feedback_mode'] == 'velocity_free_beta'
    np.testing.assert_allclose(state['velocity'], state['rotation'] @ state['beta_body'][3:])
    np.testing.assert_allclose(state['body_rate'], state['beta_body'][:3])
    np.testing.assert_array_equal(state['acceleration'], np.zeros(3))
    assert state['raw_velocity'][0] == 99.
    assert np.linalg.norm(state['velocity']) < .15


@pytest.mark.parametrize('mode,cls', [('pose_observer', PayloadStateObserver),
                                     ('velocity_free_beta', VelocityFreePayloadObserver)])
def test_parameter_selection_and_real_csv_fields(tmp_path, mode, cls):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path,
        {'slung_payload': {'state_feedback_mode': mode}})
    try:
        assert type(manager.payload_state_observer) is cls
        o = manager.payload_state_observer
        for i in range(5): state = o.update([0,0,.6], np.eye(3), 1.+i*.01)
        state.update(valid=True, received_time=1.04)
        vehicle = manager.vehicles[0]
        vehicle.set_payload_diagnostics(state)
        row = vehicle._base_log_row(1.04, vehicle.latest_state)
        manager.logger.write(row); manager.logger.flush()
        with open(manager.logger.path, newline='') as stream: saved = list(csv.DictReader(stream))[-1]
        assert saved['payload_state_feedback_mode'] == mode
        if mode == 'velocity_free_beta':
            assert float(saved['payload_beta_omega_body_x']) == 0.
            assert saved['payload_acceleration_available'] == '0'
    finally:
        manager.logger.close()


@pytest.mark.parametrize('kwargs', [dict(kappa_position=0.), dict(kappa_attitude=21.),
    dict(attitude_weights=[1,1,1]), dict(attitude_weights=[1,-1,2])])
def test_bad_beta_parameters_fail_before_startup(kwargs):
    with pytest.raises(ValueError): observer(**kwargs)
