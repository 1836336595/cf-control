"""Transport rate feedforward uses the final force frame and actual elapsed time."""
from dataclasses import replace
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_ctbr_controller_v2 import _controller_module, _config, _state, _target, _slung_manager


def case(**options):
    module = _controller_module()
    cfg = dict(transport_attitude_mode='feedforward_p', omega_c_method='log_difference')
    cfg.update(options)
    controller = module.GeometricCtbrController(_config(module, **cfg))
    target = _target()
    target.update(transport_mode=True, transport_blend=1.,
                  desired_force_override=np.array([0.,0.,.5]),
                  desired_force_dot_override=np.zeros(3),
                  transport_attitude_gain=np.array([12.,12.,6.]),
                  transport_rate_gain=np.array([9.,9.,9.]))
    return controller, _state(), target


def test_feedforward_is_added_in_measured_body_frame_without_rate_damping_or_attitude_i():
    c,s,t=case()
    s['body_rate']=np.array([.7,-.5,.4])
    s['rotation']=Rotation.from_rotvec([.04,-.03,.02]).as_matrix()
    c.attitude_integral[:]=.3
    a=c.compute(s,t,.02)
    np.testing.assert_array_equal(a['computed_body_rate'],np.zeros(3))
    t.update(yaw=.015,desired_force_override=np.array([.015,-.01,.5]))
    b=c.compute(s,t,.02)
    expected=Rotation.from_matrix(a['computed_rotation'].T@b['computed_rotation']).as_rotvec()/.02
    np.testing.assert_allclose(b['computed_body_rate'],expected,atol=1e-12)
    np.testing.assert_allclose(b['raw_body_rate_command'],
        s['rotation'].T@b['computed_rotation']@expected-t['transport_attitude_gain']*b['attitude_error'],atol=1e-12)
    np.testing.assert_array_equal(c.attitude_integral,[.3]*3)
    assert b['omega_c_method']=='transport_log_difference'


def test_diff_uses_elapsed_time_before_integrator_dt_clipping():
    c,s,t=case()
    c.compute(s,t,.02)
    t['yaw']=.04
    out=c.compute(s,t,.08)
    assert out['computed_body_rate'][2]==pytest.approx(.5)


def test_control_timestamp_not_cached_reference_tick_owns_feedforward_time():
    c,s,t=case()
    t.update(attitude_control_time_s=10.,transport_full_pid_tick=1.)
    c.compute(s,t,.01)
    t.update(yaw=.02,attitude_control_time_s=10.04)
    assert c.compute(s,t,.01)['computed_body_rate'][2]==pytest.approx(.5)
    # Recomputing the same timestamp supplies no new derivative sample.
    assert np.all(c.compute(s,t,.01)['computed_body_rate']==0.)


@pytest.mark.parametrize('event',['reset','reentry','long_gap','zero_force','filter_reset'])
def test_discontinuous_entry_has_no_derivative_kick(event):
    c,s,t=case()
    c.compute(s,t,.01)
    step=.01
    if event=='reset':c.reset()
    if event=='reentry':c.compute(s,dict(t,independent_mode=True),.01)
    if event=='long_gap':step=.2
    if event=='zero_force':c.compute(s,dict(t,desired_force_override=np.zeros(3)),.01)
    if event=='filter_reset':t['transport_filter_reset']=True
    t['yaw']=.2
    np.testing.assert_array_equal(c.compute(s,t,step)['computed_body_rate'],np.zeros(3))


def test_same_final_reference_does_not_replay_old_feedforward_on_cached_hold():
    c,s,t=case()
    c.compute(s,t,.01)
    t['yaw']=.01
    assert c.compute(s,t,.01)['computed_body_rate'][2]==pytest.approx(1.)
    t['transport_full_pid_force_hold']=True
    np.testing.assert_allclose(c.compute(s,t,.01)['computed_body_rate'],0.,atol=1e-12)


def test_feedforward_differentiates_filtered_limited_force_frame_not_raw_requested_force():
    c,s,t=case(max_tilt_rad=math.radians(10.),transport_force_filter_tau_s=.02)
    first=c.compute(s,t,.01)
    t['desired_force_override']=np.array([.5,0.,.5])
    second=c.compute(s,t,.01)
    expected=Rotation.from_matrix(first['computed_rotation'].T@second['computed_rotation']).as_rotvec()/.01
    np.testing.assert_allclose(second['computed_body_rate'],expected,atol=1e-12)
    assert np.linalg.norm(second['computed_body_rate']) < math.radians(10.)/.01


def test_feedforward_history_survives_normal_transport_landing_transition():
    c,s,t=case()
    t['flight_phase']='hover'
    c.compute(s,t,.02)
    t.update(yaw=.01,flight_phase='landing')
    assert c.compute(s,t,.02)['computed_body_rate'][2]==pytest.approx(.5)


def test_rollback_mode_keeps_original_damping_law():
    c,s,t=case(transport_attitude_mode='rate_damping')
    s['body_rate']=np.array([.01,-.02,.03])
    t['yaw']=.01
    out=c.compute(s,t,.02)
    np.testing.assert_allclose(out['raw_body_rate_command'],
        -t['transport_attitude_gain']*out['attitude_error']-t['transport_rate_gain']*s['body_rate'])
    np.testing.assert_array_equal(out['computed_body_rate'],np.zeros(3))
    assert out['omega_c_method']=='slung_load_matlab'


def test_handoff_blends_independent_p_and_new_feedforward_p_with_output_limits():
    c,s,t=case(max_body_rate=np.array([.2,.2,.2]),independent_attitude_gain=np.array([4.,4.,4.]))
    t.update(transport_blend=.5,handoff_force=np.array([0.,0.,.5]))
    c.compute(s,t,.01)
    t['yaw']=.02
    out=c.compute(s,t,.01)
    expected=.5*(-np.array([4.,4.,4.])*out['attitude_error'])+.5*(
        out['computed_rotation']@out['computed_body_rate']-t['transport_attitude_gain']*out['attitude_error'])
    np.testing.assert_allclose(out['raw_body_rate_command'],expected,atol=1e-12)
    assert out['body_rate_limit_active']
    assert max(abs(out['body_rate_command']))<=.2


def test_new_mode_does_not_use_attitude_integral_even_without_handoff_force():
    c,s,t=case(independent_attitude_gain=np.array([4.,4.,4.]),attitude_integral_gain=np.ones(3))
    c.attitude_integral[:]=5.
    t.update(transport_blend=.3,yaw=.02)
    out=c.compute(s,t,.01)
    np.testing.assert_allclose(out['raw_body_rate_command'],
        -.7*4.*out['attitude_error']-.3*t['transport_attitude_gain']*out['attitude_error'],atol=1e-12)
    np.testing.assert_array_equal(c.attitude_integral,[5.]*3)


def test_actual_node_logs_selected_mode_and_difference_method(tmp_path,monkeypatch):
    import csv
    from test_slung_landing_failure_containment import flight_case
    _,manager,_,_,tick=flight_case(tmp_path,monkeypatch)
    try:
        tick(18.02)
        manager.logger.flush()
        with open(manager.logger.path) as stream:rows=list(csv.DictReader(stream))
        transport=[r for r in rows if r['transport_mode']=='1']
        assert transport
        assert {r['transport_attitude_mode'] for r in transport}=={
            manager.vehicles[0].controller.config.transport_attitude_mode}
        assert {r['omega_c_method'] for r in transport}=={'transport_log_difference'}
    finally:manager.logger.close()


@pytest.mark.parametrize('changes',[{'transport_attitude_mode':'bad'},
    {'omega_c_method':'analytic'},{'use_body_rate_feedforward':False}])
def test_invalid_feedforward_mode_configuration_fails_before_control(changes):
    with pytest.raises(ValueError,match='transport_attitude_mode|feedforward_p'):case(**changes)


def test_live_yaml_enables_all_three_transport_controllers(tmp_path):
    m=_controller_module()
    manager,_=_slung_manager(m,tmp_path,use_live_payload_trajectory=True)
    try:
        for v in manager.vehicles:
            assert v.controller.config.transport_attitude_mode=='feedforward_pi'
            assert v.controller.config.omega_c_method=='log_difference'
            assert v.controller.config.use_body_rate_feedforward
    finally:manager.logger.close()
