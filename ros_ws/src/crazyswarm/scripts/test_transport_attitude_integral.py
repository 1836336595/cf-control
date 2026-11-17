"""Bounded aircraft attitude I augments the transport rate command only."""
import numpy as np
import pytest
from test_transport_attitude_feedforward import case


def setup():
    c,s,t=case(transport_attitude_mode='feedforward_pi',
        transport_attitude_integral_gain=(.4,.4,.2),
        transport_attitude_integral_limit_rps=(.15,.15,.1),
        transport_attitude_integral_rate_limit_rps2=(.02,.02,.01))
    t.update(yaw=.05,flight_phase='hover',transport_attitude_integral_learning=True)
    return c,s,t


def tick(c,s,t,k):
    t['attitude_control_time_s']=10.+.02*k
    return c.compute(s,t,.02)


def test_constant_attitude_error_accumulates_compensating_rate_without_touching_force():
    c,s,t=setup()
    first=tick(c,s,t,0)
    for k in range(1,101):last=tick(c,s,t,k)
    bias=last['transport_attitude_integral_rate']
    assert bias[2]>0.
    assert bias[2]==pytest.approx(.2*np.sin(.05)*2.)
    np.testing.assert_allclose(last['raw_body_rate_command']-first['raw_body_rate_command'],bias,atol=1e-12)
    np.testing.assert_array_equal(last['desired_force'],first['desired_force'])
    np.testing.assert_array_equal(c.attitude_integral,0.)


@pytest.mark.parametrize('fault',['cached','bad_derivatives','held_ekf','landing','ramp','external','large_error','long_gap'])
def test_fault_freezes_but_preserves_existing_bias(fault):
    c,s,t=setup()
    for k in range(20):tick(c,s,t,k)
    before=c.transport_attitude_integral_rate.copy()
    if fault=='cached':t['transport_full_pid_force_hold']=True
    if fault=='bad_derivatives':s['derivatives_valid']=False
    if fault=='held_ekf':s['ekf_kinematics_held']=True
    if fault=='landing':t['flight_phase']='landing'
    if fault=='ramp':t['transport_blend']=.5
    if fault=='external':t['transport_attitude_integral_learning']=False
    if fault=='large_error':t['yaw']=1.
    k=40 if fault=='long_gap' else 20
    out=tick(c,s,t,k)
    np.testing.assert_array_equal(c.transport_attitude_integral_rate,before)
    assert not out['transport_attitude_integral_learning']


def test_duplicate_tick_does_not_integrate_twice_and_reset_clears_bias():
    c,s,t=setup()
    tick(c,s,t,0);tick(c,s,t,1)
    before=c.transport_attitude_integral_rate.copy()
    tick(c,s,t,1)
    np.testing.assert_array_equal(c.transport_attitude_integral_rate,before)
    c.reset()
    np.testing.assert_array_equal(c.transport_attitude_integral_rate,0.)


def test_reversed_timestamp_cannot_make_an_already_used_tick_learn_again():
    c,s,t=setup()
    tick(c,s,t,0);tick(c,s,t,2)
    before=c.transport_attitude_integral_rate.copy()
    tick(c,s,t,1);tick(c,s,t,2)
    np.testing.assert_array_equal(c.transport_attitude_integral_rate,before)


def test_bias_and_increment_have_separate_limits():
    c,s,t=setup();t['yaw']=.2
    previous=np.zeros(3)
    for k in range(1000):
        out=tick(c,s,t,k);b=out['transport_attitude_integral_rate']
        assert np.all(abs(b)<=np.array([.15,.15,.1])+1e-12)
        assert np.all(abs(b-previous)<=np.array([.02,.02,.01])*.02+1e-12)
        previous=b.copy()
    assert b[2]==pytest.approx(.1)


def test_rate_saturation_cannot_wind_up_integral_but_opposite_error_can_unwind():
    c,s,t=setup();c.config.max_body_rate=np.array([.2,.2,.2])
    for k in range(15):tick(c,s,t,k)
    assert c.transport_attitude_integral_rate[2]==0.
    c.transport_attitude_integral_rate[2]=.08
    t['yaw']=-.02
    tick(c,s,t,15)
    tick(c,s,t,16)  # The preceding yaw step has feedforward; now the reference is stationary.
    assert c.transport_attitude_integral_rate[2]<.08


def test_independent_exit_clears_transport_bias_and_normal_attitude_i_is_separate():
    c,s,t=setup()
    for k in range(20):tick(c,s,t,k)
    assert c.transport_attitude_integral_rate[2]>0.
    c.attitude_integral[:]=.2
    c.compute(s,dict(t,independent_mode=True),.02)
    np.testing.assert_array_equal(c.transport_attitude_integral_rate,0.)
    np.testing.assert_array_equal(c.attitude_integral,[.2]*3)


def test_current_force_limit_freezes_integral():
    c,s,t=setup()
    for k in range(20):tick(c,s,t,k)
    previous=c.transport_attitude_integral_rate.copy()
    t['desired_force_override']=np.array([0.,0.,2.])
    out=tick(c,s,t,20)
    assert out['transport_attitude_integral_freeze_reason']=='force_limited'
    np.testing.assert_array_equal(c.transport_attitude_integral_rate,previous)


def test_fleet_native_pi_gate_and_cached_hold_are_logged(tmp_path,monkeypatch):
    import csv
    from test_slung_landing_failure_containment import flight_case
    from test_slung_transport_integral_integration import fresh,ERRORS
    module,manager,anchors,published,tick_fleet=flight_case(tmp_path,monkeypatch)
    try:
        manager.payload_flight_validation_enabled=False
        tick_fleet(18.02)
        before=[v.controller.transport_attitude_integral_rate.copy() for v in manager.vehicles]
        fresh(module,manager,anchors,18.04,ERRORS)
        manager.latest_payload_state['derivatives_valid']=False
        monkeypatch.setattr(module.time,'monotonic',lambda:18.04)
        manager._timer_callback(None)
        for i,v in enumerate(manager.vehicles):
            np.testing.assert_array_equal(v.controller.transport_attitude_integral_rate,before[i])
            assert not v.last_control_command['transport_attitude_integral_learning']
            assert v.last_control_command['transport_attitude_integral_freeze_reason']=='payload_cached_hold'
        manager.logger.flush()
        with open(manager.logger.path) as f:rows=list(csv.DictReader(f))
        for row in rows[-3:]:
            assert row['transport_attitude_mode']=='feedforward_pi'
            assert row['transport_attitude_integral_learning']=='0'
            assert row['transport_attitude_integral_freeze_reason']=='payload_cached_hold'
            assert all(np.isfinite(float(row['transport_attitude_integral_rate_'+a])) for a in 'xyz')
    finally:manager.logger.close()


@pytest.mark.parametrize('bad',[(-1.,.4,.2),(True,.4,.2),(.4,np.nan,.2),(.4,.2)])
def test_invalid_integral_gain_fails_before_control(bad):
    with pytest.raises(ValueError,match='transport_attitude_integral'):
        case(transport_attitude_mode='feedforward_pi',transport_attitude_integral_gain=bad)
