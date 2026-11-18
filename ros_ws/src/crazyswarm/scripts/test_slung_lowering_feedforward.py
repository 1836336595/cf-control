"""Lowering-only rate shaping must preserve P/I and control-clock causality."""
import csv
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from test_transport_attitude_feedforward import case
from ctbr_transport_vehicle_controller import validate_landing_feedforward_settings


def lowering(target):
    target.update(flight_phase='landing', flight_validation_landing_stage='payload_lowering',
        payload_landing_attitude_feedforward_enabled=True,
        payload_landing_attitude_feedforward_filter_tau_s=.10,
        payload_landing_attitude_feedforward_limit_rps=[.6,.6,.5])
    return target


def test_spike_is_bounded_before_world_filter_and_attitude_feedback_remains_direct():
    c,s,t=case()
    lowering(t)
    c.compute(s,t,.02)
    t['yaw']=.12
    out=c.compute(s,t,.02)
    assert out['transport_landing_feedforward_raw'][2] == pytest.approx(6.)
    expected=(1.-math.exp(-.02/.1))*.5
    assert out['computed_body_rate'][2] == pytest.approx(expected)
    np.testing.assert_allclose(out['raw_body_rate_command'],
        out['computed_rotation']@out['computed_body_rate']
        -t['transport_attitude_gain']*out['attitude_error'])
    assert out['transport_landing_feedforward_limit_active']
    tail=c.compute(s,t,.02)['computed_body_rate'][2]
    assert tail == pytest.approx(expected*math.exp(-.02/.1))


@pytest.mark.parametrize('stage',['', 'vehicle_descent', 'payload_lowering'])
def test_scope_requires_lowering_and_enabled(stage):
    c,s,t=case()
    lowering(t);t['flight_validation_landing_stage']=stage
    if stage=='payload_lowering':t['payload_landing_attitude_feedforward_enabled']=False
    c.compute(s,t,.02);t['yaw']=.02
    out=c.compute(s,t,.02)
    assert out['computed_body_rate'][2] == pytest.approx(1.)
    assert not out['transport_landing_feedforward_active']


def test_filter_transports_history_in_world_axes_and_keeps_desired_body_axis_bounds():
    c,_,t=case();lowering(t)
    r1=Rotation.from_rotvec([.2,-.1,.3]).as_matrix()
    r2=Rotation.from_rotvec([-.4,.3,1.2]).as_matrix()
    raw1=np.array([.4,.5,.2]);raw2=np.array([.2,-.3,.4]);alpha=1-math.exp(-.02/.1)
    a=c._shape_transport_landing_feedforward(raw1,r1,t,.02)
    np.testing.assert_allclose(a,alpha*raw1,atol=1e-14)
    world=r1@a+alpha*(r2@raw2-r1@a)
    b=c._shape_transport_landing_feedforward(raw2,r2,t,.02)
    np.testing.assert_allclose(b,r2.T@world,atol=1e-14)
    assert np.all(abs(b)<=np.array([.6,.6,.5]))


def test_duplicate_control_time_does_not_advance_or_replay_filter():
    c,s,t=case();lowering(t);t['attitude_control_time_s']=1.
    c.compute(s,t,.02);t.update(yaw=.04,attitude_control_time_s=1.02)
    c.compute(s,t,.02);before=c._landing_feedforward_world.copy()
    out=c.compute(s,t,.02)
    np.testing.assert_array_equal(out['computed_body_rate'],np.zeros(3))
    np.testing.assert_array_equal(c._landing_feedforward_world,before)
    t.update(yaw=.05,attitude_control_time_s=1.04)
    assert c.compute(s,t,.02)['transport_landing_feedforward_raw'][2]==pytest.approx(.5)


@pytest.mark.parametrize('event',['reset','independent','gap','backward','degenerate','filter_reset'])
def test_discontinuities_clear_filter_state_without_old_rate_tail(event):
    c,s,t=case();lowering(t);t['attitude_control_time_s']=1.
    c.compute(s,t,.02);t.update(yaw=.06,attitude_control_time_s=1.02)
    c.compute(s,t,.02);assert np.linalg.norm(c._landing_feedforward_world)>0
    t['attitude_control_time_s']=1.04
    if event=='reset':c.reset()
    if event=='independent':
        c.compute(s,dict(t,independent_mode=True),.02)
    if event=='gap':t['attitude_control_time_s']=1.3
    if event=='backward':t['attitude_control_time_s']=1.01
    if event=='degenerate':t['desired_force_override']=np.zeros(3)
    if event=='filter_reset':t['transport_filter_reset']=True
    out=c.compute(s,t,.02)
    np.testing.assert_array_equal(out['computed_body_rate'],np.zeros(3))
    assert c._landing_feedforward_world is None


def test_cached_payload_still_uses_live_force_frame_and_unload_preserves_shaping():
    c,s,t=case();lowering(t);c.compute(s,t,.02)
    t.update(yaw=.04,transport_full_pid_force_hold=True,payload_landing_unloading=True)
    out=c.compute(s,t,.02)
    assert out['transport_landing_feedforward_active']
    assert out['computed_body_rate'][2]>0
    t.update(flight_validation_landing_stage='vehicle_descent',independent_mode=True)
    c.compute(s,t,.02);assert c._landing_feedforward_world is None


@pytest.mark.parametrize("event", ["degenerate", "disable"])
def test_duplicate_tick_cannot_hide_an_invalid_frame_or_disabled_mode(event):
    c,s,t=case();lowering(t);t['attitude_control_time_s']=1.
    c.compute(s,t,.02);t.update(yaw=.06,attitude_control_time_s=1.02)
    c.compute(s,t,.02)
    if event=='degenerate':
        t['desired_force_override']=np.zeros(3)
    else:
        t['payload_landing_attitude_feedforward_enabled']=False
    out=c.compute(s,t,.02)
    np.testing.assert_array_equal(out['computed_body_rate'],np.zeros(3))
    assert c._landing_feedforward_world is None
    assert not out['transport_landing_feedforward_active']


@pytest.mark.parametrize('enabled,tau,limits',[(1,.1,[.6]*3),(True,True,[.6]*3),
    (True,-.1,[.6]*3),(True,float('nan'),[.6]*3),(True,.1,[.6,0,.5]),
    (True,.1,[.6,float('inf'),.5]),(True,.1,[.6,.5]),(True,.1,[True,.5,.5])])
def test_invalid_settings_reject_at_parameter_boundary(enabled,tau,limits):
    with pytest.raises(ValueError):validate_landing_feedforward_settings(enabled,tau,limits)


@pytest.mark.parametrize("update", [
    {"landing_attitude_feedforward_enabled": 1},
    {"landing_attitude_feedforward_filter_tau_s": -.1},
    {"landing_attitude_feedforward_limit_rps": [.6, 0., .5]},
])
def test_invalid_flight_settings_fail_actual_loader(tmp_path, update):
    from test_ctbr_controller_v2 import _controller_module, _slung_manager
    with pytest.raises(RuntimeError, match="slung_payload"):
        _slung_manager(_controller_module(), tmp_path, {"slung_payload": update})


def test_real_fleet_lowering_and_csv_preserve_raw_applied_and_release_mode(tmp_path,monkeypatch):
    from test_slung_landing_failure_containment import flight_case
    _,manager,_,_,tick=flight_case(tmp_path,monkeypatch)
    try:
        tick(18.02);tick(18.04);manager.logger.flush()
        with open(manager.logger.path) as stream:rows=list(csv.DictReader(stream))
        rows=[r for r in rows if r['flight_validation_landing_stage']=='payload_lowering']
        assert rows
        assert all(r['transport_landing_feedforward_active']=='1' for r in rows[-3:])
        assert all(float(r['transport_landing_feedforward_filter_tau_s'])==.10 for r in rows[-3:])
        for r in rows[-3:]:
            assert all(math.isfinite(float(r['transport_landing_feedforward_raw_'+axis])) for axis in 'xyz')
            assert all(abs(float(r['transport_landing_feedforward_applied_'+axis]))<=bound+1e-12
                       for axis,bound in zip('xyz',[.6,.6,.5]))
    finally:manager.logger.close()
