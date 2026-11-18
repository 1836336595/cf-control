"""Lowering scheduling changes only rotational feedback and derivative FF."""
import csv
import numpy as np
import pytest
from test_slung_reference_cascade import _case
from test_transport_attitude_feedforward import case
from test_slung_lowering_feedforward import lowering


def test_scaled_se3_feedback_preserves_translation_support_and_reference_inertia():
    core, state, vehicles, target = _case('reference_cascade')
    core.config.se3_W=np.diag([.001,.002,.003])
    state.update(feedback_mode='velocity_free_beta',
                 auxiliary_rotation=np.eye(3),auxiliary_position=np.array([.001,-.002,.003]))
    first=core.compute(state,vehicles,target,.01)
    scaled=core.compute(state,vehicles,dict(target,payload_attitude_feedback_scale=.3),.01)
    # Scaling does not change the auxiliary dynamics or reference inertial law.
    np.testing.assert_array_equal(first['payload_se3_upsilon'],scaled['payload_se3_upsilon'])
    np.testing.assert_array_equal(first['payload_se3_psi_error'],scaled['payload_se3_psi_error'])
    np.testing.assert_array_equal(first['payload_se3_psi_tilde'],scaled['payload_se3_psi_tilde'])
    np.testing.assert_array_equal(first['desired_force'],scaled['desired_force'])
    np.testing.assert_allclose(scaled['payload_se3_control_wrench'][:3],
        first['payload_se3_upsilon'][:3]+.3*(first['payload_se3_control_wrench'][:3]-first['payload_se3_upsilon'][:3]),atol=1e-15)
    np.testing.assert_allclose(scaled['desired_moment'],core.config.se3_moment_limit_nm*np.tanh(
        scaled['payload_se3_control_wrench'][:3]/core.config.se3_moment_limit_nm),atol=1e-15)
    assert scaled['payload_attitude_feedback_scale']==.3


def test_zero_lowering_ff_retains_raw_evidence_and_direct_attitude_p():
    c,s,t=case();lowering(t);c.compute(s,t,.02)
    t.update(yaw=.08,payload_landing_attitude_feedforward_gain=0.)
    out=c.compute(s,t,.02)
    assert out['transport_landing_feedforward_raw'][2]==pytest.approx(4.)
    np.testing.assert_array_equal(out['computed_body_rate'],np.zeros(3))
    np.testing.assert_allclose(out['raw_body_rate_command'],-t['transport_attitude_gain']*out['attitude_error'])
    assert out['transport_landing_feedforward_active'] and out['transport_landing_feedforward_gain']==0.
    t.update(flight_validation_landing_stage='',yaw=.10)
    out=c.compute(s,t,.02)
    assert out['computed_body_rate'][2]==pytest.approx(1.)
    assert out['transport_landing_feedforward_gain']==1.


def test_real_mission_schedule_is_continuous_from_hover_and_logs_both_authorities(tmp_path,monkeypatch):
    from test_slung_landing_failure_containment import flight_case
    _,manager,_,_,tick=flight_case(tmp_path,monkeypatch)
    try:
        tick(18.02)
        start=manager.payload_landing_start_time
        for j in range(1,100):tick(start+.02*j)
        # Quintic cancellation can overshoot 1 by a few ULPs close to the
        # endpoint. A zero target gain must never become a negative gain.
        tick(start+2.*.99999999)
        tick(start+2.)
        manager.logger.flush()
        with open(manager.logger.path) as stream:rows=list(csv.DictReader(stream))
        lower=[r for r in rows if r['flight_validation_landing_stage']=='payload_lowering' and r['vehicle_id']=='3']
        scales=np.array([float(r['payload_attitude_feedback_scale']) for r in lower])
        # actual gain in the pure controller may reset at long/first intervals;
        # after 2 seconds the scheduled target is zero and stays zero.
        assert scales[0]==pytest.approx(1.)
        assert scales[-1]==pytest.approx(.3)
        assert np.all(np.diff(scales)<=1e-12)
        assert np.max(abs(np.diff(scales)))<.02
        for vehicle in manager.vehicles:
            target=manager.last_transport_overrides[id(vehicle)]
            assert target['payload_landing_attitude_feedforward_gain']==pytest.approx(0.)
            assert target['payload_attitude_feedback_scale']==pytest.approx(.3)
        assert float(lower[-1]['transport_landing_feedforward_gain'])==pytest.approx(0.)
    finally:manager.logger.close()


@pytest.mark.parametrize('name',['landing_payload_attitude_feedback_scale','landing_attitude_feedforward_gain'])
@pytest.mark.parametrize('bad',[True,-.1,1.1,float('nan'),'0.3'])
def test_bad_scheduling_parameters_reject_initialization(tmp_path,name,bad):
    from test_ctbr_controller_v2 import _controller_module,_slung_manager
    with pytest.raises(RuntimeError,match='slung_payload'):
        _slung_manager(_controller_module(),tmp_path,{'slung_payload':{name:bad}})
