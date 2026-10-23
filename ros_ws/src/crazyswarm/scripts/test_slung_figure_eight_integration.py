"""The real fleet manager must track both circular lobes with tangent heading."""

import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_attitude_reference import sample


def figure_case(tmp_path, **changes):
    payload = dict(trajectory=dict(mode="figure_eight", radius_m=.5, center_xy=[0.,0.],
        speed_mps=.1, revolutions=1., ramp_s=5., settle_s=3., direction="ccw"),
        transport_force_mode="reference_cascade", transport_attitude_trim_enabled=False,
        flight_validation_enabled=True, flight_validation_transport_s=100.)
    payload.update(changes)
    manager, anchors = _slung_manager(_controller_module(),tmp_path, {
        "ctbr_trajectory":{"hover_duration_s":100.}, "slung_payload":payload})
    manager.payload_transport_active=True
    manager.payload_transport_start_time=0.
    manager.payload_transport_start_position=manager.latest_payload_state["position"].copy()
    manager.payload_transport_vehicle_start_positions=anchors.copy()
    manager.payload_handoff_reference_positions=anchors.copy()
    sample(manager,0.)
    return manager


def test_figure_eight_starts_at_crossing_with_initial_tangent_heading(tmp_path):
    m=figure_case(tmp_path)
    try:
        t=next(iter(sample(m,19.).values()))
        np.testing.assert_allclose(t["payload_target_position"],[0.,0.,.6],atol=1e-12)
        np.testing.assert_allclose(t["payload_target_rotation"][:,0],[0.,-1.,0.],atol=1e-12)
        np.testing.assert_allclose(t["payload_target_body_rate"],0.,atol=1e-12)
    finally:m.logger.close()


def test_real_manager_figure_eight_exact_lobes_and_tangent_heading(tmp_path):
    m=figure_case(tmp_path)
    try:
        sample(m,19.)
        duration=m.payload_circle_reference.duration/2.
        for elapsed in [7.,15.,25.,duration+7.,duration+15.,duration+25.]:
            t=next(iter(sample(m,20.+elapsed).values()))
            center=np.array([.5 if elapsed<duration else -.5,0.,.6])
            p,v,r=t["payload_target_position"],t["payload_target_velocity"],t["payload_target_rotation"]
            assert np.linalg.norm(p-center)==pytest.approx(.5,abs=1e-12)
            assert p[2]==pytest.approx(.6)
            np.testing.assert_allclose(r[:,0],v/np.linalg.norm(v),atol=1e-12)
            assert t["payload_trajectory_mode"]=="figure_eight"
            assert t["payload_trajectory_phase"]=="figure_eight"
    finally:m.logger.close()


@pytest.mark.parametrize("now",[21.5,30.,57.9,67.,90.5])
def test_figure_eight_full_formation_and_attitude_reference_derivatives(tmp_path,now):
    m=figure_case(tmp_path)
    try:
        h=1e-3
        left,middle,right=[sample(m,t) for t in [now-h,now,now+h]]
        rho=m.slung_controller.config.attachment_points_m
        for i,v in enumerate(m.vehicles):
            a,b,c=[items[id(v)] for items in [left,middle,right]]
            for p,d,dd in [("payload_target_position","payload_target_velocity","payload_target_acceleration"),
                ("position","velocity","acceleration"),
                ("position_hold_position","position_hold_velocity","position_hold_acceleration")]:
                np.testing.assert_allclose((c[p]-a[p])/(2*h),b[d],atol=5e-8)
                np.testing.assert_allclose((c[p]-2*b[p]+a[p])/h**2,b[dd],atol=8e-8)
            r=b["payload_target_rotation"];w=b["payload_target_body_rate"]
            hat=np.array([[0.,-w[2],w[1]],[w[2],0.,-w[0]],[-w[1],w[0],0.]])
            np.testing.assert_allclose((c["payload_target_rotation"]-a["payload_target_rotation"])/(2*h),r@hat,atol=5e-8)
            np.testing.assert_allclose((c["payload_target_body_rate"]-a["payload_target_body_rate"])/(2*h),
                b["payload_target_body_rate_dot"],atol=5e-8)
            cable=b["payload_target_position"]+r@rho[:,i]-b["position_hold_position"]
            assert np.isclose(np.linalg.norm(cable),m.payload_transport_initial_link_distances[i],atol=1e-12)
    finally:m.logger.close()


def test_figure_eight_does_not_cut_off_at_old_sixty_second_deadline(tmp_path):
    with pytest.raises(RuntimeError,match="figure_eight|八字|轨迹"):
        figure_case(tmp_path,flight_validation_transport_s=60.)


def test_figure_eight_crossing_finishes_with_continuous_heading_and_zero_rates(tmp_path):
    m=figure_case(tmp_path)
    try:
        sample(m,19.)
        ref=m.payload_circle_reference
        for now in [ref.start_time+ref.duration/2.,ref.end_time]:
            t=next(iter(sample(m,now).values()))
            np.testing.assert_allclose(t["payload_target_position"],[0.,0.,.6],atol=1e-12)
            for key in ["payload_target_velocity","payload_target_acceleration","payload_target_body_rate","payload_target_body_rate_dot"]:
                np.testing.assert_allclose(t[key],0.,atol=1e-12)
            np.testing.assert_allclose(t["payload_target_rotation"][:,0],[0.,-1.,0.],atol=1e-12)
    finally:m.logger.close()


def test_figure_eight_uses_rotating_trim_and_freezes_slow_learning(tmp_path):
    m=figure_case(tmp_path,transport_attitude_trim_enabled=True)
    try:
        sample(m,19.)
        goal=m.payload_transport_goal_rotation.copy()
        angle=.1
        pitch=np.array([[np.cos(angle),0.,np.sin(angle)],[0.,1.,0.],[-np.sin(angle),0.,np.cos(angle)]])
        m.payload_attitude_trim.update(goal@pitch,goal,19.1,learning=True)
        before=m.payload_attitude_trim.integral.copy()
        t=sample(m,30.)
        np.testing.assert_array_equal(m.payload_attitude_trim.integral,before)
        assert all(v["transport_attitude_trim_freeze_reason"]=="reference_moving" for v in t.values())
    finally:m.logger.close()


@pytest.mark.parametrize("now",[3.,10.,30.,67.])
@pytest.mark.parametrize("trim_enabled",[False,True])
def test_figure_eight_interrupted_landing_preserves_current_pose_rate_and_formation(tmp_path,now,trim_enabled):
    m=figure_case(tmp_path,transport_attitude_trim_enabled=trim_enabled)
    try:
        old=sample(m,now)
        first=next(iter(old.values()))
        m.last_transport_overrides=old
        m.latest_payload_state.update(position=first["payload_target_position"].copy(),
            rotation=first["payload_target_rotation"].copy(),velocity=first["payload_target_velocity"].copy(),
            acceleration=first["payload_target_acceleration"].copy(),body_rate=first["payload_target_body_rate"].copy())
        for v in m.vehicles:
            v.latest_state["position"]=old[id(v)]["position"].copy()
            v.trajectory.begin_controlled_landing(now,v.latest_state["position"],0.,"figure_eight_test_exit")
        new=sample(m,now)
        for v in m.vehicles:
            for key in ["payload_target_rotation","payload_target_body_rate","payload_target_body_rate_dot",
                        "position","velocity","acceleration","position_hold_position",
                        "position_hold_velocity","position_hold_acceleration"]:
                np.testing.assert_allclose(new[id(v)][key],old[id(v)][key],atol=2e-10,err_msg=key)
    finally:m.logger.close()


def test_startup_exit_preserves_tilted_attitude_and_brakes_all_three_axes(tmp_path):
    m=figure_case(tmp_path)
    try:
        from slung_reference import _rotation_exp
        m.payload_transport_start_rotation=_rotation_exp(np.array([.15,-.1,.05]))
        now=3.
        old=sample(m,now)
        first=next(iter(old.values()))
        assert np.linalg.norm(first["payload_target_body_rate"][:2]) > .001
        m.last_transport_overrides=old
        m.latest_payload_state.update(position=first["payload_target_position"].copy(),
            rotation=first["payload_target_rotation"].copy(),velocity=first["payload_target_velocity"].copy(),
            acceleration=first["payload_target_acceleration"].copy(),body_rate=first["payload_target_body_rate"].copy())
        for v in m.vehicles:
            v.latest_state["position"]=old[id(v)]["position"].copy()
            v.trajectory.begin_controlled_landing(now,v.latest_state["position"],0.,"startup_exit")
        current=sample(m,now)
        for v in m.vehicles:
            for key in ["payload_target_rotation","payload_target_body_rate","payload_target_body_rate_dot",
                        "position","velocity","acceleration"]:
                np.testing.assert_allclose(current[id(v)][key],old[id(v)][key],atol=2e-10)
        h=1e-3
        left,middle,right=[sample(m,t) for t in [now+1.-h,now+1.,now+1.+h]]
        for v in m.vehicles:
            a,b,c=[items[id(v)] for items in [left,middle,right]]
            np.testing.assert_allclose((c["position"]-a["position"])/(2*h),b["velocity"],atol=5e-8)
            np.testing.assert_allclose((c["velocity"]-a["velocity"])/(2*h),b["acceleration"],atol=5e-8)
    finally:m.logger.close()
