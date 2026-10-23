"""Fixed-centroid cone attitude must reach the real cooperative controller."""

import numpy as np
import pytest
import csv

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_attitude_reference import sample
from slung_load_controller import payload_ground_clearance


def cone_case(directory, **changes):
    payload=dict(trajectory=dict(mode="attitude_cone", center_xy=[0.,0.],
        cone_half_angle_deg=10.,cone_period_s=30.,cone_transition_s=5.,
        revolutions=1.,settle_s=3.),transport_force_mode="reference_cascade",
        transport_attitude_trim_enabled=False,flight_validation_enabled=True,
        flight_validation_transport_s=100.)
    payload.update(changes)
    manager,anchors=_slung_manager(_controller_module(),directory,{
        "ctbr_trajectory":{"hover_duration_s":100.},"slung_payload":payload})
    manager.payload_transport_active=True
    manager.payload_transport_start_time=0.
    manager.payload_transport_start_position=manager.latest_payload_state["position"].copy()
    manager.payload_transport_vehicle_start_positions=anchors.copy()
    manager.payload_handoff_reference_positions=anchors.copy()
    sample(manager,0.)
    return manager


def test_cone_centroid_is_fixed_and_body_z_makes_ten_degree_cone(tmp_path):
    m=cone_case(tmp_path)
    try:
        directions=[]
        for now in [25.,30.,35.,40.,45.,50.,55.]:
            item=next(iter(sample(m,now).values()))
            np.testing.assert_allclose(item["payload_target_position"],[0.,0.,.6],atol=1e-12)
            np.testing.assert_allclose(item["payload_target_velocity"],0.,atol=1e-12)
            np.testing.assert_allclose(item["payload_target_acceleration"],0.,atol=1e-12)
            z=item["payload_target_rotation"][:,2]
            assert np.rad2deg(np.arccos(np.clip(z[2],-1.,1.)))==pytest.approx(10.,abs=1e-10)
            assert item["payload_trajectory_mode"]=="attitude_cone"
            directions.append(z)
        assert np.ptp(np.array(directions)[:,0])>.3
        assert np.ptp(np.array(directions)[:,1])>.2
    finally:m.logger.close()


@pytest.mark.parametrize("now",[21.5,25.,31.,42.,55.,57.5])
def test_cone_pose_rate_and_both_aircraft_reference_derivatives_match(tmp_path,now):
    m=cone_case(tmp_path)
    try:
        # Acceleration is continuous at phase joins; jerk changes there, so
        # a centered velocity derivative converges linearly at those joins.
        h=5e-6 if now in (25.,55.) else 1e-3
        left,middle,right=[sample(m,t) for t in [now-h,now,now+h]]
        rho=m.slung_controller.config.attachment_points_m
        for i,vehicle in enumerate(m.vehicles):
            a,b,c=[items[id(vehicle)] for items in [left,middle,right]]
            for p,v,acc in [("position","velocity","acceleration"),
                           ("position_hold_position","position_hold_velocity","position_hold_acceleration")]:
                np.testing.assert_allclose((c[p]-a[p])/(2*h),b[v],atol=2e-7)
                np.testing.assert_allclose((c[v]-a[v])/(2*h),b[acc],atol=2e-7)
            r=b["payload_target_rotation"];w=b["payload_target_body_rate"]
            cross=np.array([[0.,-w[2],w[1]],[w[2],0.,-w[0]],[-w[1],w[0],0.]])
            np.testing.assert_allclose((c["payload_target_rotation"]-a["payload_target_rotation"])/(2*h),r@cross,atol=2e-7)
            np.testing.assert_allclose((c["payload_target_body_rate"]-a["payload_target_body_rate"])/(2*h),b["payload_target_body_rate_dot"],atol=2e-7)
            cable=b["payload_target_position"]+r@rho[:,i]-b["position_hold_position"]
            assert np.linalg.norm(cable)==pytest.approx(m.payload_transport_initial_link_distances[i],abs=1e-10)
    finally:m.logger.close()


def test_cone_nominal_geometry_can_realize_static_gravity_wrench(tmp_path):
    m=cone_case(tmp_path)
    try:
        for now in [25.,40.]:
            targets=sample(m,now);first=next(iter(targets.values()))
            payload=dict(valid=True,position=first["payload_target_position"],rotation=first["payload_target_rotation"],
                         velocity=np.zeros(3),body_rate=np.zeros(3),acceleration=np.zeros(3))
            vehicles=[dict(position=targets[id(v)]["position_hold_position"],velocity=np.zeros(3),mass=v.controller.config.mass) for v in m.vehicles]
            target=dict(position=payload["position"],rotation=payload["rotation"],velocity=np.zeros(3),acceleration=np.zeros(3),
                        payload_attitude_gain=m.payload_attitude_gain,payload_rate_gain=m.payload_rate_gain)
            m.slung_controller.reset()
            result=m.slung_controller.compute(payload,vehicles,target,1/70)
            force=np.array([-o["desired_tension"]*o["link_direction"] for o in result["vehicles"]])
            np.testing.assert_allclose(force.sum(axis=0),[0.,0.,.023*9.80665],atol=1e-11)
            body_forces=force@payload["rotation"]
            moment=np.cross(m.slung_controller.config.attachment_points_m.T,body_forces).sum(axis=0)
            np.testing.assert_allclose(moment,0.,atol=1e-11)
    finally:m.logger.close()


def test_cone_finishes_level_at_rest_and_keeps_slow_trim_frozen_while_moving(tmp_path):
    m=cone_case(tmp_path,transport_attitude_trim_enabled=True)
    try:
        a=next(iter(sample(m,30.).values()))
        assert a["transport_attitude_trim_freeze_reason"]=="reference_moving"
        final=next(iter(sample(m,60.).values()))
        assert final["payload_trajectory_phase"]=="complete"
        np.testing.assert_allclose(final["payload_target_rotation"],np.eye(3),atol=1e-12)
        np.testing.assert_allclose(final["payload_target_body_rate"],0.,atol=1e-12)
        np.testing.assert_allclose(final["payload_target_body_rate_dot"],0.,atol=1e-12)
    finally:m.logger.close()


@pytest.mark.parametrize("now",[22.5,35.,57.5])
def test_interrupted_cone_landing_is_continuous_and_levels_before_reference_touchdown(tmp_path,now):
    m=cone_case(tmp_path)
    try:
        old=sample(m,now);first=next(iter(old.values()))
        m.last_transport_overrides=old
        m.latest_payload_state.update(position=first["payload_target_position"].copy(),rotation=first["payload_target_rotation"].copy(),
            velocity=first["payload_target_velocity"].copy(),acceleration=first["payload_target_acceleration"].copy(),body_rate=first["payload_target_body_rate"].copy())
        for v in m.vehicles:
            v.latest_state["position"]=old[id(v)]["position"].copy()
            v.trajectory.begin_controlled_landing(now,v.latest_state["position"],0.,"cone_exit_test")
        new=sample(m,now)
        for v in m.vehicles:
            for key in ["position","velocity","acceleration","payload_target_rotation","payload_target_body_rate","payload_target_body_rate_dot"]:
                np.testing.assert_allclose(new[id(v)][key],old[id(v)][key],atol=1e-10)
        assert m.payload_landing_profile.duration>=10.
        later=next(iter(sample(m,now+10.).values()))
        np.testing.assert_allclose(later["payload_target_rotation"],np.eye(3),atol=1e-12)
    finally:m.logger.close()


def test_real_flight_csv_records_actual_desired_cone_axis_and_angle(tmp_path):
    m=cone_case(tmp_path)
    try:
        now=35.
        overrides=sample(m,now)
        vehicle=m.vehicles[0]
        reference=overrides[id(vehicle)]
        vehicle.payload_diagnostics=dict(m.latest_payload_state,rotation=reference["payload_target_rotation"].copy())
        state=dict(vehicle.latest_state,filtered_velocity=np.zeros(3),filtered_acceleration=np.zeros(3))
        target=vehicle._target_for(state,now);target.update(reference)
        command=vehicle.controller.compute(state,target,1/70)
        vehicle._write_log(now,state,target,command,control_state=state)
        m.logger.flush()
        with open(m.logger.path) as f:row=next(csv.DictReader(f))
        assert float(row["payload_cone_half_angle_deg"])==pytest.approx(10.)
        assert float(row["payload_target_cone_half_angle_deg"])==pytest.approx(10.)
        assert float(row["payload_cone_axis_error_deg"])==pytest.approx(0.,abs=1e-6)
        np.testing.assert_allclose([float(row["payload_target_body_z_"+a]) for a in "xyz"],reference["payload_target_rotation"][:,2])
    finally:m.logger.close()


def _interrupt_cone(manager, now, height=.6, velocity=None, acceleration=None):
    manager.payload_target_height_m=height
    manager.payload_circle_reference=None
    old=sample(manager,now)
    velocity=np.zeros(3) if velocity is None else np.asarray(velocity,dtype=float)
    acceleration=np.zeros(3) if acceleration is None else np.asarray(acceleration,dtype=float)
    for item in old.values():
        item["payload_target_velocity"]=velocity.copy()
        item["payload_target_acceleration"]=acceleration.copy()
        for key in ("velocity","position_hold_velocity"):
            item[key]=item[key]+velocity
        for key in ("acceleration","position_hold_acceleration"):
            item[key]=item[key]+acceleration
    first=next(iter(old.values()))
    manager.last_transport_overrides=old
    manager.latest_payload_state.update(position=first["payload_target_position"].copy(),
        rotation=first["payload_target_rotation"].copy(),velocity=velocity.copy(),
        acceleration=acceleration.copy(),body_rate=first["payload_target_body_rate"].copy())
    for vehicle in manager.vehicles:
        vehicle.latest_state["position"]=old[id(vehicle)]["position"].copy()
        # The fixture starts at taut anchors rather than aircraft ground.
        # Use the physical ground height so low payload cases stay airborne.
        vehicle.trajectory.start_position[2]=.007
        vehicle.trajectory.begin_controlled_landing(now,vehicle.latest_state["position"],0.,"cone_exit_test")
    return old,sample(manager,now)


@pytest.mark.parametrize("height",[.04,.6])
def test_interrupted_cone_holds_height_until_level_and_never_intersects_ground(tmp_path,height):
    manager=cone_case(tmp_path)
    try:
        now=35.
        _interrupt_cone(manager,now,height)
        profile=manager.payload_landing_profile
        assert profile is not None
        for time in np.linspace(now,now+profile.duration,201):
            targets=sample(manager,float(time))
            assert targets
            item=next(iter(targets.values()))
            assert payload_ground_clearance(item["payload_target_position"],
                item["payload_target_rotation"],manager.slung_controller.config.payload_size_m)>=-1e-10
            if time<now+10.-1e-9:
                assert item["payload_target_position"][2]==pytest.approx(height,abs=1e-12)
        endpoint=next(iter(targets.values()))
        np.testing.assert_allclose(endpoint["payload_target_rotation"],np.eye(3),atol=1e-12)
        assert endpoint["payload_target_position"][2]==pytest.approx(.025,abs=1e-12)
    finally:manager.logger.close()


def test_interrupted_cone_level_geometry_still_realizes_static_gravity_wrench(tmp_path):
    manager=cone_case(tmp_path)
    try:
        now=35.
        _interrupt_cone(manager,now)
        targets=sample(manager,now+10.)
        first=next(iter(targets.values()))
        rotation=first["payload_target_rotation"]
        payload=dict(valid=True,position=first["payload_target_position"],rotation=rotation,
            velocity=np.zeros(3),acceleration=np.zeros(3),body_rate=np.zeros(3))
        vehicles=[dict(position=targets[id(vehicle)]["position_hold_position"],
            velocity=np.zeros(3),mass=vehicle.controller.config.mass) for vehicle in manager.vehicles]
        target=dict(position=payload["position"],rotation=rotation,velocity=np.zeros(3),acceleration=np.zeros(3),
            payload_attitude_gain=manager.payload_attitude_gain,payload_rate_gain=manager.payload_rate_gain)
        manager.slung_controller.reset()
        result=manager.slung_controller.compute(payload,vehicles,target,1/70)
        forces=np.asarray([-output["desired_tension"]*output["link_direction"] for output in result["vehicles"]])
        np.testing.assert_allclose(forces.sum(axis=0),[0.,0.,.023*9.80665],atol=1e-11)
        moment=np.cross(manager.slung_controller.config.attachment_points_m.T,forces@rotation).sum(axis=0)
        np.testing.assert_allclose(moment,0.,atol=1e-11)
    finally:manager.logger.close()


@pytest.mark.parametrize("height",[.05,.6])
def test_interrupted_cone_brakes_measured_translation_without_resetting_pva(tmp_path,height):
    manager=cone_case(tmp_path)
    try:
        now=35.
        velocity=np.array([.004,-.003,-.002])
        acceleration=np.array([.003,.002,-.001])
        old,new=_interrupt_cone(manager,now,height,velocity,acceleration)
        assert new
        first=next(iter(new.values()))
        np.testing.assert_array_equal(first["payload_target_velocity"],velocity)
        np.testing.assert_array_equal(first["payload_target_acceleration"],acceleration)
        for vehicle in manager.vehicles:
            for key in ("position","velocity","acceleration","position_hold_position",
                        "position_hold_velocity","position_hold_acceleration"):
                np.testing.assert_allclose(new[id(vehicle)][key],old[id(vehicle)][key],atol=1e-10)
        for time in np.linspace(now,now+manager.payload_landing_profile.duration,201):
            item=next(iter(sample(manager,float(time)).values()))
            assert payload_ground_clearance(item["payload_target_position"],item["payload_target_rotation"],
                manager.slung_controller.config.payload_size_m)>=-1e-10
        for time in (now+.03,now+.2,now+5.,now+7.,now+10.2):
            h=5e-6 if time==now+5. else 1e-4
            left,middle,right=[sample(manager,t) for t in (time-h,time,time+h)]
            for vehicle in manager.vehicles:
                a,b,c=[targets[id(vehicle)] for targets in (left,middle,right)]
                for p,v,acc in (("payload_target_position","payload_target_velocity","payload_target_acceleration"),
                    ("position","velocity","acceleration"),("position_hold_position","position_hold_velocity","position_hold_acceleration")):
                    np.testing.assert_allclose((c[p]-a[p])/(2*h),b[v],atol=2e-7)
                    np.testing.assert_allclose((c[v]-a[v])/(2*h),b[acc],atol=2e-7)
    finally:manager.logger.close()


def test_already_level_completed_cone_does_not_add_a_landing_wait(tmp_path):
    manager=cone_case(tmp_path)
    try:
        _interrupt_cone(manager,100.)
        targets=sample(manager,101.)
        assert next(iter(targets.values()))["payload_target_position"][2]<.6
    finally:manager.logger.close()


def test_infeasible_low_cone_descent_uses_supported_planning_retry(tmp_path,monkeypatch):
    manager=cone_case(tmp_path)
    try:
        import ctbr_payload_mission
        monkeypatch.setattr(ctbr_payload_mission.rospy,"logerr",lambda *_args:None,raising=False)
        old,held=_interrupt_cone(manager,35.,.04,[0.,0.,-.12],np.zeros(3))
        assert manager.payload_landing_profile is None
        assert manager.payload_landing_failure_count==1
        assert manager.payload_landing_retry_count==1
        assert manager.payload_landing_failure_reason
        assert manager.payload_transport_active and not manager.global_abort_reason
        assert held
        for vehicle in manager.vehicles:
            target=held[id(vehicle)]
            assert manager._finite_transport_override(target)
            assert target["transport_full_pid_force_hold"]
            np.testing.assert_array_equal(target["desired_force_override"],old[id(vehicle)]["desired_force_override"])
    finally:manager.logger.close()
