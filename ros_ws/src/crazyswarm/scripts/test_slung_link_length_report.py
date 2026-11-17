"""Stable geometric measurements are diagnostic and never calibrate control."""
import numpy as np
import pytest

from slung_link_length_report import SlungLinkLengthReporter

RHO = np.array([[.04, 0., .025], [-.04, .03, .025], [-.04, -.03, .025]])
LENGTHS = np.array([.8592, .8608, .8566])


def sample(reporter, t, eligible=True, tilt=0., offset=0.):
    a = np.deg2rad(tilt)
    rotation = np.array([[np.cos(a), 0., np.sin(a)], [0., 1., 0.], [-np.sin(a), 0., np.cos(a)]])
    center = np.array([offset, 0., .6])
    vehicles = center + RHO @ rotation.T + np.column_stack([np.zeros(3), np.zeros(3), LENGTHS])
    return reporter.update(t, center, rotation, vehicles, eligible)


def test_rotated_mount_points_give_correct_lengths_and_do_not_use_nominal():
    reporter = SlungLinkLengthReporter(RHO, window_s=1.)
    result = None
    for t in np.arange(0., 1.001, .02):
        result = sample(reporter, t, tilt=2.)
    np.testing.assert_allclose(result['median_m'], LENGTHS, atol=1e-12)
    assert result['samples'] == 51
    assert result['window_s'] >= 1.
    assert result['median_tilt_deg'] == pytest.approx(2.)


def test_duplicate_frames_do_not_supply_evidence_and_long_gap_restarts():
    reporter = SlungLinkLengthReporter(RHO, window_s=1.)
    for _ in range(100):
        assert sample(reporter, .1) is None
    assert sample(reporter, 2.) is None
    assert reporter.sample_count == 1


@pytest.mark.parametrize('case', ['moving','tilted','not_eligible'])
def test_motion_and_unlevel_or_invalid_windows_cannot_report(case):
    reporter = SlungLinkLengthReporter(RHO, window_s=1.)
    for t in np.arange(0., 2., .02):
        assert sample(reporter, t, eligible=case!='not_eligible',
                      tilt=8. if case=='tilted' else 0.,
                      offset=.1*t if case=='moving' else 0.) is None


def test_rearmed_after_a_new_stable_episode_and_throttled_while_steady():
    reporter = SlungLinkLengthReporter(RHO, window_s=1.)
    events=[]
    for t in np.arange(0., 4., .02):
        if sample(reporter,t) is not None: events.append(t)
    assert len(events)==1
    sample(reporter,4.,eligible=False)
    events=[]
    for t in np.arange(4.02,5.1,.02):
        if sample(reporter,t) is not None: events.append(t)
    assert len(events)==1


def test_invalid_parameters_fail_before_runtime():
    with pytest.raises(ValueError):SlungLinkLengthReporter(RHO, window_s=-1.)


def test_window_filters_a_single_distance_outlier_with_median_not_maximum():
    reporter=SlungLinkLengthReporter(RHO,window_s=1.)
    result=None
    for index,t in enumerate(np.arange(0.,1.001,.02)):
        center=np.array([0.,0.,.6])
        vehicles=center+RHO+np.column_stack([np.zeros(3),np.zeros(3),LENGTHS])
        if index==25:vehicles[0,2]+=.004
        result=reporter.update(t,center,np.eye(3),vehicles)
    np.testing.assert_allclose(result['median_m'],LENGTHS,atol=1e-12)


def test_diagnostic_switch_does_not_change_cooperative_commands(tmp_path):
    from test_slung_transport_integral_integration import manager_case,fresh
    from copy import deepcopy
    module,manager,anchors=manager_case(tmp_path)
    try:
        fresh(module,manager,anchors,18.)
        # The diagnostic only appends scalar fields to already computed references.
        outputs=manager._transport_overrides(18.)
        original=deepcopy(outputs)
        nominal=manager.payload_nominal_link_lengths.copy()
        effective=manager.slung_controller.config.link_lengths_m.copy()
        states=[dict(position=v.latest_state['position'],sample_time=118.) for v in manager.vehicles]
        manager._report_payload_link_lengths(18.,manager.latest_payload_state,states,outputs,False)
        for key in outputs:
            for field in ['desired_force_override','desired_force_dot_override','position_hold_position',
                          'position_hold_velocity','payload_target_rotation']:
                np.testing.assert_array_equal(outputs[key][field],original[key][field])
        np.testing.assert_array_equal(manager.payload_nominal_link_lengths,nominal)
        np.testing.assert_array_equal(manager.slung_controller.config.link_lengths_m,effective)
    finally:
        manager.logger.close()


def test_real_manager_reports_and_logs_live_lengths_without_changing_effective_lengths(tmp_path):
    from test_slung_transport_integral_integration import manager_case, fresh
    module, manager, anchors = manager_case(tmp_path)
    try:
        before = manager.slung_controller.config.link_lengths_m.copy()
        logs=[]
        module.rospy.loginfo=lambda *args:logs.append(args)
        for t in np.arange(18.,21.2,.02):
            fresh(module,manager,anchors,t)
            output=manager._transport_overrides(t)
        assert any('绳长统计' in str(row[0]) for row in logs)
        assert output[id(manager.vehicles[0])]['link_length_report_sequence'] >= 1
        from slung_load_controller import cable_distances
        d=cable_distances(manager.latest_payload_state['position'],manager.latest_payload_state['rotation'],
                         manager.slung_controller.config.attachment_points_m,
                         [v.latest_state['position'] for v in manager.vehicles])
        np.testing.assert_allclose(manager.payload_link_measured_distances,d)
        np.testing.assert_array_equal(manager.slung_controller.config.link_lengths_m,before)
        vehicle=manager.vehicles[0]
        state=dict(vehicle.latest_state,filtered_velocity=np.zeros(3),filtered_acceleration=np.zeros(3))
        target=vehicle._target_for(state,t)
        target.update(output[id(vehicle)])
        vehicle.set_payload_diagnostics(manager.latest_payload_state)
        command=vehicle.controller.compute(state,target,.02)
        vehicle._write_log(t,state,target,command)
        manager.logger.flush()
        import csv
        with open(manager.logger.path) as stream:
            row=list(csv.DictReader(stream))[-1]
        assert int(row['link_length_report_sequence']) >= 1
        assert float(row['link_length_report_median_m']) == pytest.approx(d[0])
    finally:
        manager.logger.close()


@pytest.mark.parametrize('bad', ['moving_reference','landing','async_sources','unlevel_target','rejected_history'])
def test_manager_excludes_motion_landing_and_bad_feedback_from_report(tmp_path,bad):
    from test_slung_transport_integral_integration import manager_case,fresh
    module,manager,anchors=manager_case(tmp_path)
    try:
        for t in np.arange(18.,21.2,.02):
            fresh(module,manager,anchors,t)
            outputs=manager._transport_overrides(t)
            # Feed the actual diagnostic entry with the relevant eligibility change.
            states=[dict(position=v.latest_state['position'],sample_time=100.+t) for v in manager.vehicles]
            if bad=='moving_reference':
                for out in outputs.values():out['payload_target_velocity']=np.array([.1,0,0])
            if bad=='async_sources':states[0]['sample_time']-=.2
            if bad=='unlevel_target':
                for out in outputs.values():out['payload_target_rotation']=np.diag([1.,0.,0.])
            if bad=='rejected_history':
                manager.latest_payload_state['rejected_count']=int(t*100)
            manager._report_payload_link_lengths(t,manager.latest_payload_state,states,outputs,bad=='landing')
        assert manager.payload_link_length_reporter.sequence==0
    finally:
        manager.logger.close()
