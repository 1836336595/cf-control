"""Effective lengths must update the model without jumping aircraft targets."""

import numpy as np
import pytest
import sys
from types import SimpleNamespace

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from slung_prelift_trajectory import SlungPreliftTrajectory


def setup_manager(tmp_path, enabled=True):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path, {'slung_payload': {
        # This fixture proves the configured 5 s / 15 mm calibration contract;
        # keep it independent of later operator-edited flight YAML values.
        'link_lengths_m':[.709,.699,.654], 'link_calibration_enabled':enabled,
        'link_calibration_window_s':5.0, 'takeup_distance_tolerance_m':.015}})
    lengths = np.array([.715,.703,.679])
    rho = manager.slung_controller.config.attachment_points_m.T
    manager.latest_payload_state.update(position=np.array([0.,0.,.034]), velocity=np.zeros(3),
                                        body_rate=np.zeros(3), rotation=np.eye(3))
    for vehicle, point, length, radial in zip(manager.vehicles, rho, lengths, ([1,0],[-.6,.8],[-.6,-.8])):
        offset=np.array([.25*radial[0],.25*radial[1],np.sqrt(length**2-.25**2)])
        vehicle.latest_state.update(position=manager.latest_payload_state['position']+point+offset,
                                    filtered_velocity=np.zeros(3),filtered_acceleration=np.zeros(3),
                                    filter_derivatives_valid=True)
    manager.payload_takeup_start_time = 0.
    manager.payload_takeup_start_positions = np.array([v.latest_state['position'] for v in manager.vehicles])
    manager.payload_takeup_target_positions = manager.payload_takeup_start_positions.copy()
    return module, manager, lengths


def sample(manager, now):
    sys.modules[type(manager).__module__].rospy.Time.now = lambda: SimpleNamespace(to_sec=lambda:100.+now)
    manager.latest_payload_state.update(received_time=now,sample_time=100.+now)
    for v in manager.vehicles:
        v.latest_state.update(received_time=now,mocap_sample_time_s=100.+now)
        v.latest_ekf_state = dict(position=v.latest_state['position'],received_time=now,
            filtered_velocity=np.zeros(3),filtered_acceleration=np.zeros(3),filter_derivatives_valid=True)
    manager._payload_takeup_overrides(now)


def test_length_calibration_updates_model_and_target_distance_once(tmp_path):
    _, manager, actual = setup_manager(tmp_path)
    try:
        nominal = manager.payload_nominal_link_lengths.copy()
        for now in np.arange(9.,14.101,.02):
            sample(manager,float(now))
        np.testing.assert_allclose(manager.slung_controller.config.link_lengths_m,actual,atol=1e-9)
        np.testing.assert_allclose(manager.payload_takeup_target_distance_m,actual,atol=1e-9)
        assert np.isclose(manager.slung_controller.config.link_length_m,np.mean(actual))
        np.testing.assert_array_equal(manager.payload_nominal_link_lengths,nominal)
        assert manager.payload_link_calibration_applied
        for v in manager.vehicles:
            v.latest_state['position'][2] += .005
        sample(manager,15.)
        np.testing.assert_allclose(manager.slung_controller.config.link_lengths_m,actual,atol=1e-9)
        assert not manager._payload_takeup_ready(15.)  # Grounded load still cannot hand off.
    finally:
        manager.logger.close()


def test_calibration_keeps_active_prelift_position_velocity_acceleration(tmp_path):
    _, manager, actual = setup_manager(tmp_path)
    try:
        measured = np.array([v.latest_state['position'] for v in manager.vehicles])
        trajectory = SlungPreliftTrajectory(**manager.payload_prelift_parameters)
        trajectory.start(measured,measured,8.)
        trajectory.stage = 'rising'
        trajectory.speed = .03
        manager.payload_prelift_reference = trajectory
        for now in np.arange(9.,14.001,.02):sample(manager,float(now))
        before = trajectory.snapshot()
        # Repeated frame must neither apply twice nor replace trajectory state.
        sample(manager,14.)
        assert manager.payload_prelift_reference is trajectory
        assert manager.payload_link_calibration_applied
        assert np.max(abs(trajectory.positions-before['positions'])) < .002
        assert np.max(abs(trajectory.velocities-before['velocities'])) < .002
        assert np.max(abs(trajectory.accelerations-before['accelerations'])) < .06
    finally:
        manager.logger.close()


def test_post_takeup_calibration_holds_reference_before_smooth_realignment(tmp_path):
    _, manager, _ = setup_manager(tmp_path)
    try:
        positions=[]
        for now in np.arange(9.,14.701,.02):
            sample(manager,float(now))
            positions.append(manager.payload_last_takeup_reference['positions'].copy())
        assert manager.payload_link_calibration_applied
        assert manager.payload_prelift_reference is not None
        changes=np.diff(np.array(positions),axis=0)
        assert np.max(np.abs(changes)) < .002
        assert manager.payload_prelift_reference.stage == 'aligning'
        effective=manager.slung_controller.config.link_lengths_m.copy()
        manager._reset_payload_handoff()
        assert manager.payload_link_calibration_applied
        np.testing.assert_array_equal(manager.slung_controller.config.link_lengths_m,effective)
    finally:manager.logger.close()


def test_disabled_or_stale_calibration_never_changes_model(tmp_path):
    for enabled in (False,True):
        folder=tmp_path/str(enabled);folder.mkdir()
        _, manager, _ = setup_manager(folder,enabled)
        try:
            nominal=manager.slung_controller.config.link_lengths_m.copy()
            for now in np.arange(9.,15.,.02):
                manager.latest_payload_state['received_time']=0.
                manager._payload_takeup_overrides(float(now))
            np.testing.assert_array_equal(manager.slung_controller.config.link_lengths_m,nominal)
            assert not manager.payload_link_calibration_applied
        finally:manager.logger.close()


def test_invalid_snapshot_discards_unapplied_candidate_and_restarts_window(tmp_path):
    _,manager,_=setup_manager(tmp_path)
    try:
        # An accepted robust window may be awaiting a non-outlier current frame.
        manager.payload_link_calibration_pending=np.array([.715,.703,.679])
        manager.payload_link_calibrator.applied_time=100.
        manager.payload_link_calibrator.status='applied'
        manager.latest_payload_state['received_time']=0.
        manager._payload_takeup_overrides(10.)
        assert manager.payload_link_calibration_pending is None
        assert manager.payload_link_calibrator.applied_time is None
        assert not manager.payload_link_calibration_applied
    finally:manager.logger.close()


def test_failed_geometry_rebuild_leaves_nominal_model_intact(tmp_path):
    _,manager,_=setup_manager(tmp_path)
    try:
        nominal=manager.slung_controller.config.link_lengths_m.copy()
        def fail(_rotation): raise ValueError('test geometry failure')
        manager.slung_controller.equilibrium_link_units=fail
        for now in np.arange(9.,14.101,.02):sample(manager,float(now))
        assert not manager.payload_link_calibration_applied
        np.testing.assert_array_equal(manager.slung_controller.config.link_lengths_m,nominal)
        np.testing.assert_array_equal(manager.payload_takeup_target_distance_m,nominal)
    finally:manager.logger.close()


@pytest.mark.parametrize('name,value',[('link_calibration_enabled','false'),
    ('link_calibration_window_s',0),('link_calibration_max_adjustment_m',-.01)])
def test_invalid_link_calibration_parameters_fail_at_startup(tmp_path,name,value):
    module=_controller_module()
    with pytest.raises(RuntimeError,match='link_calibration'):
        _slung_manager(module,tmp_path,{'slung_payload':{name:value}})
