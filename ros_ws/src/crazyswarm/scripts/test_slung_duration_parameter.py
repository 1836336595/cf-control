"""The payload deadline must not be cut short by the ordinary hover timer."""
import pytest
from test_ctbr_controller_v2 import _controller_module, _slung_manager


def test_only_payload_duration_needs_extending_for_long_hover(tmp_path):
    module=_controller_module()
    manager,_=_slung_manager(module,tmp_path,{
        'slung_payload':{'trajectory':{'mode':'hover'},'flight_validation_enabled':True,
                         'flight_validation_transport_s':150.},
        'ctbr_trajectory':{'hover_duration_s':20.}})
    try:
        assert manager.payload_flight_validation_transport_s==150.
        for vehicle in manager.vehicles:
            assert vehicle.trajectory_config.hover_duration_s>=150.
            vehicle.trajectory.phase='hover'
            vehicle.trajectory.phase_start_time=0.
            assert vehicle.trajectory.evaluate(vehicle.latest_state,120.)['flight_phase']=='hover'
    finally:manager.logger.close()


def test_disabled_payload_deadline_preserves_ordinary_hover_duration(tmp_path):
    module=_controller_module()
    manager,_=_slung_manager(module,tmp_path,{
        'slung_payload':{'flight_validation_enabled':False,'flight_validation_transport_s':150.},
        'ctbr_trajectory':{'hover_duration_s':20.}})
    try:
        assert all(v.trajectory_config.hover_duration_s==20. for v in manager.vehicles)
    finally:manager.logger.close()


def test_cone_lands_once_after_endpoint_even_with_a_long_hover_deadline(tmp_path):
    module=_controller_module()
    manager,_=_slung_manager(module,tmp_path,{
        'slung_payload':{'trajectory':{'mode':'attitude_cone','revolutions':2.,'cone_period_s':30.},
                         'flight_validation_enabled':True,'flight_validation_transport_s':100.},
        'ctbr_trajectory':{'hover_duration_s':20.}})
    try:
        assert manager.payload_flight_validation_transport_s == pytest.approx(90.)
        assert all(v.trajectory_config.wait_for_payload_trajectory for v in manager.vehicles)
        manager.payload_transport_active = True
        manager.payload_transport_start_time = 0.
        assert not manager._check_payload_flight_validation(90.)
        manager.payload_endpoint_sent = True
        assert manager._check_payload_flight_validation(90.02)
        assert manager.payload_validation_landing_stage == "payload_lowering"
        assert not manager._check_payload_flight_validation(90.04)
    finally:manager.logger.close()
