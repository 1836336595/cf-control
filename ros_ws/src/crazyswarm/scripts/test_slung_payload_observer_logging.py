"""The command's load state and the latest source must remain distinguishable."""

import csv
import math

import numpy as np

from test_ctbr_controller_v2 import _controller_module, _slung_manager


def test_csv_keeps_control_snapshot_and_new_rejected_source_separate(tmp_path):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    try:
        vehicle = manager.vehicles[0]
        controlled = dict(manager.latest_payload_state,
            position=np.array([.1, .2, .6]), received_time=9.98,
            sample_time=100.0, source_stamp=100.0,
            observer_status="updated", observer_updated=True,
            observer_quality_valid=True, input_dt_s=.01, update_dt_s=.01,
            position_innovation=np.array([.001, -.002, .003]),
            attitude_innovation=np.array([.01, .02, -.03]),
            reset_count=1, rejected_count=2, consecutive_rejected_count=0,
            body_rate_clipped=False)
        source = dict(controlled, position=np.array([.1, .2, .6]),
            raw_position=np.array([1., 2., 3.]), raw_rotation=np.eye(3),
            received_time=10.09, source_stamp=100.11,
            observer_status="attitude_innovation", observer_updated=False,
            observer_quality_valid=False, input_dt_s=.0001,
            update_dt_s=.11, rejected_count=3, consecutive_rejected_count=1,
            attitude_innovation=np.array([.5, 0., 0.]))
        vehicle.set_payload_diagnostics(controlled)
        vehicle.payload_source_diagnostics = source
        vehicle.payload_control_time = 10.0
        vehicle.payload_control_reused = True
        row = vehicle._base_log_row(10.1, vehicle.latest_state)
        # The ordinary payload fields describe the state that made the force.
        assert row["payload_position_z"] == .6
        assert row["payload_observer_status"] == "updated"
        assert row["payload_control_sample_time_s"] == 100.
        assert math.isclose(row["payload_control_age_s"], .1)
        assert row["payload_control_reused"] == 1
        # New raw source diagnostics cannot overwrite the controlled state.
        assert row["payload_source_observer_status"] == "attitude_innovation"
        assert row["payload_source_observer_quality_valid"] == 0
        assert row["payload_source_stamp_s"] == 100.11
        assert row["payload_source_raw_position_z"] == 3.
        assert row["payload_source_observer_rejected_count"] == 3
        assert row["payload_source_observer_input_dt_s"] == .0001
        assert row["payload_source_observer_attitude_innovation_x"] == .5
        assert math.isclose(row["payload_source_received_age_s"], .01)
        manager.logger.write(row)
        manager.logger.flush()
        with open(manager.logger.path, newline="") as stream:
            saved = list(csv.DictReader(stream))[-1]
        assert saved["payload_control_reused"] == "1"
        assert saved["payload_observer_status"] == "updated"
        assert saved["payload_source_observer_status"] == "attitude_innovation"
        assert len(module.FlightCsvLogger.FIELDS) == len(set(module.FlightCsvLogger.FIELDS))
    finally:
        manager.logger.close()


def test_observer_logging_remains_compatible_without_observer_metadata(tmp_path):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    try:
        vehicle = manager.vehicles[0]
        vehicle.set_payload_diagnostics(manager.latest_payload_state)
        row = vehicle._base_log_row(.1, vehicle.latest_state)
        assert row["payload_observer_status"] == ""
        assert row["payload_control_reused"] == 0
        assert math.isnan(row["payload_control_age_s"])
        assert math.isnan(row["payload_source_stamp_s"])
        manager.logger.write(row)
        # The first real observer frame has no previous interval yet.
        initial = dict(manager.latest_payload_state, sample_time=None,
                       source_stamp=None, input_dt_s=None, update_dt_s=None)
        vehicle.set_payload_diagnostics(initial)
        vehicle.payload_source_diagnostics = initial
        row = vehicle._base_log_row(.1, vehicle.latest_state)
        assert math.isnan(row["payload_observer_input_dt_s"])
        assert math.isnan(row["payload_control_sample_time_s"])
        manager.logger.write(row)
    finally:
        manager.logger.close()
