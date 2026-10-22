"""Fleet CSV should distinguish each term in cooperative vehicle support."""

import csv
import math

import numpy as np

from test_ctbr_controller_v2 import _controller_module, _slung_manager


def test_transport_force_terms_and_applied_control_dt_reach_flight_csv(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        now = 1.0
        manager.payload_transport_active = True
        manager.payload_transport_start_time = 0.0
        manager.payload_transport_start_position = manager.latest_payload_state["position"].copy()
        manager.payload_transport_vehicle_start_positions = anchors.copy()
        manager.payload_handoff_reference_positions = anchors.copy()
        manager.payload_handoff_forces = np.tile([.08, 0., .85], (3, 1))
        manager.payload_control_dt = .004
        manager.latest_payload_state["received_time"] = now
        for current in manager.vehicles:
            current.latest_state["received_time"] = now
            current.latest_state["filtered_velocity"] = np.zeros(3)
            current.latest_state["filtered_acceleration"] = np.zeros(3)
        vehicle = manager.vehicles[0]
        targets = manager._transport_overrides(now)
        target = vehicle._target_for(vehicle.latest_state, now)
        target.update(targets[id(vehicle)])
        state = vehicle.latest_state
        command = vehicle.controller.compute(state, target, manager.payload_control_dt)
        vehicle._write_log(now, state, target, command, control_state=state)
        manager.logger.flush()

        with open(manager.logger.path, newline="") as stream:
            row = next(csv.DictReader(stream))

        output = targets[id(vehicle)]
        for key, columns in (
            ("raw_link_force", ("raw_link_force_x", "raw_link_force_y", "raw_link_force_z")),
            ("link_force", ("limited_link_force_x", "limited_link_force_y", "limited_link_force_z")),
            ("kinematic_perpendicular_force", (
                "kinematic_perpendicular_force_x", "kinematic_perpendicular_force_y",
                "kinematic_perpendicular_force_z")),
        ):
            np.testing.assert_allclose([float(row[name]) for name in columns], output[key])
        assert int(float(row["link_force_rate_limited"])) == int(output["link_force_rate_limited"])
        assert int(float(row["link_force_magnitude_limited"])) == int(output["link_force_magnitude_limited"])
        assert math.isclose(float(row["payload_control_dt_s"]), manager.payload_control_dt)
        np.testing.assert_allclose(
            [float(row[f"desired_force_{axis}"]) for axis in "xyz"],
            command["desired_force"],
        )
    finally:
        manager.logger.close()


def test_absent_link_force_terms_are_nan_not_synthetic_zeros(tmp_path):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    try:
        vehicle = manager.vehicles[0]
        state = dict(vehicle.latest_state, filtered_velocity=np.zeros(3),
                     filtered_acceleration=np.zeros(3))
        target = vehicle._target_for(state, 1.0)
        command = vehicle.controller.compute(state, target, .01)
        vehicle._write_log(1.0, state, target, command, control_state=state)
        manager.logger.flush()
        with open(manager.logger.path, newline="") as stream:
            row = next(csv.DictReader(stream))
        for name in (
            "raw_link_force_x", "raw_link_force_y", "raw_link_force_z",
            "limited_link_force_x", "limited_link_force_y", "limited_link_force_z",
            "kinematic_perpendicular_force_x", "kinematic_perpendicular_force_y",
            "kinematic_perpendicular_force_z", "payload_control_dt_s",
        ):
            assert math.isnan(float(row[name]))
        assert math.isnan(float(row["link_force_rate_limited"]))
        assert math.isnan(float(row["link_force_magnitude_limited"]))
    finally:
        manager.logger.close()
