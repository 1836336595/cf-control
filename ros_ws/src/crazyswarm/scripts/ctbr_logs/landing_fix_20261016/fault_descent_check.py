"""Verify a supported height-fault exit from a known quiet simulated state.

This injects the controller transition only. It does not change physical state,
sensor data, mass, motor parameters, safety limits, or reproduce a real fault.
"""

import csv
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml


SCRIPTS = Path('/home/csy/my_project/crazyswarm/ros_ws/src/crazyswarm/scripts')
sys.path.insert(0, str(SCRIPTS))
import simulate_slung_handoff as replay


directory = Path(__file__).resolve().parent / 'supported_height_fault_descent'
directory.mkdir(parents=True, exist_ok=True)
metadata = dict(
    injection='height_drop controller transition after 4.0 s of transport',
    physical_state_modified=False, sensor_data_modified=False,
    reproduces_recorded_fault=False, flight_validation_enabled=False,
    verify_controlled_fault_descent=True,
)
overrides = {'slung_payload': {'flight_validation_enabled': False}}
(directory / 'candidate.yaml').write_text(yaml.safe_dump(overrides, sort_keys=False))
(directory / 'injection_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
original_loader = replay._controller_module


def instrumented_controller():
    module = original_loader()
    original_guard = module.MultiCtbrControllerNode._check_payload_physical_safety

    def guard(manager, now):
        triggered = original_guard(manager, now)
        if triggered:
            return True
        if (manager.payload_transport_active
                and manager.payload_transport_start_time is not None
                and now - manager.payload_transport_start_time >= 4.0
                and not manager.payload_controlled_exit_reason
                and not metadata.get('injected', False)):
            metadata['injected'] = True
            metadata['injection_time_s'] = float(now)
            metadata['handoff_time_s'] = float(manager.payload_transport_start_time)
            manager.payload_safety_reason = 'height_drop'
            manager._begin_payload_emergency_landing(now, 'height_drop: injected verification')
            return True
        return False

    module.MultiCtbrControllerNode._check_payload_physical_safety = guard
    return module


replay._controller_module = instrumented_controller
started = time.perf_counter()
try:
    result = replay.run_simulation(
        directory, parameter_overrides=overrides, verify_controlled_fault_descent=True)
finally:
    replay._controller_module = original_loader
metadata['wall_time_s'] = time.perf_counter() - started
metadata['summary_success'] = result['success']
metadata['summary_failure'] = result['failure']
if metadata.get('injected'):
    with Path(result['controller_csv']).open() as stream:
        rows = list(csv.DictReader(stream))
    jumps = {}
    for vehicle in (3, 4, 5):
        own = [row for row in rows if row['vehicle_id'] == str(vehicle)]
        before = max((row for row in own if float(row['control_time_s']) < metadata['injection_time_s']),
                     key=lambda row: float(row['control_time_s']))
        after = min((row for row in own if float(row['control_time_s']) >= metadata['injection_time_s']),
                    key=lambda row: float(row['control_time_s']))
        jumps[str(vehicle)] = dict(
            before_time_s=float(before['control_time_s']), after_time_s=float(after['control_time_s']),
            before_desired_force_z_n=float(before['desired_force_z']),
            after_desired_force_z_n=float(after['desired_force_z']),
            desired_force_z_step_n=float(after['desired_force_z']) - float(before['desired_force_z']),
            command_thrust_step_n=float(after['command_thrust_newton']) - float(before['command_thrust_newton']),
            before_phase=before['flight_phase'], after_phase=after['flight_phase'],
        )
    metadata['transition_diagnostics'] = jumps
    metadata['all_transition_vertical_force_steps_below_0_02_n'] = bool(
        all(abs(item['desired_force_z_step_n']) < .02 for item in jumps.values()))
metadata['whole_flight_spacing_at_least_0_30_m'] = bool(result['minimum_vehicle_distance_m'] >= .30)
metadata['settled_all_zero_completion'] = bool(result['all_vehicles_zero']
    and result['controlled_fault_descent_completed'])
(directory / 'injection_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
print(json.dumps(dict(metadata=metadata, summary=result), indent=2), flush=True)
