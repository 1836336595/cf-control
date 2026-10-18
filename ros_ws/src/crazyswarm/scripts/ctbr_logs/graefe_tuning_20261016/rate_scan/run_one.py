"""Run one host attitude/rate candidate in a fresh Python process."""

import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import time

import yaml


SCRIPTS = Path('/home/csy/my_project/crazyswarm/ros_ws/src/crazyswarm/scripts')
sys.path.insert(0, str(SCRIPTS))
from simulate_slung_handoff import run_simulation


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('name')
    parser.add_argument('attitude', type=float)
    parser.add_argument('rate', type=float)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    directory = root / args.name
    directory.mkdir(parents=True, exist_ok=True)
    overrides = deepcopy(yaml.safe_load((root / 'baseline_parameters.yaml').read_text()))
    changed = {}
    for vehicle in (3, 4, 5):
        key = 'ctbr_controller_cf%d' % vehicle
        changed[key] = dict(
            transport_attitude_gain=[args.attitude, args.attitude, args.attitude / 2],
            transport_rate_gain=[args.rate, args.rate, args.rate / 2],
        )
        overrides[key].update(changed[key])
    (directory / 'candidate.yaml').write_text(yaml.safe_dump(changed, sort_keys=False))
    started = time.perf_counter()
    result = run_simulation(
        directory, parameter_overrides=overrides,
        post_handoff_s=60.0, target_payload_height_m=0.60,
    )
    elapsed = time.perf_counter() - started
    metadata = dict(name=args.name, host_attitude_gain=changed['ctbr_controller_cf3']['transport_attitude_gain'],
                    host_rate_gain=changed['ctbr_controller_cf3']['transport_rate_gain'],
                    wall_time_s=elapsed, summary=result)
    (directory / 'run_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps({key: result.get(key) for key in (
        'success', 'failure', 'handoff_time_s', 'final_time_s', 'observed_post_handoff_s',
        'maximum_body_rate_rps', 'maximum_tilt_deg', 'minimum_clearance_m',
        'minimum_vehicle_distance_m', 'rate_saturation_fraction',
        'final_payload_position_error_m', 'payload_safety_reason',
    )} | dict(name=args.name, wall_time_s=elapsed)), flush=True)
