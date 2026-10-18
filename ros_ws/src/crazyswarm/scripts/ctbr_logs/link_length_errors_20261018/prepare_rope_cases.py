#!/usr/bin/env python3
"""Freeze paired, constant-per-run physical rope errors; no simulation launch."""

from copy import deepcopy
import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parent
SCRIPTS = ROOT.parents[1]
sys.path.insert(0, str(SCRIPTS))
from test_ctbr_controller_v2 import _controller_module, _controller_parameter_roots, _slung_manager


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    baseline_path = ROOT / 'frozen_baseline.yaml'
    if (ROOT / 'cases.json').exists():
        raise RuntimeError('cases.json already exists; preserve the frozen experiment rather than regenerating it')
    roots = _controller_parameter_roots()
    roots.update(yaml.safe_load((SCRIPTS.parent / 'config/slung_payload.yaml').read_text()))
    payload = roots['slung_payload']
    nominal = np.asarray(payload['link_lengths_m'], dtype=float)
    rho = np.asarray(payload['attachment_points_m'], dtype=float)
    payload_position = np.array([0., 0., float(payload['size_m'][2]) / 2.])
    module = _controller_module()
    with tempfile.TemporaryDirectory(prefix='rope-case-geometry-') as directory:
        manager, _ = _slung_manager(module, directory, roots)
        try:
            nominal_anchors = manager._payload_takeup_positions().copy()
        finally:
            manager.logger.close()
    source_csv = SCRIPTS / 'ctbr_logs/multi_ctbr_20261018_162455.csv'
    with source_csv.open() as stream:
        actual = [row for row in csv.DictReader(stream) if row['vehicle_id'] == '3']
    handoff = next(float(row['control_time_s']) for row in actual if row['flight_phase'] == 'tension_ramp')
    fault = next(float(row['control_time_s']) for row in actual if row['flight_phase'] == 'emergency_landing')
    timing = [float(row['controller_tick_interval_s']) for row in actual
              if handoff <= float(row['control_time_s']) < fault]
    if not timing or not np.all(np.isfinite(timing)) or min(timing) <= 0:
        raise ValueError('recorded control intervals must be finite and positive')
    roots['slung_payload']['simulation']['control_intervals_s'] = timing
    baseline_path.write_text(yaml.safe_dump(roots, sort_keys=False))
    references = {
        'source_csv': dict(path=str(source_csv), sha256=digest(source_csv)),
        'frozen_baseline': dict(path=str(baseline_path), sha256=digest(baseline_path)),
    }
    for name in ['ctbr_controller.py', 'slung_load_controller.py', 'simulate_slung_handoff.py',
                 'slung_link_calibration.py', 'slung_prelift_trajectory.py']:
        references[name] = dict(path=str(SCRIPTS / name), sha256=digest(SCRIPTS / name))
    definitions = []
    def add(name, family, error, matched=False, auto=False, damping=.001, pair=None):
        definitions.append(dict(name=name, family=family, physical_error_m=list(map(float, error)),
                                matched_controller_lengths=matched, calibration_enabled=auto,
                                payload_rotational_damping=float(damping), pair=pair))
    zero = np.zeros(3)
    observed = np.array([.024, .002, .017])
    latest = np.array([.0283, .0060, .0208])
    add('nominal_00', 'zero_error', zero)
    for name, error in [('observed', observed), ('latest', latest)]:
        add(name + '_mismatch', name, error, pair=name)
        add(name + '_matched', name, error, matched=True, pair=name)
    rng = np.random.default_rng(20261018)
    for index, error in enumerate(rng.uniform(-.03, .03, size=(6, 3))):
        pair = 'random_%02d' % index
        add(pair + '_mismatch', 'random', error, pair=pair)
        add(pair + '_matched', 'random', error, matched=True, pair=pair)
    add('observed_auto', 'auto_calibration', observed, auto=True, pair='observed')
    add('weak_c0_nominal', 'weak_damping_zero', zero, damping=0.)
    add('weak_c0_observed_mismatch', 'weak_damping_observed', observed, damping=0., pair='weak_c0_observed')
    add('weak_c0_observed_matched', 'weak_damping_observed', observed, matched=True, damping=0., pair='weak_c0_observed')
    for index, case in enumerate(definitions):
        physical = nominal + np.array(case['physical_error_m'])
        attachment = payload_position + rho
        horizontal2 = np.sum((nominal_anchors[:, :2] - attachment[:, :2])**2, axis=1)
        if np.any(physical**2 <= horizontal2):
            raise ValueError('physical rope cannot reach frozen vehicle XY')
        positions = nominal_anchors.copy()
        positions[:, 2] = attachment[:, 2] + np.sqrt(physical**2 - horizontal2)
        np.testing.assert_allclose(np.linalg.norm(positions - attachment, axis=1), physical,
                                   rtol=0, atol=1e-12)
        controller_lengths = physical if case['matched_controller_lengths'] else nominal
        frozen = deepcopy(roots)
        frozen['slung_payload'].update(
            link_lengths_m=controller_lengths.tolist(), flight_validation_enabled=False,
            target_payload_height_m=.60, post_handoff_s=60.,
            link_calibration_enabled=case['calibration_enabled'])
        frozen['slung_payload']['simulation'].update(
            physical_link_lengths_m=physical.tolist(), initial_vehicle_positions_m=positions.tolist(),
            payload_rotational_damping=case['payload_rotational_damping'])
        directory = ROOT / 'cases' / case['name']
        directory.mkdir(parents=True, exist_ok=True)
        parameters = directory / 'parameters.yaml'
        parameters.write_text(yaml.safe_dump(frozen, sort_keys=False))
        case.update(index=index, worker=index % 3, controller_lengths_m=controller_lengths.tolist(),
                    physical_lengths_m=physical.tolist(), initial_vehicle_positions_m=positions.tolist(),
                    parameters_path=str(parameters.resolve()), parameters_sha256=digest(parameters),
                    output_directory=str((directory / 'replay').resolve()))
        (directory / 'case.json').write_text(json.dumps(case, indent=2) + '\n')
    assert len(definitions) == 21
    for pair in set(case['pair'] for case in definitions if case['pair']):
        cases = [case for case in definitions if case['pair'] == pair]
        for case in cases[1:]:
            assert case['physical_lengths_m'] == cases[0]['physical_lengths_m']
            assert case['initial_vehicle_positions_m'] == cases[0]['initial_vehicle_positions_m']
    manifest = dict(
        seed=20261018, case_count=len(definitions), cases=definitions, references=references,
        nominal_lengths_m=nominal.tolist(), nominal_takeup_anchors_m=nominal_anchors.tolist(),
        payload_initial_center_m=payload_position.tolist(), attachment_points_m=rho.tolist(),
        timing_source='latest 162455 only: CF3 unique fleet rows from handoff to first emergency',
        control_interval_pattern=dict(sample_count=len(timing), minimum_s=min(timing), maximum_s=max(timing),
                                      mean_s=float(np.mean(timing)), handoff_time_s=handoff, fault_time_s=fault),
        design_notes=[
            'Physical rope errors are sampled once per case and held fixed, never varied per frame.',
            'Matched/mismatched pairs use exactly the same physical ropes and initial positions. Controller lengths, and therefore their reference geometry, differ by design.',
            'Initial vehicle XY is frozen from nominal TAKEUP anchors; Z is solved from each physical rope sphere with identity payload rotation and grounded payload center.',
            'No calibration runs except observed_auto; the existing 0.030 m trigger and 0.030 m adjustment bound are retained, including their inability to apply strictly-above-trigger corrections.',
            'Primary 18 cases retain payload_rotational_damping=0.001; three explicitly named weak_c0 cases set it to zero.',
            'Payload bandwidth, damping, masses, motor model, safety protections and all other controller/plant parameters retain the frozen baseline.',
            'All cases disable the short validation timer and require the full 60 s after handoff at payload target height 0.60 m.',
            'User measurement endpoints are load attachment to CF mocap rigid-body origin, matching this model convention; pose/frame calibration errors and tendon solver compliance remain limitations.',
        ])
    (ROOT / 'cases.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(dict(cases=len(definitions), manifest=str(ROOT / 'cases.json'),
                         timing=manifest['control_interval_pattern'],
                         nominal=nominal.tolist(), random_errors_m=[case['physical_error_m']
                           for case in definitions if case['family']=='random' and not case['matched_controller_lengths']]),
                     indent=2))


if __name__ == '__main__':
    main()
