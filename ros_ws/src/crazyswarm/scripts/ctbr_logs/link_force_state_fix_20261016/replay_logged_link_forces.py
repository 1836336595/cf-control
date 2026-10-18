#!/usr/bin/env python3
"""Approximate offline force-path replay: identical inputs, different saved state.

This is not a closed-loop replay or a prediction of the corrected flight. Both
paths call the current production limiter on the same reconstructed correction
and kinematic force. Only the value retained for the next call differs.
"""

import argparse
import csv
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


SCRIPTS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCRIPTS))
from slung_load_controller import SlungLoadController


def vector(row, prefix):
    return np.array([float(row[prefix + '_' + axis]) for axis in 'xyz'])


def statistics(values):
    values = np.asarray(values, dtype=float)
    return dict(count=len(values), mean=float(np.mean(values)),
                rms=float(np.sqrt(np.mean(values * values))),
                p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)),
                p99=float(np.percentile(values, 99)), maximum=float(np.max(values)))


def summarize(rows):
    result = dict(samples=len(rows))
    for field in ('raw_correction_norm_n', 'kinematic_force_norm_n', 'state_path_difference_n',
                  'legacy_correction_norm_n', 'fixed_correction_norm_n',
                  'legacy_correction_rate_nps', 'fixed_correction_rate_nps',
                  'geometry_direction_residual', 'geometry_anchor_residual_m'):
        valid = [row[field] for row in rows if np.isfinite(row[field])]
        result[field] = statistics(valid)
    for flag in ('raw_magnitude_limited', 'legacy_rate_limited', 'fixed_rate_limited',
                 'legacy_only_rate_limited', 'legacy_magnitude_violation',
                 'fixed_magnitude_violation', 'legacy_output_rate_violation',
                 'fixed_output_rate_violation'):
        result[flag + '_count'] = sum(row[flag] for row in rows)
    result['changed_by_saved_state_count'] = sum(row['state_path_difference_n'] > 1e-9 for row in rows)
    changed = [row for row in rows if row['state_path_difference_n'] > 1e-9]
    result['changed_sample_dt_ms'] = (statistics([row['dt_s'] * 1000 for row in changed])
                                      if changed else None)
    result['changed_sample_dt_below_10_ms_count'] = sum(row['dt_s'] < .010 for row in changed)
    result['legacy_only_rate_limited_fraction'] = result['legacy_only_rate_limited_count'] / len(rows)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=SCRIPTS / 'ctbr_logs/multi_ctbr_20261016_213804.csv')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with args.source.open() as stream:
        source = list(csv.DictReader(stream))
    handoff = next(float(row['control_time_s']) for row in source
                   if row['vehicle_id'] == '3' and row['flight_phase'] == 'tension_ramp')
    selected = [row for row in source if handoff <= float(row['control_time_s']) < handoff + 10
                and row['flight_phase'] in ('tension_ramp', 'height_correction')]
    payload = yaml.safe_load((SCRIPTS.parent / 'config/slung_payload.yaml').read_text())['slung_payload']
    vehicles = yaml.safe_load((SCRIPTS.parent / 'config/ctbr_vehicle.yaml').read_text())
    mass = float(payload['mass_kg'])
    size = np.asarray(payload['size_m'], dtype=float)
    inertia = mass / 12 * np.array([size[1]**2 + size[2]**2,
                                  size[0]**2 + size[2]**2, size[0]**2 + size[1]**2])
    rho = np.asarray(payload['attachment_points_m'], dtype=float)
    limit = float(payload['link_force_correction_limit_n'])
    slew = float(payload['link_force_correction_rate_limit_nps'])
    kq, kw = float(payload['link_kq']), float(payload['link_komega'])
    # The isolated private method uses only these two configuration values.
    limiter = SlungLoadController.__new__(SlungLoadController)
    limiter.config = SimpleNamespace(link_force_correction_limit_n=limit,
                                     link_force_correction_rate_limit_nps=slew)
    histories = {vehicle: dict(legacy=None, fixed=None, previous_legacy_output=None,
                               previous_fixed_output=None) for vehicle in (3, 4, 5)}
    output = []
    for row in selected:
        vehicle = int(row['vehicle_id'])
        index = vehicle - 3
        vehicle_mass = float(vehicles['ctbr_controller_cf%d' % vehicle]['mass_kg'])
        length = float(row['link_length_effective_m'] or row['link_length_nominal_m'])
        measured_length = float(row['link_length_measured_m'])
        dt = float(np.clip(float(row['controller_tick_interval_s']), .001, .05))
        error = vector(row, 'payload_attitude_error')
        sine = float(np.linalg.norm(error))
        if sine >= 1:
            raise ValueError('principal SO(3) reconstruction is ambiguous at this logged error')
        rotvec = error if sine < 1e-12 else error * (np.arcsin(sine) / sine)
        rotation = Rotation.from_rotvec(rotvec).as_matrix()
        q = vector(row, 'link_direction')
        q /= np.linalg.norm(q)
        projector = np.eye(3) - np.outer(q, q)
        omega = vector(row, 'payload_body_rate')
        payload_velocity = vector(row, 'payload_velocity')
        vehicle_velocity = vector(row, 'control_velocity')
        offset_world = rotation @ rho[index]
        relative_velocity = (payload_velocity + np.cross(rotation @ omega, offset_world)
                             - vehicle_velocity)
        qdot = projector @ relative_velocity / max(measured_length, 1e-6)
        omega_link = np.cross(q, qdot)
        eq = vector(row, 'link_direction_error')
        scale = float(row['effective_transport_link_gain_scale'])
        raw_force = vehicle_mass * length * np.cross(q, -scale * (kq * eq + kw * omega_link))
        force = vector(row, 'payload_desired_force')
        moment = vector(row, 'payload_desired_moment')
        omega_dot = (moment - np.cross(omega, inertia * omega)) / inertia
        ai = force / mass + rotation @ (
            np.cross(omega, np.cross(omega, rho[index])) + np.cross(omega_dot, rho[index]))
        kinematic = vehicle_mass * projector @ ai
        history = histories[vehicle]
        legacy, _ = limiter._limit_link_force(raw_force, history['legacy'], dt)
        fixed, _ = limiter._limit_link_force(raw_force, history['fixed'], dt)
        magnitude = np.linalg.norm(raw_force)
        bounded = raw_force * min(1., limit / max(magnitude, 1e-12))
        legacy_limited = np.linalg.norm(legacy - bounded) > 1e-12
        fixed_limited = np.linalg.norm(fixed - bounded) > 1e-12
        if not fixed_limited:
            np.testing.assert_allclose(fixed, bounded, rtol=0, atol=1e-12)
        previous_legacy = history['previous_legacy_output']
        previous_fixed = history['previous_fixed_output']
        legacy_rate = (float('nan') if previous_legacy is None
                       else np.linalg.norm(legacy - previous_legacy) / dt)
        fixed_rate = (float('nan') if previous_fixed is None
                      else np.linalg.norm(fixed - previous_fixed) / dt)
        geometry = (vector(row, 'payload_position') + offset_world - vector(row, 'position'))
        geometry_q = geometry / max(np.linalg.norm(geometry), 1e-12)
        result = dict(
            control_time_s=float(row['control_time_s']),
            handoff_elapsed_s=float(row['control_time_s']) - handoff,
            vehicle_id=vehicle, dt_s=dt, link_gain_scale=scale,
            vehicle_mass_kg=vehicle_mass, effective_length_m=length,
            raw_correction_norm_n=float(magnitude), kinematic_force_norm_n=float(np.linalg.norm(kinematic)),
            legacy_correction_norm_n=float(np.linalg.norm(legacy)),
            fixed_correction_norm_n=float(np.linalg.norm(fixed)),
            legacy_correction_rate_nps=legacy_rate, fixed_correction_rate_nps=fixed_rate,
            state_path_difference_n=float(np.linalg.norm(legacy - fixed)),
            geometry_direction_residual=float(np.linalg.norm(geometry_q - q)),
            geometry_anchor_residual_m=float(np.linalg.norm(geometry - measured_length * q)),
            raw_magnitude_limited=bool(magnitude > limit + 1e-12),
            legacy_rate_limited=bool(legacy_limited), fixed_rate_limited=bool(fixed_limited),
            legacy_only_rate_limited=bool(legacy_limited and not fixed_limited),
            legacy_magnitude_violation=bool(np.linalg.norm(legacy) > limit + 1e-9),
            fixed_magnitude_violation=bool(np.linalg.norm(fixed) > limit + 1e-9),
            legacy_output_rate_violation=bool(legacy_rate > slew + 1e-9),
            fixed_output_rate_violation=bool(fixed_rate > slew + 1e-9),
        )
        for name, value in (('raw_correction', raw_force), ('kinematic_force', kinematic),
                            ('legacy_correction', legacy), ('fixed_correction', fixed)):
            result.update({name + '_' + axis + '_n': float(item)
                           for axis, item in zip('xyz', value)})
        output.append(result)
        # This is the only difference between paths. The kinematic contribution
        # is added to both physical outputs, but belonged in neither history.
        history['legacy'] = legacy + kinematic
        history['fixed'] = fixed.copy()
        history['previous_legacy_output'] = legacy.copy()
        history['previous_fixed_output'] = fixed.copy()
    with (args.output / 'logged_link_force_replay.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output[0]))
        writer.writeheader()
        writer.writerows(output)
    metrics = dict(
        analysis_type='approximate_offline_force_path_replay', source=str(args.source.resolve()),
        closed_loop_simulation=False, corrected_flight_prediction=False,
        production_limiter_source=str(Path(sys.modules[SlungLoadController.__module__].__file__).resolve()),
        handoff_time_s=handoff, analyzed_duration_s=10.,
        reconstruction=dict(payload_mass_kg=mass, payload_size_m=size.tolist(),
            payload_inertia_kgm2=inertia.tolist(), attachment_points_m=rho.tolist(),
            link_kq=kq, link_komega=kw, correction_limit_n=limit, correction_rate_limit_nps=slew,
            masses_kg={str(i): vehicles['ctbr_controller_cf%d' % i]['mass_kg'] for i in (3,4,5)}),
        assumptions=[
            'Desired payload rotation Rd is identity; payload R is reconstructed on the principal SO(3) branch from sin(theta)*axis attitude error.',
            'Payload and vehicle CSV rows are asynchronous snapshots; reconstructed geometry and velocities therefore differ slightly from the exact controller input.',
            'Payload mass/inertia, attachment points, link gains and limiter settings use the current parameter file; effective cable lengths use the logged values.',
            'Initial correction histories are empty at handoff, matching the newly activated slung controller.',
            'No physical measurements or later flight states change between the two paths; only limiter history changes.',
            'The previous perpendicular-force limiter history is emulated as limited correction plus kinematic force; the fixed history stores limited correction only.',
        ], windows={})
    for label, lower, upper in [('all_transport', 0., 10.), ('first_2_s', 0., 2.), ('last_3_s', 7., 10.)]:
        window = [row for row in output if lower <= row['handoff_elapsed_s'] < upper]
        metrics['windows'][label] = dict(all_vehicles=summarize(window), vehicles={
            str(vehicle): summarize([row for row in window if row['vehicle_id'] == vehicle])
            for vehicle in (3, 4, 5)})
    (args.output / 'logged_link_force_replay.json').write_text(json.dumps(metrics, indent=2) + '\n')
    lines = [
        '# Approximate logged force-path replay', '',
        'Source: `multi_ctbr_20261016_213804.csv`, the first 10 seconds after the 48.283755 s handoff. '
        'The current production `_limit_link_force` runs twice with identical reconstructed raw correction, '
        'kinematic force and clipped 1–50 ms control intervals. The old path retains `limited + kinematic`; '
        'the corrected path retains `limited`. This does not change recorded inputs or simulate the resulting flight.', '',
        '| Window | Vehicle samples | Outputs changed by saved state | Old-only slew limiting | Corrected genuine slew limiting | Difference RMS / max (N) | Old correction slew violations | Corrected slew violations |',
        '|---|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for name, window in metrics['windows'].items():
        item = window['all_vehicles']
        lines.append('| %s | %d | %d | %d | %d | %.6f / %.6f | %d | %d |' % (
            name, item['samples'], item['changed_by_saved_state_count'],
            item['legacy_only_rate_limited_count'], item['fixed_rate_limited_count'],
            item['state_path_difference_n']['rms'], item['state_path_difference_n']['maximum'],
            item['legacy_output_rate_violation_count'], item['fixed_output_rate_violation_count']))
    all_metrics = metrics['windows']['all_transport']['all_vehicles']
    lines += ['',
        'The corrected path has %d magnitude-limited and %d slew-limited samples. '
        'The old history changes %d of %d vehicle samples; %d changed samples have intervals below 10 ms. '
        'The largest old correction slew is %.6f N/s versus %.6f N/s for the corrected path (configured cap %.3f N/s).' % (
            all_metrics['raw_magnitude_limited_count'], all_metrics['fixed_rate_limited_count'],
            all_metrics['changed_by_saved_state_count'], all_metrics['samples'],
            all_metrics['changed_sample_dt_below_10_ms_count'],
            all_metrics['legacy_correction_rate_nps']['maximum'],
            all_metrics['fixed_correction_rate_nps']['maximum'], slew), '',
        'Reconstruction limitations: desired payload rotation is assumed to be identity; the principal SO(3) '
        'rotation is recovered from logged attitude error. Per-vehicle payload/vehicle rows are asynchronous snapshots, '
        'and physical masses, geometry and limiter coefficients come from the current parameter files. '
        'The resulting anchor geometry residual is RMS %.6f m, maximum %.6f m. '
        'This fit supports an approximate force-path comparison; it does not recover exact controller inputs or '
        'establish the corrected closed-loop behavior.' % (
            all_metrics['geometry_anchor_residual_m']['rms'],
            all_metrics['geometry_anchor_residual_m']['maximum']), '',
        'Reproduce with `python3 replay_logged_link_forces.py`. CSV includes every raw/kinematic/old/fixed vector '
        'and event flag; JSON includes per-window and per-vehicle statistics. The PNG compares force norms.',
    ]
    (args.output / 'logged_link_force_replay.md').write_text('\n'.join(lines) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for axis, vehicle in zip(axes, (3, 4, 5)):
        own = [row for row in output if row['vehicle_id'] == vehicle]
        times = [row['handoff_elapsed_s'] for row in own]
        for name, label in [('legacy_correction_norm_n', 'Old saved state'),
                            ('fixed_correction_norm_n', 'Corrected saved state'),
                            ('kinematic_force_norm_n', 'Kinematic compensation')]:
            axis.plot(times, [row[name] for row in own], label=label, linewidth=1.)
        axis.axhline(limit, color='black', linewidth=.7, linestyle=':')
        axis.set_ylabel('CF%d force norm (N)' % vehicle)
        axis.grid(alpha=.25)
    axes[0].legend(loc='upper left', ncol=3)
    axes[-1].set_xlabel('Seconds after handoff')
    figure.suptitle('Approximate offline force-path replay: identical logged inputs\n'
                   'Only the limiter saved state differs; this is not a closed-loop prediction')
    figure.tight_layout()
    figure.savefig(args.output / 'logged_link_force_replay.png', dpi=160)
    plt.close(figure)
    print(json.dumps({name: item['all_vehicles'] for name, item in metrics['windows'].items()}, indent=2))


if __name__ == '__main__':
    main()
