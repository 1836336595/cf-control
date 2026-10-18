#!/usr/bin/env python3
"""Analyze frozen rope experiments without conflating gate failures and instability.

Trace values are MuJoCo truth at irregular control instants. Statistics integrate
piecewise-linear signals in physical time; no noise spectrum is inferred from
quiet deterministic traces. Completed-case metrics are cached to avoid repeatedly
reading large CSVs while independent workers are still producing results.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
IDS = (3, 4, 5)
WINDOWS = (('handoff_0_2_s', 0., 2.), ('handoff_2_5_s', 2., 5.),
           ('handoff_5_8_s', 5., 8.), ('handoff_40_60_s', 40., 60.))
FIELDS = ('control_time_s', 'payload_roll_rad', 'payload_pitch_rad', 'payload_yaw_rad',
          'payload_clearance_m', 'physical_cable_length_m', 'physical_cable_limit_m',
          'payload_position_x', 'payload_position_y', 'payload_position_z',
          'payload_body_rate_x', 'payload_body_rate_y', 'payload_body_rate_z',
          'position_x', 'position_y', 'position_z', 'body_rate_x', 'body_rate_y', 'body_rate_z',
          'command_rate_x', 'command_rate_y', 'command_rate_z', 'command_thrust_n')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_trace(path):
    by_vehicle = {i: [] for i in IDS}
    with path.open() as stream:
        for row in csv.DictReader(stream):
            vehicle = int(row['vehicle_id'])
            by_vehicle[vehicle].append([float(row[name]) for name in FIELDS])
    result = {}
    for vehicle, rows in by_vehicle.items():
        matrix = np.asarray(rows, dtype=float)
        if not len(matrix):
            raise ValueError('trace is missing CF%d' % vehicle)
        result[vehicle] = {name: matrix[:, index] for index, name in enumerate(FIELDS)}
    return result


def vectors(trace, prefix):
    return np.column_stack([trace[prefix + '_' + axis] for axis in 'xyz'])


def clipped_signal(time, value, lower, upper):
    """Clip to observed support and interpolate the two physical-time bounds."""
    start, stop = max(lower, time[0]), min(upper, time[-1])
    if stop <= start:
        return None, None
    selected = (time > start) & (time < stop)
    clipped_time = np.r_[start, time[selected], stop]
    value = np.asarray(value)
    if value.ndim == 1:
        clipped_value = np.interp(clipped_time, time, value)
    else:
        clipped_value = np.column_stack([np.interp(clipped_time, time, value[:, i])
                                          for i in range(value.shape[1])])
    return clipped_time, clipped_value


def time_statistics(time, value, lower, upper):
    sample_time, sample = clipped_signal(time, value, lower, upper)
    if sample_time is None:
        return None
    duration = sample_time[-1] - sample_time[0]
    integral = getattr(np, 'trapezoid', np.trapz)
    mean = integral(sample, sample_time, axis=0) / duration
    variance = np.maximum(0., integral((sample - mean)**2, sample_time, axis=0) / duration)
    rms = np.sqrt(integral(sample**2, sample_time, axis=0) / duration)
    def serial(value):
        return float(value) if np.ndim(value) == 0 else value.tolist()
    return dict(duration_s=float(duration), observed_start_s=float(sample_time[0]),
                observed_end_s=float(sample_time[-1]),
                mean=serial(mean), std=serial(np.sqrt(variance)), rms=serial(rms),
                minimum=serial(np.min(sample, axis=0)), maximum=serial(np.max(sample, axis=0)),
                maximum_absolute=serial(np.max(np.abs(sample), axis=0)))


def rotation_matrices(trace):
    roll, pitch, yaw = (trace['payload_' + axis + '_rad'] for axis in ('roll', 'pitch', 'yaw'))
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    result = np.empty((len(roll), 3, 3))
    result[:, 0, 0] = cy * cp
    result[:, 0, 1] = cy * sp * sr - sy * cr
    result[:, 0, 2] = cy * sp * cr + sy * sr
    result[:, 1, 0] = sy * cp
    result[:, 1, 1] = sy * sp * sr + cy * cr
    result[:, 1, 2] = sy * sp * cr - cy * sr
    result[:, 2, 0] = -sp
    result[:, 2, 1] = cp * sr
    result[:, 2, 2] = cp * cr
    return result


def failure_category(summary):
    if summary.get('success'):
        return 'full_60_s_success'
    failure = summary.get('failure') or 'unknown'
    if summary.get('handoff_time_s') is None:
        return ('pre_handoff_gate_timeout' if failure in ('handoff_timeout', 'hover_not_reached')
                else 'pre_handoff_protection_or_invalid_state')
    if failure in ('post_handoff_window_incomplete', 'hover_not_reached'):
        return 'post_handoff_incomplete_window'
    return 'post_handoff_protection_or_failure'


def analyze_case(case, manifest):
    output = Path(case['output_directory'])
    summary_path, metadata_path, trace_path = (output / name for name in
                                              ('summary.json', 'run_metadata.json', 'simulation_trace.csv'))
    record = {key: case[key] for key in ('name', 'family', 'pair', 'physical_error_m',
              'matched_controller_lengths', 'calibration_enabled', 'payload_rotational_damping',
              'controller_lengths_m', 'physical_lengths_m')}
    if not all(path.exists() for path in (summary_path, metadata_path, trace_path)):
        error = output / 'process_error.json'
        record['status'] = 'process_error' if error.exists() else 'pending'
        if error.exists():
            record['process_error'] = json.loads(error.read_text())
        return record
    signature = dict(summary_sha256=digest(summary_path), trace_bytes=trace_path.stat().st_size,
                     trace_mtime_ns=trace_path.stat().st_mtime_ns, analysis_version=1)
    cache_path = output / 'rope_analysis.json'
    if cache_path.exists():
        cached = json.loads(cache_path.read_text())
        if cached.get('cache_signature') == signature:
            return cached
    summary, metadata = json.loads(summary_path.read_text()), json.loads(metadata_path.read_text())
    trace = load_trace(trace_path)
    time = trace[3]['control_time_s']
    handoff = summary.get('handoff_time_s')
    rho = np.array(manifest['attachment_points_m'], dtype=float)
    record.update(status='complete', cache_signature=signature,
        outcome_category=failure_category(summary), success=bool(summary['success']),
        failure=summary.get('failure'), terminal_reason=summary.get('terminal_reason'),
        handoff_time_s=handoff, observed_post_handoff_s=summary.get('observed_post_handoff_s'),
        final_time_s=summary['final_time_s'], last_trace_time_s=float(time[-1]),
        final_phase=summary.get('phases', ['unknown'])[-1], phases=summary.get('phases'),
        safety_reason=summary.get('payload_safety_reason'),
        minimum_vehicle_distance_m=summary.get('minimum_vehicle_distance_m'),
        minimum_transport_clearance_m=summary.get('minimum_clearance_m'),
        maximum_body_rate_rps=summary.get('maximum_body_rate_rps'),
        final_payload_position_error_m=summary.get('final_payload_position_error_m'),
        rate_saturation_fraction=summary.get('rate_saturation_fraction'),
        calibration_status=summary.get('link_calibration_status'),
        calibration_reason=summary.get('link_calibration_reason'),
        calibration_applied=summary.get('link_calibration_applied'),
        controller_initial_lengths_m=summary.get('nominal_link_lengths_m'),
        controller_final_lengths_m=summary.get('effective_link_lengths_m'),
        physical_lengths_verified_m=summary.get('physical_link_lengths_m'),
        wall_time_s=metadata.get('wall_time_s'),
        initial_cable_error_from_summary_m=summary.get('initial_physical_cable_errors_m'),
        windows={}, cable_diagnostics={})
    payload_rotation = rotation_matrices(trace[3])
    payload_position = vectors(trace[3], 'payload_position')
    payload_rpy = np.rad2deg(np.column_stack([trace[3]['payload_' + axis + '_rad']
                                             for axis in ('roll', 'pitch', 'yaw')]))
    payload_rate = vectors(trace[3], 'payload_body_rate')
    all_rate = np.column_stack([vectors(trace[vehicle], 'body_rate') for vehicle in IDS])
    observed_end = float(time[-1])
    for name, lower, upper in WINDOWS:
        if handoff is None:
            record['windows'][name] = dict(available=False, complete=False, reason='handoff_not_reached')
            continue
        lower, upper = handoff + lower, handoff + upper
        statistics = time_statistics(time, payload_rpy, lower, upper)
        if statistics is None:
            record['windows'][name] = dict(available=False, complete=False, reason='window_not_observed')
            continue
        own = {}
        for vehicle in IDS:
            own[str(vehicle)] = dict(
                body_rate_radps=time_statistics(time, vectors(trace[vehicle], 'body_rate'), lower, upper),
                command_rate_radps=time_statistics(time, vectors(trace[vehicle], 'command_rate'), lower, upper))
        record['windows'][name] = dict(available=True, complete=observed_end >= upper-1e-9,
            payload_rpy_deg=statistics,
            payload_body_rate_radps=time_statistics(time, payload_rate, lower, upper),
            cf4_command_rate_radps=time_statistics(time, vectors(trace[4], 'command_rate'), lower, upper),
            all_vehicle_body_rate_rms_radps=time_statistics(time, all_rate, lower, upper),
            vehicles=own)
    record['post_handoff_truth'] = (dict(
        payload_rpy_deg=time_statistics(time, payload_rpy, handoff, observed_end),
        payload_body_rate_radps=time_statistics(time, payload_rate, handoff, observed_end))
        if handoff is not None else None)
    for index, vehicle in enumerate(IDS):
        own = trace[vehicle]
        extension = own['physical_cable_length_m'] - own['physical_cable_limit_m']
        geometry = payload_position + np.einsum('nij,j->ni', payload_rotation, rho[index]) - vectors(own, 'position')
        reconstructed_length = np.linalg.norm(geometry, axis=1)
        residual = reconstructed_length - own['physical_cable_length_m']
        positions = vectors(own, 'position')
        pair_dt = np.diff(time)
        secant_velocity = np.diff(positions, axis=0) / pair_dt[:, None]
        first_second = time[1:] <= 1.0 + 1e-9
        ground = own['payload_clearance_m'] <= .03
        before = np.ones(len(time), dtype=bool) if handoff is None else time < handoff
        after = np.zeros(len(time), dtype=bool) if handoff is None else time >= handoff
        def limits(mask):
            return (dict(minimum_delta_m=float(np.min(extension[mask])),
                         maximum_extension_m=float(max(0., np.max(extension[mask]))),
                         maximum_signed_delta_m=float(np.max(extension[mask])),
                         positive_extension_sample_count=int(np.sum(extension[mask] > 1e-6)))
                    if np.any(mask) else None)
        record['cable_diagnostics'][str(vehicle)] = dict(
            initial_error_from_summary_m=summary.get('initial_physical_cable_errors_m', [None]*3)[index],
            initial_error_from_trace_m=float(extension[0]), prehandoff=limits(before),
            posthandoff=limits(after), payload_near_ground=limits(ground),
            geometry_vs_tendon_residual_max_abs_m=float(np.max(np.abs(residual))),
            geometry_vs_tendon_residual_rms_m=float(np.sqrt(np.mean(residual**2))),
            first_1_s_position_secant_speed_max_mps=(float(np.max(np.linalg.norm(secant_velocity[first_second],axis=1)))
                                                    if np.any(first_second) else None),
            first_1_s_position_secant_vz_max_abs_mps=(float(np.max(np.abs(secant_velocity[first_second,2])))
                                                    if np.any(first_second) else None))
    cache_path.write_text(json.dumps(record, indent=2) + '\n')
    return record


def make_plots(records, cases):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    complete = [record for record in records if record['status'] == 'complete']
    figure, axis = plt.subplots(figsize=(15, max(7, len(records)*.39)))
    axis.axis('off')
    body = []
    for record in records:
        truth = record.get('post_handoff_truth') or {}
        rpy = truth.get('payload_rpy_deg')
        max_angles = '-' if not rpy else '/'.join('%.2f'%x for x in rpy['maximum_absolute'])
        body.append([record['name'], ','.join('%+.1f'%(x*1000) for x in record['physical_error_m']),
                     'yes' if record['matched_controller_lengths'] else 'no',
                     record.get('outcome_category',record['status']),
                     '-' if record.get('handoff_time_s') is None else '%.2f'%record['handoff_time_s'],
                     '-' if record.get('observed_post_handoff_s') is None else '%.2f'%record['observed_post_handoff_s'],
                     max_angles, '-' if record.get('minimum_vehicle_distance_m') is None else
                     '%.3f'%record['minimum_vehicle_distance_m']])
    table = axis.table(cellText=body, colLabels=['Case','Physical error mm','Matched','Outcome','Handoff s',
                                               'Observed s','Max |R/P/Y| deg','Min D m'],
                       cellLoc='left',colLoc='left',loc='center',
                       colWidths=[.20,.15,.06,.24,.08,.08,.13,.07])
    table.auto_set_font_size(False);table.set_fontsize(8);table.scale(1,1.6)
    figure.suptitle('Frozen physical rope errors: all cases (pending rows retained)\n'
                   'Failure before handoff is reported as gate/protection, not proof of transport instability',fontsize=12)
    figure.tight_layout();figure.savefig(ROOT/'rope_comparison_overview.png',dpi=160);plt.close(figure)
    if not complete:
        return
    by_name={record['name']:record for record in complete}
    score={}
    for record in complete:
        if record['family']!='random':continue
        p=record.get('post_handoff_truth') or {};rpy=p.get('payload_rpy_deg')
        score[record['pair']]=max(score.get(record['pair'],0.),max(rpy['maximum_absolute']) if rpy else 0.)
    worst=sorted(score,key=score.get,reverse=True)[:2]
    groups=[('Nominal / supplied / latest pairs',['nominal_00','observed_mismatch','observed_matched',
                                                 'latest_mismatch','latest_matched']),
            ('Worst two completed random pairs',[pair+suffix for pair in worst for suffix in ('_mismatch','_matched')]),
            ('Zero artificial payload rotational damping',['weak_c0_nominal','weak_c0_observed_mismatch',
                                                           'weak_c0_observed_matched'])]
    figure,axes=plt.subplots(3,2,figsize=(13,10),sharex=True)
    for line,(title,names) in enumerate(groups):
        for name in names:
            if name not in by_name or by_name[name].get('handoff_time_s') is None:continue
            case=next(case for case in cases if case['name']==name)
            trace=load_trace(Path(case['output_directory'])/'simulation_trace.csv')[3]
            t=trace['control_time_s']-by_name[name]['handoff_time_s'];keep=t>=0
            for column,angle in enumerate(('pitch','yaw')):
                axes[line,column].plot(t[keep],np.rad2deg(trace['payload_'+angle+'_rad'][keep]),
                                       linewidth=1.,label=name)
        for column,angle in enumerate(('pitch','yaw')):
            axes[line,column].set_title(title+' — '+angle)
            axes[line,column].set_ylabel('Truth angle (deg)');axes[line,column].grid(alpha=.25)
            handles,labels=axes[line,column].get_legend_handles_labels()
            if handles:axes[line,column].legend(fontsize=7)
    for axis in axes[-1]:axis.set_xlabel('Seconds after handoff')
    figure.suptitle('Identical physical pair initialization; only controller rope knowledge differs\n'
                   'No sensor noise added; traces may end at gates or protection rather than complete 60 s')
    figure.tight_layout();figure.savefig(ROOT/'rope_selected_truth_traces.png',dpi=160);plt.close(figure)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-plots',action='store_true')
    args=parser.parse_args()
    manifest=json.loads((ROOT/'cases.json').read_text())
    records=[]
    for case in manifest['cases']:
        try:records.append(analyze_case(case,manifest))
        except (ValueError,OSError,json.JSONDecodeError,KeyError) as error:
            records.append(dict(name=case['name'],family=case['family'],pair=case['pair'],
                physical_error_m=case['physical_error_m'],matched_controller_lengths=case['matched_controller_lengths'],
                status='awaiting_complete_artifacts',analysis_error=str(error)))
    paired=[]
    for pair in sorted({case['pair'] for case in manifest['cases'] if case['pair']}):
        mismatch=next((r for r in records if r['name']==pair+'_mismatch'),None)
        matched=next((r for r in records if r['name']==pair+'_matched'),None)
        if mismatch and matched:paired.append(dict(pair=pair,mismatch=mismatch['name'],matched=matched['name'],
            both_complete=all(r['status']=='complete' for r in (mismatch,matched)),
            mismatch_outcome=mismatch.get('outcome_category',mismatch['status']),
            matched_outcome=matched.get('outcome_category',matched['status'])))
    result=dict(cases_manifest_sha256=digest(ROOT/'cases.json'),total_cases=len(records),
        complete_cases=sum(r['status']=='complete' for r in records),records=records,pairs=paired,
        statistical_method='piecewise-linear interpolation at window boundaries; physical-time trapezoidal mean/variance/RMS, never per-row weighting',
        caveats=[
            'These are deterministic simulated cases using frozen Graefe/Mellinger plant, configured sampled EKF and repeated measured timing. They do not reproduce every real-flight disturbance.',
            'No PSD/noise claims are made for quiet deterministic signals.',
            'MuJoCo tendon constraints are soft. Positive length-minus-limit is numerical constraint extension, not calibrated physical rope elasticity.',
            'First-second speed is secant speed from truth positions at control instants, not an independent high-rate velocity or collision sensor.',
            'Window completeness is explicit; truncated post-handoff windows must not be compared as full 60-second passes.',
            'An exit before handoff proves that gate/protection did not complete for that initialization; it does not establish unstable cooperative transport.',
        ])
    (ROOT/'rope_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    columns=['name','family','status','outcome_category','physical_error_mm','matched_controller_lengths',
             'calibration_applied','calibration_status','calibration_reason','handoff_time_s',
             'observed_post_handoff_s','failure','safety_reason','final_time_s','minimum_vehicle_distance_m',
             'maximum_body_rate_rps','post_max_abs_roll_deg','post_max_abs_pitch_deg','post_max_abs_yaw_deg',
             'final_payload_position_error_m','controller_initial_lengths_m','controller_final_lengths_m',
             'physical_lengths_verified_m','wall_time_s']
    with (ROOT/'rope_comparison.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader()
        for record in records:
            row={key:record.get(key) for key in columns}
            row['physical_error_mm']=[value*1000 for value in record['physical_error_m']]
            truth=record.get('post_handoff_truth') or {};rpy=truth.get('payload_rpy_deg')
            for index,angle in enumerate(('roll','pitch','yaw')):
                row['post_max_abs_'+angle+'_deg']=rpy['maximum_absolute'][index] if rpy else None
            writer.writerow(row)
    if not args.no_plots:make_plots(records,manifest['cases'])
    print(json.dumps(dict(total=len(records),complete=result['complete_cases'],
        outcomes={str(outcome):sum(r.get('outcome_category',r['status'])==outcome for r in records)
                  for outcome in {r.get('outcome_category',r['status']) for r in records}}),indent=2))


if __name__=='__main__':main()
