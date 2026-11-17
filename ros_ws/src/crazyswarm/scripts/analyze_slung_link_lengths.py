#!/usr/bin/env python3
"""Recompute cable endpoint lengths from a recorded fleet CSV, not frozen fields."""

import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
import yaml
from slung_link_length_report import SlungLinkLengthReporter


def rotation_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def number(row, key):
    try:
        return float(row.get(key) or 'nan')
    except ValueError:
        return math.nan


def vector(row, prefix):
    return np.array([number(row, prefix+'_'+axis) for axis in 'xyz'])


def analyze(path, config, config_path):
    points = np.asarray(config['attachment_points_m'], dtype=float)
    reporter = SlungLinkLengthReporter(points,
        window_s=config.get('link_length_report_window_s', 3.),
        level_tolerance_deg=config.get('link_length_report_level_tolerance_deg', 3.))
    rows = []
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            if row.get('vehicle_id') in ('3','4','5'):
                rows.append(row)
    frames = {}
    for row in rows:
        frames.setdefault(number(row, 'control_time_s'), {})[int(row['vehicle_id'])] = row
    handoff = next((t for t, frame in frames.items()
                    if frame.get(3, {}).get('flight_phase') == 'tension_ramp'), None)
    accepted = []
    reports = []
    seen = set()
    history = None
    statuses = Counter()
    for t, frame in frames.items():
        if set(frame) != {3,4,5}:
            continue
        row = frame[3]
        if handoff is None or t < handoff:
            continue
        if any(r['flight_phase'] in ('landing','landing_settle','landed','emergency_landing','aborted')
               for r in frame.values()):
            reporter.reset_window()
            continue
        if number(row, 'payload_control_reused') == 1.:
            continue  # An unchanged command snapshot supplies no new pose evidence.
        stamp = number(row, 'payload_control_sample_time_s')
        if not math.isfinite(stamp):
            continue  # Older logs without source timing cannot support synchronized inference here.
        if stamp in seen:
            continue
        seen.add(stamp)
        stamps = [number(frame[vid], 'ros_time_s')-number(frame[vid], 'mocap_sample_age_s') for vid in (3,4,5)]
        valid = bool(number(row, 'payload_state_valid') == 1.
                     and number(row, 'payload_filter_derivatives_valid') == 1.
                     and np.all(np.isfinite(stamps)) and np.max(np.abs(np.asarray(stamps)-stamp)) <= .03
                     and -.02 <= number(row,'ros_time_s')-stamp <= .10)
        if not valid:
            reporter.reset_window()
            continue
        token=(number(row,'payload_observer_reset_count'), number(row,'payload_observer_rejected_count'))
        if history != token:
            reporter.reset_window()
            history=token
        angles = [number(row,'payload_'+axis+'_rad') for axis in ('roll','pitch','yaw')]
        desired = [number(row,'payload_target_'+axis+'_rad') for axis in ('roll','pitch','yaw')]
        position = vector(row,'payload_position')
        positions = np.array([vector(frame[vid],'position') for vid in (3,4,5)])
        if not all(np.all(np.isfinite(a)) for a in (angles,desired,position,positions)):
            reporter.reset_window()
            continue
        rotation = rotation_from_rpy(*angles)
        target = rotation_from_rpy(*desired)
        lengths = np.linalg.norm(positions-(position+points@rotation.T),axis=1)
        clearance = position[2]-.5*np.abs(rotation[2])@np.asarray(config['size_m'])
        stationary = bool(target[2,2] >= math.cos(math.radians(reporter.level_tolerance_deg))
            and np.linalg.norm(vector(row,'payload_target_velocity')) < .01
            and np.linalg.norm(vector(row,'payload_target_body_rate')) < .01)
        eligible = bool(t-handoff >= max(config['tension_ramp_s']+config['reference_lift_s'],
                                        config.get('transport_attitude_reference_s',0.))
            and stationary and clearance >= .10
            and all(number(r,'desired_tension_signed_n') > 0. for r in frame.values()))
        report = reporter.update(stamp,position,rotation,positions,eligible)
        if eligible:statuses[reporter.status]+=1
        tilt = math.degrees(math.acos(float(np.clip(rotation[2,2],-1.,1.))))
        accepted.append(dict(control_time_s=t,h_s=t-handoff,sample_time=stamp,
            lengths_m=lengths,tilt_deg=tilt,eligible=eligible,stationary=stationary,
            logged_measured_m=[number(frame[v],'link_length_measured_m') for v in (3,4,5)]))
        if report is not None:
            reports.append(dict(report,control_time_s=t,h_s=t-handoff))
    # Explicitly labeled descriptive window, even if full online eligibility was not met.
    tail=[]
    if accepted:
        end=accepted[-1]['control_time_s']
        tail=[r for r in accepted if r['control_time_s']>=end-10. and r['eligible']]
    def statistics(records):
        if not records:return None
        lengths=np.array([r['lengths_m'] for r in records])
        return dict(samples=len(records),h_interval_s=[records[0]['h_s'],records[-1]['h_s']],
            median_m=np.median(lengths,axis=0),mean_m=np.mean(lengths,axis=0),
            p10_m=np.percentile(lengths,10,axis=0),p90_m=np.percentile(lengths,90,axis=0),
            tilt_median_deg=float(np.median([r['tilt_deg'] for r in records])),
            tilt_max_deg=max(r['tilt_deg'] for r in records))
    return dict(source=str(path.resolve()),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        geometry_config=str(config_path.resolve()),geometry_note='Uses supplied config mount points and box dimensions. Verify these match the historical flight. Endpoints: payload mount to aircraft mocap origin. No physical rope tension sensor.',
        attachment_points_m=points,nominal_lengths_m=config['link_lengths_m'],
        handoff_control_time_s=handoff,stable_reports=reports,
        stationary_window_status_counts=dict(statuses),
        last_10s_stationary_reference_summary=statistics(tail),
        note='Tail summary is descriptive; only stable_reports meet the full online pose-window criterion. Cached/duplicate load snapshots and >30 ms endpoint skew are excluded.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path,nargs='+')
    parser.add_argument('--config',type=Path,default=Path(__file__).resolve().parent.parent/'config/slung_payload.yaml')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    config=yaml.safe_load(args.config.read_text())['slung_payload']
    results=[analyze(path,config,args.config) for path in args.csv]
    text=json.dumps(results,ensure_ascii=False,indent=2,
                    default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item())+'\n'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(text)
    for result in results:
        print(Path(result['source']).name,'stable_reports=',len(result['stable_reports']))
        for report in result['stable_reports']:
            print(' h=%.2f median=%s p10-p90 spread(mm)=%s' %
                  (report['h_s'],np.round(report['median_m'],5),np.round((report['p90_m']-report['p10_m'])*1000,2)))
        print(' tail:',result['last_10s_stationary_reference_summary'])


if __name__=='__main__':
    main()
