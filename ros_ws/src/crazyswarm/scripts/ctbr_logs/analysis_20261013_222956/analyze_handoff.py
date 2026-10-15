"""Offline diagnosis of logged handoff rates; this script never sends commands."""
import csv
import json
from pathlib import Path
from collections import Counter
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
scripts = root.parents[1]
source = scripts / 'ctbr_logs/multi_ctbr_20261013_222956.csv'
config = yaml.safe_load((scripts.parent / 'config/ctbr_vehicle.yaml').read_text())
with source.open(newline='') as stream:
    rows = list(csv.DictReader(stream))

def number(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return float('nan')

vehicles = {v: [r for r in rows if r['vehicle_id'] == v] for v in '345'}
ramp = {v: [r for r in rr if r['flight_phase'] == 'tension_ramp'] for v, rr in vehicles.items()}
t0 = number(ramp['3'][0], 'control_time_s')
summary = dict(source=source.name, handoff_time_s=t0, vehicles={})
fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
colors = ['#2563eb', '#ed8b12', '#169a70']
for v, color in zip('345', colors):
    rr = ramp[v]
    cfg = config['ctbr_controller_cf' + v]
    t = np.array([number(r, 'control_time_s') - t0 for r in rr])
    blend = np.array([number(r, 'transport_blend') for r in rr])
    attitude_error = np.array([[number(r, 'attitude_error_' + a) for a in 'xyz'] for r in rr])
    rate = np.array([[number(r, 'body_rate_' + a) for a in 'xyz'] for r in rr])
    command = np.array([[number(r, 'command_rate_' + a) for a in 'xyz'] for r in rr])
    p_term = -((1 - blend[:, None]) * np.asarray(cfg['independent_attitude_gain'])
               + blend[:, None] * np.asarray(cfg['transport_attitude_gain'])) * attitude_error
    d_term = -blend[:, None] * np.asarray(cfg['transport_rate_gain']) * rate
    reconstructed = np.clip(p_term + d_term, -np.asarray(cfg['max_body_rate_radps']),
                             np.asarray(cfg['max_body_rate_radps']))
    clipped = np.any(np.abs(command[:, :2]) >= 2.99999, axis=1)
    index = int(np.flatnonzero(clipped)[0])
    first = rr[0]
    prior = [r for r in vehicles[v] if number(r, 'control_time_s') < t0][-1]
    summary['vehicles']['CF' + v] = dict(
        reconstruction_max_abs_error=float(np.max(np.abs(reconstructed-command))),
        first_rate_saturation_time_s=number(rr[index], 'control_time_s'),
        first_rate_saturation_elapsed_s=float(t[index]),
        blend_at_first_saturation=float(blend[index]),
        max_abs_measured_roll_pitch_rate_rps=float(np.max(np.abs(rate[:, :2]))),
        saturation_fraction_after_1p2s=float(np.mean(clipped[t >= 1.2])),
        command_thrust_before_handoff_N=number(prior, 'command_thrust_newton'),
        command_thrust_first_handoff_N=number(first, 'command_thrust_newton'),
        handoff_force_z_before_N=number(prior, 'desired_force_z'),
        handoff_force_z_after_N=number(first, 'desired_force_z'),
        ekf_status_counts=dict(Counter(r['ekf_status'] for r in rr)),
        invalid_count=sum(bool(r['invalid_reason']) for r in rr),
    )
    axes[0, 1].plot(t, [number(r, 'command_thrust_newton') for r in rr], color=color, label='CF' + v)
    axes[1, 1].plot(t, np.linalg.norm(command[:, :2], axis=1), color=color, label='CF' + v)
    if v == '5':
        axes[1, 0].plot(t, command[:, 1], color='#2563eb', label='CF5 pitch-rate command')
        axes[1, 0].plot(t, rate[:, 1], color='#ed8b12', label='CF5 measured pitch rate')

rr = ramp['3']
t = np.array([number(r, 'control_time_s') - t0 for r in rr])
axes[0, 0].plot(t, [number(r, 'payload_position_z') for r in rr], label='Recorded payload center z', color='#2563eb')
axes[0, 0].plot(t, [number(r, 'payload_target_z') for r in rr], '--', label='Payload target z', color='#d83235')
axes[0, 0].plot(t, [number(r, 'payload_ground_clearance_m') for r in rr], ':', label='Stale clearance diagnostic', color='#777777')
axes[0, 0].set(title='Payload falls before the 5 s handoff finishes', ylabel='Height (m)')
axes[0, 1].set(title='No initial thrust step; commands remain nonzero', ylabel='Collective-thrust command (N)')
axes[1, 0].axhline(3, color='gray', linestyle=':')
axes[1, 0].axhline(-3, color='gray', linestyle=':')
axes[1, 0].set(title='Rates become unstable at only 6–8% transport blend', ylabel='Pitch rate (rad/s)', xlim=(0.5, 2.0))
axes[1, 1].axhline(3*np.sqrt(2), color='gray', linestyle=':', label='Both roll/pitch commands saturated')
axes[1, 1].set(title='Roll and pitch remain saturated after 1.2 s', ylabel='Commanded roll/pitch rate norm (rad/s)')
for ax in axes.flat:
    ax.grid(alpha=.2)
    ax.legend(fontsize=8)
    ax.set_xlabel('Seconds after handoff')
    ax.axvspan(1.03, 1.16, color='#e9b949', alpha=.18)
fig.suptitle('multi_ctbr_20261013_222956 — logged handoff diagnosis', fontsize=14)
fig.savefig(root / 'handoff_diagnostics.png', dpi=160)

summary['payload'] = dict(
    target_z_m=number(rr[0], 'payload_target_z'),
    first_z_m=number(rr[0], 'payload_position_z'),
    lowest_center_z_m=min(number(r, 'payload_position_z') for r in rr),
    first_center_below_0p1m_time_s=number(next(r for r in rr if number(r, 'payload_position_z') < .1), 'control_time_s'),
    ground_clearance_diagnostic_unique_values=sorted(set(number(r, 'payload_ground_clearance_m') for r in rr)),
    measured_angular_rate_norm_at_handoff=float(np.linalg.norm([number(rr[0], 'payload_body_rate_' + a) for a in 'xyz'])),
    attitude_error_norm_at_handoff=float(np.linalg.norm([number(rr[0], 'payload_attitude_error_' + a) for a in 'xyz'])),
)
(root / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
print(json.dumps(summary, indent=2, ensure_ascii=False))
