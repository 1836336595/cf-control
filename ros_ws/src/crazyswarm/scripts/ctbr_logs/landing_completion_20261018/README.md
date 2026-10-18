# Offline aircraft landing completion reproduction

All artifacts in this directory are offline; no ROS, radio, or hardware was used.
`frozen_sources/` and `frozen_parameters.yaml` preserve the pre-fix controller,
trajectory, and parameters. `frozen_source_hashes.json` records their SHA-256s.
No production file is edited by either analysis runner.

Conditions are a 23 g payload, three 0.850 m cables, `reference_cascade`, position
hold gain 1.5, a fixed 70 Hz host callback, Mellinger + Graefe motor dynamics,
100 Hz mocap delayed 10 ms, 50 Hz sampled EKF delayed 20 ms and host-filtered,
5 ms command delay, and exactly 10 s transport before planned landing. As in
the existing replay, the physical simulation starts at airborne TAKEUP anchors;
the independent ground takeoff is outside this reproduction.

The complete tested exit is physical payload lowering, measured stable payload
touchdown, the full 2 s unload, aircraft descent, and completion only after all
three command zero and remain within 3 cm of physical ground center with low
vertical velocity for 0.5 s. There is no state, phase, contact, or completion
clamping. The existing replay's `set_validation_ground_reference` overwrites
only trajectory ground reference to the physical model body center, 0.007 m;
it does not overwrite actual physics state.

Baseline results:

| Case | Final physical aircraft z [m] | Outcome |
| --- | --- | --- |
| Nominal | 0.006998, 0.006998, 0.006998 | All zero, complete at 50.014 s |
| +5% force after unload | 0.128151, 0.126203, 0.131296 | Sustained hover, incomplete at 90 s |
| +7.5% force after unload | 0.184515, 0.182742, 0.187376 | Sustained hover, incomplete at 90 s |
| +10% force after unload | 0.261258, 0.258775, 0.263030 | Sustained hover, incomplete at 90 s |
| +10% force from start | 0.993322, 0.963658, 0.966482 | Payload never contacts ground; touchdown timeout at 46.214 s |

The post-unload disturbances use the native actuator's `efficiency` coefficient
to scale each rotor's physical force, after the real controller has confirmed
contact and completed unloading. For these runs, payload contact is confirmed
at 40.842857 s and aircraft descent begins at 42.842857 s. This is a controlled
disturbance that isolates the reported post-release symptom. It does not claim
that physical calibration changed suddenly in the reported flight. The separate
constant-gain run shows that an error present during the whole flight can first
prevent payload touchdown, and must not be described as a successful unloading
reproduction.

Why nominal simulation previously passed: it combines the same configured
vehicle masses and an unperturbed thrust mapping with a corrected, exactly known
ground reference. That validates the nominal exit sequence but misses calibration
residuals and ground-origin errors. During aircraft descent the independent
vertical P gain is only 4 /s², the vertical I gain is 0.8, and I is bounded to
0.20. Its maximum I correction is only 0.16 m/s². At +7.5% force the I state
hits 0.20 and steady height error remains about 0.18 m, with actual thrust equal
to vehicle weight. At +10% force the height error exceeds the 0.20 m integral
gate, so integration stops near 0.084–0.088 instead of reaching its limit. A
fixed ground-height reference with zero reference velocity therefore admits
these stable nonzero-height equilibria.

Current-source verification (controller SHA-256
`884a31cfc34c60476032af5e85f3f9aea1ab093895252271a0abd819a3b42a43`):

| Case | Final physical aircraft z [m] | Completion [s] | Peak powered descent [m/s] | Peak landing body rate [rad/s] |
| --- | --- | --- | --- | --- |
| Nominal | All 0.006998 | 50.000 | 0.25074 | 0.77494 |
| +7.5% after unload | All 0.006998 | 55.271 | 0.25390 | 0.77791 |
| +10% after unload | All 0.006998 | 57.814 | 0.25230 | 0.77830 |

All three current cases have `validation_completed=true`, `all_vehicles_zero=true`,
ground-speed magnitude below 1e-10 m/s at completion, and peak landing tilt
3.4023 degrees. Contact confirmation and full unloading retain the exact baseline
times above. Current sources and frozen sources have identical trajectory and
replay hashes; only the controller differs for these runs. Full input parameters
including the new vertical-integral limits are recorded per run.

The peak powered descent is calculated from consecutive physical truth-height
samples while both endpoint thrust commands are positive. The existing 0.08 m
landing-height tolerance allows commands to turn zero while the aircraft is still
a few centimeters above the floor. The remaining fall is included in the physical
simulation: its peak downward speed is 0.918 m/s for baseline nominal, 0.646 m/s
for current nominal, and 1.076/1.070 m/s for current +7.5/+10% cases. Completion
requires actual subsequent settled ground contact. These checks demonstrate
landing completion and bounded powered descent, and do not establish gentle
touchdown under the existing early zero-output gate.

Reproduce the observed-height baseline from the scripts directory:

```bash
/home/csy/miniconda3/envs/udaan/bin/python ctbr_logs/landing_completion_20261018/run_landing_mapping.py --source frozen --gain 1.075 --gain-onset vehicle_descent --time-limit 90 --output ctbr_logs/landing_completion_20261018/baseline_release_gain1075
```

Run exactly the same parameters and physical disturbance using current sources:

```bash
/home/csy/miniconda3/envs/udaan/bin/python ctbr_logs/landing_completion_20261018/run_landing_mapping.py --source current --gain 1.075 --gain-onset vehicle_descent --time-limit 90 --output ctbr_logs/landing_completion_20261018/current_release_gain1075
```

Use `--gain 1.0` for nominal, `1.05` for +5%, or `1.1` for +10%. Omit
`--gain-onset vehicle_descent` to apply the motor error from the start. A
successful runner invocation means artifacts were written; actual flight
completion is explicitly reported as `success` and `validation_completed` in
each `summary.json` and `landing_metrics.json`.

Rebuild `comparison.json` and `landing_mapping_comparison.png` from completed
runs without rerunning physics:

```bash
/home/csy/miniconda3/envs/udaan/bin/python ctbr_logs/landing_completion_20261018/analyze_landing_mapping.py
```
