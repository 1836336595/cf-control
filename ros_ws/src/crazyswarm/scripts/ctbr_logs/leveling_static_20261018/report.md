# Offline physical leveling diagnostic

These are hypothesis tests, not identification of any real aircraft's motor coefficients. No production source was edited, and no physical state, pose bias, phase or completion flag was forced.

All runs use a frozen snapshot of the current reference cascade, 12 s consistent full R/cable reference, hold scales 2.0 ramp and 1.5 steady, load attitude bandwidth [1,1,.45] Hz and damping ratio .9. The reference is level from 12 s onward. The real control interval sequence, 50 Hz sampled/filtered EKF, 20 ms pose/EKF delay, 10 ms command delay, rotation noise [.005,.004,.004] rad and zero physical load rotational damping are retained. All runs cover the complete physical 60 s post-handoff window.

The disturbance is the native per-vehicle motor_thrust_efficiency. It multiplies all four rotor lift forces and resulting roll/pitch moments, while retaining rotor dynamics, reactive yaw torque, rope mechanics and ground contacts. The configured bridge raw scales remain [1.0213949773,1.0174765859,1.0266666884] in all runs; native efficiency 1 is the model baseline, not a claim that actual thrust equals commanded thrust.

Physical assumptions: vehicle masses [43.4,44.0,42.4] g for CF3/4/5, centered uniform 23 g box [.08,.06,.05] m, identical true/nominal cables .85 m, no rope calibration. Top attachment points are [+.04,0,+.025],[-.04,+.03,+.025],[-.04,-.03,+.025] m relative to box center. ROS world is z-up and R=Rz(yaw) Ry(pitch) Rx(roll). Trace vehicle positions are true body centers. Vehicle pose feedback uses those same body-center coordinates with the configured sampling delay. The replay starts at airborne TAKEUP anchors; it is not a replay of aircraft ground takeoff. Reported hold errors are synchronized true body-center XYZ minus each vehicle's actual position_hold_position, not ordinary target XYZ.

| Case | 50–60 s true mean R/P/Y (deg) | R/P std (deg) | Final load position error (m) | Outcome |
|---|---:|---:|---:|---|
| Native efficiency [1, 1, 1] | -1.122, -3.178, +0.086 | 0.086, 0.168 | 0.0434 | pass |
| [1, 1.05, 1.10] from start | -6.476, +20.631, -0.521 | 0.051, 0.058 | 0.1609 | hover_position_error |
| [1, 1.05, 1.10] at handoff | -6.491, +20.550, -0.517 | 0.050, 0.058 | 0.1608 | hover_position_error |

| Case | Vehicle | 50–60 s true XYZ minus hold reference (m) | Command / actual thrust mean (N) |
|---|---|---:|---:|
| Native efficiency [1, 1, 1] | CF3 | -0.0846, -0.0007, +0.0669 | 0.5259 / 0.5450 |
| Native efficiency [1, 1, 1] | CF4 | +0.0322, -0.0339, +0.0638 | 0.4643 / 0.4836 |
| Native efficiency [1, 1, 1] | CF5 | +0.0404, +0.0418, +0.0713 | 0.4443 / 0.4722 |
| [1, 1.05, 1.10] from start | CF3 | -0.0671, -0.0037, +0.1042 | 0.4797 / 0.5016 |
| [1, 1.05, 1.10] from start | CF4 | +0.0205, -0.0321, +0.1703 | 0.4504 / 0.4938 |
| [1, 1.05, 1.10] from start | CF5 | +0.0394, +0.0528, +0.1973 | 0.4316 / 0.5057 |
| [1, 1.05, 1.10] at handoff | CF3 | -0.0672, -0.0037, +0.1040 | 0.4798 / 0.5018 |
| [1, 1.05, 1.10] at handoff | CF4 | +0.0205, -0.0320, +0.1704 | 0.4503 / 0.4937 |
| [1, 1.05, 1.10] at handoff | CF5 | +0.0395, +0.0528, +0.1973 | 0.4316 / 0.5056 |

A [0,+5%,+10%] actual rotor lift mismatch produces a steady ~20.6 degree pitch error after a level full-attitude reference has completed. Applying it from simulation start at airborne TAKEUP anchors or exactly at handoff gives the same settled result, isolating the mechanism from the prelift history. There is no rate saturation; all original safety/quality checks run. The two biased cases are correctly marked hover_position_error at the end (about .161 m), rather than treated as successful flights.

This supports testing bounded per-vehicle hold integral/disturbance compensation and improving the per-aircraft thrust mapping. It does not prove that the real log's tilt is caused by thrust mapping: load rigid-body frame tilt, center-of-mass offset, rope/attachment geometry and tension identification remain distinct hypotheses.

Artifacts: source_snapshot/ and config_snapshot/ contain frozen controller/config dependencies; physics_source_snapshot/ preserves native rotor and MuJoCo implementations. Every case has candidate.json, candidate.yaml, run_metadata.json, input_parameters.yaml, summary.json, leveling_metrics.json, controller CSV and truth trace CSV. Source hashes are recorded at the root.
