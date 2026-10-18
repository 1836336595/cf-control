# Offline bounded differential hold-integral comparison

All cases retain the frozen production controller, common load support trim, actual physical rotor model, measured control timing, sampled EKF, delays, zero load rotational damping and rotation noise from report.md. Production files were not edited. The experimental control is implemented only by wrapping the manager's _transport_overrides in the offline runner.

For each aircraft e_i = measured position minus actual hold reference. The probe removes the fleet mean independently on XYZ and integrates db_i/dt = -Ki (e_i - mean(e)). A common scale applied to the complete fleet enforces per-aircraft norm caps of .03 N and .002 N/s while retaining sum(b_i)=sum(db_i/dt)=0. The force and its actual bounded derivative are added to desired_force_override and desired_force_dot_override. The existing transport blend remains responsible for blending them into the applied aircraft force.

Learning starts only after max(attitude reference duration, tension ramp + load lift duration) = max(12,5+12)=17 s. The wrapper checks valid position/source-derivative flags, prior command force/tilt/rate/collective limits, and landing/fault state. Those failure paths were not injected in this study; production adoption requires explicit reset/landing/invalid-EKF handling and behavioral tests. Bias trace and command-source manifests are saved per integral case.

| Case | Mean true R/P 50–60 s (deg) | R/P std (deg) | Differential hold mean-error RMS (mm) | Final load XYZ error norm (m) | Outcome |
|---|---:|---:|---:|---:|---|
| nominal | -1.122, -3.178 | 0.086, 0.168 | 37.55 | 0.0434 | pass |
| nominal_diffI002 | -0.379, -0.118 | 0.067, 0.054 | 3.02 | 0.0611 | pass |
| unequal_from_start | -6.476, +20.631 | 0.051, 0.058 | 40.52 | 0.1609 | hover_position_error |
| unequal_diffI002 | -1.106, +2.970 | 0.148, 0.369 | 4.91 | 0.1409 | hover_position_error |
| unequal_diffI005 | -0.629, +1.253 | 0.060, 0.044 | 2.43 | 0.1405 | hover_position_error |
| unequal_bw2_noI | -5.147, +14.820 | 0.067, 0.050 | 38.04 | 0.1457 | hover_position_error |

The differential RMS above measures shape error: remove the fleet mean from the three per-aircraft mean XYZ hold errors, square all nine components, average and take the square root. It deliberately excludes the common translation error.

| Integral case | First learning after handoff (s) | Max bias norm (N) | Max bias rate (N/s) | Max norm of sum(b) (N) |
|---|---:|---:|---:|---:|
| nominal_diffI002 | 17.01394 | 0.022723 | 0.001593 | 4.48e-17 |
| unequal_diffI002 | 17.01438 | 0.028572 | 0.001667 | 1.05e-16 |
| unequal_diffI005 | 17.01438 | 0.030000 | 0.002000 | 1.61e-16 |

No integral case changes the production common load support trim or injects net fleet force. Thus a reduction in shape/attitude error does not establish correction of common thrust-mapping error or common load height/position. The 60 s success/failure result and final load position norm are reported separately.

The 2 Hz case changes only attitude_bandwidth_hz from [1,1,.45] to [2,2,.45] under the same [1,1.05,1.10] rotor-efficiency mismatch. It is a diagnostic comparison in one model/noise configuration, not a recommendation to deploy 2 Hz gains.

The current 20 s transport window permits only about 3 s of learning after the 17 s reference-completion gate. In the unequal-efficiency hypothesis test, true pitch at 20 s is 17.77 deg with Ki=.02 and 17.18 deg with Ki=.05, versus 20.36 deg without integral. Neither integral therefore establishes leveling within a 20 s flight. Ki=.02 first holds both |roll| and |pitch| below 5 deg for 2 s starting at 43.16 s after handoff; Ki=.05 does so from 30.67 s and below 2 deg from 38.21 s. See integral_settling_time.json for sampled trajectories and the precise criterion.
