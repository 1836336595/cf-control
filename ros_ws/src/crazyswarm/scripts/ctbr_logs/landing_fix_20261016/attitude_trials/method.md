# Payload attitude bandwidth and noise conditioning comparison

These runs use the existing real-controller replay and the Gräfe 2026 four-motor
plant. Candidate YAML files are complete parameter snapshots. Every run uses a
fresh Python process; `run_metadata.json` records source hashes before/after.
All hashes stayed unchanged during the reported trials.

Normal planned validation landing is disabled. Each successful trial observes
60.014 s after handoff. The physical plant, aircraft/payload masses, host gains
`transport_attitude_gain=[12,12,6]` and `transport_rate_gain=[.25,.25,.125]`, and
motor model are identical within each comparison. These are hover tests; root
task separately verifies physical landing.

The noise stress injects independent rotation-vector perturbations into measured
payload orientation before the real observer callback, at the 100 Hz pose stream:
sigma `[.005,.004,.004] rad`, seed `20261016`. Physical rotation is unchanged by
the measurement perturbation. This is a sensitivity hypothesis informed by
high-frequency observed-rate residuals, **not a measurement of NOKOV accuracy**.
The real CSV lacks raw payload quaternion history and cannot separate physical
swing from orientation noise. A single deterministic seed does not constitute
an exhaustive uncertainty study.

## Bandwidth trials

The nominal bandwidth trials and `noise_bw*` use pose/command delays 10/5 ms.
Bandwidths are `[2,2,.45]`, `[1.2,1.2,.45]` and `[1,1,.45] Hz` with damping ratio
.9. Reducing bandwidth lowers noise-induced rate/thrust fluctuations, but also
weakens stiffness and increases steady payload pitch bias: approximately
5.08°, 8.55° and 9.93° respectively. Passing position/rate checks does not
establish small attitude tracking error.

## Noise conditioning trials with larger delays

All four conditioning cases use bandwidth `[2,2,.45] Hz` and pose/command delays
20/10 ms, preserving static attitude stiffness:

| Candidate | Observer angular-rate gain | Attitude damping ratio | Result |
| --- | ---: | ---: | --- |
| `baseline_noise_delay` | .30 | .90 | Full 60 s passed |
| `observer_rate_010` | .10 | .90 | Distance failure after 7.80 s |
| `observer_rate_015` | .15 | .90 | Distance failure after 9.87 s |
| `damping_06` | .30 | .60 | Full 60 s passed |

The lower observer gains add delay to the estimated angular-rate response and
did not improve this closed-loop test. They should not replace .30 based only
on apparent noise suppression.

The `.60` damping candidate reduces final-20-s aircraft angular-rate vector RMS
by 39–42% and thrust-command standard deviation by 36–40% relative to the
matched `.90` baseline. Mean payload pitch remains 5.07° versus 5.15°; minimum
spacing remains approximately .3855 m versus .3854 m. Both complete cases have
zero commanded-rate saturation. This supports `.60` as a candidate for further
landing/disturbance verification; it does not establish real-flight readiness.

`conditioning_comparison_metrics.json` provides all numerical metrics. For failed
runs, fields named `final_20s_*` cover only the available post-handoff interval;
`actual_metrics_window_s` and `steady_window_complete_20s` identify that limitation.
RMS is `sqrt(mean(p²+q²+r²))` from physical simulated aircraft rates.
Thrust variability is standard deviation around the time-window mean, reported
separately for host-command and physical motor force.

The charts use the same vertical scale across candidates and show the final
5 s of completed 60 s runs. Regenerate them with:

```bash
python plot_noise_comparison.py
python plot_noise_comparison.py --conditioning
```
