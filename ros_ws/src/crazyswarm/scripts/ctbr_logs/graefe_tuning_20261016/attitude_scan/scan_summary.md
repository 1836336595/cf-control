# Graefe plant: outer attitude-gain scan

All runs use the real slung-load host controller with the Graefe 2026 rotor plant, sampled 50 Hz filtered EKF feedback, 20 ms EKF delay, 100 Hz pose samples with 10 ms delay, 5 ms command delay, and the same safety thresholds. The outer transport rate gain is fixed at `[2, 2, 1]` for R3/R9/R12/R18; R9_D025 uses `[0.5, 0.5, 0.25]`. Exact overlays and each run's `input_parameters.yaml` and `summary.json` are in their run directories.

| Run | Attitude gain | Rate gain | Post-handoff observed | Outcome | Min vehicle spacing | Min load clearance | Max body rate | Max tilt | Rate saturation |
|---|---:|---:|---:|---|---:|---:|---:|---:|---:|
| R3 | [3, 3, 1.5] | [2, 2, 1] | 7.229 s | fail: distance | 0.296 m | 0.325 m | 4.619 rad/s | 8.889° | 7.5% |
| R9 | [9, 9, 4.5] | [2, 2, 1] | 5.900 s | fail: distance | 0.298 m | 0.275 m | 4.792 rad/s | 11.178° | 10.2% |
| R12 | [12, 12, 6.0] | [2, 2, 1] | 5.557 s | fail: distance | 0.288 m | 0.265 m | 4.953 rad/s | 11.053° | 12.9% |
| R18 | [18, 18, 9.0] | [2, 2, 1] | 5.029 s | fail: distance | 0.293 m | 0.265 m | 4.760 rad/s | 11.186° | 13.1% |
| R9_D025 | [9, 9, 4.5] | [0.5, 0.5, 0.25] | 60.014 s | pass | 0.366 m | 0.336 m | 0.018 rad/s | 6.875° | 0.0% |

## Interpretation

At fixed rate gain `[2, 2, 1]`, all four attitude settings reached the distance safety guard before the required 60 s observation. The guard time worsened from 7.23 s after handoff at R3 to 5.03 s at R18, while peak body rate reached 4.6–5.0 rad/s and post-handoff rate saturation rose from 7.5% to about 13%. R3 first lost geometric spacing while rates were still small: minimum pair separation fell from 0.434 m at 29.5 s to 0.342 m at 34 s and 0.316 m at 35 s. For R9, spacing remained about 0.40–0.43 m through 34 s, then rates climbed from 0.05 rad/s at 34 s to 2.88 rad/s by 35.2 s, followed by distance loss. Thus the fixed `[2, 2, 1]` rate-feedback setting masks a rate-instability region at R9 and above, while R3 also has insufficient geometric margin before that late oscillation.

R9 with rate gain reduced to `[0.5, 0.5, 0.25]` completed 60.014 s after handoff. It ended at payload height 0.600001 m with 0.000021 m target-position error, minimum vehicle separation 0.366 m, minimum payload clearance 0.336 m, maximum body-rate 0.0178 rad/s, 6.87° max tilt, and no rate saturation. This is a simulation candidate for further robustness testing; the single nominal run does not establish robustness to greater delays, initial tilt, motor-force reduction, or hardware differences.

Failed runs stopped at the safety guard and therefore do not satisfy the 60 s observation requirement. No live controller/configuration was changed.
