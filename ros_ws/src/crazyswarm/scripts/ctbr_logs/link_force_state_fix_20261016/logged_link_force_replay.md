# Approximate logged force-path replay

Source: `multi_ctbr_20261016_213804.csv`, the first 10 seconds after the 48.283755 s handoff. The current production `_limit_link_force` runs twice with identical reconstructed raw correction, kinematic force and clipped 1–50 ms control intervals. The old path retains `limited + kinematic`; the corrected path retains `limited`. This does not change recorded inputs or simulate the resulting flight.

| Window | Vehicle samples | Outputs changed by saved state | Old-only slew limiting | Corrected genuine slew limiting | Difference RMS / max (N) | Old correction slew violations | Corrected slew violations |
|---|---:|---:|---:|---:|---:|---:|---:|
| all_transport | 2100 | 100 | 100 | 0 | 0.016644 / 0.170165 | 9 | 0 |
| first_2_s | 420 | 22 | 22 | 0 | 0.019734 / 0.159845 | 5 | 0 |
| last_3_s | 630 | 60 | 60 | 0 | 0.023921 / 0.170165 | 4 | 0 |

The corrected path has 0 magnitude-limited and 0 slew-limited samples. The old history changes 100 of 2100 vehicle samples; 55 changed samples have intervals below 10 ms. The largest old correction slew is 38.864663 N/s versus 2.492049 N/s for the corrected path (configured cap 20.000 N/s).

Reconstruction limitations: desired payload rotation is assumed to be identity; the principal SO(3) rotation is recovered from logged attitude error. Per-vehicle payload/vehicle rows are asynchronous snapshots, and physical masses, geometry and limiter coefficients come from the current parameter files. The resulting anchor geometry residual is RMS 0.000534 m, maximum 0.002264 m. This fit supports an approximate force-path comparison; it does not recover exact controller inputs or establish the corrected closed-loop behavior.

Reproduce with `python3 replay_logged_link_forces.py`. CSV includes every raw/kinematic/old/fixed vector and event flag; JSON includes per-window and per-vehicle statistics. The PNG compares force norms.
