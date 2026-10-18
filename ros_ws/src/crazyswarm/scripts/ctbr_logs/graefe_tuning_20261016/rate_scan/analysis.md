# Graefe 2026 transport rate scan

Six independent fresh-process runs used the frozen baseline in `baseline_parameters.yaml`. Only host `transport_attitude_gain` and `transport_rate_gain` were varied. The 500 Hz firmware Mellinger gains, Graefe motor coefficients/dynamics, mass/inertia, thrust mapping, all safety limits, 0.60 m target, and 60 s post-handoff window were preserved. The aggregate script checked every saved input parameter difference and verified identical effective physical plants across all six runs.

| Candidate | Host attitude gain | Host rate gain | Full window (s) | Peak body rate (rad/s) | Min spacing (m) | Min clearance (m) | Peak tilt (deg) | Wall time (s) |
|---|---|---|---:|---:|---:|---:|---:|---:|
| r6_d0 | [6.0, 6.0, 3.0] | [0.0, 0.0, 0.0] | 60.0143 | 0.01776 | 0.333592 | 0.335084 | 6.511 | 39.65 |
| r6_d025 | [6.0, 6.0, 3.0] | [0.25, 0.25, 0.125] | 60.0143 | 0.01785 | 0.333613 | 0.335097 | 6.510 | 39.59 |
| r6_d05 | [6.0, 6.0, 3.0] | [0.5, 0.5, 0.25] | 60.0143 | 0.01794 | 0.333638 | 0.335112 | 6.508 | 39.85 |
| r6_d1 | [6.0, 6.0, 3.0] | [1.0, 1.0, 0.5] | 60.0143 | 0.01799 | 0.333684 | 0.335157 | 6.503 | 39.79 |
| r12_d025 | [12.0, 12.0, 6.0] | [0.25, 0.25, 0.125] | 60.0143 | 0.01982 | 0.385995 | 0.335783 | 7.112 | 42.05 |
| r12_d1 | [12.0, 12.0, 6.0] | [1.0, 1.0, 0.5] | 60.0143 | 0.02018 | 0.386017 | 0.335783 | 7.122 | 42.81 |

All six runs returned `success=true`, with zero rate saturation and no safety reason. They handed off at 29.442857 s and ended at 89.457143 s, covering 60.014286 s after handoff. Final payload position errors were below 0.000018 m.

The preferred candidate for further validation is `r12_d1`: host attitude `[12, 12, 6]`, host rate `[1, 1, 0.5]`. Its minimum spacing of 0.386017 m provides 0.086017 m above the unchanged 0.30 m limit. `r6_d1` changes only the host rate gains and also passes, with 0.333684 m minimum spacing. The candidate YAML is copied to `preferred_candidate.yaml`; no shared runtime configuration was edited.

These results cover the nominal initialized replay and its configured deterministic delays. They do not establish hardware stability or robustness to all model errors and initial disturbances. Each run contains `candidate.yaml`, complete `input_parameters.yaml`, `summary.json`, controller CSV, simulation trace, and `run_metadata.json`.

Reproduce one candidate with:

```bash
/home/csy/miniconda3/envs/udaan/bin/python run_one.py r12_d1_replay 12 1
```
