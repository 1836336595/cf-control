"""Rebuild the comparison figure/table from saved 70 Hz simulation results."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
cases = ["baseline_10ms", "candidate_10ms", "candidate_20ms_tilt",
         "candidate_slow_rate", "candidate_fast_rate"]
labels = ["Live gains, 10/5 ms delay", "Candidate, 10/5 ms delay",
          "Candidate, 20/10 ms + 5 deg", "Candidate, slow inner loop",
          "Candidate, fast inner loop"]
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
metrics = []
for case, label in zip(cases, labels):
    result = json.loads((root / case / "summary.json").read_text())
    with (root / case / "simulation_trace.csv").open() as stream:
        rows = [r for r in csv.DictReader(stream) if r["vehicle_id"] == "5"]
    times = np.array([float(r["control_time_s"]) for r in rows])
    dt = np.diff(times)
    assert np.allclose(dt, 1/70, rtol=0, atol=1e-10), case
    result["verified_rate_hz"] = float(1/np.mean(dt))
    metrics.append(dict(case=case, **result))
    elapsed = np.array([float(r["handoff_elapsed_s"]) for r in rows])
    mask = elapsed >= 0
    elapsed = elapsed[mask]
    arrays = [np.array([float(r[field]) for r in rows])[mask]
              for field in ("command_rate_x", "body_rate_x", "payload_clearance_m")]
    for ax, values in zip(axes, arrays):
        ax.plot(elapsed, values, label=label, linewidth=1.0, alpha=0.85)
axes[0].set_ylabel("CF5 roll command (rad/s)")
axes[1].set_ylabel("CF5 roll rate (rad/s)")
axes[2].set_ylabel("Payload clearance (m)")
axes[2].set_xlabel("Time after handoff (s)")
axes[0].legend(fontsize=8, loc="upper right", ncol=2)
for ax in axes:
    ax.grid(True, alpha=0.25)
fig.suptitle("70 Hz external control, 100 Hz sampled pose, delayed feedback/commands")
fig.tight_layout()
fig.savefig(root / "comparison.png", dpi=180)
plt.close(fig)
(root / "comparison.json").write_text(json.dumps(metrics, indent=2) + "\n")
print("| Case | Verified Hz | Rate saturation | Min clearance (m) | Final position error (m) | Result |")
print("| --- | ---: | ---: | ---: | ---: | --- |")
for r in metrics:
    print(f"| {r['case']} | {r['verified_rate_hz']:.6f} | "
          f"{r['rate_saturation_fraction']*100:.2f}% | {r['minimum_clearance_m']:.3f} | "
          f"{r['final_payload_position_error_m']:.3f} | {r['failure'] or 'passed'} |")
