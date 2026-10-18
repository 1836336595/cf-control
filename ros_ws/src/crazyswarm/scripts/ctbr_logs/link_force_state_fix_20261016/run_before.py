#!/usr/bin/env python3
"""Run pre-fix control arithmetic with a diagnostics-only compatibility adapter.

Snapshot files stay immutable. The adapter adds flags expected by the current
logger; it does not replace any legacy force output, limiter state, or command.
"""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parents[1]
sys.path.insert(0, str(SCRIPTS))


def load(name, source):
    spec = importlib.util.spec_from_file_location(name, source)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def add_legacy_diagnostics(module):
    """Add two output flags without changing legacy computations or history."""
    original_compute = module.SlungLoadController.compute
    def compute(self, *args, **kwargs):
        result = original_compute(self, *args, **kwargs)
        if result is None:
            return result
        for output in result["vehicles"]:
            raw_force = np.asarray(output["raw_link_force"], dtype=float)
            norm = float(np.linalg.norm(raw_force))
            limit = float(self.config.link_force_correction_limit_n)
            bounded = raw_force * min(1.0, limit / max(norm, 1e-12))
            output["link_force_magnitude_limited"] = norm > limit
            output["link_force_rate_limited"] = bool(np.linalg.norm(output["link_force"] - bounded) > 1e-12)
        return result
    module.SlungLoadController.compute = compute
    return original_compute


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=HERE / "before_nominal")
    parser.add_argument("--candidate", type=Path, help="Optional full or partial parameter overlay")
    parser.add_argument("--current-replay", action="store_true", help="Use current schedule-capable replay with immutable old controller")
    args = parser.parse_args()
    started = time.perf_counter()
    legacy = load("slung_load_controller", HERE / "slung_load_controller_before.py")
    add_legacy_diagnostics(legacy)
    replay_source = SCRIPTS / "simulate_slung_handoff.py" if args.current_replay else HERE / "simulate_slung_handoff_before.py"
    replay = load("simulate_slung_handoff_before", replay_source)
    # Preserve the production script's path-based lookup of the Udaan checkout.
    replay.__file__ = str(SCRIPTS / "simulate_slung_handoff.py")
    overrides = yaml.safe_load(args.candidate.read_text()) if args.candidate else {}
    payload = overrides.setdefault("slung_payload", {})
    payload.update(flight_validation_enabled=False, post_handoff_s=60.0)
    summary = replay.run_simulation(args.output, overrides)
    elapsed = time.perf_counter() - started
    metadata = {"elapsed_wall_s": elapsed, "python": sys.executable,
                "diagnostic_compatibility_adapter": "Add magnitude/rate-limited output flags only; legacy forces, commands and limiter history unchanged.",
                "controller_snapshot_sha256": hashlib.sha256((HERE / "slung_load_controller_before.py").read_bytes()).hexdigest(),
                "replay_source": str(replay_source),
                "replay_snapshot_sha256": hashlib.sha256(replay_source.read_bytes()).hexdigest()}
    (args.output / "before_run_metadata.json").write_text(json.dumps(metadata, indent=2))
    print(json.dumps(dict(summary=summary, metadata=metadata), indent=2))
    return 0 if summary["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
