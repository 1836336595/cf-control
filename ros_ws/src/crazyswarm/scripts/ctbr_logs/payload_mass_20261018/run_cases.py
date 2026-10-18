#!/usr/bin/env python3
"""Run frozen experiment cases in fresh Udaan processes; keep all outputs."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", type=int, choices=(0, 1, 2), required=True)
    args = parser.parse_args()
    manifest = json.loads((HERE / "cases.json").read_text())
    cases = manifest["cases"][args.worker::3]
    for case in cases:
        started = time.monotonic()
        output = Path(case["output"])
        with (output / "console.txt").open("w") as stream:
            process = subprocess.run([
                sys.executable, str(SCRIPTS / "simulate_slung_handoff.py"),
                "--candidate", case["candidate"], "--output", str(output),
            ], cwd=SCRIPTS, stdout=stream, stderr=subprocess.STDOUT)
        report = dict(name=case["name"], returncode=process.returncode,
                      wall_s=time.monotonic() - started)
        summary = output / "summary.json"
        if summary.exists():
            result = json.loads(summary.read_text())
            report.update({name: result[name] for name in (
                "success", "failure", "observed_post_handoff_s", "maximum_body_rate_rps",
                "minimum_vehicle_distance_m", "final_payload_position_error_m")})
        (output / "run_status.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
        if process.returncode and not summary.exists():
            raise RuntimeError("Simulation crashed; inspect " + str(output / "console.txt"))


if __name__ == "__main__":
    main()
