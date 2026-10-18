#!/usr/bin/env python3
"""Run frozen lightweight/long-rope candidates without ROS or radio."""
import argparse
import json
import os
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
    cases = json.loads((HERE / "cases.json").read_text())["cases"]
    for case in cases[args.worker::3]:
        output = Path(case["output"])
        start = time.monotonic()
        with (output / "console.txt").open("w") as stream:
            process = subprocess.run([
                sys.executable, str(SCRIPTS / "simulate_slung_handoff.py"),
                "--candidate", case["candidate"], "--output", case["output"],
            ], cwd=SCRIPTS, stdout=stream, stderr=subprocess.STDOUT)
        report = dict(name=case["name"], returncode=process.returncode,
                      wall_s=time.monotonic() - start)
        path = output / "summary.json"
        if path.exists():
            data = json.loads(path.read_text())
            report.update({k: data[k] for k in (
                "success", "failure", "observed_post_handoff_s", "validation_completed",
                "all_vehicles_zero", "minimum_vehicle_distance_m", "maximum_body_rate_rps",
            )})
        (output / "run_status.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
