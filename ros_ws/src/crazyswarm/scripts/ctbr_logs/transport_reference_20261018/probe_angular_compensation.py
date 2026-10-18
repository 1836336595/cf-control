#!/usr/bin/env python3
"""Offline ablation of commanded payload angular-acceleration compensation."""

import argparse
import inspect
import json
from pathlib import Path
import sys
import textwrap

import yaml

SCRIPTS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCRIPTS))
import simulate_slung_handoff as replay
import slung_load_controller as load


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scale", type=float)
    args = parser.parse_args()
    assert 0 <= args.scale <= 1
    root = Path(__file__).resolve().parent
    output = root / ("probe_angular_scale_" + str(args.scale).replace(".", "p"))
    output.mkdir(exist_ok=True)
    source = textwrap.dedent(inspect.getsource(load.SlungLoadController.compute))
    original = "- r0 @ _hat(rho) @ omega0_dot_cmd"
    assert source.count(original) == 1
    source = source.replace(original, "- {} * r0 @ _hat(rho) @ omega0_dot_cmd".format(args.scale))
    namespace = dict(load.__dict__)
    exec(compile(source, str(output / "compute_probe.py"), "exec"), namespace)
    load.SlungLoadController.compute = namespace["compute"]
    (output / "compute_probe.py").write_text(source)
    params = yaml.safe_load((root / "consistent_observed_q_damping000/candidate.yaml").read_text())
    (output / "candidate.yaml").write_text(yaml.safe_dump(params, sort_keys=False))
    result = replay.run_simulation(output, params)
    print(json.dumps({name: result[name] for name in ("success", "failure",
        "observed_post_handoff_s", "minimum_vehicle_distance_m", "maximum_body_rate_rps")}))


if __name__ == "__main__":
    main()
