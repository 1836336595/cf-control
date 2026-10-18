#!/usr/bin/env python3
"""Compare cascaded position support without the delayed cable-direction loop."""

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
    parser.add_argument("mode", choices=("reference", "requested"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    output = root / ("probe_cascaded_" + args.mode)
    output.mkdir(exist_ok=True)
    source = textwrap.dedent(inspect.getsource(load.SlungLoadController.compute))
    original = """        ai = (
            desired_load_acceleration + gravity
            + r0 @ (_hat(omega0) @ (_hat(omega0) @ rho))
            - r0 @ _hat(rho) @ omega0_dot_cmd
        )"""
    replacement = """        ai = (
            ad + gravity
            + rd @ (_hat(omegad) @ (_hat(omegad) @ rho))
            - rd @ _hat(rho) @ omegadd
        )"""
    if args.mode == "reference":
        assert source.count(original) == 1
        source = source.replace(original, replacement)
        namespace = dict(load.__dict__)
        exec(compile(source, str(output / "compute_probe.py"), "exec"), namespace)
        load.SlungLoadController.compute = namespace["compute"]
    (output / "compute_probe.py").write_text(source)
    params = yaml.safe_load((root / "equilibrium_observed_q_damping000/candidate.yaml").read_text())
    params['slung_payload']['transport_link_gain_scale'] = 0.0
    params['slung_payload']['transport_link_steady_gain_scale'] = 0.0
    (output / "candidate.yaml").write_text(yaml.safe_dump(params, sort_keys=False))
    result = replay.run_simulation(output, params)
    print(json.dumps({name: result[name] for name in ("success", "failure",
        "observed_post_handoff_s", "minimum_vehicle_distance_m", "maximum_body_rate_rps")}))


if __name__ == "__main__":
    main()
