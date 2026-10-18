#!/usr/bin/env python3
"""Offline hypothesis probe only: compare allocated vs projected kinematics."""

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
    parser.add_argument("mode", choices=("angular", "both"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    output = root / ("probe_projected_" + args.mode)
    output.mkdir(exist_ok=True)
    source = textwrap.dedent(inspect.getsource(load.SlungLoadController.compute))
    marker = "    if self.previous_link_units is None:\n"
    replacement = """    projected_tensions = np.maximum(-np.sum(link_units * mu_desired, axis=0), 0.0)
    projected_mu = -link_units * projected_tensions[None, :]
    projected_moment = np.sum(np.cross(self.config.attachment_points_m.T,
                                       (r0.T @ projected_mu).T), axis=0)
    omega0_dot_cmd = np.linalg.solve(
        inertia, projected_moment - np.cross(omega0, inertia @ omega0))
"""
    if args.mode == "both":
        replacement += "    kinematic_specific_force = np.sum(projected_mu, axis=1) / self.config.payload_mass_kg\n"
        assert source.count("            desired_load_acceleration + gravity\n") == 1
        source = source.replace("            desired_load_acceleration + gravity\n",
                                "            kinematic_specific_force\n")
    assert source.count(marker) == 1
    source = source.replace(marker, replacement + marker)
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
