#!/usr/bin/env python3
"""Build grounded, taut physical start states from the logged handoff geometry."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml


HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "multi_ctbr_20261018_211815.csv"
RHO = np.array([[.04, 0., .025], [-.04, .03, .025], [-.04, -.03, .025]])


def vector(row, name):
    return np.array([float(row[name + "_" + axis]) for axis in "xyz"])


def prepare(use_measured_intervals=False):
    records = list(csv.DictReader(SOURCE.open()))
    handoff = next(float(row["control_time_s"]) for row in records
                   if row["flight_phase"] == "tension_ramp")
    fleet = sorted([row for row in records if abs(float(row["control_time_s"]) - handoff) < 1e-8],
                   key=lambda row: int(row["vehicle_id"]))
    center = vector(fleet[0], "payload_position")
    aircraft = np.array([vector(row, "position") for row in fleet])
    directions = np.array([vector(row, "link_direction") for row in fleet])

    def residual(rotvec):
        links = center + RHO @ Rotation.from_rotvec(rotvec).as_matrix().T - aircraft
        return (links / np.linalg.norm(links, axis=1)[:, None] - directions).ravel()

    solution = least_squares(residual, np.zeros(3), xtol=1e-12, ftol=1e-12, gtol=1e-12)
    rotation = Rotation.from_rotvec(solution.x)
    matrix = rotation.as_matrix()
    rpy = rotation.as_euler("xyz", degrees=True)
    ground_center = np.array([0., 0., np.abs(matrix[2]) @ np.array([.04, .03, .025])])
    # Preserve measured world q; impose the user-specified physical .850 m
    # length instead of copying measured 9-14 mm overlength into MuJoCo.
    initial_aircraft = ground_center + RHO @ matrix.T - .85 * directions
    measured_lengths = np.linalg.norm(center + RHO @ matrix.T - aircraft, axis=1)
    intervals_file = HERE / "control_intervals_latest.json"
    intervals = (json.loads(intervals_file.read_text())["control_intervals_s"]
                 if use_measured_intervals else [])
    cases = {}
    for name, damping in (("baseline_observed_q_damping001", .001),
                          ("baseline_observed_q_damping000", 0.)):
        candidate = {"ctbr_controller": {"control_rate_hz": 70.}, "slung_payload": {
            "mass_kg": .023, "link_lengths_m": [.85, .85, .85],
            "target_payload_height_m": .60, "post_handoff_s": 60.,
            "flight_validation_enabled": False, "link_calibration_enabled": False,
            "transport_force_mode": "coupled",
            "transport_attitude_reference_enabled": False,
            "transport_equilibrium_reference_enabled": False,
            "ramp_position_hold_gain_scale": 2., "transport_position_hold_gain_scale": .5,
            "simulation": {"initial_payload_rpy_deg": rpy.tolist(),
                "initial_vehicle_positions_m": initial_aircraft.tolist(),
                "physical_link_lengths_m": [.85, .85, .85],
                "payload_rotational_damping": damping, "ekf_sample_rate_hz": 70.,
                "control_intervals_s": intervals, "time_limit_s": 160.,
                "bridge_thrust_raw_scale": [float(row["thrust_raw_scale"]) for row in fleet]}}}
        directory = HERE / name
        directory.mkdir(exist_ok=True)
        (directory / "candidate.yaml").write_text(yaml.safe_dump(candidate, sort_keys=False))
        cases[name] = str(directory / "candidate.yaml")
    geometry = {
        "source": str(SOURCE), "source_handoff_time_s": handoff,
        "source_payload_center_m": center.tolist(), "source_aircraft_positions_m": aircraft.tolist(),
        "source_payload_rpy_deg_inferred_from_q": rpy.tolist(),
        "source_q_fit_residual_norm": float(np.linalg.norm(solution.fun)),
        "source_measured_rope_distances_m": measured_lengths.tolist(),
        "initial_payload_ground_center_m": ground_center.tolist(),
        "initial_payload_rotation": matrix.tolist(), "initial_world_q": directions.tolist(),
        "initial_vehicle_positions_m": initial_aircraft.tolist(), "physical_link_lengths_m": [.85]*3,
        "initial_minimum_vehicle_distance_m": min(float(np.linalg.norm(initial_aircraft[a]-initial_aircraft[b]))
            for a,b in ((0,1),(0,2),(1,2))),
        "caveats": [
            "The rotation fits the controller's logged q and geometry; it is not an independent attitude measurement.",
            "The source distances exceed .850m; preserve q and use .850m to avoid impossible initial tendon overlength.",
            "Grounded physical initial tilt may relax before handoff; measure actual handoff attitude in simulation_trace.csv.",
            "This is a physical geometry perturbation, not a complete replay of aircraft/payload velocities or controller state.",
            "No synthetic mocap orientation offset or descent behavior change is used."],
        "control_interval_pattern_sample_count": len(intervals), "cases": cases}
    (HERE / "start_geometry.json").write_text(json.dumps(geometry, indent=2) + "\n")
    print(json.dumps(geometry, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--measured-intervals", action="store_true",
                        help="Use the recorded fleet interval sequence instead of fixed 70 Hz")
    prepare(parser.parse_args().measured_intervals)
